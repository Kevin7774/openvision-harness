use serde::{Deserialize, Serialize};
use std::fs;
use std::path::Path;

pub const TASK_PROPOSAL_SCHEMA_VERSION: u32 = 2;

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Task {
    Grasp,
    PickPlace,
}

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum GraspIntent {
    TopDown,
}

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum SpatialRelation {
    On,
    Inside,
    LeftOf,
    RightOf,
    InFrontOf,
    Behind,
    Near,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct ObjectQuery {
    #[serde(default)]
    pub object_id: Option<String>,
    pub description: String,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct DestinationQuery {
    pub relation: SpatialRelation,
    pub reference: ObjectQuery,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct GraspPreferences {
    pub intent: GraspIntent,
    #[serde(default)]
    pub preferred_closing_axis: Option<[f64; 3]>,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct ProposalConstraints {
    #[serde(default = "default_true")]
    pub object_visible: bool,
    #[serde(default = "default_true")]
    pub avoid_table_collision: bool,
}

impl Default for ProposalConstraints {
    fn default() -> Self {
        Self {
            object_visible: true,
            avoid_table_collision: true,
        }
    }
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum SuccessPredicate {
    TargetHeld,
    TargetAtDestination,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct TaskProposal {
    pub schema_version: u32,
    pub task: Task,
    pub command: String,
    pub target: ObjectQuery,
    #[serde(default)]
    pub destination: Option<DestinationQuery>,
    pub grasp: GraspPreferences,
    #[serde(default)]
    pub constraints: ProposalConstraints,
    pub success_predicates: Vec<SuccessPredicate>,
    #[serde(default)]
    pub reasoning: String,
    #[serde(default)]
    pub prediction: String,
}

#[derive(Clone, Debug)]
pub struct GraspPlanRequest {
    pub object_id: String,
    pub intent: GraspIntent,
    pub preferred_closing_axis: Option<[f64; 3]>,
    pub constraints: ProposalConstraints,
}

impl TaskProposal {
    pub fn yellow_block_grasp() -> Self {
        Self {
            schema_version: TASK_PROPOSAL_SCHEMA_VERSION,
            task: Task::Grasp,
            command: "grasp the yellow block".into(),
            target: ObjectQuery {
                object_id: Some("yellow_block".into()),
                description: "yellow block".into(),
            },
            destination: None,
            grasp: GraspPreferences {
                intent: GraspIntent::TopDown,
                preferred_closing_axis: None,
            },
            constraints: ProposalConstraints::default(),
            success_predicates: vec![SuccessPredicate::TargetHeld],
            reasoning:
                "Use the measured yellow-component detector and evaluate grasp geometry in Rust"
                    .into(),
            prediction:
                "At least one top-down grasp candidate will satisfy IK and contact geometry".into(),
        }
    }

    pub fn read(path: &Path) -> Result<Self, String> {
        let bytes = fs::read(path).map_err(|error| format!("{}: {error}", path.display()))?;
        let value: serde_json::Value = serde_json::from_slice(&bytes)
            .map_err(|error| format!("invalid proposal {}: {error}", path.display()))?;
        let schema_version = value
            .get("schema_version")
            .and_then(serde_json::Value::as_u64)
            .ok_or_else(|| format!("proposal {} has no schema_version", path.display()))?;
        let proposal = match schema_version {
            1 => {
                let legacy: LegacyProposal = serde_json::from_value(value).map_err(|error| {
                    format!("invalid legacy proposal {}: {error}", path.display())
                })?;
                legacy.upgrade()
            }
            version if version == u64::from(TASK_PROPOSAL_SCHEMA_VERSION) => {
                serde_json::from_value(value)
                    .map_err(|error| format!("invalid task proposal {}: {error}", path.display()))?
            }
            version => return Err(format!("unsupported proposal schema version {version}")),
        };
        proposal.validate()?;
        Ok(proposal)
    }

    pub fn validate(&self) -> Result<(), String> {
        if self.schema_version != TASK_PROPOSAL_SCHEMA_VERSION {
            return Err(format!(
                "unsupported proposal schema version {}",
                self.schema_version
            ));
        }
        if self.command.trim().is_empty() {
            return Err("proposal command must not be empty".into());
        }
        validate_query(&self.target, "target")?;
        if self.task == Task::PickPlace {
            let destination = self
                .destination
                .as_ref()
                .ok_or_else(|| "pick_place requires a destination".to_string())?;
            validate_query(&destination.reference, "destination reference")?;
        } else if self.destination.is_some() {
            return Err("grasp proposal must not include a destination".into());
        }
        if self.success_predicates.is_empty() {
            return Err("proposal must include at least one success predicate".into());
        }
        if !self
            .success_predicates
            .iter()
            .any(|predicate| matches!(predicate, SuccessPredicate::TargetHeld))
        {
            return Err("proposal success predicates must require target_held".into());
        }
        let requires_destination = self
            .success_predicates
            .iter()
            .any(|predicate| matches!(predicate, SuccessPredicate::TargetAtDestination));
        if (self.task == Task::PickPlace) != requires_destination {
            return Err(
                "target_at_destination is required exactly for pick_place proposals".into(),
            );
        }
        if !self.constraints.object_visible {
            return Err("grasp planning requires constraints.object_visible=true".into());
        }
        if !self.constraints.avoid_table_collision {
            return Err(
                "the current planner does not allow table-collision checks to be disabled".into(),
            );
        }
        validate_axis(self.grasp.preferred_closing_axis)?;
        Ok(())
    }

    pub fn grasp_request(&self) -> Result<GraspPlanRequest, String> {
        self.validate()?;
        let object_id = self.target.object_id.clone().ok_or_else(|| {
            "target must be grounded to object_id before geometry planning".to_string()
        })?;
        if object_id != "yellow_block" {
            return Err(format!(
                "object_id {object_id:?} is not supported by the current measured detector"
            ));
        }
        Ok(GraspPlanRequest {
            object_id,
            intent: self.grasp.intent,
            preferred_closing_axis: self.grasp.preferred_closing_axis,
            constraints: self.constraints.clone(),
        })
    }
}

fn validate_query(query: &ObjectQuery, name: &str) -> Result<(), String> {
    if query.description.trim().is_empty() {
        return Err(format!("{name} description must not be empty"));
    }
    if query
        .object_id
        .as_ref()
        .is_some_and(|object_id| object_id.trim().is_empty())
    {
        return Err(format!("{name} object_id must not be empty"));
    }
    Ok(())
}

fn validate_axis(axis: Option<[f64; 3]>) -> Result<(), String> {
    let Some(axis) = axis else {
        return Ok(());
    };
    if !axis.iter().all(|value| value.is_finite()) {
        return Err("preferred_closing_axis contains non-finite values".into());
    }
    let horizontal_norm = (axis[0] * axis[0] + axis[1] * axis[1]).sqrt();
    if horizontal_norm < 1e-6 {
        return Err("preferred_closing_axis must have a horizontal component".into());
    }
    Ok(())
}

#[derive(Deserialize)]
struct LegacyProposal {
    task: LegacyTask,
    object_id: String,
    intent: GraspIntent,
    #[serde(default)]
    preferred_closing_axis: Option<[f64; 3]>,
    #[serde(default)]
    constraints: ProposalConstraints,
    #[serde(default)]
    reasoning: String,
    #[serde(default)]
    prediction: String,
}

#[derive(Deserialize)]
#[serde(rename_all = "snake_case")]
enum LegacyTask {
    Grasp,
}

impl LegacyProposal {
    fn upgrade(self) -> TaskProposal {
        let _task = self.task;
        TaskProposal {
            schema_version: TASK_PROPOSAL_SCHEMA_VERSION,
            task: Task::Grasp,
            command: format!("grasp {}", self.object_id),
            target: ObjectQuery {
                description: self.object_id.replace('_', " "),
                object_id: Some(self.object_id.clone()),
            },
            destination: None,
            grasp: GraspPreferences {
                intent: self.intent,
                preferred_closing_axis: self.preferred_closing_axis,
            },
            constraints: self.constraints,
            success_predicates: vec![SuccessPredicate::TargetHeld],
            reasoning: self.reasoning,
            prediction: self.prediction,
        }
    }
}

fn default_true() -> bool {
    true
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_proposal_is_valid_and_semantic() {
        let proposal = TaskProposal::yellow_block_grasp();
        assert!(proposal.validate().is_ok());
        assert_eq!(proposal.task, Task::Grasp);
        assert_eq!(proposal.grasp.intent, GraspIntent::TopDown);
    }

    #[test]
    fn pick_place_requires_a_destination() {
        let mut proposal = TaskProposal::yellow_block_grasp();
        proposal.task = Task::PickPlace;
        assert!(proposal.validate().is_err());
    }

    #[test]
    fn unresolved_semantics_validate_but_cannot_enter_geometry() {
        let mut proposal = TaskProposal::yellow_block_grasp();
        proposal.target.object_id = None;
        assert!(proposal.validate().is_ok());
        assert!(proposal.grasp_request().is_err());
    }

    #[test]
    fn vertical_only_closing_preference_is_rejected() {
        let mut proposal = TaskProposal::yellow_block_grasp();
        proposal.grasp.preferred_closing_axis = Some([0.0, 0.0, 1.0]);
        assert!(proposal.validate().is_err());
    }

    #[test]
    fn proposal_cannot_disable_the_existing_table_constraint() {
        let mut proposal = TaskProposal::yellow_block_grasp();
        proposal.constraints.avoid_table_collision = false;
        assert!(proposal.validate().is_err());
    }
}
