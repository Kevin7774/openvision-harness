use serde::{Deserialize, Serialize};
use std::fs;
use std::path::Path;

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Task {
    Grasp,
}

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum GraspIntent {
    TopDown,
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
pub struct VisionHarnessProposal {
    pub schema_version: u32,
    pub task: Task,
    pub object_id: String,
    pub intent: GraspIntent,
    #[serde(default)]
    pub preferred_closing_axis: Option<[f64; 3]>,
    #[serde(default)]
    pub constraints: ProposalConstraints,
    #[serde(default)]
    pub reasoning: String,
    #[serde(default)]
    pub prediction: String,
}

impl VisionHarnessProposal {
    pub fn yellow_block() -> Self {
        Self {
            schema_version: 1,
            task: Task::Grasp,
            object_id: "yellow_block".into(),
            intent: GraspIntent::TopDown,
            preferred_closing_axis: None,
            constraints: ProposalConstraints::default(),
            reasoning:
                "Use the measured yellow-component detector and evaluate grasp geometry in Rust"
                    .into(),
            prediction:
                "At least one top-down grasp candidate will satisfy IK and contact geometry".into(),
        }
    }

    pub fn read(path: &Path) -> Result<Self, String> {
        let bytes = fs::read(path).map_err(|error| format!("{}: {error}", path.display()))?;
        let proposal: Self = serde_json::from_slice(&bytes)
            .map_err(|error| format!("invalid proposal {}: {error}", path.display()))?;
        proposal.validate()?;
        Ok(proposal)
    }

    pub fn validate(&self) -> Result<(), String> {
        if self.schema_version != 1 {
            return Err(format!(
                "unsupported proposal schema version {}",
                self.schema_version
            ));
        }
        if self.object_id.trim().is_empty() {
            return Err("proposal object_id must not be empty".into());
        }
        if self.object_id != "yellow_block" {
            return Err(format!(
                "object_id {:?} is not supported by the current measured detector",
                self.object_id
            ));
        }
        if !self.constraints.object_visible {
            return Err("grasp planning requires constraints.object_visible=true".into());
        }
        if !self.constraints.avoid_table_collision {
            return Err(
                "the current planner does not allow table-collision checks to be disabled".into(),
            );
        }
        if let Some(axis) = self.preferred_closing_axis {
            if !axis.iter().all(|value| value.is_finite()) {
                return Err("preferred_closing_axis contains non-finite values".into());
            }
            let horizontal_norm = (axis[0] * axis[0] + axis[1] * axis[1]).sqrt();
            if horizontal_norm < 1e-6 {
                return Err("preferred_closing_axis must have a horizontal component".into());
            }
        }
        Ok(())
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
        let proposal = VisionHarnessProposal::yellow_block();
        assert!(proposal.validate().is_ok());
        assert_eq!(proposal.task, Task::Grasp);
        assert_eq!(proposal.intent, GraspIntent::TopDown);
    }

    #[test]
    fn vertical_only_closing_preference_is_rejected() {
        let mut proposal = VisionHarnessProposal::yellow_block();
        proposal.preferred_closing_axis = Some([0.0, 0.0, 1.0]);
        assert!(proposal.validate().is_err());
    }

    #[test]
    fn proposal_cannot_disable_the_existing_table_constraint() {
        let mut proposal = VisionHarnessProposal::yellow_block();
        proposal.constraints.avoid_table_collision = false;
        assert!(proposal.validate().is_err());
    }
}
