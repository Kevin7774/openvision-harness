use super::types::{MotionAction, TaskEvent, TaskEventRecord, TaskSnapshot, TaskStage};
use crate::proposal::{Task, TaskProposal};
use crate::taskpack::TaskPackRegistry;
use harness_contracts::TaskDescriptor;

pub struct TaskExecutive {
    snapshot: TaskSnapshot,
    registry: TaskPackRegistry,
}

impl TaskExecutive {
    /// Create an executive backed by the shipped task packs.
    pub fn new(proposal: TaskProposal) -> Result<Self, String> {
        Self::new_with_registry(proposal, TaskPackRegistry::with_default_packs())
    }

    /// Create an executive backed by a specific registry. The executive grounds
    /// a `TargetLocked` object against this registry, so an object no pack can
    /// handle is rejected at lock time rather than discovered later in geometry.
    pub fn new_with_registry(
        proposal: TaskProposal,
        registry: TaskPackRegistry,
    ) -> Result<Self, String> {
        proposal.validate()?;
        Ok(Self {
            registry,
            snapshot: TaskSnapshot {
                schema_version: 1,
                proposal,
                stage: TaskStage::Observe,
                attempt: 1,
                observation_frame_id: None,
                target_object_id: None,
                destination_object_id: None,
                selected_grasp_rank: None,
                selected_place_rank: None,
                servo_step_count: 0,
                last_prediction_match: None,
                last_improvement_ratio: None,
                last_evidence_confidence: None,
                last_repair: None,
                failure_reason: None,
                event_count: 0,
                last_event_at_ns: None,
            },
        })
    }

    pub fn replay(
        proposal: TaskProposal,
        events: impl IntoIterator<Item = TaskEventRecord>,
    ) -> Result<TaskSnapshot, String> {
        let mut executive = Self::new(proposal)?;
        for record in events {
            executive.apply(record)?;
        }
        Ok(executive.snapshot)
    }

    pub fn apply(&mut self, record: TaskEventRecord) -> Result<(), String> {
        if self.snapshot.stage == TaskStage::Complete || self.snapshot.stage == TaskStage::Failed {
            return Err(format!(
                "task is terminal at {:?}; no further events are accepted",
                self.snapshot.stage
            ));
        }
        if self
            .snapshot
            .last_event_at_ns
            .is_some_and(|last| record.at_ns <= last)
        {
            return Err("task event timestamps must increase strictly".into());
        }

        let next_stage = match (&self.snapshot.stage, &record.event) {
            (TaskStage::Observe, TaskEvent::ObservationCaptured { frame_id }) => {
                require_nonempty(frame_id, "observation frame_id")?;
                self.snapshot.observation_frame_id = Some(frame_id.clone());
                TaskStage::LockTarget
            }
            (TaskStage::LockTarget, TaskEvent::TargetLocked { object_id }) => {
                require_nonempty(object_id, "target object_id")?;
                let expected = self
                    .snapshot
                    .proposal
                    .target
                    .object_id
                    .as_deref()
                    .ok_or_else(|| "proposal target is not grounded to object_id".to_string())?;
                if object_id != expected {
                    return Err(format!(
                        "locked target {object_id:?} does not match proposal {expected:?}"
                    ));
                }
                // Grounding now goes through the task-pack registry, not a
                // hardcoded object name. A target no pack can handle is rejected
                // here rather than surfacing later as a geometry failure.
                let descriptor = TaskDescriptor {
                    task: match self.snapshot.proposal.task {
                        Task::Grasp => "grasp".into(),
                        Task::PickPlace => "pick_place".into(),
                    },
                    object_id: Some(object_id.clone()),
                    description: self.snapshot.proposal.target.description.clone(),
                };
                if !self.registry.can_ground(&descriptor) {
                    return Err(format!(
                        "no task pack can ground locked target {object_id:?}"
                    ));
                }
                self.snapshot.target_object_id = Some(object_id.clone());
                TaskStage::Geometry
            }
            (TaskStage::Geometry, TaskEvent::GeometryReady) => TaskStage::GenerateGrasp,
            (TaskStage::GenerateGrasp, TaskEvent::GraspCandidatesGenerated { feasible_count }) => {
                if *feasible_count == 0 {
                    TaskStage::Diagnose
                } else {
                    TaskStage::ValidateGrasp
                }
            }
            (TaskStage::ValidateGrasp, TaskEvent::GraspCandidateValidated { rank }) => {
                self.snapshot.selected_grasp_rank = Some(*rank);
                TaskStage::Approach
            }
            (TaskStage::Approach, TaskEvent::MotionCompleted { action }) => {
                require_action(*action, MotionAction::Approach)?;
                TaskStage::Servo
            }
            (
                TaskStage::Servo,
                TaskEvent::ServoStepReconciled {
                    before_frame_id,
                    after_frame_id,
                    prediction_match,
                    converged,
                    stalled,
                    improvement_ratio,
                },
            ) => {
                require_nonempty(before_frame_id, "servo before_frame_id")?;
                require_nonempty(after_frame_id, "servo after_frame_id")?;
                if before_frame_id == after_frame_id {
                    return Err("servo reconciliation requires distinct frames".into());
                }
                let expected = self
                    .snapshot
                    .observation_frame_id
                    .as_deref()
                    .ok_or_else(|| "servo has no current observation frame".to_string())?;
                if before_frame_id != expected {
                    return Err(format!(
                        "servo reconciliation starts at {before_frame_id:?}, expected {expected:?}"
                    ));
                }
                if !improvement_ratio.is_finite() {
                    return Err("servo improvement_ratio must be finite".into());
                }
                if *converged && *stalled {
                    return Err("servo step cannot be both converged and stalled".into());
                }
                self.snapshot.servo_step_count = self
                    .snapshot
                    .servo_step_count
                    .checked_add(1)
                    .ok_or_else(|| "servo step counter overflow".to_string())?;
                self.snapshot.last_prediction_match = Some(*prediction_match);
                self.snapshot.last_improvement_ratio = Some(*improvement_ratio);
                self.snapshot.observation_frame_id = Some(after_frame_id.clone());
                if !prediction_match || *stalled {
                    TaskStage::Diagnose
                } else if *converged {
                    TaskStage::Grasp
                } else {
                    TaskStage::Servo
                }
            }
            (TaskStage::Grasp, TaskEvent::MotionCompleted { action }) => {
                require_action(*action, MotionAction::Grasp)?;
                TaskStage::Lift
            }
            (TaskStage::Lift, TaskEvent::MotionCompleted { action }) => {
                require_action(*action, MotionAction::Lift)?;
                TaskStage::VerifyGrasp
            }
            (
                TaskStage::VerifyGrasp,
                TaskEvent::GraspVerified {
                    object_held,
                    confidence,
                },
            ) => {
                validate_confidence(*confidence)?;
                self.snapshot.last_evidence_confidence = Some(*confidence);
                if !object_held {
                    TaskStage::Diagnose
                } else if self.snapshot.proposal.task == Task::PickPlace {
                    TaskStage::LockDestination
                } else {
                    TaskStage::Complete
                }
            }
            (TaskStage::LockDestination, TaskEvent::DestinationLocked { object_id }) => {
                require_nonempty(object_id, "destination object_id")?;
                let expected = self
                    .snapshot
                    .proposal
                    .destination
                    .as_ref()
                    .and_then(|destination| destination.reference.object_id.as_deref())
                    .ok_or_else(|| {
                        "proposal destination is not grounded to object_id".to_string()
                    })?;
                if object_id != expected {
                    return Err(format!(
                        "locked destination {object_id:?} does not match proposal {expected:?}"
                    ));
                }
                self.snapshot.destination_object_id = Some(object_id.clone());
                TaskStage::PlaceGeometry
            }
            (TaskStage::PlaceGeometry, TaskEvent::PlaceGeometryReady) => TaskStage::GeneratePlace,
            (TaskStage::GeneratePlace, TaskEvent::PlaceCandidatesGenerated { feasible_count }) => {
                if *feasible_count == 0 {
                    TaskStage::Diagnose
                } else {
                    TaskStage::ValidatePlace
                }
            }
            (TaskStage::ValidatePlace, TaskEvent::PlaceCandidateValidated { rank }) => {
                self.snapshot.selected_place_rank = Some(*rank);
                TaskStage::Place
            }
            (TaskStage::Place, TaskEvent::MotionCompleted { action }) => {
                require_action(*action, MotionAction::Place)?;
                TaskStage::VerifyPlace
            }
            (
                TaskStage::VerifyPlace,
                TaskEvent::PlaceVerified {
                    predicates_satisfied,
                    confidence,
                },
            ) => {
                validate_confidence(*confidence)?;
                self.snapshot.last_evidence_confidence = Some(*confidence);
                if *predicates_satisfied {
                    TaskStage::Complete
                } else {
                    TaskStage::Diagnose
                }
            }
            (TaskStage::Diagnose, TaskEvent::DiagnosisCompleted { resume_at, repair }) => {
                require_nonempty(repair, "repair")?;
                validate_resume_stage(*resume_at)?;
                self.snapshot.attempt = self
                    .snapshot
                    .attempt
                    .checked_add(1)
                    .ok_or_else(|| "task attempt counter overflow".to_string())?;
                self.snapshot.last_repair = Some(repair.clone());
                *resume_at
            }
            (_, TaskEvent::Aborted { reason }) => {
                require_nonempty(reason, "abort reason")?;
                self.snapshot.failure_reason = Some(reason.clone());
                TaskStage::Failed
            }
            (stage, event) => {
                return Err(format!(
                    "event {event:?} is invalid while task is at {stage:?}"
                ));
            }
        };

        self.snapshot.stage = next_stage;
        self.snapshot.event_count += 1;
        self.snapshot.last_event_at_ns = Some(record.at_ns);
        Ok(())
    }
}

fn require_nonempty(value: &str, name: &str) -> Result<(), String> {
    if value.trim().is_empty() {
        Err(format!("{name} must not be empty"))
    } else {
        Ok(())
    }
}

fn require_action(actual: MotionAction, expected: MotionAction) -> Result<(), String> {
    if actual == expected {
        Ok(())
    } else {
        Err(format!(
            "motion completion is for {actual:?}, expected {expected:?}"
        ))
    }
}

fn validate_confidence(confidence: f64) -> Result<(), String> {
    if confidence.is_finite() && (0.0..=1.0).contains(&confidence) {
        Ok(())
    } else {
        Err("evidence confidence must be finite and within [0, 1]".into())
    }
}

fn validate_resume_stage(stage: TaskStage) -> Result<(), String> {
    match stage {
        TaskStage::Observe
        | TaskStage::Geometry
        | TaskStage::PlaceGeometry
        | TaskStage::GenerateGrasp
        | TaskStage::Servo
        | TaskStage::GeneratePlace => Ok(()),
        _ => Err(format!("diagnosis cannot resume directly at {stage:?}")),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::proposal::{DestinationQuery, ObjectQuery, SpatialRelation, SuccessPredicate};

    fn event(at_ns: u64, event: TaskEvent) -> TaskEventRecord {
        TaskEventRecord { at_ns, event }
    }

    fn pick_place() -> TaskProposal {
        let mut proposal = TaskProposal::yellow_block_grasp();
        proposal.task = Task::PickPlace;
        proposal.command = "把黄色方块放到绿色托盘里面".into();
        proposal.destination = Some(DestinationQuery {
            relation: SpatialRelation::Inside,
            reference: ObjectQuery {
                object_id: Some("green_tray".into()),
                description: "绿色托盘".into(),
            },
        });
        proposal
            .success_predicates
            .push(SuccessPredicate::TargetAtDestination);
        proposal
    }

    fn executive_at_servo() -> TaskExecutive {
        let mut executive = TaskExecutive::new(TaskProposal::yellow_block_grasp()).unwrap();
        let events = [
            TaskEvent::ObservationCaptured {
                frame_id: "frame-1".into(),
            },
            TaskEvent::TargetLocked {
                object_id: "yellow_block".into(),
            },
            TaskEvent::GeometryReady,
            TaskEvent::GraspCandidatesGenerated { feasible_count: 1 },
            TaskEvent::GraspCandidateValidated { rank: 0 },
            TaskEvent::MotionCompleted {
                action: MotionAction::Approach,
            },
        ];
        for (index, item) in events.into_iter().enumerate() {
            assert!(executive.apply(event(index as u64 + 1, item)).is_ok());
        }
        executive
    }

    #[test]
    fn pick_place_happy_path_reaches_complete() {
        let events = vec![
            event(
                1,
                TaskEvent::ObservationCaptured {
                    frame_id: "frame-1".into(),
                },
            ),
            event(
                2,
                TaskEvent::TargetLocked {
                    object_id: "yellow_block".into(),
                },
            ),
            event(3, TaskEvent::GeometryReady),
            event(4, TaskEvent::GraspCandidatesGenerated { feasible_count: 2 }),
            event(5, TaskEvent::GraspCandidateValidated { rank: 0 }),
            event(
                6,
                TaskEvent::MotionCompleted {
                    action: MotionAction::Approach,
                },
            ),
            event(
                7,
                TaskEvent::ServoStepReconciled {
                    before_frame_id: "frame-1".into(),
                    after_frame_id: "frame-2".into(),
                    prediction_match: true,
                    converged: true,
                    stalled: false,
                    improvement_ratio: 0.75,
                },
            ),
            event(
                8,
                TaskEvent::MotionCompleted {
                    action: MotionAction::Grasp,
                },
            ),
            event(
                9,
                TaskEvent::MotionCompleted {
                    action: MotionAction::Lift,
                },
            ),
            event(
                10,
                TaskEvent::GraspVerified {
                    object_held: true,
                    confidence: 0.9,
                },
            ),
            event(
                11,
                TaskEvent::DestinationLocked {
                    object_id: "green_tray".into(),
                },
            ),
            event(12, TaskEvent::PlaceGeometryReady),
            event(
                13,
                TaskEvent::PlaceCandidatesGenerated { feasible_count: 1 },
            ),
            event(14, TaskEvent::PlaceCandidateValidated { rank: 0 }),
            event(
                15,
                TaskEvent::MotionCompleted {
                    action: MotionAction::Place,
                },
            ),
            event(
                16,
                TaskEvent::PlaceVerified {
                    predicates_satisfied: true,
                    confidence: 0.8,
                },
            ),
        ];
        let result = TaskExecutive::replay(pick_place(), events);
        assert!(result.is_ok());
        assert_eq!(
            result.ok().map(|snapshot| snapshot.stage),
            Some(TaskStage::Complete)
        );
    }

    #[test]
    fn failed_evidence_enters_diagnosis_and_minimal_repair() {
        let mut executive = TaskExecutive::new(TaskProposal::yellow_block_grasp()).unwrap();
        let events = [
            TaskEvent::ObservationCaptured {
                frame_id: "frame-1".into(),
            },
            TaskEvent::TargetLocked {
                object_id: "yellow_block".into(),
            },
            TaskEvent::GeometryReady,
            TaskEvent::GraspCandidatesGenerated { feasible_count: 1 },
            TaskEvent::GraspCandidateValidated { rank: 0 },
            TaskEvent::MotionCompleted {
                action: MotionAction::Approach,
            },
            TaskEvent::ServoStepReconciled {
                before_frame_id: "frame-1".into(),
                after_frame_id: "frame-2".into(),
                prediction_match: true,
                converged: true,
                stalled: false,
                improvement_ratio: 0.75,
            },
            TaskEvent::MotionCompleted {
                action: MotionAction::Grasp,
            },
            TaskEvent::MotionCompleted {
                action: MotionAction::Lift,
            },
            TaskEvent::GraspVerified {
                object_held: false,
                confidence: 0.95,
            },
        ];
        for (index, item) in events.into_iter().enumerate() {
            assert!(executive.apply(event(index as u64 + 1, item)).is_ok());
        }
        assert_eq!(executive.snapshot.stage, TaskStage::Diagnose);
        assert!(executive
            .apply(event(
                11,
                TaskEvent::DiagnosisCompleted {
                    resume_at: TaskStage::Servo,
                    repair: "re-observe and correct closing-axis alignment".into(),
                }
            ))
            .is_ok());
        assert_eq!(executive.snapshot.stage, TaskStage::Servo);
        assert_eq!(executive.snapshot.attempt, 2);
    }

    #[test]
    fn out_of_order_event_is_rejected() {
        let mut executive = TaskExecutive::new(TaskProposal::yellow_block_grasp()).unwrap();
        assert!(executive.apply(event(1, TaskEvent::GeometryReady)).is_err());
    }

    #[test]
    fn reconciled_servo_steps_bind_frames_and_drive_the_state_machine() {
        let mut executive = executive_at_servo();
        assert!(executive
            .apply(event(
                7,
                TaskEvent::ServoStepReconciled {
                    before_frame_id: "frame-1".into(),
                    after_frame_id: "frame-2".into(),
                    prediction_match: true,
                    converged: false,
                    stalled: false,
                    improvement_ratio: 0.45,
                },
            ))
            .is_ok());
        assert_eq!(executive.snapshot.stage, TaskStage::Servo);
        assert_eq!(
            executive.snapshot.observation_frame_id.as_deref(),
            Some("frame-2")
        );
        assert_eq!(executive.snapshot.servo_step_count, 1);

        assert!(executive
            .apply(event(
                8,
                TaskEvent::ServoStepReconciled {
                    before_frame_id: "frame-2".into(),
                    after_frame_id: "frame-3".into(),
                    prediction_match: true,
                    converged: true,
                    stalled: false,
                    improvement_ratio: 0.70,
                },
            ))
            .is_ok());
        assert_eq!(executive.snapshot.stage, TaskStage::Grasp);
        assert_eq!(executive.snapshot.servo_step_count, 2);
    }

    #[test]
    fn servo_prediction_mismatch_enters_diagnosis() {
        let mut executive = executive_at_servo();
        assert!(executive
            .apply(event(
                7,
                TaskEvent::ServoStepReconciled {
                    before_frame_id: "frame-1".into(),
                    after_frame_id: "frame-2".into(),
                    prediction_match: false,
                    converged: false,
                    stalled: false,
                    improvement_ratio: -0.25,
                },
            ))
            .is_ok());
        assert_eq!(executive.snapshot.stage, TaskStage::Diagnose);
        assert_eq!(executive.snapshot.last_prediction_match, Some(false));
    }
}
