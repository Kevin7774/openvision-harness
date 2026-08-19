use crate::proposal::TaskProposal;
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum TaskStage {
    Observe,
    LockTarget,
    Geometry,
    GenerateGrasp,
    ValidateGrasp,
    Approach,
    Servo,
    Grasp,
    Lift,
    VerifyGrasp,
    LockDestination,
    PlaceGeometry,
    GeneratePlace,
    ValidatePlace,
    Place,
    VerifyPlace,
    Diagnose,
    Complete,
    Failed,
}

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum MotionAction {
    Approach,
    Grasp,
    Lift,
    Place,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(tag = "event", rename_all = "snake_case")]
pub enum TaskEvent {
    ObservationCaptured {
        frame_id: String,
    },
    TargetLocked {
        object_id: String,
    },
    DestinationLocked {
        object_id: String,
    },
    GeometryReady,
    PlaceGeometryReady,
    GraspCandidatesGenerated {
        feasible_count: usize,
    },
    GraspCandidateValidated {
        rank: usize,
    },
    MotionCompleted {
        action: MotionAction,
    },
    ServoConverged,
    GraspVerified {
        object_held: bool,
        confidence: f64,
    },
    PlaceCandidatesGenerated {
        feasible_count: usize,
    },
    PlaceCandidateValidated {
        rank: usize,
    },
    PlaceVerified {
        predicates_satisfied: bool,
        confidence: f64,
    },
    DiagnosisCompleted {
        resume_at: TaskStage,
        repair: String,
    },
    Aborted {
        reason: String,
    },
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct TaskEventRecord {
    pub at_ns: u64,
    #[serde(flatten)]
    pub event: TaskEvent,
}

#[derive(Clone, Debug, Serialize)]
pub struct TaskSnapshot {
    pub schema_version: u32,
    pub proposal: TaskProposal,
    pub stage: TaskStage,
    pub attempt: u32,
    pub observation_frame_id: Option<String>,
    pub target_object_id: Option<String>,
    pub destination_object_id: Option<String>,
    pub selected_grasp_rank: Option<usize>,
    pub selected_place_rank: Option<usize>,
    pub last_evidence_confidence: Option<f64>,
    pub last_repair: Option<String>,
    pub failure_reason: Option<String>,
    pub event_count: usize,
    pub last_event_at_ns: Option<u64>,
}
