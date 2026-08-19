//! Hardware-independent contracts for the vision harness.
//!
//! This crate is the answer to Step 1's first two findings:
//!
//! - **"Generic interface" was naming, not a boundary.** [`ports`] defines the
//!   five — and only five — replaceable seams a robot platform implements.
//! - **Single-machine binding lived in the core.** [`profile`] and
//!   [`calibration`] give measured facts a home *outside* the core, with a
//!   staleness binding that makes a copied-from-another-robot calibration
//!   detectable rather than silently trusted.
//!
//! It depends on nothing but `serde`. No hardware, no ROS, no perception maths.

pub mod calibration;
pub mod ports;
pub mod profile;

pub use calibration::{CalibrationBinding, CalibrationManifest, CalibrationStatus, RobotIdentity};
pub use ports::{
    EpisodeEvidence, Judgement, KinematicsValidator, Motion, MotionExecutor, MotionOutcome,
    Observation, ObservationSource, OutcomeJudge, ReachRequest, ReachVerdict, TaskDescriptor,
    TaskSkill,
};
pub use profile::{PlanningLimits, RobotProfile, ToolGeometry};
