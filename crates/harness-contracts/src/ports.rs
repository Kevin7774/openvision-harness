//! The five replaceable boundaries a robot platform must implement.
//!
//! These are the only traits in the harness. They exist because Step 1 found
//! that the "generic interface" was naming, not a substitutable boundary: every
//! module was exposed flat from `xr1-vision/src/lib.rs`, and the real seams
//! (where does a frame come from, who moves the arm, is this reachable, did the
//! physical result match, which capability ran) were expressed only through
//! Python filenames, environment variables, and JSON over subprocess pipes.
//!
//! A second robot — even an identical XR1 — swaps *implementations of these
//! traits* plus a [`crate::profile::RobotProfile`], and changes no core source.
//!
//! Deliberately small. The Step 1 rebuild order says "add ~five traits only at
//! genuinely replaceable boundaries, not one interface per function." Types are
//! intentionally opaque handles (`serde_json::Value`, ids) so this crate carries
//! no hardware, no ROS, and no perception maths.

use serde::{Deserialize, Serialize};

/// Where observations come from. A platform implements this over ROS images, a
/// RealSense stream, a replay directory, or a simulator — the core never knows
/// which.
pub trait ObservationSource {
    type Error;

    /// Capture one fresh, self-describing observation. The returned frame must
    /// carry its own id and capture time so downstream freshness gates work
    /// without trusting the source's clock discipline blindly.
    fn observe(&mut self) -> Result<Observation, Self::Error>;
}

/// A single captured observation, identified and timestamped so the executive
/// can reason about freshness and bind servo steps to distinct frames.
#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct Observation {
    pub frame_id: String,
    pub captured_at_ns: u64,
    /// Opaque payload (image paths, intrinsics, state). The core interprets it
    /// through a task pack, not through this contract.
    pub payload: serde_json::Value,
}

/// Who actually moves the robot, and under what envelope. A platform implements
/// this over its arm controller; the core hands it a bounded, pre-validated
/// motion and never touches a joint directly.
pub trait MotionExecutor {
    type Error;

    /// Execute exactly one bounded motion. Implementations must fail closed on
    /// stale state or a busy command channel rather than move blind.
    fn execute(&mut self, motion: &Motion) -> Result<MotionOutcome, Self::Error>;
}

/// A bounded motion request. `max_joint_delta_rad` is the caller-declared bound;
/// a [`KinematicsValidator`] must have already confirmed the target respects it.
#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct Motion {
    pub action: String,
    pub joint_targets: Vec<(String, f64)>,
    pub max_joint_delta_rad: f64,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct MotionOutcome {
    pub completed: bool,
    pub reason: String,
}

/// Is a target reachable and safe for this specific URDF and workstation? A
/// platform binds this to its own kinematics/MoveIt backend. The core asks; it
/// does not assume any particular arm geometry.
pub trait KinematicsValidator {
    type Error;

    /// Validate a candidate pose against joint limits, reach, and the floor
    /// defined by the active profile. Returns the decision plus the margins that
    /// justified it, so a rejection is auditable rather than opaque.
    fn validate(&self, request: &ReachRequest) -> Result<ReachVerdict, Self::Error>;
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct ReachRequest {
    /// Target tool pose as [x, y, z, roll, pitch, yaw] in the robot base frame.
    pub tool_pose: [f64; 6],
    pub current_joints: Vec<(String, f64)>,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct ReachVerdict {
    pub reachable: bool,
    pub min_limit_margin_rad: f64,
    pub min_tip_z_m: f64,
    pub reason: String,
}

/// Did the physical result match the predicate? This is the boundary that Step 1
/// singled out: a self-improving loop is only a data engine if the judge is much
/// better than the policy, so the judge must be allowed to say "I don't know."
pub trait OutcomeJudge {
    type Error;

    /// Judge one episode's outcome against its declared success predicate.
    /// Implementations MUST return [`Judgement::Abstain`] when the evidence is
    /// insufficient rather than guessing — a confident wrong label forms the
    /// positive feedback loop the design explicitly forbids.
    fn judge(&self, episode: &EpisodeEvidence) -> Result<Judgement, Self::Error>;
}

/// The immutable evidence for one attempt. Kept opaque here; the evaluation crate
/// (future Step) defines the concrete episode schema on top of this.
#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct EpisodeEvidence {
    pub episode_id: String,
    pub robot_id: String,
    pub predicate: String,
    pub evidence: serde_json::Value,
}

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Judgement {
    Success,
    Failure,
    /// Evidence insufficient. Not a label — an explicit refusal to label.
    Abstain,
}

/// One pluggable capability (grasp a block, pour, open a drawer). A task pack
/// implements this; the executive dispatches to it by name. The yellow-block
/// logic that Step 1 found welded into the core becomes the first implementation
/// of this trait, living in a task pack rather than in `harness-core`.
pub trait TaskSkill {
    type Error;

    /// Stable identifier the executive dispatches on (e.g. "yellow_block.pick_place").
    fn skill_id(&self) -> &str;

    /// Can this skill handle the given semantic task? Lets the executive pick a
    /// skill without hardcoding a match arm per capability.
    fn can_handle(&self, task: &TaskDescriptor) -> bool;
}

/// A semantic task, decoupled from any single object detector. `object_id` is a
/// grounding hint, not a hardcoded gate — the Step 1 finding that `object_id`
/// only ever accepted `"yellow_block"` is exactly what this indirection removes.
#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct TaskDescriptor {
    pub task: String,
    pub object_id: Option<String>,
    pub description: String,
}
