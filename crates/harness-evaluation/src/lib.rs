//! Evaluation and rollout for the self-improving loop.
//!
//! Step 1 of the harness assessment scored self-evolution 0.5/10: the reasoning
//! existed in ADR 0005, but none of the machinery existed in code. This crate is
//! the part of that machinery which is decidable in software:
//!
//! | Step 1 gap | Here |
//! |---|---|
//! | Episode data standard | [`episode::Episode`], [`episode::EpisodeLog`] |
//! | Outcome judge with abstain | [`judge::LayeredJudge`] over the `OutcomeJudge` port |
//! | Cross-robot data isolation | [`episode::FleetScope`] |
//! | Mislabel feedback guard | [`judge::AbstainMonitor`], [`golden::GoldenSet`] |
//! | Policy/model registry | [`policy::PolicyRegistry`] |
//! | Baseline/challenger comparison | [`promotion::evaluate`] |
//! | Automatic promotion criteria | [`promotion::PromotionCriteria`] |
//! | Shadow / canary / rollback | [`lifecycle::Rollout`] |
//!
//! # What this crate does not do
//!
//! **It does not train anything.** There is no model, no optimiser, and no
//! gradient here. A challenger arrives as a [`policy::PolicyArtifact`] that some
//! offline process produced; this crate decides whether that artifact is allowed
//! to serve. Training belongs outside, on frozen data, exactly as ADR 0005
//! requires -- and building it needs episodes that only a robot can produce.
//!
//! **It does not make the loop autonomous.** Every type here is a decision
//! procedure over evidence someone else collected. Nothing in this crate moves a
//! robot, and nothing edits source on one: ADR 0005's loop is offline-trained,
//! gated, and reversible, not an agent rewriting the harness in place.
//!
//! So the honest claim after this crate exists is *"the promotion path is
//! implemented and tested"*, not *"the harness self-evolves"*. The latter needs a
//! fleet, a trainer, and the automatic reset that ADR 0005 identifies as the real
//! ceiling.

pub mod episode;
pub mod golden;
pub mod judge;
pub mod lifecycle;
pub mod policy;
pub mod promotion;

pub use episode::{Episode, EpisodeLog, FleetScope, Label};
pub use golden::{GoldenItem, GoldenScore, GoldenSet};
pub use judge::{
    information_ceiling, standard_error, AbstainHealth, AbstainMonitor, Channel, ChannelVerdict,
    JudgeQuality, LayeredJudge,
};
pub use lifecycle::{Rollout, RolloutEvent, RolloutRecord, RolloutStage};
pub use policy::{PolicyArtifact, PolicyRegistry};
pub use promotion::{evaluate, Arm, PromotionCriteria, PromotionDecision, PromotionRequest};
