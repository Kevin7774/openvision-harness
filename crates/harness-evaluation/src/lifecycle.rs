//! The rollout lifecycle: shadow, canary, promote, roll back.
//!
//! Step 1 finding #5 listed "shadow evaluation", "canary deployment" and
//! "rollback" as absent. This is the state machine, built in the same shape as
//! `xr1-vision`'s `TaskExecutive`: evidence-gated transitions, strictly increasing
//! timestamps, and terminal states that refuse further events.
//!
//! The ordering is not configurable, because each stage answers a question the
//! next one depends on:
//!
//! ```text
//! Registered --shadow--> Shadow --gate passes--> Canary --gate passes--> Promoted
//!      |                   |                        |                      |
//!      +---------- Rejected/RolledBack -------------+----------------------+
//! ```
//!
//! - **Shadow** runs the challenger without letting it command the robot, so a
//!   policy that would have broken something is discovered before it can.
//! - **Canary** gives it a bounded slice of real episodes. ADR 0005's acceptance
//!   gate -- mean time between human interventions -- can only be observed here.
//! - **Rollback is always available** from Canary and Promoted, and needs no gate.
//!   A rollback that has to argue its case is a rollback that happens too late.

use crate::promotion::PromotionDecision;

/// Where a challenger is in its rollout.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RolloutStage {
    Registered,
    Shadow,
    Canary,
    Promoted,
    RolledBack,
    Rejected,
}

impl RolloutStage {
    pub fn is_terminal(self) -> bool {
        matches!(
            self,
            RolloutStage::Promoted | RolloutStage::RolledBack | RolloutStage::Rejected
        )
    }
}

/// What happened to a challenger.
#[derive(Clone, Debug)]
pub enum RolloutEvent {
    /// Begin shadow evaluation: the challenger runs but cannot command the robot.
    ShadowStarted,
    /// A promotion gate was evaluated. Only a `Promote` advances the stage.
    GateEvaluated(PromotionDecision),
    /// Give the challenger a bounded share of live episodes.
    CanaryStarted { share_percent: u8 },
    /// Return to the previous policy. Always permitted; never gated.
    RollbackRequested { reason: String },
}

#[derive(Clone, Debug)]
pub struct RolloutRecord {
    pub at_ns: u64,
    pub event: RolloutEvent,
}

/// One challenger's rollout, including the policy it would replace.
#[derive(Clone, Debug)]
pub struct Rollout {
    challenger_policy_id: String,
    /// The policy to return to. A rollout without a rollback target is refused.
    incumbent_policy_id: String,
    stage: RolloutStage,
    canary_share_percent: Option<u8>,
    gates_passed: usize,
    last_event_at_ns: Option<u64>,
    history: Vec<String>,
}

impl Rollout {
    /// Highest share of live episodes a canary may take. A canary large enough to
    /// matter to the fleet is not a canary.
    pub const MAX_CANARY_SHARE_PERCENT: u8 = 25;

    pub fn new(challenger_policy_id: &str, incumbent_policy_id: &str) -> Result<Self, String> {
        if challenger_policy_id.trim().is_empty() || incumbent_policy_id.trim().is_empty() {
            return Err("a rollout needs both a challenger and an incumbent".into());
        }
        if challenger_policy_id == incumbent_policy_id {
            return Err("a policy cannot be its own rollback target".into());
        }
        Ok(Self {
            challenger_policy_id: challenger_policy_id.into(),
            incumbent_policy_id: incumbent_policy_id.into(),
            stage: RolloutStage::Registered,
            canary_share_percent: None,
            gates_passed: 0,
            last_event_at_ns: None,
            history: Vec::new(),
        })
    }

    pub fn stage(&self) -> RolloutStage {
        self.stage
    }

    pub fn challenger_policy_id(&self) -> &str {
        &self.challenger_policy_id
    }

    /// The policy that serves right now: the challenger only once promoted.
    pub fn serving_policy_id(&self) -> &str {
        match self.stage {
            RolloutStage::Promoted => &self.challenger_policy_id,
            _ => &self.incumbent_policy_id,
        }
    }

    pub fn canary_share_percent(&self) -> Option<u8> {
        self.canary_share_percent
    }

    pub fn history(&self) -> &[String] {
        &self.history
    }

    pub fn apply(&mut self, record: RolloutRecord) -> Result<(), String> {
        if self.stage.is_terminal() {
            return Err(format!(
                "rollout of {} is terminal at {:?}; no further events are accepted",
                self.challenger_policy_id, self.stage
            ));
        }
        if self
            .last_event_at_ns
            .is_some_and(|last| record.at_ns <= last)
        {
            return Err("rollout event timestamps must increase strictly".into());
        }

        let next = match (self.stage, &record.event) {
            // Rollback is accepted from any live stage, and is never gated.
            (_, RolloutEvent::RollbackRequested { reason }) => {
                if reason.trim().is_empty() {
                    return Err("a rollback must state a reason".into());
                }
                self.history.push(format!("rolled back: {reason}"));
                RolloutStage::RolledBack
            }
            (RolloutStage::Registered, RolloutEvent::ShadowStarted) => {
                self.history.push("shadow started".into());
                RolloutStage::Shadow
            }
            (RolloutStage::Shadow, RolloutEvent::GateEvaluated(decision)) => {
                self.record_gate(decision);
                match decision {
                    // Shadow passing does not deploy anything; it earns a canary.
                    PromotionDecision::Promote { .. } => RolloutStage::Shadow,
                    PromotionDecision::Hold { .. } => RolloutStage::Shadow,
                    PromotionDecision::Reject { .. } => RolloutStage::Rejected,
                }
            }
            (RolloutStage::Shadow, RolloutEvent::CanaryStarted { share_percent }) => {
                if self.gates_passed == 0 {
                    return Err(
                        "a canary requires a passing shadow gate; deploying an unevaluated \
                         challenger to real episodes is what shadow mode exists to prevent"
                            .into(),
                    );
                }
                self.validate_share(*share_percent)?;
                self.canary_share_percent = Some(*share_percent);
                self.history.push(format!("canary at {share_percent}%"));
                RolloutStage::Canary
            }
            (RolloutStage::Canary, RolloutEvent::GateEvaluated(decision)) => {
                self.record_gate(decision);
                match decision {
                    PromotionDecision::Promote { .. } => RolloutStage::Promoted,
                    PromotionDecision::Hold { .. } => RolloutStage::Canary,
                    PromotionDecision::Reject { .. } => RolloutStage::Rejected,
                }
            }
            (stage, event) => {
                return Err(format!(
                    "event {event:?} is invalid while the rollout is at {stage:?}"
                ));
            }
        };

        self.stage = next;
        self.last_event_at_ns = Some(record.at_ns);
        Ok(())
    }

    fn record_gate(&mut self, decision: &PromotionDecision) {
        match decision {
            PromotionDecision::Promote { margin, .. } => {
                self.gates_passed += 1;
                self.history
                    .push(format!("gate passed (margin {margin:.4})"));
            }
            PromotionDecision::Hold { reason } => self.history.push(format!("gate held: {reason}")),
            PromotionDecision::Reject { reason } => {
                self.history.push(format!("gate rejected: {reason}"))
            }
        }
    }

    fn validate_share(&self, share_percent: u8) -> Result<(), String> {
        if share_percent == 0 {
            return Err("a canary at 0% observes nothing".into());
        }
        if share_percent > Self::MAX_CANARY_SHARE_PERCENT {
            return Err(format!(
                "canary share {share_percent}% exceeds the {}% cap",
                Self::MAX_CANARY_SHARE_PERCENT
            ));
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn promote() -> PromotionDecision {
        PromotionDecision::Promote {
            margin: 0.12,
            baseline_rate: 0.30,
            challenger_rate: 0.42,
        }
    }

    fn hold() -> PromotionDecision {
        PromotionDecision::Hold {
            reason: "not enough episodes".into(),
        }
    }

    fn reject() -> PromotionDecision {
        PromotionDecision::Reject {
            reason: "challenger is worse".into(),
        }
    }

    fn event(at_ns: u64, event: RolloutEvent) -> RolloutRecord {
        RolloutRecord { at_ns, event }
    }

    fn rollout() -> Rollout {
        Rollout::new("v2", "baseline").unwrap()
    }

    #[test]
    fn the_happy_path_is_shadow_gate_canary_gate_promoted() {
        let mut rollout = rollout();
        assert_eq!(rollout.serving_policy_id(), "baseline");
        rollout
            .apply(event(1, RolloutEvent::ShadowStarted))
            .unwrap();
        rollout
            .apply(event(2, RolloutEvent::GateEvaluated(promote())))
            .unwrap();
        assert_eq!(rollout.stage(), RolloutStage::Shadow); // shadow does not deploy
        rollout
            .apply(event(3, RolloutEvent::CanaryStarted { share_percent: 10 }))
            .unwrap();
        assert_eq!(rollout.stage(), RolloutStage::Canary);
        // Still serving the incumbent while the canary runs.
        assert_eq!(rollout.serving_policy_id(), "baseline");
        rollout
            .apply(event(4, RolloutEvent::GateEvaluated(promote())))
            .unwrap();
        assert_eq!(rollout.stage(), RolloutStage::Promoted);
        assert_eq!(rollout.serving_policy_id(), "v2");
    }

    #[test]
    fn shadow_cannot_be_skipped() {
        let mut rollout = rollout();
        let error = rollout
            .apply(event(1, RolloutEvent::CanaryStarted { share_percent: 5 }))
            .unwrap_err();
        assert!(error.contains("invalid while the rollout is at Registered"));
    }

    #[test]
    fn a_canary_needs_a_passing_shadow_gate() {
        let mut rollout = rollout();
        rollout
            .apply(event(1, RolloutEvent::ShadowStarted))
            .unwrap();
        // A held gate is not a pass.
        rollout
            .apply(event(2, RolloutEvent::GateEvaluated(hold())))
            .unwrap();
        let error = rollout
            .apply(event(3, RolloutEvent::CanaryStarted { share_percent: 10 }))
            .unwrap_err();
        assert!(error.contains("requires a passing shadow gate"), "{error}");
    }

    #[test]
    fn promotion_requires_a_canary_not_just_a_shadow_pass() {
        let mut rollout = rollout();
        rollout
            .apply(event(1, RolloutEvent::ShadowStarted))
            .unwrap();
        rollout
            .apply(event(2, RolloutEvent::GateEvaluated(promote())))
            .unwrap();
        rollout
            .apply(event(3, RolloutEvent::GateEvaluated(promote())))
            .unwrap();
        // Two shadow passes still do not promote.
        assert_eq!(rollout.stage(), RolloutStage::Shadow);
        assert_eq!(rollout.serving_policy_id(), "baseline");
    }

    #[test]
    fn a_rejecting_gate_ends_the_rollout() {
        let mut rollout = rollout();
        rollout
            .apply(event(1, RolloutEvent::ShadowStarted))
            .unwrap();
        rollout
            .apply(event(2, RolloutEvent::GateEvaluated(reject())))
            .unwrap();
        assert_eq!(rollout.stage(), RolloutStage::Rejected);
        assert!(rollout
            .apply(event(3, RolloutEvent::ShadowStarted))
            .is_err());
    }

    #[test]
    fn rollback_is_available_from_shadow_and_canary_without_a_gate() {
        for stage_events in [
            vec![RolloutEvent::ShadowStarted],
            vec![
                RolloutEvent::ShadowStarted,
                RolloutEvent::GateEvaluated(promote()),
                RolloutEvent::CanaryStarted { share_percent: 10 },
            ],
        ] {
            let mut rollout = rollout();
            let mut at = 0;
            for event_kind in stage_events {
                at += 1;
                rollout.apply(event(at, event_kind)).unwrap();
            }
            at += 1;
            rollout
                .apply(event(
                    at,
                    RolloutEvent::RollbackRequested {
                        reason: "operator saw a collision".into(),
                    },
                ))
                .unwrap();
            assert_eq!(rollout.stage(), RolloutStage::RolledBack);
            assert_eq!(rollout.serving_policy_id(), "baseline");
        }
    }

    #[test]
    fn a_promoted_policy_can_still_be_rolled_back_by_a_new_rollout() {
        // Promoted is terminal for *this* rollout; reverting is a new decision
        // recorded against a fresh rollout, so the history of each stays honest.
        let mut rollout = rollout();
        for (at, kind) in [
            (1, RolloutEvent::ShadowStarted),
            (2, RolloutEvent::GateEvaluated(promote())),
            (3, RolloutEvent::CanaryStarted { share_percent: 10 }),
            (4, RolloutEvent::GateEvaluated(promote())),
        ] {
            rollout.apply(event(at, kind)).unwrap();
        }
        assert_eq!(rollout.stage(), RolloutStage::Promoted);
        assert!(rollout
            .apply(event(
                5,
                RolloutEvent::RollbackRequested {
                    reason: "regression".into()
                }
            ))
            .is_err());
        let revert = Rollout::new("baseline", "v2").unwrap();
        assert_eq!(revert.serving_policy_id(), "v2");
    }

    #[test]
    fn a_rollback_must_state_a_reason() {
        let mut rollout = rollout();
        rollout
            .apply(event(1, RolloutEvent::ShadowStarted))
            .unwrap();
        assert!(rollout
            .apply(event(
                2,
                RolloutEvent::RollbackRequested {
                    reason: "  ".into()
                }
            ))
            .is_err());
    }

    #[test]
    fn canary_share_is_bounded() {
        let mut rollout = rollout();
        rollout
            .apply(event(1, RolloutEvent::ShadowStarted))
            .unwrap();
        rollout
            .apply(event(2, RolloutEvent::GateEvaluated(promote())))
            .unwrap();
        assert!(rollout
            .apply(event(3, RolloutEvent::CanaryStarted { share_percent: 0 }))
            .is_err());
        assert!(rollout
            .apply(event(3, RolloutEvent::CanaryStarted { share_percent: 90 }))
            .is_err());
        assert!(rollout
            .apply(event(3, RolloutEvent::CanaryStarted { share_percent: 25 }))
            .is_ok());
    }

    #[test]
    fn events_must_advance_in_time() {
        let mut rollout = rollout();
        rollout
            .apply(event(5, RolloutEvent::ShadowStarted))
            .unwrap();
        assert!(rollout
            .apply(event(5, RolloutEvent::GateEvaluated(hold())))
            .is_err());
    }

    #[test]
    fn a_policy_cannot_be_its_own_rollback_target() {
        assert!(Rollout::new("v2", "v2").is_err());
        assert!(Rollout::new("", "baseline").is_err());
    }
}
