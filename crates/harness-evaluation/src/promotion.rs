//! The promotion gate: may this challenger replace the baseline?
//!
//! Step 1 finding #5 listed "baseline/challenger comparison" and "automatic
//! promotion conditions" as absent. This is that gate, and every criterion traces
//! to something ADR 0005 states:
//!
//! - **The margin must beat the judge's bias.** If the observed improvement is no
//!   larger than the judge's systematic bias, the bias alone could have produced
//!   all of it. More episodes cannot fix this -- past `n*` they add no information
//!   -- so the gate reports that the *judge* must improve, not the sample size.
//! - **The margin must beat sampling noise.** Compared against the two-proportion
//!   standard error, with a minimum sample size so the normal approximation is
//!   defensible.
//! - **Abstention must be healthy.** Too high means no signal; a collapse means
//!   the judge learned to be confidently wrong, which is disqualifying rather
//!   than encouraging.
//! - **The acceptance gate is MTBH, not success rate.** ADR 0005 sets the target
//!   at mean time between human interventions >= 8 h. A policy that scores better
//!   but needs a human every hour has not improved the thing that matters.
//! - **The golden set must predate the challenger and stay out of training.**
//!
//! The gate deliberately has no "override" parameter. A promotion that needs one
//! is a human decision, made outside this type.

use crate::golden::GoldenSet;
use crate::judge::{self, AbstainHealth, AbstainMonitor, JudgeQuality};
use crate::policy::PolicyArtifact;

/// Thresholds a challenger must clear.
#[derive(Clone, Debug)]
pub struct PromotionCriteria {
    /// Minimum decided episodes per arm.
    pub min_episodes_per_arm: usize,
    /// How many standard errors the margin must exceed (1.96 ~ 95%).
    pub noise_sigmas: f64,
    /// How many times the judge's bias the margin must exceed.
    pub judge_bias_factor: f64,
    /// ADR 0005's acceptance gate, in hours.
    pub min_mtbh_hours: f64,
}

impl Default for PromotionCriteria {
    /// Defaults chosen to be defensible rather than permissive: 200 decided
    /// episodes per arm keeps the normal approximation honest at the observed
    /// p ~ 0.3, the margin must clear ~95% sampling noise, and it must be at
    /// least twice the judge's bias so bias alone cannot explain it.
    fn default() -> Self {
        Self {
            min_episodes_per_arm: 200,
            noise_sigmas: 1.96,
            judge_bias_factor: 2.0,
            min_mtbh_hours: 8.0,
        }
    }
}

/// One arm of the comparison.
#[derive(Clone, Copy, Debug)]
pub struct Arm {
    pub successes: usize,
    pub decided: usize,
}

impl Arm {
    pub fn new(successes: usize, decided: usize) -> Result<Self, String> {
        if successes > decided {
            return Err("successes cannot exceed decided episodes".into());
        }
        Ok(Self { successes, decided })
    }

    pub fn rate(&self) -> Option<f64> {
        if self.decided == 0 {
            return None;
        }
        Some(self.successes as f64 / self.decided as f64)
    }
}

/// Everything the gate needs. Assembled by the caller from the ledger, the judge
/// and the registry, so the gate itself stays a pure decision.
#[derive(Clone, Debug)]
pub struct PromotionRequest<'a> {
    pub baseline: Arm,
    pub challenger: Arm,
    pub challenger_artifact: &'a PolicyArtifact,
    pub judge: &'a JudgeQuality,
    pub golden: &'a GoldenSet,
    pub challenger_abstain_rate: f64,
    pub abstain_monitor: &'a AbstainMonitor,
    /// Observed mean time between human interventions during the canary, in hours.
    pub observed_mtbh_hours: f64,
}

/// The gate's verdict. `Hold` means "not yet, and here is what would change it";
/// `Reject` means the comparison is invalid or the challenger is worse.
#[derive(Clone, Debug, PartialEq)]
pub enum PromotionDecision {
    Promote {
        margin: f64,
        baseline_rate: f64,
        challenger_rate: f64,
    },
    Hold {
        reason: String,
    },
    Reject {
        reason: String,
    },
}

impl PromotionDecision {
    pub fn is_promote(&self) -> bool {
        matches!(self, PromotionDecision::Promote { .. })
    }
}

/// Evaluate a challenger against the baseline.
pub fn evaluate(request: &PromotionRequest<'_>, criteria: &PromotionCriteria) -> PromotionDecision {
    // Validity first: an invalid comparison is not a "not yet".
    if !request.golden.predates(request.challenger_artifact.created_at_ns) {
        return PromotionDecision::Reject {
            reason: format!(
                "golden set {} was frozen at {} but the challenger was created at {}; a golden set \
                 that postdates the challenger cannot certify it",
                request.golden.golden_set_id,
                request.golden.frozen_at_ns,
                request.challenger_artifact.created_at_ns
            ),
        };
    }
    if let Err(error) = request
        .golden
        .assert_disjoint_from_training(&request.challenger_artifact.trained_on_episode_ids)
    {
        return PromotionDecision::Reject { reason: error };
    }

    let (Some(baseline_rate), Some(challenger_rate)) =
        (request.baseline.rate(), request.challenger.rate())
    else {
        return PromotionDecision::Hold {
            reason: "both arms need at least one decided episode".into(),
        };
    };

    if request.baseline.decided < criteria.min_episodes_per_arm
        || request.challenger.decided < criteria.min_episodes_per_arm
    {
        return PromotionDecision::Hold {
            reason: format!(
                "need {} decided episodes per arm, have baseline={} challenger={}",
                criteria.min_episodes_per_arm, request.baseline.decided, request.challenger.decided
            ),
        };
    }

    // Abstention health, before any credit is given for the score itself.
    match request.abstain_monitor.assess(request.challenger_abstain_rate) {
        AbstainHealth::Healthy => {}
        AbstainHealth::TooHigh { rate, ceiling } => {
            return PromotionDecision::Hold {
                reason: format!(
                    "abstention rate {rate:.3} exceeds the usable ceiling {ceiling:.3}"
                ),
            };
        }
        AbstainHealth::SuspiciousDrop { rate, baseline } => {
            return PromotionDecision::Reject {
                reason: format!(
                    "abstention collapsed from {baseline:.3} to {rate:.3}; a sudden drop usually \
                     means the judge learned to be confidently wrong, so this comparison cannot be \
                     trusted"
                ),
            };
        }
    }

    let margin = challenger_rate - baseline_rate;
    if margin <= 0.0 {
        return PromotionDecision::Reject {
            reason: format!(
                "challenger is not better: {challenger_rate:.3} vs baseline {baseline_rate:.3}"
            ),
        };
    }

    // The judge's bias floor. Past the information ceiling more episodes cannot
    // help, so say so instead of asking for a bigger sample.
    let bias_floor = criteria.judge_bias_factor * request.judge.bias;
    if margin <= bias_floor {
        let ceiling = request
            .judge
            .information_ceiling_at(challenger_rate)
            .map(|n| format!("{n:.0}"))
            .unwrap_or_else(|| "unbounded".into());
        return PromotionDecision::Hold {
            reason: format!(
                "margin {margin:.4} does not exceed {:.1}x the judge's bias {:.4} (floor \
                 {bias_floor:.4}); judge {} adds no information past ~{ceiling} episodes at this \
                 rate, so the judge must improve rather than the sample grow",
                criteria.judge_bias_factor, request.judge.bias, request.judge.judge_version
            ),
        };
    }

    // Sampling noise on the difference of two proportions.
    let (Some(baseline_se), Some(challenger_se)) = (
        judge::standard_error(baseline_rate, request.baseline.decided),
        judge::standard_error(challenger_rate, request.challenger.decided),
    ) else {
        return PromotionDecision::Hold {
            reason: "cannot compute sampling error for these arms".into(),
        };
    };
    let difference_se = (baseline_se * baseline_se + challenger_se * challenger_se).sqrt();
    let required = criteria.noise_sigmas * difference_se;
    if margin <= required {
        return PromotionDecision::Hold {
            reason: format!(
                "margin {margin:.4} does not exceed {:.2} sigma of sampling noise ({required:.4})",
                criteria.noise_sigmas
            ),
        };
    }

    // ADR 0005: the acceptance gate is MTBH, not success rate.
    if !(request.observed_mtbh_hours.is_finite()
        && request.observed_mtbh_hours >= criteria.min_mtbh_hours)
    {
        return PromotionDecision::Hold {
            reason: format!(
                "mean time between human interventions is {:.2} h, below the {:.2} h acceptance \
                 gate; success rate is not the gate",
                request.observed_mtbh_hours, criteria.min_mtbh_hours
            ),
        };
    }

    PromotionDecision::Promote {
        margin,
        baseline_rate,
        challenger_rate,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::golden::{GoldenItem, GoldenSet};
    use harness_contracts::Judgement;

    fn golden() -> GoldenSet {
        GoldenSet::new(
            "golden-1",
            100,
            vec![GoldenItem {
                episode_id: "g1".into(),
                truth: Judgement::Success,
            }],
        )
        .unwrap()
    }

    fn artifact() -> PolicyArtifact {
        PolicyArtifact {
            policy_id: "v2".into(),
            content_hash: "hash-v2".into(),
            parent_policy_id: None,
            trained_on_episode_ids: vec!["t1".into()],
            created_at_ns: 200, // after the golden set was frozen
        }
    }

    fn judge_quality() -> JudgeQuality {
        JudgeQuality {
            judge_version: "layered-v1".into(),
            bias: 0.01,
            measured_on_samples: 400,
        }
    }

    /// A clearly-better challenger that satisfies every criterion.
    fn healthy<'a>(
        artifact: &'a PolicyArtifact,
        judge: &'a JudgeQuality,
        golden: &'a GoldenSet,
        monitor: &'a AbstainMonitor,
    ) -> PromotionRequest<'a> {
        PromotionRequest {
            baseline: Arm::new(300, 1_000).unwrap(),   // 0.300
            challenger: Arm::new(420, 1_000).unwrap(), // 0.420
            challenger_artifact: artifact,
            judge,
            golden,
            challenger_abstain_rate: 0.19,
            abstain_monitor: monitor,
            observed_mtbh_hours: 9.0,
        }
    }

    #[test]
    fn a_clearly_better_challenger_is_promoted() {
        let (a, j, g, m) = (artifact(), judge_quality(), golden(), AbstainMonitor::with_baseline(0.20));
        let decision = evaluate(&healthy(&a, &j, &g, &m), &PromotionCriteria::default());
        assert!(decision.is_promote(), "{decision:?}");
    }

    #[test]
    fn a_margin_inside_the_judges_bias_holds_and_blames_the_judge() {
        let (a, g, m) = (artifact(), golden(), AbstainMonitor::with_baseline(0.20));
        // A 5 pp-biased judge cannot resolve a 12 pp margin at 2x bias... it can.
        // Make the bias 8 pp so the 2x floor (16 pp) exceeds the 12 pp margin.
        let coarse = JudgeQuality {
            judge_version: "coarse-v0".into(),
            bias: 0.08,
            measured_on_samples: 400,
        };
        let decision = evaluate(&healthy(&a, &coarse, &g, &m), &PromotionCriteria::default());
        match decision {
            PromotionDecision::Hold { reason } => {
                assert!(reason.contains("judge must improve"), "{reason}");
                assert!(reason.contains("adds no information past"), "{reason}");
            }
            other => panic!("expected Hold, got {other:?}"),
        }
    }

    #[test]
    fn a_margin_inside_sampling_noise_holds() {
        let (a, j, g, m) = (artifact(), judge_quality(), golden(), AbstainMonitor::with_baseline(0.20));
        let mut request = healthy(&a, &j, &g, &m);
        // 3 pp apart on 200 each: above the 2 pp bias floor, below ~1.96 sigma.
        request.baseline = Arm::new(60, 200).unwrap();
        request.challenger = Arm::new(66, 200).unwrap();
        match evaluate(&request, &PromotionCriteria::default()) {
            PromotionDecision::Hold { reason } => assert!(reason.contains("sampling noise"), "{reason}"),
            other => panic!("expected Hold, got {other:?}"),
        }
    }

    #[test]
    fn too_few_episodes_holds() {
        let (a, j, g, m) = (artifact(), judge_quality(), golden(), AbstainMonitor::with_baseline(0.20));
        let mut request = healthy(&a, &j, &g, &m);
        request.baseline = Arm::new(3, 10).unwrap();
        request.challenger = Arm::new(8, 10).unwrap();
        match evaluate(&request, &PromotionCriteria::default()) {
            PromotionDecision::Hold { reason } => assert!(reason.contains("decided episodes per arm")),
            other => panic!("expected Hold, got {other:?}"),
        }
    }

    #[test]
    fn a_worse_challenger_is_rejected() {
        let (a, j, g, m) = (artifact(), judge_quality(), golden(), AbstainMonitor::with_baseline(0.20));
        let mut request = healthy(&a, &j, &g, &m);
        request.challenger = Arm::new(200, 1_000).unwrap();
        match evaluate(&request, &PromotionCriteria::default()) {
            PromotionDecision::Reject { reason } => assert!(reason.contains("not better")),
            other => panic!("expected Reject, got {other:?}"),
        }
    }

    #[test]
    fn a_collapsed_abstention_rate_rejects_even_a_better_score() {
        let (a, j, g, m) = (artifact(), judge_quality(), golden(), AbstainMonitor::with_baseline(0.20));
        let mut request = healthy(&a, &j, &g, &m);
        // Score looks great, but the judge stopped abstaining.
        request.challenger_abstain_rate = 0.01;
        match evaluate(&request, &PromotionCriteria::default()) {
            PromotionDecision::Reject { reason } => {
                assert!(reason.contains("confidently wrong"), "{reason}")
            }
            other => panic!("expected Reject, got {other:?}"),
        }
    }

    #[test]
    fn failing_the_mtbh_gate_holds_even_with_a_better_success_rate() {
        let (a, j, g, m) = (artifact(), judge_quality(), golden(), AbstainMonitor::with_baseline(0.20));
        let mut request = healthy(&a, &j, &g, &m);
        request.observed_mtbh_hours = 1.5;
        match evaluate(&request, &PromotionCriteria::default()) {
            PromotionDecision::Hold { reason } => {
                assert!(reason.contains("success rate is not the gate"), "{reason}")
            }
            other => panic!("expected Hold, got {other:?}"),
        }
    }

    #[test]
    fn a_golden_set_that_postdates_the_challenger_rejects() {
        let (j, m) = (judge_quality(), AbstainMonitor::with_baseline(0.20));
        let late = GoldenSet::new(
            "golden-late",
            5_000, // frozen long after the challenger
            vec![GoldenItem { episode_id: "g1".into(), truth: Judgement::Success }],
        )
        .unwrap();
        let a = artifact();
        match evaluate(&healthy(&a, &j, &late, &m), &PromotionCriteria::default()) {
            PromotionDecision::Reject { reason } => assert!(reason.contains("cannot certify")),
            other => panic!("expected Reject, got {other:?}"),
        }
    }

    #[test]
    fn a_challenger_trained_on_the_golden_set_rejects() {
        let (j, m) = (judge_quality(), AbstainMonitor::with_baseline(0.20));
        let g = golden();
        let mut leaky = artifact();
        leaky.trained_on_episode_ids = vec!["t1".into(), "g1".into()];
        match evaluate(&healthy(&leaky, &j, &g, &m), &PromotionCriteria::default()) {
            PromotionDecision::Reject { reason } => assert!(reason.contains("leaked into training")),
            other => panic!("expected Reject, got {other:?}"),
        }
    }

    #[test]
    fn an_arm_cannot_have_more_successes_than_trials() {
        assert!(Arm::new(11, 10).is_err());
    }
}
