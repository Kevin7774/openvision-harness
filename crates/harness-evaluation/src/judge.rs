//! The layered judge, its bias arithmetic, and the abstention monitor.
//!
//! ADR 0005 is unusually specific about all three, so this module implements what
//! it states rather than a generic scoring interface.
//!
//! **The judge must be much better than the policy, and that is arithmetic.** At
//! success rate `p` the standard error over `n` episodes is `sqrt(p(1-p)/n)`. A
//! judge with systematic bias `b` stops adding information once noise falls to
//! `b`, at `n* = p(1-p) / b²`. Past `n*` the loop keeps burning robot hours and
//! keeps drawing a normal-looking curve while learning nothing. That is why
//! [`information_ceiling`] exists and why the promotion gate consults it.
//!
//! **The judge is layered because the sensors force it to be.** Scene-level
//! questions go to the head ZED at one fixed pose; the grasp instant goes to the
//! wrist camera plus `pos_mm` stall, which is the only non-visual independent
//! channel on the robot. When the two channels disagree the label is
//! [`Judgement::Abstain`] and the episode stays out of the comparison.
//!
//! **A falling abstention rate is a warning, not progress.** ADR 0005: "a sudden
//! drop usually means the judge learned to be confidently wrong." So
//! [`AbstainMonitor`] flags a drop, and the promotion gate treats it as
//! disqualifying rather than as an improvement.

use harness_contracts::Judgement;
use serde::{Deserialize, Serialize};

/// Episodes past which a judge of bias `b` adds no information about a policy at
/// success rate `p`: `n* = p(1-p) / b²`.
///
/// Returns `None` when `b` is zero or the inputs are not a probability -- an
/// unbiased judge has no ceiling, and a caller claiming zero bias should have to
/// handle that explicitly rather than receive `inf`.
pub fn information_ceiling(success_rate: f64, judge_bias: f64) -> Option<f64> {
    if !(0.0..=1.0).contains(&success_rate)
        || !judge_bias.is_finite()
        || judge_bias <= 0.0
        || !success_rate.is_finite()
    {
        return None;
    }
    Some(success_rate * (1.0 - success_rate) / (judge_bias * judge_bias))
}

/// Standard error of a success-rate estimate: `sqrt(p(1-p)/n)`.
pub fn standard_error(success_rate: f64, trials: usize) -> Option<f64> {
    if trials == 0 || !(0.0..=1.0).contains(&success_rate) {
        return None;
    }
    Some((success_rate * (1.0 - success_rate) / trials as f64).sqrt())
}

/// A judge's measured quality. `bias` is a *systematic* offset in probability
/// points, established against a golden set -- not a guess.
#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct JudgeQuality {
    pub judge_version: String,
    /// Systematic bias as a fraction (0.05 == 5 pp).
    pub bias: f64,
    /// Golden-set size the bias was measured on. A bias from ten samples is not
    /// a bias estimate.
    pub measured_on_samples: usize,
}

impl JudgeQuality {
    /// Episodes past which this judge cannot resolve improvement at rate `p`.
    pub fn information_ceiling_at(&self, success_rate: f64) -> Option<f64> {
        information_ceiling(success_rate, self.bias)
    }
}

/// One channel's reading. `Unavailable` is distinct from a negative reading: a
/// dead camera must not look like "not grasped".
#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum ChannelVerdict {
    Success,
    Failure,
    Unavailable,
}

/// The two independent channels ADR 0005 allows, named so a reader can tell which
/// physical capability each depends on.
#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Channel {
    /// Head ZED at the pinned inspection pose. Centimetre-scale scene geometry.
    SceneZed,
    /// Wrist camera plus `pos_mm` stall -- the only non-visual independent channel.
    GraspInstant,
}

/// Combines the two channels into one ruling, abstaining on disagreement.
#[derive(Clone, Debug)]
pub struct LayeredJudge {
    quality: JudgeQuality,
}

impl LayeredJudge {
    pub fn new(quality: JudgeQuality) -> Self {
        Self { quality }
    }

    pub fn quality(&self) -> &JudgeQuality {
        &self.quality
    }

    /// Rule on one episode from both channels.
    ///
    /// Abstains when the channels disagree, and when either channel is missing --
    /// a single-channel ruling has no cross-check, and this judge's whole claim to
    /// low bias is that two independent channels agreed.
    pub fn rule(&self, scene: ChannelVerdict, grasp: ChannelVerdict) -> (Judgement, String) {
        match (scene, grasp) {
            (ChannelVerdict::Success, ChannelVerdict::Success) => {
                (Judgement::Success, "both channels agree: success".into())
            }
            (ChannelVerdict::Failure, ChannelVerdict::Failure) => {
                (Judgement::Failure, "both channels agree: failure".into())
            }
            (ChannelVerdict::Unavailable, _) | (_, ChannelVerdict::Unavailable) => (
                Judgement::Abstain,
                "a channel was unavailable; a single-channel ruling has no cross-check".into(),
            ),
            _ => (
                Judgement::Abstain,
                format!("channels disagree (scene={scene:?}, grasp={grasp:?})"),
            ),
        }
    }
}

/// Reads the two channels out of an episode's evidence payload.
///
/// A field that is absent or not one of `"success"` / `"failure"` becomes
/// [`ChannelVerdict::Unavailable`], which abstains. Parsing leniently into
/// `Failure` would turn a malformed payload into a verdict about the robot.
fn channel_from_evidence(evidence: &serde_json::Value, field: &str) -> ChannelVerdict {
    match evidence.get(field).and_then(serde_json::Value::as_str) {
        Some("success") => ChannelVerdict::Success,
        Some("failure") => ChannelVerdict::Failure,
        _ => ChannelVerdict::Unavailable,
    }
}

/// The layered judge as an implementation of the `OutcomeJudge` port, so it plugs
/// into the contract `harness-contracts` defines rather than a bespoke interface.
///
/// Expects `evidence` to carry `scene` and `grasp` string fields. Anything else
/// abstains, which is the whole point of the port having an `Abstain` variant.
impl harness_contracts::OutcomeJudge for LayeredJudge {
    // Ruling never fails: an unreadable payload is an abstention, not an error.
    type Error = std::convert::Infallible;

    fn judge(
        &self,
        episode: &harness_contracts::EpisodeEvidence,
    ) -> Result<Judgement, Self::Error> {
        let scene = channel_from_evidence(&episode.evidence, "scene");
        let grasp = channel_from_evidence(&episode.evidence, "grasp");
        Ok(self.rule(scene, grasp).0)
    }
}

/// Watches the abstention rate for the drop ADR 0005 warns about.
#[derive(Clone, Debug)]
pub struct AbstainMonitor {
    /// The rate this judge historically abstains at, from the golden set or the
    /// incumbent's own history.
    baseline_rate: f64,
    /// How far below baseline is tolerated before the drop is called suspicious.
    drop_tolerance: f64,
    /// Highest abstention rate that still counts as a usable data stream.
    ceiling: f64,
}

/// What the monitor concluded. Only `Healthy` may support a promotion.
#[derive(Clone, Debug, PartialEq)]
pub enum AbstainHealth {
    Healthy,
    /// Abstaining so often the stream carries little signal.
    TooHigh { rate: f64, ceiling: f64 },
    /// Abstention collapsed. ADR 0005: usually the judge learned to be
    /// confidently wrong, which is exactly the mislabel feedback loop.
    SuspiciousDrop { rate: f64, baseline: f64 },
}

impl AbstainMonitor {
    pub fn new(baseline_rate: f64, drop_tolerance: f64, ceiling: f64) -> Self {
        Self {
            baseline_rate,
            drop_tolerance,
            ceiling,
        }
    }

    /// A defensible default: a judge that abstains on more than a third of
    /// episodes is not usable, and a fall of more than 10 pp below its own
    /// baseline is treated as the judge breaking rather than improving.
    pub fn with_baseline(baseline_rate: f64) -> Self {
        Self::new(baseline_rate, 0.10, 0.33)
    }

    pub fn assess(&self, rate: f64) -> AbstainHealth {
        if rate > self.ceiling {
            return AbstainHealth::TooHigh {
                rate,
                ceiling: self.ceiling,
            };
        }
        if rate + self.drop_tolerance < self.baseline_rate {
            return AbstainHealth::SuspiciousDrop {
                rate,
                baseline: self.baseline_rate,
            };
        }
        AbstainHealth::Healthy
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_information_ceiling_reproduces_adr_0005() {
        // ADR 0005's table at p = 0.30, to the integers it publishes.
        let at = |bias: f64| information_ceiling(0.30, bias).unwrap().round() as i64;
        assert_eq!(at(0.05), 84);
        assert_eq!(at(0.02), 525);
        assert_eq!(at(0.01), 2_100);
        assert_eq!(at(0.005), 8_400);
    }

    #[test]
    fn a_worse_judge_has_a_lower_ceiling() {
        // Halving the bias should quadruple the usable episode count.
        let coarse = information_ceiling(0.30, 0.04).unwrap();
        let fine = information_ceiling(0.30, 0.02).unwrap();
        assert!((fine / coarse - 4.0).abs() < 1e-9);
    }

    #[test]
    fn a_zero_or_invalid_bias_has_no_ceiling() {
        assert!(information_ceiling(0.30, 0.0).is_none());
        assert!(information_ceiling(0.30, -0.01).is_none());
        assert!(information_ceiling(1.5, 0.01).is_none());
        assert!(information_ceiling(f64::NAN, 0.01).is_none());
    }

    #[test]
    fn standard_error_shrinks_with_the_square_root_of_n() {
        let small = standard_error(0.30, 100).unwrap();
        let large = standard_error(0.30, 400).unwrap();
        assert!((small / large - 2.0).abs() < 1e-9);
        assert!(standard_error(0.30, 0).is_none());
    }

    fn judge() -> LayeredJudge {
        LayeredJudge::new(JudgeQuality {
            judge_version: "layered-v1".into(),
            bias: 0.02,
            measured_on_samples: 400,
        })
    }

    #[test]
    fn agreeing_channels_decide_and_disagreeing_channels_abstain() {
        let judge = judge();
        assert_eq!(
            judge.rule(ChannelVerdict::Success, ChannelVerdict::Success).0,
            Judgement::Success
        );
        assert_eq!(
            judge.rule(ChannelVerdict::Failure, ChannelVerdict::Failure).0,
            Judgement::Failure
        );
        // ADR 0005: when they disagree, label uncertain and keep it out.
        assert_eq!(
            judge.rule(ChannelVerdict::Success, ChannelVerdict::Failure).0,
            Judgement::Abstain
        );
        assert_eq!(
            judge.rule(ChannelVerdict::Failure, ChannelVerdict::Success).0,
            Judgement::Abstain
        );
    }

    #[test]
    fn a_missing_channel_abstains_rather_than_ruling_alone() {
        let judge = judge();
        // A dead wrist camera must not read as "not grasped".
        let (judgement, reason) = judge.rule(ChannelVerdict::Success, ChannelVerdict::Unavailable);
        assert_eq!(judgement, Judgement::Abstain);
        assert!(reason.contains("no cross-check"));
        assert_eq!(
            judge.rule(ChannelVerdict::Unavailable, ChannelVerdict::Failure).0,
            Judgement::Abstain
        );
    }

    #[test]
    fn a_collapsing_abstention_rate_is_suspicious_not_an_improvement() {
        let monitor = AbstainMonitor::with_baseline(0.20);
        assert_eq!(monitor.assess(0.18), AbstainHealth::Healthy);
        // A fall from 20% to 2% is the judge becoming confidently wrong.
        assert!(matches!(
            monitor.assess(0.02),
            AbstainHealth::SuspiciousDrop { .. }
        ));
    }

    #[test]
    fn the_port_impl_rules_from_evidence_and_abstains_on_anything_odd() {
        use harness_contracts::{EpisodeEvidence, OutcomeJudge};
        let judge = judge();
        let evidence = |value: serde_json::Value| EpisodeEvidence {
            episode_id: "e1".into(),
            robot_id: "xr1-thor-001".into(),
            predicate: "target_held".into(),
            evidence: value,
        };
        assert_eq!(
            judge
                .judge(&evidence(serde_json::json!({"scene":"success","grasp":"success"})))
                .unwrap(),
            Judgement::Success
        );
        assert_eq!(
            judge
                .judge(&evidence(serde_json::json!({"scene":"failure","grasp":"failure"})))
                .unwrap(),
            Judgement::Failure
        );
        // Disagreement, a missing channel, and a malformed channel all abstain --
        // none of them silently become "failure".
        for payload in [
            serde_json::json!({"scene":"success","grasp":"failure"}),
            serde_json::json!({"scene":"success"}),
            serde_json::json!({"scene":"success","grasp":true}),
            serde_json::json!({}),
        ] {
            assert_eq!(
                judge.judge(&evidence(payload.clone())).unwrap(),
                Judgement::Abstain,
                "{payload} must abstain"
            );
        }
    }

    #[test]
    fn an_abstention_rate_above_the_ceiling_is_unusable() {
        let monitor = AbstainMonitor::with_baseline(0.20);
        assert!(matches!(
            monitor.assess(0.60),
            AbstainHealth::TooHigh { .. }
        ));
    }
}
