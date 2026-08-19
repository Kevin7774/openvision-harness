//! The frozen golden set.
//!
//! ADR 0005: "Keep a frozen golden set that never enters training and re-run it on
//! every judge change; without it, judge drift is invisible, and in a positive
//! feedback loop an invisible drift is fatal."
//!
//! Two properties make this more than a test fixture, and both are enforced here:
//!
//! 1. **It never enters training.** [`GoldenSet::assert_disjoint_from_training`]
//!    refuses a training set that overlaps it. A golden set that leaked into
//!    training measures memorisation, not drift.
//! 2. **It was frozen before the thing it judges.** A golden set created *after* a
//!    challenger could have been shaped by that challenger's behaviour, which is
//!    precisely how a mislabel loop certifies itself.
//!    [`GoldenSet::predates`] is the check.

use harness_contracts::Judgement;
use serde::{Deserialize, Serialize};

/// One frozen, human-established item. `truth` is never `Abstain`: a golden item
/// whose answer is unknown cannot score a judge.
#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct GoldenItem {
    pub episode_id: String,
    pub truth: Judgement,
}

/// A frozen evaluation set with a fixed creation time.
#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct GoldenSet {
    pub golden_set_id: String,
    pub frozen_at_ns: u64,
    pub items: Vec<GoldenItem>,
}

/// How a judge scored against the golden set.
#[derive(Clone, Debug, PartialEq)]
pub struct GoldenScore {
    pub scored: usize,
    pub agreed: usize,
    pub abstained: usize,
    /// Signed systematic bias: how much more often this judge says success than
    /// the truth does. Positive means optimistic.
    pub bias: f64,
}

impl GoldenScore {
    pub fn agreement_rate(&self) -> Option<f64> {
        if self.scored == 0 {
            return None;
        }
        Some(self.agreed as f64 / self.scored as f64)
    }
}

impl GoldenSet {
    pub fn new(golden_set_id: &str, frozen_at_ns: u64, items: Vec<GoldenItem>) -> Result<Self, String> {
        if items.is_empty() {
            return Err("a golden set must contain at least one item".into());
        }
        if items.iter().any(|item| item.truth == Judgement::Abstain) {
            return Err("a golden item cannot have Abstain as its truth".into());
        }
        Ok(Self {
            golden_set_id: golden_set_id.into(),
            frozen_at_ns,
            items,
        })
    }

    /// Whether this set was frozen strictly before `created_at_ns`.
    pub fn predates(&self, created_at_ns: u64) -> bool {
        self.frozen_at_ns < created_at_ns
    }

    /// Refuse a training set that overlaps the golden set.
    pub fn assert_disjoint_from_training(&self, training_episode_ids: &[String]) -> Result<(), String> {
        let leaked = self
            .items
            .iter()
            .filter(|item| training_episode_ids.contains(&item.episode_id))
            .map(|item| item.episode_id.as_str())
            .collect::<Vec<_>>();
        if leaked.is_empty() {
            Ok(())
        } else {
            Err(format!(
                "golden set {} leaked into training: {}",
                self.golden_set_id,
                leaked.join(", ")
            ))
        }
    }

    /// Score a judge's rulings against the frozen truth.
    ///
    /// `rulings` maps episode id to what the judge said. Abstentions are counted
    /// but do not score as agreement or disagreement -- an abstention is a refusal
    /// to answer, and penalising it would push the judge toward guessing.
    pub fn score(&self, rulings: &[(String, Judgement)]) -> GoldenScore {
        let mut scored = 0;
        let mut agreed = 0;
        let mut abstained = 0;
        let mut judge_successes = 0i64;
        let mut truth_successes = 0i64;
        for item in &self.items {
            let Some((_, ruling)) = rulings.iter().find(|(id, _)| *id == item.episode_id) else {
                continue;
            };
            if *ruling == Judgement::Abstain {
                abstained += 1;
                continue;
            }
            scored += 1;
            if *ruling == item.truth {
                agreed += 1;
            }
            if *ruling == Judgement::Success {
                judge_successes += 1;
            }
            if item.truth == Judgement::Success {
                truth_successes += 1;
            }
        }
        let bias = if scored == 0 {
            0.0
        } else {
            (judge_successes - truth_successes) as f64 / scored as f64
        };
        GoldenScore {
            scored,
            agreed,
            abstained,
            bias,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn golden() -> GoldenSet {
        GoldenSet::new(
            "golden-1",
            100,
            vec![
                GoldenItem { episode_id: "g1".into(), truth: Judgement::Success },
                GoldenItem { episode_id: "g2".into(), truth: Judgement::Success },
                GoldenItem { episode_id: "g3".into(), truth: Judgement::Failure },
                GoldenItem { episode_id: "g4".into(), truth: Judgement::Failure },
            ],
        )
        .unwrap()
    }

    #[test]
    fn a_golden_set_needs_items_and_cannot_have_unknown_truth() {
        assert!(GoldenSet::new("g", 1, vec![]).is_err());
        assert!(GoldenSet::new(
            "g",
            1,
            vec![GoldenItem { episode_id: "x".into(), truth: Judgement::Abstain }]
        )
        .is_err());
    }

    #[test]
    fn a_golden_set_that_leaked_into_training_is_refused() {
        let golden = golden();
        assert!(golden
            .assert_disjoint_from_training(&["t1".into(), "t2".into()])
            .is_ok());
        let error = golden
            .assert_disjoint_from_training(&["t1".into(), "g3".into()])
            .unwrap_err();
        assert!(error.contains("leaked into training"));
    }

    #[test]
    fn a_golden_set_frozen_after_the_challenger_cannot_certify_it() {
        let golden = golden(); // frozen at 100
        assert!(golden.predates(101));
        // A set created after the challenger could have been shaped by it.
        assert!(!golden.predates(100));
        assert!(!golden.predates(99));
    }

    #[test]
    fn a_perfect_judge_scores_full_agreement_and_no_bias() {
        let score = golden().score(&[
            ("g1".into(), Judgement::Success),
            ("g2".into(), Judgement::Success),
            ("g3".into(), Judgement::Failure),
            ("g4".into(), Judgement::Failure),
        ]);
        assert_eq!(score.scored, 4);
        assert_eq!(score.agreed, 4);
        assert_eq!(score.agreement_rate(), Some(1.0));
        assert_eq!(score.bias, 0.0);
    }

    #[test]
    fn an_optimistic_judge_shows_positive_bias() {
        // Calls one true failure a success: 3 successes claimed vs 2 true, over 4
        // scored items == +25 pp optimistic.
        let score = golden().score(&[
            ("g1".into(), Judgement::Success),
            ("g2".into(), Judgement::Success),
            ("g3".into(), Judgement::Success),
            ("g4".into(), Judgement::Failure),
        ]);
        assert_eq!(score.agreed, 3);
        assert!((score.bias - 0.25).abs() < 1e-9);
    }

    #[test]
    fn abstentions_are_counted_but_do_not_score_as_disagreement() {
        let score = golden().score(&[
            ("g1".into(), Judgement::Success),
            ("g2".into(), Judgement::Abstain),
            ("g3".into(), Judgement::Failure),
            ("g4".into(), Judgement::Abstain),
        ]);
        assert_eq!(score.scored, 2);
        assert_eq!(score.agreed, 2);
        assert_eq!(score.abstained, 2);
        assert_eq!(score.agreement_rate(), Some(1.0));
    }
}
