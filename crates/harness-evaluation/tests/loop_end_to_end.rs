//! Drives the whole promotion path end to end, the way an operator would: record
//! episodes, judge them through the port, compare arms, and walk the rollout.
//!
//! These tests exist because the unit tests prove each piece in isolation. What
//! they cannot show is that the pieces compose -- that a ledger's tally is what
//! the gate wants, and that the gate's verdict is what the lifecycle consumes.

use harness_contracts::{EpisodeEvidence, Judgement, OutcomeJudge};
use harness_evaluation::{
    evaluate, AbstainMonitor, Arm, Episode, EpisodeLog, FleetScope, GoldenItem, GoldenSet,
    JudgeQuality, Label, LayeredJudge, PolicyArtifact, PolicyRegistry, PromotionCriteria,
    PromotionDecision, PromotionRequest, Rollout, RolloutEvent, RolloutRecord, RolloutStage,
};

const POSE: &str = "head_pitch_p40";

fn scope() -> FleetScope {
    FleetScope::single("xr1-thor-001", "bench-a", "urdf-abc", POSE)
}

fn episode(id: &str, policy_id: &str, at_ns: u64) -> Episode {
    Episode {
        episode_id: id.into(),
        robot_id: "xr1-thor-001".into(),
        station_id: "bench-a".into(),
        urdf_hash: "urdf-abc".into(),
        policy_id: policy_id.into(),
        inspection_pose_id: POSE.into(),
        recorded_at_ns: at_ns,
        observation: serde_json::json!({"frame": id}),
        action: serde_json::json!({"grasp": "top_down"}),
        outcome: serde_json::json!({"pos_mm": 149}),
        label: None,
    }
}

fn judge() -> LayeredJudge {
    LayeredJudge::new(JudgeQuality {
        judge_version: "layered-v1".into(),
        bias: 0.01,
        measured_on_samples: 400,
    })
}

/// Record `total` episodes for `policy_id`, of which `successes` had both channels
/// agree on success and `abstentions` had the wrist channel drop out. Labels are
/// produced by the judge through the `OutcomeJudge` port, not hand-written.
fn run_arm(
    log: &mut EpisodeLog,
    judge: &LayeredJudge,
    policy_id: &str,
    successes: usize,
    total: usize,
    abstentions: usize,
    id_offset: u64,
) {
    for index in 0..total {
        let id = format!("{policy_id}-e{index}");
        let at = id_offset + index as u64 + 1;
        log.append(episode(&id, policy_id, at)).unwrap();

        let evidence = if index < abstentions {
            // Wrist camera unavailable: the judge must abstain, not guess.
            serde_json::json!({ "scene": "success" })
        } else if index < abstentions + successes {
            serde_json::json!({ "scene": "success", "grasp": "success" })
        } else {
            serde_json::json!({ "scene": "failure", "grasp": "failure" })
        };
        let judgement = judge
            .judge(&EpisodeEvidence {
                episode_id: id.clone(),
                robot_id: "xr1-thor-001".into(),
                predicate: "target_held".into(),
                evidence,
            })
            .unwrap();
        log.label(
            &id,
            Label {
                judgement,
                judge_version: judge.quality().judge_version.clone(),
                labelled_at_ns: at + 1,
                reason: "layered channels".into(),
            },
        )
        .unwrap();
    }
}

fn golden_set() -> GoldenSet {
    GoldenSet::new(
        "golden-1",
        100,
        vec![
            GoldenItem { episode_id: "gold-1".into(), truth: Judgement::Success },
            GoldenItem { episode_id: "gold-2".into(), truth: Judgement::Failure },
        ],
    )
    .unwrap()
}

#[test]
fn a_better_challenger_walks_shadow_canary_and_promotion() {
    let judge = judge();
    let mut log = EpisodeLog::new(scope());

    // Baseline at ~30% over 1000 decided; challenger at ~42%. Both abstain on
    // 200 of 1200 recorded episodes, so the rate is steady at ~17%.
    run_arm(&mut log, &judge, "baseline", 300, 1_200, 200, 0);
    run_arm(&mut log, &judge, "v2", 420, 1_200, 200, 10_000);

    let (baseline_successes, baseline_decided) = log.tally("baseline");
    let (challenger_successes, challenger_decided) = log.tally("v2");
    assert_eq!((baseline_successes, baseline_decided), (300, 1_000));
    assert_eq!((challenger_successes, challenger_decided), (420, 1_000));

    // The abstentions are visible and identical across arms.
    let baseline_abstain = log.abstain_rate("baseline").unwrap();
    let challenger_abstain = log.abstain_rate("v2").unwrap();
    assert!((baseline_abstain - challenger_abstain).abs() < 1e-12);

    let mut registry = PolicyRegistry::new();
    registry
        .register(PolicyArtifact {
            policy_id: "baseline".into(),
            content_hash: "hash-baseline".into(),
            parent_policy_id: None,
            trained_on_episode_ids: vec![],
            created_at_ns: 150,
        })
        .unwrap();
    let challenger = PolicyArtifact {
        policy_id: "v2".into(),
        content_hash: "hash-v2".into(),
        parent_policy_id: Some("baseline".into()),
        trained_on_episode_ids: vec!["baseline-e0".into()],
        created_at_ns: 200,
    };
    registry.register(challenger.clone()).unwrap();
    registry.activate("baseline").unwrap();

    let golden = golden_set();
    let monitor = AbstainMonitor::with_baseline(baseline_abstain);
    let criteria = PromotionCriteria::default();
    let request = PromotionRequest {
        baseline: Arm::new(baseline_successes, baseline_decided).unwrap(),
        challenger: Arm::new(challenger_successes, challenger_decided).unwrap(),
        challenger_artifact: &challenger,
        judge: judge.quality(),
        golden: &golden,
        challenger_abstain_rate: challenger_abstain,
        abstain_monitor: &monitor,
        observed_mtbh_hours: 9.0,
    };

    let mut rollout = Rollout::new("v2", "baseline").unwrap();
    rollout
        .apply(RolloutRecord { at_ns: 1, event: RolloutEvent::ShadowStarted })
        .unwrap();

    // Shadow gate passes, but nothing is deployed yet.
    let shadow_decision = evaluate(&request, &criteria);
    assert!(shadow_decision.is_promote(), "{shadow_decision:?}");
    rollout
        .apply(RolloutRecord { at_ns: 2, event: RolloutEvent::GateEvaluated(shadow_decision) })
        .unwrap();
    assert_eq!(rollout.stage(), RolloutStage::Shadow);
    assert_eq!(rollout.serving_policy_id(), "baseline");

    // Canary takes a bounded slice; the incumbent still serves.
    rollout
        .apply(RolloutRecord {
            at_ns: 3,
            event: RolloutEvent::CanaryStarted { share_percent: 10 },
        })
        .unwrap();
    assert_eq!(rollout.serving_policy_id(), "baseline");

    // Canary gate passes, and only now does the challenger serve.
    rollout
        .apply(RolloutRecord { at_ns: 4, event: RolloutEvent::GateEvaluated(evaluate(&request, &criteria)) })
        .unwrap();
    assert_eq!(rollout.stage(), RolloutStage::Promoted);
    assert_eq!(rollout.serving_policy_id(), "v2");

    registry.activate(rollout.serving_policy_id()).unwrap();
    assert_eq!(registry.active().map(|a| a.policy_id.as_str()), Some("v2"));

    // The rollback target is a real, registered artifact with resolvable lineage.
    let lineage = registry.lineage("v2").unwrap();
    assert_eq!(
        lineage.iter().map(|a| a.policy_id.as_str()).collect::<Vec<_>>(),
        vec!["v2", "baseline"]
    );
}

#[test]
fn a_judge_that_stops_abstaining_cannot_promote_even_with_a_better_score() {
    // The mislabel feedback loop: the challenger's arm looks better, but it looks
    // better because the judge stopped saying "uncertain".
    let judge = judge();
    let mut log = EpisodeLog::new(scope());
    run_arm(&mut log, &judge, "baseline", 300, 1_200, 200, 0);
    // Same success count, but abstentions collapse from 200 to 5.
    run_arm(&mut log, &judge, "v2", 420, 1_200, 5, 10_000);

    let baseline_abstain = log.abstain_rate("baseline").unwrap();
    let challenger_abstain = log.abstain_rate("v2").unwrap();
    assert!(challenger_abstain < baseline_abstain);

    let challenger = PolicyArtifact {
        policy_id: "v2".into(),
        content_hash: "hash-v2".into(),
        parent_policy_id: None,
        trained_on_episode_ids: vec![],
        created_at_ns: 200,
    };
    let golden = golden_set();
    let monitor = AbstainMonitor::with_baseline(baseline_abstain);
    let (bs, bd) = log.tally("baseline");
    let (cs, cd) = log.tally("v2");

    let decision = evaluate(
        &PromotionRequest {
            baseline: Arm::new(bs, bd).unwrap(),
            challenger: Arm::new(cs, cd).unwrap(),
            challenger_artifact: &challenger,
            judge: judge.quality(),
            golden: &golden,
            challenger_abstain_rate: challenger_abstain,
            abstain_monitor: &monitor,
            observed_mtbh_hours: 9.0,
        },
        &PromotionCriteria::default(),
    );
    match decision {
        PromotionDecision::Reject { reason } => {
            assert!(reason.contains("confidently wrong"), "{reason}")
        }
        other => panic!("a collapsed abstention rate must not promote, got {other:?}"),
    }
}

#[test]
fn a_canary_regression_rolls_back_to_the_incumbent() {
    let mut rollout = Rollout::new("v2", "baseline").unwrap();
    rollout
        .apply(RolloutRecord { at_ns: 1, event: RolloutEvent::ShadowStarted })
        .unwrap();
    rollout
        .apply(RolloutRecord {
            at_ns: 2,
            event: RolloutEvent::GateEvaluated(PromotionDecision::Promote {
                margin: 0.12,
                baseline_rate: 0.30,
                challenger_rate: 0.42,
            }),
        })
        .unwrap();
    rollout
        .apply(RolloutRecord { at_ns: 3, event: RolloutEvent::CanaryStarted { share_percent: 10 } })
        .unwrap();
    // Rollback needs no gate and no argument beyond a stated reason.
    rollout
        .apply(RolloutRecord {
            at_ns: 4,
            event: RolloutEvent::RollbackRequested {
                reason: "canary drove the fingertip into the table".into(),
            },
        })
        .unwrap();
    assert_eq!(rollout.stage(), RolloutStage::RolledBack);
    assert_eq!(rollout.serving_policy_id(), "baseline");
    assert!(rollout
        .history()
        .iter()
        .any(|entry| entry.contains("rolled back")));
}

#[test]
fn episodes_from_a_second_robot_cannot_enter_the_first_robots_evaluation() {
    let mut log = EpisodeLog::new(scope());
    let mut other = episode("foreign-1", "v2", 1);
    other.robot_id = "xr1-thor-002".into();
    assert!(log.append(other).is_err());
    // And a robot moved to another bench is a different measurement setup.
    let mut moved = episode("moved-1", "v2", 2);
    moved.station_id = "bench-b".into();
    assert!(log.append(moved).is_err());
    assert!(log.is_empty());
}
