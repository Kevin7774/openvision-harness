//! The immutable episode record and the append-only ledger.
//!
//! Step 1 finding #5 listed "Episode data standard" and "cross-robot data
//! isolation" as absent. Both are here, and both are shaped by ADR 0005 rather
//! than by generality:
//!
//! - **Every judgement must be made at the same head pose**, or the judge's bias
//!   drifts with head motion, and "a few tens of episodes of drift is enough to
//!   void the data stream". So an episode records the inspection pose it was
//!   judged at, and a scope pins one pose. Episodes at another pose are not
//!   comparable and are refused, not silently averaged in.
//! - **If the station moves, the workspace spec and the judge's extrinsics both
//!   silently expire, and no sensor reports it.** So an episode records
//!   `station_id` and `urdf_hash`, and a scope refuses anything outside it.
//!
//! Episodes are immutable: [`EpisodeLog`] appends and never mutates. A label is
//! attached once, by [`EpisodeLog::label`], and a second attempt is an error --
//! relabelling is how a positive feedback loop launders its own mistakes.

use serde::{Deserialize, Serialize};

/// What happened in one attempt. Observation, action and outcome are opaque
/// payloads: this crate decides *whether* an episode counts, not what it means.
#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct Episode {
    pub episode_id: String,
    /// Which physical robot produced this. Cross-robot isolation depends on it.
    pub robot_id: String,
    /// Which workstation. ADR 0005: moving the base silently expires calibration.
    pub station_id: String,
    /// The URDF the robot was running. A rebuild invalidates comparability.
    pub urdf_hash: String,
    /// The policy under test when this episode ran.
    pub policy_id: String,
    /// The fixed inspection pose the judge observed from (ADR 0005 pins
    /// `head_pitch` at the +40 degree limit for the entire loop).
    pub inspection_pose_id: String,
    pub recorded_at_ns: u64,
    pub observation: serde_json::Value,
    pub action: serde_json::Value,
    pub outcome: serde_json::Value,
    /// Attached once, after the fact. `None` until a judge has ruled.
    #[serde(default)]
    pub label: Option<Label>,
}

/// A judge's ruling plus the evidence trail behind it.
#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct Label {
    pub judgement: harness_contracts::Judgement,
    /// Which judge version ruled. A judge change must invalidate comparisons.
    pub judge_version: String,
    pub labelled_at_ns: u64,
    pub reason: String,
}

/// The set of robots, stations and URDFs an evaluation is allowed to draw from,
/// plus the one inspection pose every judgement must share.
#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct FleetScope {
    pub scope_id: String,
    /// Allowed `(robot_id, station_id)` pairs. A robot at a different station is
    /// a different measurement setup, not the same robot.
    pub stations: Vec<(String, String)>,
    pub urdf_hash: String,
    pub inspection_pose_id: String,
}

impl FleetScope {
    /// Single-robot scope: the honest default while only one robot exists.
    pub fn single(
        robot_id: &str,
        station_id: &str,
        urdf_hash: &str,
        inspection_pose_id: &str,
    ) -> Self {
        Self {
            scope_id: format!("{robot_id}@{station_id}"),
            stations: vec![(robot_id.to_string(), station_id.to_string())],
            urdf_hash: urdf_hash.to_string(),
            inspection_pose_id: inspection_pose_id.to_string(),
        }
    }

    /// Why this episode does not belong to this scope, or `Ok(())`.
    pub fn admits(&self, episode: &Episode) -> Result<(), String> {
        let pair = (episode.robot_id.clone(), episode.station_id.clone());
        if !self.stations.contains(&pair) {
            return Err(format!(
                "episode {} is from {}@{}, which is outside scope {}",
                episode.episode_id, episode.robot_id, episode.station_id, self.scope_id
            ));
        }
        if episode.urdf_hash != self.urdf_hash {
            return Err(format!(
                "episode {} ran on URDF {:?}, scope {} requires {:?}",
                episode.episode_id, episode.urdf_hash, self.scope_id, self.urdf_hash
            ));
        }
        if episode.inspection_pose_id != self.inspection_pose_id {
            return Err(format!(
                "episode {} was judged from pose {:?}, scope {} pins {:?}; judgements from \
                 different head poses are not comparable",
                episode.episode_id,
                episode.inspection_pose_id,
                self.scope_id,
                self.inspection_pose_id
            ));
        }
        Ok(())
    }
}

/// Append-only episode ledger for exactly one scope.
pub struct EpisodeLog {
    scope: FleetScope,
    episodes: Vec<Episode>,
}

impl EpisodeLog {
    pub fn new(scope: FleetScope) -> Self {
        Self {
            scope,
            episodes: Vec::new(),
        }
    }

    pub fn scope(&self) -> &FleetScope {
        &self.scope
    }

    pub fn episodes(&self) -> &[Episode] {
        &self.episodes
    }

    pub fn len(&self) -> usize {
        self.episodes.len()
    }

    pub fn is_empty(&self) -> bool {
        self.episodes.is_empty()
    }

    /// Append one episode. Refuses out-of-scope episodes, duplicate ids, and
    /// records that arrive already labelled -- a label is applied here, under
    /// this ledger's rules, or not at all.
    pub fn append(&mut self, episode: Episode) -> Result<(), String> {
        self.scope.admits(&episode)?;
        if episode.label.is_some() {
            return Err(format!(
                "episode {} arrived pre-labelled; labels are attached through the ledger",
                episode.episode_id
            ));
        }
        if self
            .episodes
            .iter()
            .any(|existing| existing.episode_id == episode.episode_id)
        {
            return Err(format!("episode {} is already recorded", episode.episode_id));
        }
        self.episodes.push(episode);
        Ok(())
    }

    /// Attach a label to a recorded episode, once. A second label is refused:
    /// silently overwriting a ruling is how a bad judge erases its own evidence.
    pub fn label(&mut self, episode_id: &str, label: Label) -> Result<(), String> {
        let episode = self
            .episodes
            .iter_mut()
            .find(|episode| episode.episode_id == episode_id)
            .ok_or_else(|| format!("episode {episode_id} is not recorded"))?;
        if let Some(existing) = &episode.label {
            return Err(format!(
                "episode {episode_id} is already labelled {:?} by judge {}; relabelling is not \
                 allowed",
                existing.judgement, existing.judge_version
            ));
        }
        episode.label = Some(label);
        Ok(())
    }

    /// Episodes for one policy that carry a decided (non-abstained) label.
    /// Abstentions are deliberately excluded: ADR 0005 keeps `uncertain` out of
    /// the training set and out of the comparison.
    pub fn decided_for(&self, policy_id: &str) -> Vec<&Episode> {
        self.episodes
            .iter()
            .filter(|episode| episode.policy_id == policy_id)
            .filter(|episode| {
                matches!(
                    episode.label.as_ref().map(|label| label.judgement),
                    Some(harness_contracts::Judgement::Success)
                        | Some(harness_contracts::Judgement::Failure)
                )
            })
            .collect()
    }

    /// Successes and decided trials for one policy.
    pub fn tally(&self, policy_id: &str) -> (usize, usize) {
        let decided = self.decided_for(policy_id);
        let successes = decided
            .iter()
            .filter(|episode| {
                episode.label.as_ref().map(|label| label.judgement)
                    == Some(harness_contracts::Judgement::Success)
            })
            .count();
        (successes, decided.len())
    }

    /// Share of labelled episodes for one policy that abstained.
    pub fn abstain_rate(&self, policy_id: &str) -> Option<f64> {
        let labelled = self
            .episodes
            .iter()
            .filter(|episode| episode.policy_id == policy_id && episode.label.is_some())
            .collect::<Vec<_>>();
        if labelled.is_empty() {
            return None;
        }
        let abstained = labelled
            .iter()
            .filter(|episode| {
                episode.label.as_ref().map(|label| label.judgement)
                    == Some(harness_contracts::Judgement::Abstain)
            })
            .count();
        Some(abstained as f64 / labelled.len() as f64)
    }
}

#[cfg(test)]
pub(crate) mod fixtures {
    use super::*;

    pub fn scope() -> FleetScope {
        FleetScope::single("xr1-thor-001", "bench-a", "urdf-abc", "head_pitch_p40")
    }

    pub fn episode(id: &str, policy_id: &str, at_ns: u64) -> Episode {
        Episode {
            episode_id: id.into(),
            robot_id: "xr1-thor-001".into(),
            station_id: "bench-a".into(),
            urdf_hash: "urdf-abc".into(),
            policy_id: policy_id.into(),
            inspection_pose_id: "head_pitch_p40".into(),
            recorded_at_ns: at_ns,
            observation: serde_json::json!({}),
            action: serde_json::json!({}),
            outcome: serde_json::json!({}),
            label: None,
        }
    }

    pub fn label(judgement: harness_contracts::Judgement) -> Label {
        Label {
            judgement,
            judge_version: "layered-v1".into(),
            labelled_at_ns: 10,
            reason: "test".into(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::fixtures::*;
    use super::*;
    use harness_contracts::Judgement;

    #[test]
    fn an_episode_from_another_robot_is_refused() {
        let mut log = EpisodeLog::new(scope());
        let mut foreign = episode("e1", "policy-a", 1);
        foreign.robot_id = "xr1-thor-002".into();
        let error = log.append(foreign).unwrap_err();
        assert!(error.contains("outside scope"));
    }

    #[test]
    fn an_episode_from_another_station_is_refused() {
        let mut log = EpisodeLog::new(scope());
        let mut moved = episode("e1", "policy-a", 1);
        moved.station_id = "bench-b".into();
        assert!(log.append(moved).is_err());
    }

    #[test]
    fn an_episode_judged_from_a_different_head_pose_is_refused() {
        // ADR 0005: every judgement at the same head pose, or the judge's bias
        // drifts with head motion.
        let mut log = EpisodeLog::new(scope());
        let mut tilted = episode("e1", "policy-a", 1);
        tilted.inspection_pose_id = "head_pitch_p10".into();
        let error = log.append(tilted).unwrap_err();
        assert!(error.contains("not comparable"));
    }

    #[test]
    fn a_rebuilt_urdf_is_refused() {
        let mut log = EpisodeLog::new(scope());
        let mut rebuilt = episode("e1", "policy-a", 1);
        rebuilt.urdf_hash = "urdf-xyz".into();
        assert!(log.append(rebuilt).is_err());
    }

    #[test]
    fn episodes_are_immutable_once_labelled() {
        let mut log = EpisodeLog::new(scope());
        log.append(episode("e1", "policy-a", 1)).unwrap();
        assert!(log.label("e1", label(Judgement::Success)).is_ok());
        let error = log.label("e1", label(Judgement::Failure)).unwrap_err();
        assert!(error.contains("relabelling is not allowed"));
    }

    #[test]
    fn pre_labelled_and_duplicate_episodes_are_refused() {
        let mut log = EpisodeLog::new(scope());
        let mut pre = episode("e1", "policy-a", 1);
        pre.label = Some(label(Judgement::Success));
        assert!(log.append(pre).is_err());

        log.append(episode("e2", "policy-a", 2)).unwrap();
        assert!(log.append(episode("e2", "policy-a", 3)).is_err());
    }

    #[test]
    fn abstentions_are_excluded_from_the_tally_but_counted_in_the_rate() {
        let mut log = EpisodeLog::new(scope());
        for (index, judgement) in [
            Judgement::Success,
            Judgement::Success,
            Judgement::Failure,
            Judgement::Abstain,
        ]
        .into_iter()
        .enumerate()
        {
            let id = format!("e{index}");
            log.append(episode(&id, "policy-a", index as u64 + 1)).unwrap();
            log.label(&id, label(judgement)).unwrap();
        }
        // 2 of 3 decided; the abstention is not a failure.
        assert_eq!(log.tally("policy-a"), (2, 3));
        assert_eq!(log.abstain_rate("policy-a"), Some(0.25));
    }
}
