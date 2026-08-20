//! Policy artifacts and their registry.
//!
//! Step 1 finding #5 listed "Policy/model registry" as absent. A policy here is an
//! *artifact*, not a running process: an id, a content hash, the dataset it was
//! trained on, and its parent. Lineage matters because a rollback target must be a
//! real prior artifact, and because "trained on the golden set" has to be a
//! checkable claim.

use serde::{Deserialize, Serialize};

/// An immutable, content-addressed policy artifact.
#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct PolicyArtifact {
    pub policy_id: String,
    /// Hash of the artifact bytes. Two artifacts with one id and different
    /// hashes are a mistake, not a version.
    pub content_hash: String,
    /// The policy this was trained from, if any. `None` marks the baseline.
    pub parent_policy_id: Option<String>,
    /// Episode ids this was trained on. Checked against the golden set.
    pub trained_on_episode_ids: Vec<String>,
    pub created_at_ns: u64,
}

/// Registry of known artifacts, plus which one is currently serving.
#[derive(Debug, Default)]
pub struct PolicyRegistry {
    artifacts: Vec<PolicyArtifact>,
    active_policy_id: Option<String>,
}

impl PolicyRegistry {
    pub fn new() -> Self {
        Self::default()
    }

    /// Register an artifact. Refuses a duplicate id, a content-hash conflict, and
    /// a parent that is not registered -- lineage must be resolvable.
    pub fn register(&mut self, artifact: PolicyArtifact) -> Result<(), String> {
        if let Some(existing) = self.get(&artifact.policy_id) {
            return if existing.content_hash == artifact.content_hash {
                Err(format!(
                    "policy {} is already registered",
                    artifact.policy_id
                ))
            } else {
                Err(format!(
                    "policy {} is already registered with content hash {:?}, refusing {:?}",
                    artifact.policy_id, existing.content_hash, artifact.content_hash
                ))
            };
        }
        if let Some(parent) = &artifact.parent_policy_id {
            if self.get(parent).is_none() {
                return Err(format!(
                    "policy {} names parent {parent}, which is not registered",
                    artifact.policy_id
                ));
            }
        }
        self.artifacts.push(artifact);
        Ok(())
    }

    pub fn get(&self, policy_id: &str) -> Option<&PolicyArtifact> {
        self.artifacts
            .iter()
            .find(|artifact| artifact.policy_id == policy_id)
    }

    pub fn active(&self) -> Option<&PolicyArtifact> {
        self.active_policy_id
            .as_deref()
            .and_then(|policy_id| self.get(policy_id))
    }

    /// Make an artifact the serving policy. Only a registered artifact can serve.
    pub fn activate(&mut self, policy_id: &str) -> Result<(), String> {
        if self.get(policy_id).is_none() {
            return Err(format!("policy {policy_id} is not registered"));
        }
        self.active_policy_id = Some(policy_id.to_string());
        Ok(())
    }

    /// The chain from an artifact back to the baseline, nearest parent first.
    /// Returns an error on a lineage cycle rather than looping forever.
    pub fn lineage(&self, policy_id: &str) -> Result<Vec<&PolicyArtifact>, String> {
        let mut chain = Vec::new();
        let mut seen = Vec::new();
        let mut current = Some(policy_id.to_string());
        while let Some(id) = current {
            if seen.contains(&id) {
                return Err(format!(
                    "policy lineage for {policy_id} contains a cycle at {id}"
                ));
            }
            seen.push(id.clone());
            let artifact = self
                .get(&id)
                .ok_or_else(|| format!("policy {id} is not registered"))?;
            chain.push(artifact);
            current = artifact.parent_policy_id.clone();
        }
        Ok(chain)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn artifact(id: &str, parent: Option<&str>, at_ns: u64) -> PolicyArtifact {
        PolicyArtifact {
            policy_id: id.into(),
            content_hash: format!("hash-{id}"),
            parent_policy_id: parent.map(str::to_string),
            trained_on_episode_ids: vec![],
            created_at_ns: at_ns,
        }
    }

    #[test]
    fn registering_and_activating_tracks_the_serving_policy() {
        let mut registry = PolicyRegistry::new();
        registry.register(artifact("baseline", None, 1)).unwrap();
        assert!(registry.active().is_none());
        registry.activate("baseline").unwrap();
        assert_eq!(
            registry.active().map(|a| a.policy_id.as_str()),
            Some("baseline")
        );
    }

    #[test]
    fn an_unregistered_policy_cannot_serve() {
        let mut registry = PolicyRegistry::new();
        assert!(registry.activate("ghost").is_err());
    }

    #[test]
    fn a_content_hash_conflict_is_refused() {
        let mut registry = PolicyRegistry::new();
        registry.register(artifact("p1", None, 1)).unwrap();
        let mut impostor = artifact("p1", None, 2);
        impostor.content_hash = "different".into();
        let error = registry.register(impostor).unwrap_err();
        assert!(error.contains("refusing"));
    }

    #[test]
    fn an_unresolvable_parent_is_refused() {
        let mut registry = PolicyRegistry::new();
        let error = registry
            .register(artifact("child", Some("missing"), 1))
            .unwrap_err();
        assert!(error.contains("not registered"));
    }

    #[test]
    fn lineage_walks_back_to_the_baseline() {
        let mut registry = PolicyRegistry::new();
        registry.register(artifact("baseline", None, 1)).unwrap();
        registry
            .register(artifact("v2", Some("baseline"), 2))
            .unwrap();
        registry.register(artifact("v3", Some("v2"), 3)).unwrap();
        let chain = registry.lineage("v3").unwrap();
        let ids = chain
            .iter()
            .map(|a| a.policy_id.as_str())
            .collect::<Vec<_>>();
        assert_eq!(ids, vec!["v3", "v2", "baseline"]);
    }
}
