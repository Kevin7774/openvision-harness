//! Task-pack registry.
//!
//! Step 1 finding #3: `object_id` was a label, not pluggable grounding —
//! `grasp_request()` rejected everything except the hardcoded string
//! `"yellow_block"`. That gate is now a *registry lookup*. A second object is a
//! second pack registered here; the core's `proposal.rs` no longer names any
//! object.
//!
//! The registry holds `TaskSkill` ports (from `harness-contracts`). The default
//! registry ships the one real pack, `yellow-block-pick-place`. Adding another
//! pack is one `register` call — no edit to the proposal, planner, or executive
//! logic.

use harness_contracts::{TaskDescriptor, TaskSkill};
use yellow_block_pick_place::YellowBlockPickPlace;

/// A registry of task-pack skills. `Error` is erased to `String` at this
/// boundary so packs with different error types can coexist.
pub struct TaskPackRegistry {
    skills: Vec<Box<dyn ErasedSkill>>,
}

impl TaskPackRegistry {
    /// An empty registry. Grounds nothing until a pack is registered — useful
    /// for tests that assert the core no longer knows any object by itself.
    pub fn empty() -> Self {
        Self { skills: Vec::new() }
    }

    /// The registry the shipped harness runs with: exactly the packs that have a
    /// real, measured implementation. Today that is the yellow block.
    pub fn with_default_packs() -> Self {
        let mut registry = Self::empty();
        registry.register(YellowBlockPickPlace::new());
        registry
    }

    pub fn register<S>(&mut self, skill: S)
    where
        S: TaskSkill + 'static,
    {
        self.skills.push(Box::new(skill));
    }

    /// Ids of the registered packs, for `doctor`-style reporting.
    pub fn skill_ids(&self) -> Vec<String> {
        self.skills.iter().map(|skill| skill.skill_id()).collect()
    }

    /// The skill id that can handle this task, if any. This replaces the
    /// hardcoded `object_id != "yellow_block"` check.
    pub fn ground(&self, task: &TaskDescriptor) -> Option<String> {
        self.skills
            .iter()
            .find(|skill| skill.can_handle(task))
            .map(|skill| skill.skill_id())
    }

    /// Whether any registered pack can ground this task.
    pub fn can_ground(&self, task: &TaskDescriptor) -> bool {
        self.ground(task).is_some()
    }
}

impl Default for TaskPackRegistry {
    fn default() -> Self {
        Self::with_default_packs()
    }
}

/// Object-safe wrapper so packs with heterogeneous `Error` types share a `Vec`.
trait ErasedSkill {
    fn skill_id(&self) -> String;
    fn can_handle(&self, task: &TaskDescriptor) -> bool;
}

impl<S: TaskSkill> ErasedSkill for S {
    fn skill_id(&self) -> String {
        TaskSkill::skill_id(self).to_string()
    }

    fn can_handle(&self, task: &TaskDescriptor) -> bool {
        TaskSkill::can_handle(self, task)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn descriptor(task: &str, object_id: Option<&str>) -> TaskDescriptor {
        TaskDescriptor {
            task: task.into(),
            object_id: object_id.map(str::to_string),
            description: "d".into(),
        }
    }

    #[test]
    fn default_registry_grounds_the_yellow_block() {
        let registry = TaskPackRegistry::with_default_packs();
        assert_eq!(
            registry.ground(&descriptor("grasp", Some("yellow_block"))),
            Some("yellow_block.pick_place".into())
        );
    }

    #[test]
    fn empty_core_knows_no_objects_on_its_own() {
        // Proves the core no longer hardcodes any object: with no packs, nothing
        // grounds. Grounding lives entirely in the packs now.
        let registry = TaskPackRegistry::empty();
        assert!(!registry.can_ground(&descriptor("grasp", Some("yellow_block"))));
    }

    #[test]
    fn an_unregistered_object_does_not_ground() {
        let registry = TaskPackRegistry::with_default_packs();
        assert!(!registry.can_ground(&descriptor("grasp", Some("blue_cup"))));
    }

    #[test]
    fn a_second_pack_is_added_without_touching_core_grounding_logic() {
        // A hand-rolled second pack, registered at the call site. No edit to this
        // file's logic, to proposal.rs, or to the executive was needed — which is
        // exactly the property Step 1 said was missing.
        #[derive(Clone, Copy)]
        struct BlueCupPack;
        impl TaskSkill for BlueCupPack {
            type Error = std::convert::Infallible;
            fn skill_id(&self) -> &str {
                "blue_cup.pick_place"
            }
            fn can_handle(&self, task: &TaskDescriptor) -> bool {
                task.object_id.as_deref() == Some("blue_cup")
            }
        }
        let mut registry = TaskPackRegistry::with_default_packs();
        registry.register(BlueCupPack);
        assert_eq!(
            registry.ground(&descriptor("pick_place", Some("blue_cup"))),
            Some("blue_cup.pick_place".into())
        );
        // The original pack still grounds too.
        assert!(registry.can_ground(&descriptor("grasp", Some("yellow_block"))));
        assert_eq!(registry.skill_ids().len(), 2);
    }
}
