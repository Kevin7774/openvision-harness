//! The yellow-block pick/place task pack.
//!
//! Implements [`harness_contracts::TaskSkill`] so the executive can dispatch to
//! it by id and ask whether it can ground a given semantic task — instead of the
//! core hardcoding `if object_id != "yellow_block"`. Adding a second object is a
//! second pack, not an edit to `proposal.rs`.

pub mod detector;

use harness_contracts::{TaskDescriptor, TaskSkill};

/// The object id this pack grounds. The measured detector in [`detector`] is
/// tuned to exactly this object under the current lighting; the pack is honest
/// about that by grounding only this id.
pub const OBJECT_ID: &str = "yellow_block";

/// The stable skill id the executive dispatches on.
pub const SKILL_ID: &str = "yellow_block.pick_place";

/// Never fails to construct; holds no live state. Kept as a unit struct so a
/// registry can own a boxed `dyn TaskSkill`.
#[derive(Clone, Copy, Debug, Default)]
pub struct YellowBlockPickPlace;

impl YellowBlockPickPlace {
    pub fn new() -> Self {
        Self
    }
}

impl TaskSkill for YellowBlockPickPlace {
    // This pack's `can_handle` never fails, but the port is fallible so packs
    // that consult external config can report a real error.
    type Error = std::convert::Infallible;

    fn skill_id(&self) -> &str {
        SKILL_ID
    }

    fn can_handle(&self, task: &TaskDescriptor) -> bool {
        // Grasp or pick_place of the yellow block. The detector rejects anything
        // that is not the measured yellow signal, so grounding is honest: this
        // returns true only for the object it can actually perceive.
        matches!(task.task.as_str(), "grasp" | "pick_place")
            && task.object_id.as_deref() == Some(OBJECT_ID)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn descriptor(task: &str, object_id: Option<&str>) -> TaskDescriptor {
        TaskDescriptor {
            task: task.into(),
            object_id: object_id.map(str::to_string),
            description: "yellow block".into(),
        }
    }

    #[test]
    fn handles_yellow_block_grasp_and_pick_place() {
        let skill = YellowBlockPickPlace::new();
        assert_eq!(skill.skill_id(), "yellow_block.pick_place");
        assert!(skill.can_handle(&descriptor("grasp", Some("yellow_block"))));
        assert!(skill.can_handle(&descriptor("pick_place", Some("yellow_block"))));
    }

    #[test]
    fn refuses_other_objects_and_tasks() {
        let skill = YellowBlockPickPlace::new();
        assert!(!skill.can_handle(&descriptor("grasp", Some("green_tray"))));
        assert!(!skill.can_handle(&descriptor("pour", Some("yellow_block"))));
        assert!(!skill.can_handle(&descriptor("grasp", None)));
    }
}
