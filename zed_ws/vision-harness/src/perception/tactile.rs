use crate::runtime::{RuntimeConfig, RuntimeState, SensorFrame, SkillStage, TactilePad};

pub fn shear(pad: &TactilePad) -> f64 {
    pad.shear_force_n[0].hypot(pad.shear_force_n[1])
}

pub fn apply_transition(state: &mut RuntimeState, frame: &SensorFrame, config: &RuntimeConfig) {
    let left = &frame.tactile_left;
    let right = &frame.tactile_right;
    if state.stage == SkillStage::Approach && left.contact && right.contact
        && left.normal_force_n >= config.contact_force_n && right.normal_force_n >= config.contact_force_n {
        state.stage = SkillStage::Contact;
    } else if state.stage == SkillStage::Contact
        && left.normal_force_n >= config.hold_force_n && right.normal_force_n >= config.hold_force_n {
        state.stage = SkillStage::Grasped;
    }
}
