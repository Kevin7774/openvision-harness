use crate::perception::tactile::shear;
use crate::runtime::{RuntimeConfig, SensorFrame};
use super::freshness::sensor_age_ms;

pub fn validate(frame: &SensorFrame, config: &RuntimeConfig) -> Option<String> {
    for (name, stamp) in [("robot", frame.robot.stamp_ns), ("tactile_left", frame.tactile_left.stamp_ns), ("tactile_right", frame.tactile_right.stamp_ns)] {
        if sensor_age_ms(frame.received_at_ns, stamp)? > config.max_sensor_age_ms { return Some(format!("{name} sensor is stale")); }
    }
    if !frame.robot.collision_free { return Some("collision monitor rejected the current state".into()); }
    if frame.robot.joint_limit_margin_rad < config.min_joint_margin_rad { return Some("joint limit margin is below runtime minimum".into()); }
    if shear(&frame.tactile_left).max(shear(&frame.tactile_right)) > config.max_shear_force_n { return Some("tactile shear indicates slip or unsafe lateral load".into()); }
    None
}
