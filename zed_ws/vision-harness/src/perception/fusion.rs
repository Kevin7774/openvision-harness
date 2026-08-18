use crate::runtime::{FusedTarget, RuntimeConfig, SensorFrame, TargetObservation};
use crate::safety::freshness::sensor_age_ms;

pub fn fuse_target(frame: &SensorFrame, config: &RuntimeConfig) -> Option<FusedTarget> {
    let stereo = fresh(frame, frame.stereo_target.as_ref(), config);
    let near = fresh(frame, frame.d405_target.as_ref(), config);
    match (stereo, near) {
        (Some(global), Some(near)) => {
            if global.stamp_ns.abs_diff(near.stamp_ns) / 1_000_000 > config.max_camera_skew_ms {
                return None;
            }
            let nw = (near.confidence * 1.5).clamp(0.0, 1.0);
            let gw = global.confidence.clamp(0.0, 1.0);
            let total = nw + gw;
            if total <= f64::EPSILON { return None; }
            Some(FusedTarget {
                pose: crate::runtime::Pose3 {
                    position_m: std::array::from_fn(|i| (near.pose.position_m[i] * nw + global.pose.position_m[i] * gw) / total),
                    rotation_xyzw: near.pose.rotation_xyzw,
                },
                confidence: (near.confidence + global.confidence) * 0.5,
                source: "stereo+d405".into(),
            })
        }
        (Some(value), None) => Some(copy(value, "stereo")),
        (None, Some(value)) => Some(copy(value, "d405")),
        (None, None) => None,
    }
}

fn fresh<'a>(frame: &SensorFrame, value: Option<&'a TargetObservation>, config: &RuntimeConfig) -> Option<&'a TargetObservation> {
    let value = value?;
    let age = sensor_age_ms(frame.received_at_ns, value.stamp_ns)?;
    (age <= config.max_sensor_age_ms && value.confidence.is_finite() && value.confidence > 0.0).then_some(value)
}

fn copy(value: &TargetObservation, source: &str) -> FusedTarget {
    FusedTarget { pose: value.pose.clone(), confidence: value.confidence, source: source.into() }
}
