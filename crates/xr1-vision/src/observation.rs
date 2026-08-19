use crate::hardware::SensorStatus;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::fs;
use std::path::{Path, PathBuf};

pub const OBSERVATION_BUNDLE_SCHEMA_VERSION: u32 = 1;

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct ObservationState {
    pub frame_id: String,
    pub sensor_stamp_ns: u64,
    pub received_at_ns: u64,
    pub tf: Transform,
    pub joint_state: JointState,
    #[serde(default)]
    pub grippers: HashMap<String, GripperState>,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct JointState {
    pub received_at_ns: u64,
    pub positions_rad: HashMap<String, Option<f64>>,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct Transform {
    pub target_frame: String,
    pub source_frame: String,
    pub translation_m: Vec<f64>,
    pub rotation_xyzw: Vec<f64>,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct GripperState {
    pub received_at_ns: u64,
    pub position_mm: u32,
    pub running: Option<u32>,
    pub temperature: Option<u32>,
    pub error: Option<u32>,
}

#[derive(Debug, Deserialize)]
struct LatestObservation {
    ok: bool,
    run_id: String,
    frame_id: String,
    rgb_path: PathBuf,
    depth_path: PathBuf,
    camera_info_path: PathBuf,
    state_path: PathBuf,
    sensor_stamp_ns: u64,
    received_at_ns: u64,
    rgb_depth_delta_ms: f64,
    clock_offset_ms: f64,
    tf_ok: bool,
    depth_valid_ratio: f64,
}

#[derive(Debug, Serialize)]
pub struct ObservationBundle {
    pub schema_version: u32,
    pub run_id: String,
    pub frame_id: String,
    pub sensor_stamp_ns: u64,
    pub received_at_ns: u64,
    pub zed: ZedObservation,
    pub robot: RobotObservation,
    pub capabilities: SensorStatus,
}

#[derive(Debug, Serialize)]
pub struct ZedObservation {
    pub rgb_path: PathBuf,
    pub depth_path: PathBuf,
    pub camera_info_path: PathBuf,
    pub camera_frame: String,
    pub target_frame: String,
    pub rgb_depth_delta_ms: f64,
    pub clock_offset_ms: f64,
    pub depth_valid_ratio: f64,
}

#[derive(Debug, Serialize)]
pub struct RobotObservation {
    pub joints: JointState,
    pub tf: Transform,
    pub grippers: HashMap<String, GripperState>,
}

pub fn read_state(path: &Path) -> Result<ObservationState, String> {
    read_json(path, "observation state")
}

pub fn bundle_from_latest(
    latest_path: &Path,
    capabilities: SensorStatus,
) -> Result<ObservationBundle, String> {
    let latest: LatestObservation = read_json(latest_path, "latest observation")?;
    if !latest.ok {
        return Err(format!(
            "latest observation {} is not ok",
            latest_path.display()
        ));
    }
    if !latest.tf_ok {
        return Err(format!(
            "latest observation {} has no image-time TF",
            latest_path.display()
        ));
    }
    let state = read_state(&latest.state_path)?;
    if latest.frame_id != state.frame_id {
        return Err(format!(
            "observation frame mismatch: latest={} state={}",
            latest.frame_id, state.frame_id
        ));
    }
    if latest.sensor_stamp_ns != state.sensor_stamp_ns
        || latest.received_at_ns != state.received_at_ns
    {
        return Err("latest observation timestamps do not match state.json".into());
    }
    validate_transform(&state.tf)?;
    validate_metric("rgb_depth_delta_ms", latest.rgb_depth_delta_ms, 0.0, 50.0)?;
    validate_metric(
        "clock_offset_ms",
        latest.clock_offset_ms.abs(),
        0.0,
        2_000.0,
    )?;
    validate_metric("depth_valid_ratio", latest.depth_valid_ratio, 0.0, 1.0)?;

    Ok(ObservationBundle {
        schema_version: OBSERVATION_BUNDLE_SCHEMA_VERSION,
        run_id: latest.run_id,
        frame_id: latest.frame_id,
        sensor_stamp_ns: latest.sensor_stamp_ns,
        received_at_ns: latest.received_at_ns,
        zed: ZedObservation {
            rgb_path: latest.rgb_path,
            depth_path: latest.depth_path,
            camera_info_path: latest.camera_info_path,
            camera_frame: state.tf.source_frame.clone(),
            target_frame: state.tf.target_frame.clone(),
            rgb_depth_delta_ms: latest.rgb_depth_delta_ms,
            clock_offset_ms: latest.clock_offset_ms,
            depth_valid_ratio: latest.depth_valid_ratio,
        },
        robot: RobotObservation {
            joints: state.joint_state,
            tf: state.tf,
            grippers: state.grippers,
        },
        capabilities,
    })
}

fn validate_transform(transform: &Transform) -> Result<(), String> {
    if transform.target_frame.trim().is_empty() || transform.source_frame.trim().is_empty() {
        return Err("observation TF frame names must not be empty".into());
    }
    if transform.translation_m.len() != 3 || transform.rotation_xyzw.len() != 4 {
        return Err("observation TF must contain xyz translation and xyzw rotation".into());
    }
    if !transform
        .translation_m
        .iter()
        .chain(transform.rotation_xyzw.iter())
        .all(|value| value.is_finite())
    {
        return Err("observation TF contains non-finite values".into());
    }
    Ok(())
}

fn validate_metric(name: &str, value: f64, minimum: f64, maximum: f64) -> Result<(), String> {
    if !value.is_finite() || !(minimum..=maximum).contains(&value) {
        return Err(format!("{name}={value} is outside [{minimum}, {maximum}]"));
    }
    Ok(())
}

fn read_json<T: for<'de> Deserialize<'de>>(path: &Path, kind: &str) -> Result<T, String> {
    let bytes = fs::read(path).map_err(|error| format!("{}: {error}", path.display()))?;
    serde_json::from_slice(&bytes)
        .map_err(|error| format!("invalid {kind} {}: {error}", path.display()))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn named_hardware_frame_has_valid_image_time_transform() {
        let path = Path::new(env!("CARGO_MANIFEST_DIR")).join(
            "../../data/vista_runs/harness-upgrade-20260819/observations/20260819-005653-726107572-570151/state.json",
        );
        let state = read_state(&path);
        assert!(state.is_ok());
        let Some(state) = state.ok() else { return };
        assert_eq!(state.frame_id, "20260819-005653-726107572-570151");
        assert!(validate_transform(&state.tf).is_ok());
    }

    #[test]
    fn observation_quality_limits_match_capture_contract() {
        assert!(validate_metric("sync", 50.0, 0.0, 50.0).is_ok());
        assert!(validate_metric("sync", 50.001, 0.0, 50.0).is_err());
        assert!(validate_metric("clock", 2_000.001, 0.0, 2_000.0).is_err());
    }
}
