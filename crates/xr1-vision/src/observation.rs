use serde::Deserialize;
use std::collections::HashMap;
use std::fs;
use std::path::Path;

#[derive(Debug, Deserialize)]
pub struct ObservationState {
    pub frame_id: String,
    pub sensor_stamp_ns: u64,
    pub received_at_ns: u64,
    pub tf: Transform,
    pub joint_state: JointState,
}

#[derive(Debug, Deserialize)]
pub struct JointState {
    pub received_at_ns: u64,
    pub positions_rad: HashMap<String, Option<f64>>,
}

#[derive(Debug, Deserialize)]
pub struct Transform {
    pub target_frame: String,
    pub source_frame: String,
    pub translation_m: Vec<f64>,
    pub rotation_xyzw: Vec<f64>,
}

pub fn read_state(path: &Path) -> Result<ObservationState, String> {
    let bytes = fs::read(path).map_err(|error| format!("{}: {error}", path.display()))?;
    serde_json::from_slice(&bytes)
        .map_err(|error| format!("invalid observation state {}: {error}", path.display()))
}
