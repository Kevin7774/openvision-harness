use super::{depth, read_json, validate_intrinsics, yellow, CameraInfo};
use crate::perception::ServoSignalSample;
use image::io::Reader as ImageReader;
use serde::{Deserialize, Serialize};
use std::cmp::Ordering;
use std::collections::{BTreeMap, HashMap};
use std::path::{Path, PathBuf};

// Empty-table frame d405-20260821-195935-890001830-1788400 had a 45-pixel false positive.
const MIN_TARGET_PIXELS: usize = 100;
const MIN_D405_DEPTH_M: f64 = 0.03;
const MAX_D405_DEPTH_M: f64 = 1.00;
const EXPECTED_D405_SERIAL: &str = "262422270599";
const D405_WIDTH: usize = 848;
const D405_HEIGHT: usize = 480;
const MAX_FRAME_JOINT_DELTA_NS: u64 = 500_000_000;

#[derive(Deserialize)]
struct Latest {
    frame_id: String,
    serial: String,
    sensor_stamp_ns: u64,
    received_at_ns: u64,
    rgb_path: PathBuf,
    depth_path: PathBuf,
    camera_info_path: PathBuf,
    state_path: PathBuf,
    depth_valid_ratio: f64,
    sustained_stream_verified: bool,
}

#[derive(Deserialize)]
struct State {
    frame_id: String,
    serial: String,
    sensor_stamp_ns: u64,
    received_at_ns: u64,
    joint_state: JointState,
}

#[derive(Deserialize)]
struct JointState {
    received_at_ns: u64,
    positions_rad: HashMap<String, f64>,
}

#[derive(Clone, Debug, Serialize)]
pub struct NearFieldSignalObservation {
    pub ok: bool,
    pub schema_version: u32,
    pub mode: &'static str,
    pub serial: String,
    pub sample: ServoSignalSample,
    pub joint_received_at_ns: u64,
    pub detected_pixels: usize,
    pub target_depth_valid_pixels: usize,
    pub target_centroid_uv: [f64; 2],
    pub target_depth_m: f64,
    pub depth_valid_ratio: f64,
    pub sustained_stream_verified: bool,
}

pub fn observe(latest_path: &Path) -> Result<NearFieldSignalObservation, String> {
    let latest: Latest = read_json(latest_path)?;
    let camera: CameraInfo = read_json(&latest.camera_info_path)?;
    let state: State = read_json(&latest.state_path)?;
    validate_intrinsics(&camera)?;
    if latest.serial != EXPECTED_D405_SERIAL
        || camera.serial.as_deref() != Some(EXPECTED_D405_SERIAL)
    {
        return Err(format!(
            "near-field observation is not from D405 serial {EXPECTED_D405_SERIAL}"
        ));
    }
    if camera.width != D405_WIDTH || camera.height != D405_HEIGHT {
        return Err(format!(
            "D405 frame must use the exercised {D405_WIDTH}x{D405_HEIGHT} profile"
        ));
    }
    if latest.frame_id != state.frame_id
        || latest.serial != state.serial
        || latest.sensor_stamp_ns != state.sensor_stamp_ns
        || latest.received_at_ns != state.received_at_ns
    {
        return Err("D405 latest/state identity or timestamp mismatch".into());
    }
    if !latest.sustained_stream_verified {
        return Err("D405 observation has no bounded sustained-stream verification".into());
    }
    if state
        .joint_state
        .received_at_ns
        .abs_diff(latest.received_at_ns)
        > MAX_FRAME_JOINT_DELTA_NS
    {
        return Err("D405 frame and joint state differ by more than 500ms".into());
    }
    if !latest.depth_valid_ratio.is_finite() || latest.depth_valid_ratio < 0.50 {
        return Err(format!(
            "D405 depth valid ratio {:.3} is below 0.50",
            latest.depth_valid_ratio
        ));
    }
    let rgb = ImageReader::open(&latest.rgb_path)
        .map_err(|error| error.to_string())?
        .decode()
        .map_err(|error| error.to_string())?
        .to_rgb8();
    if rgb.width() as usize != camera.width || rgb.height() as usize != camera.height {
        return Err("D405 RGB dimensions do not match camera info".into());
    }
    let depth_image = depth::read_npy_f32(&latest.depth_path, camera.width * camera.height)?;
    let mut components = yellow::components(&rgb)
        .into_iter()
        .filter(|component| component.len() >= MIN_TARGET_PIXELS)
        .filter_map(|component| {
            let depths = component
                .iter()
                .filter_map(|index| {
                    let value = f64::from(depth_image[*index]);
                    (value.is_finite() && (MIN_D405_DEPTH_M..=MAX_D405_DEPTH_M).contains(&value))
                        .then_some(value)
                })
                .collect::<Vec<_>>();
            (depths.len() * 2 >= component.len()).then_some((component, depths))
        })
        .collect::<Vec<_>>();
    if components.is_empty() {
        return Err("D405 has no reliable yellow target in the near-field depth range".into());
    }
    if components.len() != 1 {
        return Err(format!(
            "D405 needs exactly one reliable yellow target, found {}",
            components.len()
        ));
    }
    let (target_pixels, target_depths) = components.remove(0);
    let target_u = median(
        target_pixels
            .iter()
            .map(|index| (index % camera.width) as f64)
            .collect(),
    );
    let target_v = median(
        target_pixels
            .iter()
            .map(|index| (index / camera.width) as f64)
            .collect(),
    );
    let target_depth_valid_pixels = target_depths.len();
    let target_depth_m = median(target_depths);
    let joints_rad = state
        .joint_state
        .positions_rad
        .into_iter()
        .collect::<BTreeMap<_, _>>();
    if joints_rad.values().any(|value| !value.is_finite()) {
        return Err("D405 observation joint state contains a non-finite value".into());
    }
    Ok(NearFieldSignalObservation {
        ok: true,
        schema_version: 1,
        mode: "d405_near_field_signal",
        serial: latest.serial,
        sample: ServoSignalSample {
            schema_version: 1,
            frame_id: latest.frame_id,
            received_at_ns: latest.received_at_ns,
            joints_rad,
            signal: [target_u, target_v, target_depth_m],
        },
        joint_received_at_ns: state.joint_state.received_at_ns,
        detected_pixels: target_pixels.len(),
        target_depth_valid_pixels,
        target_centroid_uv: [target_u, target_v],
        target_depth_m,
        depth_valid_ratio: latest.depth_valid_ratio,
        sustained_stream_verified: latest.sustained_stream_verified,
    })
}

fn median(mut values: Vec<f64>) -> f64 {
    if values.is_empty() {
        return f64::NAN;
    }
    values.sort_by(|left, right| left.partial_cmp(right).unwrap_or(Ordering::Equal));
    let middle = values.len() / 2;
    if values.len() % 2 == 0 {
        (values[middle - 1] + values[middle]) * 0.5
    } else {
        values[middle]
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;

    #[test]
    fn near_field_depth_bounds_cover_d405_working_range() {
        assert!((MIN_D405_DEPTH_M..=MAX_D405_DEPTH_M).contains(&0.07));
        assert!((MIN_D405_DEPTH_M..=MAX_D405_DEPTH_M).contains(&0.50));
        assert!(!(MIN_D405_DEPTH_M..=MAX_D405_DEPTH_M).contains(&0.0));
    }

    #[test]
    fn median_rejects_no_value_with_nan() {
        assert!(median(Vec::new()).is_nan());
        assert_eq!(median(vec![3.0, 1.0, 2.0]), 2.0);
        assert_eq!(median(vec![1.0, 3.0]), 2.0);
    }

    #[test]
    fn background_color_components_without_near_depth_are_not_targets() {
        let frame = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../data/sensors/d405/observations/d405-20260821-132928-181071806-572573");
        let latest_path = std::env::temp_dir().join(format!(
            "xr1-d405-background-regression-{}.json",
            std::process::id()
        ));
        let latest = serde_json::json!({
            "frame_id": "d405-20260821-132928-181071806-572573",
            "serial": EXPECTED_D405_SERIAL,
            "sensor_stamp_ns": 1787290168181071806_u64,
            "received_at_ns": 1787290168181071806_u64,
            "rgb_path": frame.join("rgb.png"),
            "depth_path": frame.join("depth.npy"),
            "camera_info_path": frame.join("camera_info.json"),
            "state_path": frame.join("state.json"),
            "depth_valid_ratio": 0.8704205974842767,
            "sustained_stream_verified": true,
        });
        fs::write(&latest_path, serde_json::to_vec(&latest).unwrap()).unwrap();

        let error = observe(&latest_path).unwrap_err();
        fs::remove_file(latest_path).unwrap();

        assert_eq!(
            error,
            "D405 has no reliable yellow target in the near-field depth range"
        );
    }

    #[test]
    fn tiny_dim_component_is_not_a_reliable_near_field_target() {
        let frame = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../data/sensors/d405/observations/d405-20260821-195935-890001830-1788400");
        let latest_path = std::env::temp_dir().join(format!(
            "xr1-d405-tiny-component-regression-{}.json",
            std::process::id()
        ));
        let latest = serde_json::json!({
            "frame_id": "d405-20260821-195935-890001830-1788400",
            "serial": EXPECTED_D405_SERIAL,
            "sensor_stamp_ns": 1787313575890001830_u64,
            "received_at_ns": 1787313575890001830_u64,
            "rgb_path": frame.join("rgb.png"),
            "depth_path": frame.join("depth.npy"),
            "camera_info_path": frame.join("camera_info.json"),
            "state_path": frame.join("state.json"),
            "depth_valid_ratio": 0.8283854166666667,
            "sustained_stream_verified": true,
        });
        fs::write(&latest_path, serde_json::to_vec(&latest).unwrap()).unwrap();

        let error = observe(&latest_path).unwrap_err();
        fs::remove_file(latest_path).unwrap();

        assert_eq!(
            error,
            "D405 has no reliable yellow target in the near-field depth range"
        );
    }
}
