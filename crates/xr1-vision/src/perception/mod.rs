mod depth;
mod geometry;
mod yellow;

use crate::observation::{read_state, ObservationState as State};
use crate::proposal::GraspPlanRequest;
use image::io::Reader as ImageReader;
use serde::{Deserialize, Serialize};
use std::fs;
use std::path::{Path, PathBuf};

#[derive(Deserialize)]
struct Latest {
    rgb_path: PathBuf,
    depth_path: PathBuf,
    camera_info_path: PathBuf,
    state_path: PathBuf,
}

#[derive(Deserialize)]
struct CameraInfo {
    width: usize,
    height: usize,
    k: Vec<f64>,
}

#[derive(Clone, Debug, Serialize)]
pub struct ObjectGeometry {
    pub object_id: String,
    pub pixel_center_uv: [f64; 2],
    pub center_base_m: [f64; 3],
    pub axes_base: [[f64; 3]; 3],
    pub extents_m: [f64; 3],
    pub detected_pixels: usize,
    pub geometry_points: usize,
}

#[derive(Debug)]
pub struct PerceptionFrame {
    pub state: State,
    pub camera_frame: String,
    pub target_frame: String,
    pub object: ObjectGeometry,
}

pub fn observe_object(
    latest_path: &Path,
    request: &GraspPlanRequest,
) -> Result<PerceptionFrame, String> {
    let latest: Latest = read_json(latest_path)?;
    let camera: CameraInfo = read_json(&latest.camera_info_path)?;
    let state = read_state(&latest.state_path)?;
    validate_intrinsics(&camera)?;
    let rgb = ImageReader::open(&latest.rgb_path)
        .map_err(|error| error.to_string())?
        .decode()
        .map_err(|error| error.to_string())?
        .to_rgb8();
    if rgb.width() as usize != camera.width || rgb.height() as usize != camera.height {
        return Err("RGB dimensions do not match camera info".into());
    }
    let depth_image = depth::read_npy_f32(&latest.depth_path, camera.width * camera.height)?;
    let points = reconstruct_target_points(&rgb, &depth_image, &camera, &state)?;
    if points.base.len() < 40 {
        return Err(format!(
            "yellow target not reliable: {} valid pixels",
            points.base.len()
        ));
    }
    let center = std::array::from_fn(|axis| {
        geometry::median(points.base.iter().map(|point| point[axis]).collect())
    });
    let pixel_center = std::array::from_fn(|axis| {
        geometry::median(points.pixels.iter().map(|pixel| pixel[axis]).collect())
    });
    let robust_points = geometry::robust_object_points(&points.base, center);
    if robust_points.len() < 40 {
        return Err(format!(
            "yellow geometry not reliable after filtering: {} points",
            robust_points.len()
        ));
    }
    let (axes, extents) = geometry::object_obb(&robust_points, center);
    Ok(PerceptionFrame {
        camera_frame: state.tf.source_frame.clone(),
        target_frame: state.tf.target_frame.clone(),
        object: ObjectGeometry {
            object_id: request.object_id.clone(),
            pixel_center_uv: pixel_center,
            center_base_m: center,
            axes_base: axes,
            extents_m: extents,
            detected_pixels: points.base.len(),
            geometry_points: robust_points.len(),
        },
        state,
    })
}

struct TargetPoints {
    base: Vec<[f64; 3]>,
    pixels: Vec<[f64; 2]>,
}

fn reconstruct_target_points(
    rgb: &image::RgbImage,
    depth_image: &[f32],
    camera: &CameraInfo,
    state: &State,
) -> Result<TargetPoints, String> {
    let (fx, fy, cx, cy) = (camera.k[0], camera.k[4], camera.k[2], camera.k[5]);
    let mut base = Vec::new();
    let mut pixels = Vec::new();
    for (index, is_target) in yellow::component_mask(rgb).into_iter().enumerate() {
        if !is_target {
            continue;
        }
        let u = index % camera.width;
        let v = index / camera.width;
        let Some(z) = depth::at_or_neighbor(depth_image, camera.width, camera.height, u, v) else {
            continue;
        };
        let camera_point = [(u as f64 - cx) * z / fx, (v as f64 - cy) * z / fy, z];
        let base_point = depth::transform_point(&state.tf, camera_point)?;
        if depth::in_manipulation_workspace(base_point) {
            base.push(base_point);
            pixels.push([u as f64, v as f64]);
        }
    }
    Ok(TargetPoints { base, pixels })
}

fn validate_intrinsics(camera: &CameraInfo) -> Result<(), String> {
    if camera.k.len() != 9 || camera.width == 0 || camera.height == 0 {
        return Err("invalid camera intrinsics".into());
    }
    if !camera.k.iter().all(|value| value.is_finite()) || camera.k[0] <= 0.0 || camera.k[4] <= 0.0 {
        return Err("camera intrinsics contain invalid focal lengths".into());
    }
    Ok(())
}

fn read_json<T: for<'de> Deserialize<'de>>(path: &Path) -> Result<T, String> {
    let data = fs::read(path).map_err(|error| format!("{}: {error}", path.display()))?;
    serde_json::from_slice(&data).map_err(|error| format!("{}: {error}", path.display()))
}
