use super::{depth, geometry, yellow, CameraInfo};
use crate::kinematics::Chain;
use crate::observation::ObservationState;
use image::RgbImage;
use serde::{Deserialize, Serialize};
use std::cmp::{Ordering, Reverse};
use std::collections::{BTreeMap, VecDeque};

const ORANGE_PAD_MIN_AREA_PX: usize = 150;
// Measured on frame 20260818-120701: the FK residual is tens of pixels, while
// the unrelated orange fruit is more than 150 px from the predicted tool.
const PAD_SEARCH_RADIUS_PX: f64 = 150.0;
// The open G2 pads measured 53.1--53.3 mm apart across the 2026-08-18 frames.
// This wider band preserves perspective variation while rejecting merged pads
// and unrelated orange components.
const MIN_PAD_SEPARATION_M: f64 = 0.035;
const MAX_PAD_SEPARATION_M: f64 = 0.095;

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct ServoSignalSample {
    pub schema_version: u32,
    pub frame_id: String,
    pub received_at_ns: u64,
    pub joints_rad: BTreeMap<String, f64>,
    /// Physical pad midpoint [u_px, v_px, optical_depth_m].
    pub signal: [f64; 3],
}

#[derive(Debug, Serialize)]
pub struct ServoSignalObservation {
    pub ok: bool,
    pub schema_version: u32,
    pub mode: &'static str,
    pub sample: ServoSignalSample,
    /// Pin this target from the first unobstructed frame for later servo steps.
    pub observed_target_signal: [f64; 3],
    pub observed_error: [f64; 3],
    pub target_anchor: &'static str,
    pub target_detected_pixels: usize,
    pub predicted_pad_midpoint_uv: [f64; 2],
    pub predicted_pad_depth_m: f64,
    pub physical_pad_midpoint_uv: [f64; 2],
    pub pad_blob_areas_px: [usize; 2],
    pub pad_separation_m: f64,
    pub orange_blobs_rejected: usize,
}

#[derive(Debug, Serialize)]
pub struct PadSignalObservation {
    pub ok: bool,
    pub schema_version: u32,
    pub mode: &'static str,
    pub sample: ServoSignalSample,
    pub predicted_pad_midpoint_uv: [f64; 2],
    pub predicted_pad_depth_m: f64,
    pub physical_pad_midpoint_uv: [f64; 2],
    pub pad_blob_areas_px: [usize; 2],
    pub pad_separation_m: f64,
    pub orange_blobs_rejected: usize,
}

pub(super) fn extract(
    rgb: &RgbImage,
    depth_image: &[f32],
    camera: &CameraInfo,
    state: &ObservationState,
    chain: &Chain,
) -> Result<ServoSignalObservation, String> {
    let pads = extract_pads(rgb, depth_image, camera, state, chain)?;
    combine_with_target(rgb, depth_image, camera, state, pads)
}

pub(super) fn extract_pads(
    rgb: &RgbImage,
    depth_image: &[f32],
    camera: &CameraInfo,
    state: &ObservationState,
    chain: &Chain,
) -> Result<PadSignalObservation, String> {
    let names = chain.names();
    let joints = names
        .iter()
        .map(|name| {
            state
                .joint_state
                .positions_rad
                .get(name)
                .and_then(|value| *value)
                .ok_or_else(|| format!("missing live joint {name}"))
        })
        .collect::<Result<Vec<_>, _>>()?;
    let joints_rad = names.iter().cloned().zip(joints.iter().copied()).collect();
    let (fixed, moving) = chain.pad_inner_points(&joints);
    let predicted_base = std::array::from_fn(|axis| (fixed[axis] + moving[axis]) * 0.5);
    let predicted_camera = depth::inverse_transform_point(&state.tf, predicted_base)?;
    if predicted_camera[2] <= 0.0 {
        return Err("predicted pad midpoint is behind the camera".into());
    }
    let predicted_uv = [
        camera.k[0] * predicted_camera[0] / predicted_camera[2] + camera.k[2],
        camera.k[4] * predicted_camera[1] / predicted_camera[2] + camera.k[5],
    ];
    extract_pads_with_prediction(
        rgb,
        depth_image,
        camera,
        state,
        joints_rad,
        [predicted_uv[0], predicted_uv[1], predicted_camera[2]],
    )
}

#[cfg(test)]
fn extract_with_prediction(
    rgb: &RgbImage,
    depth_image: &[f32],
    camera: &CameraInfo,
    state: &ObservationState,
    joints_rad: BTreeMap<String, f64>,
    predicted: [f64; 3],
) -> Result<ServoSignalObservation, String> {
    let pads =
        extract_pads_with_prediction(rgb, depth_image, camera, state, joints_rad, predicted)?;
    combine_with_target(rgb, depth_image, camera, state, pads)
}

fn combine_with_target(
    rgb: &RgbImage,
    depth_image: &[f32],
    camera: &CameraInfo,
    state: &ObservationState,
    pads: PadSignalObservation,
) -> Result<ServoSignalObservation, String> {
    let target_mask = yellow::component_mask(rgb);
    let target_pixels = target_mask
        .iter()
        .enumerate()
        .filter_map(|(index, selected)| {
            if !selected {
                return None;
            }
            let u = index % camera.width;
            let v = index / camera.width;
            let z = depth::at_or_neighbor(depth_image, camera.width, camera.height, u, v)?;
            let camera_point = [
                (u as f64 - camera.k[2]) * z / camera.k[0],
                (v as f64 - camera.k[5]) * z / camera.k[4],
                z,
            ];
            let base_point = depth::transform_point(&state.tf, camera_point).ok()?;
            depth::in_manipulation_workspace(base_point).then_some(index)
        })
        .collect::<Vec<_>>();
    if target_pixels.len() < 40 {
        return Err(format!(
            "yellow target signal is unreliable: {} pixels",
            target_pixels.len()
        ));
    }
    let mut target_rows = target_pixels
        .iter()
        .map(|index| (index / camera.width) as f64)
        .collect::<Vec<_>>();
    target_rows.sort_by(|left, right| left.partial_cmp(right).unwrap_or(Ordering::Equal));
    let top_limit = target_rows[(target_rows.len() as f64 * 0.05) as usize];
    let top_pixels = target_pixels
        .iter()
        .filter(|index| (**index / camera.width) as f64 <= top_limit)
        .copied()
        .collect::<Vec<_>>();
    let target_u = geometry::median(
        top_pixels
            .iter()
            .map(|index| (index % camera.width) as f64)
            .collect(),
    );
    let target_v = geometry::median(
        top_pixels
            .iter()
            .map(|index| (index / camera.width) as f64)
            .collect(),
    );
    let target_depth = median_component_depth(depth_image, camera, &target_pixels, "target")?;

    let observed_target_signal = [target_u, target_v, target_depth];
    let observed_error =
        std::array::from_fn(|axis| observed_target_signal[axis] - pads.sample.signal[axis]);
    Ok(ServoSignalObservation {
        ok: true,
        schema_version: 1,
        mode: "visual_servo_signal",
        sample: pads.sample,
        observed_target_signal,
        observed_error,
        target_anchor: "yellow_top_edge_5pct",
        target_detected_pixels: target_pixels.len(),
        predicted_pad_midpoint_uv: pads.predicted_pad_midpoint_uv,
        predicted_pad_depth_m: pads.predicted_pad_depth_m,
        physical_pad_midpoint_uv: pads.physical_pad_midpoint_uv,
        pad_blob_areas_px: pads.pad_blob_areas_px,
        pad_separation_m: pads.pad_separation_m,
        orange_blobs_rejected: pads.orange_blobs_rejected,
    })
}

fn extract_pads_with_prediction(
    rgb: &RgbImage,
    depth_image: &[f32],
    camera: &CameraInfo,
    state: &ObservationState,
    joints_rad: BTreeMap<String, f64>,
    predicted: [f64; 3],
) -> Result<PadSignalObservation, String> {
    let mut blobs = orange_pad_blobs(rgb);
    blobs.sort_by_key(|blob| Reverse(blob.area));
    let mut near = blobs
        .iter()
        .filter(|blob| {
            ((blob.centroid_uv[0] - predicted[0]).powi(2)
                + (blob.centroid_uv[1] - predicted[1]).powi(2))
            .sqrt()
                <= PAD_SEARCH_RADIUS_PX
        })
        .collect::<Vec<_>>();
    if near.len() < 2 {
        return Err(format!(
            "need both orange pads near FK prediction, found {}",
            near.len()
        ));
    }
    near.truncate(2);
    let pad_depths = near
        .iter()
        .map(|blob| median_component_depth(depth_image, camera, &blob.pixels, "orange pad"))
        .collect::<Result<Vec<_>, _>>()?;
    let pad_midpoint_uv = [
        (near[0].centroid_uv[0] + near[1].centroid_uv[0]) * 0.5,
        (near[0].centroid_uv[1] + near[1].centroid_uv[1]) * 0.5,
    ];
    let pad_depth = (pad_depths[0] + pad_depths[1]) * 0.5;
    let separation_px = ((near[0].centroid_uv[0] - near[1].centroid_uv[0]).powi(2)
        + (near[0].centroid_uv[1] - near[1].centroid_uv[1]).powi(2))
    .sqrt();
    let pad_separation_m = separation_px * pad_depth / ((camera.k[0] + camera.k[4]) * 0.5);
    if !(MIN_PAD_SEPARATION_M..=MAX_PAD_SEPARATION_M).contains(&pad_separation_m) {
        return Err(format!(
            "orange pad separation {pad_separation_m:.4}m is outside [{MIN_PAD_SEPARATION_M:.3}, {MAX_PAD_SEPARATION_M:.3}]m"
        ));
    }

    let signal = [pad_midpoint_uv[0], pad_midpoint_uv[1], pad_depth];
    Ok(PadSignalObservation {
        ok: true,
        schema_version: 1,
        mode: "visual_servo_pads",
        sample: ServoSignalSample {
            schema_version: 1,
            frame_id: state.frame_id.clone(),
            received_at_ns: state.received_at_ns,
            joints_rad,
            signal,
        },
        predicted_pad_midpoint_uv: [predicted[0], predicted[1]],
        predicted_pad_depth_m: predicted[2],
        physical_pad_midpoint_uv: pad_midpoint_uv,
        pad_blob_areas_px: [near[0].area, near[1].area],
        pad_separation_m,
        orange_blobs_rejected: blobs.len() - near.len(),
    })
}

struct OrangeBlob {
    area: usize,
    centroid_uv: [f64; 2],
    pixels: Vec<usize>,
}

fn orange_pad_blobs(rgb: &RgbImage) -> Vec<OrangeBlob> {
    let width = rgb.width() as usize;
    let height = rgb.height() as usize;
    let mask = rgb
        .pixels()
        .map(|pixel| {
            let [red, green, blue] = pixel.0.map(i16::from);
            // Measured on frame 20260818-120701: pads are near RGB(231,124,31).
            red > green + 55 && green > blue + 30 && red > 120
        })
        .collect::<Vec<_>>();
    let mut visited = vec![false; mask.len()];
    let mut blobs = Vec::new();
    for start in 0..mask.len() {
        if visited[start] || !mask[start] {
            continue;
        }
        visited[start] = true;
        let mut queue = VecDeque::from([start]);
        let mut pixels = Vec::new();
        while let Some(index) = queue.pop_front() {
            pixels.push(index);
            let x = index % width;
            let y = index / width;
            for neighbor_y in y.saturating_sub(1)..=(y + 1).min(height - 1) {
                for neighbor_x in x.saturating_sub(1)..=(x + 1).min(width - 1) {
                    let neighbor = neighbor_y * width + neighbor_x;
                    if mask[neighbor] && !visited[neighbor] {
                        visited[neighbor] = true;
                        queue.push_back(neighbor);
                    }
                }
            }
        }
        if pixels.len() < ORANGE_PAD_MIN_AREA_PX {
            continue;
        }
        let area = pixels.len();
        let centroid_uv = [
            pixels.iter().map(|index| index % width).sum::<usize>() as f64 / area as f64,
            pixels.iter().map(|index| index / width).sum::<usize>() as f64 / area as f64,
        ];
        blobs.push(OrangeBlob {
            area,
            centroid_uv,
            pixels,
        });
    }
    blobs
}

fn median_component_depth(
    depth_image: &[f32],
    camera: &CameraInfo,
    pixels: &[usize],
    name: &str,
) -> Result<f64, String> {
    let mut values = pixels
        .iter()
        .filter_map(|index| {
            let depth = f64::from(depth_image[*index]);
            (depth.is_finite() && (0.20..=2.0).contains(&depth)).then_some(depth)
        })
        .collect::<Vec<_>>();
    if values.len() < 3 {
        let center_u = pixels
            .iter()
            .map(|index| index % camera.width)
            .sum::<usize>()
            / pixels.len();
        let center_v = pixels
            .iter()
            .map(|index| index / camera.width)
            .sum::<usize>()
            / pixels.len();
        if let Some(value) =
            depth::at_or_neighbor(depth_image, camera.width, camera.height, center_u, center_v)
        {
            return Ok(value);
        }
        return Err(format!("{name} has no reliable depth"));
    }
    values.sort_by(|left, right| left.partial_cmp(right).unwrap_or(Ordering::Equal));
    Ok(values[values.len() / 2])
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::observation::read_state;
    use image::io::Reader as ImageReader;
    use std::fs;
    use std::path::Path;

    #[test]
    fn named_frame_extracts_both_pads_and_rejects_the_fruit() {
        let frame = Path::new(env!("CARGO_MANIFEST_DIR")).join(
            "../../data/vista_runs/yellow-block-harness/observations/20260818-120701-385142786-132823",
        );
        let camera: CameraInfo =
            serde_json::from_slice(&fs::read(frame.join("camera_info.json")).unwrap()).unwrap();
        let state = read_state(&frame.join("state.json")).unwrap();
        let rgb = ImageReader::open(frame.join("rgb.png"))
            .unwrap()
            .decode()
            .unwrap()
            .to_rgb8();
        let depth =
            depth::read_npy_f32(&frame.join("depth.npy"), camera.width * camera.height).unwrap();
        let result = extract_with_prediction(
            &rgb,
            &depth,
            &camera,
            &state,
            BTreeMap::new(),
            [703.0, 342.9, 0.4898],
        )
        .unwrap();
        assert!((result.physical_pad_midpoint_uv[0] - 726.0).abs() < 1.0);
        assert!((result.physical_pad_midpoint_uv[1] - 410.9).abs() < 1.0);
        assert_eq!(result.orange_blobs_rejected, 1);
        assert!((0.035..=0.095).contains(&result.pad_separation_m));
        assert!(result.target_detected_pixels > 100);
        assert!((600.0..=850.0).contains(&result.observed_target_signal[0]));
        assert!((300.0..=450.0).contains(&result.observed_target_signal[1]));
    }
}
