use image::io::Reader as ImageReader;
use nalgebra::{Matrix2, Vector2, Vector3};
use serde::Deserialize;
use serde_json::json;
use std::cmp::Ordering;
use std::collections::HashMap;
use std::collections::VecDeque;
use std::fs;

use std::path::{Path, PathBuf};

use crate::observation::{read_state, ObservationState as State, Transform};

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

const URDF: &str = "/opt/ros/astrabot/share/astrabot_xr1_evt2_description/urdf/astrabot_xr1_evt2_arm_description.urdf";

/// Report where a hypothetical joint vector puts the fingertips, so a staging pose
/// can be chosen without moving hardware. The floor gate in solve_pose samples the
/// straight joint-space line from the CURRENT pose and demands contact z > 0.785 at
/// every sample -- including t=0 -- so planning is impossible until the hand itself
/// is above the table. This is how you check that before committing to a move.
pub fn fk(joints: &[f64]) -> Result<(), String> {
    let chain = crate::kinematics::Chain::from_urdf(Path::new(URDF), "right_tcp_link")?;
    let names = chain.names();
    if joints.len() != names.len() {
        return Err(format!(
            "expected {} joint values, got {}",
            names.len(),
            joints.len()
        ));
    }
    let (fixed, moving) = chain.pad_inner_points(joints);
    let midpoint = [
        (fixed[0] + moving[0]) * 0.5,
        (fixed[1] + moving[1]) * 0.5,
        (fixed[2] + moving[2]) * 0.5,
    ];
    // The tool pose is what turns an offset measured in the base frame into a
    // tool-frame vector. A tool-frame model error gives the SAME tool vector at
    // every wrist orientation; a rotation error in the model does not. Without
    // this rotation the two cannot be told apart, and a one-pose fit is guesswork.
    let tool = chain.fk(joints);
    let rotation = tool.rotation.to_rotation_matrix();
    let rows: Vec<[f64; 3]> = (0..3)
        .map(|r| [rotation[(r, 0)], rotation[(r, 1)], rotation[(r, 2)]])
        .collect();
    println!(
        "{}",
        serde_json::to_string(&json!({
            "ok": true,
            "joints_rad": names.iter().cloned().zip(joints.iter().copied()).collect::<HashMap<_,_>>(),
            "fixed_pad_inner_base_m": fixed,
            "moving_pad_inner_base_m": moving,
            "pad_midpoint_base_m": midpoint,
            "tcp_origin_base_m": [tool.translation.x, tool.translation.y, tool.translation.z],
            "tool_rotation_base_rowmajor": rows,
            "floor_gate_threshold_m": 0.785,
            "clears_floor_gate": midpoint[2] > 0.785
        }))
        .map_err(|e| e.to_string())?
    );
    Ok(())
}

pub fn plan(latest_path: &Path) -> Result<(), String> {
    let latest: Latest = read_json(latest_path)?;
    let camera: CameraInfo = read_json(&latest.camera_info_path)?;
    let state: State = read_state(&latest.state_path)?;
    if camera.k.len() != 9 || camera.width == 0 || camera.height == 0 {
        return Err("invalid camera intrinsics".into());
    }
    let rgb = ImageReader::open(&latest.rgb_path)
        .map_err(|e| e.to_string())?
        .decode()
        .map_err(|e| e.to_string())?
        .to_rgb8();
    if rgb.width() as usize != camera.width || rgb.height() as usize != camera.height {
        return Err("RGB dimensions do not match camera info".into());
    }
    let depth = read_npy_f32(&latest.depth_path, camera.width * camera.height)?;
    let fx = camera.k[0];
    let fy = camera.k[4];
    let cx = camera.k[2];
    let cy = camera.k[5];
    let mut points = Vec::new();
    let mut pixels = Vec::new();
    let yellow_mask = yellow_component_mask(&rgb);
    for (index, &is_yellow) in yellow_mask.iter().enumerate() {
        if !is_yellow {
            continue;
        }
        let u = (index % camera.width) as f64;
        let v = (index / camera.width) as f64;
        let Some(z) =
            depth_at_or_neighbor(&depth, camera.width, camera.height, u as usize, v as usize)
        else {
            continue;
        };
        let camera_point = [(u - cx) * z / fx, (v - cy) * z / fy, z];
        let base_point = transform_point(&state.tf, camera_point)?;
        if !in_manipulation_workspace(base_point) {
            continue;
        }
        points.push(base_point);
        pixels.push([u, v]);
    }
    if points.len() < 40 {
        return Err(format!(
            "yellow target not reliable: {} valid pixels",
            points.len()
        ));
    }
    let center = [
        median(points.iter().map(|p| p[0]).collect()),
        median(points.iter().map(|p| p[1]).collect()),
        median(points.iter().map(|p| p[2]).collect()),
    ];
    let pixel_center = [
        median(pixels.iter().map(|p| p[0]).collect()),
        median(pixels.iter().map(|p| p[1]).collect()),
    ];
    let robust_points = robust_object_points(&points, center);
    if robust_points.len() < 40 {
        return Err(format!(
            "yellow geometry not reliable after filtering: {} points",
            robust_points.len()
        ));
    }
    let (object_axes, object_extents) = object_obb(&robust_points, center);
    let chain = crate::kinematics::Chain::from_urdf(Path::new(URDF), "right_tcp_link")?;
    let names = chain.names();
    let current = names
        .iter()
        .map(|name| {
            state
                .joint_state
                .positions_rad
                .get(name)
                .and_then(|v| *v)
                .ok_or_else(|| format!("missing live joint {name}"))
        })
        .collect::<Result<Vec<_>, _>>()?;
    let (current_fixed_pad, current_moving_pad) = chain.pad_inner_points(&current);
    let current_pad_midpoint = [
        (current_fixed_pad[0] + current_moving_pad[0]) * 0.5,
        (current_fixed_pad[1] + current_moving_pad[1]) * 0.5,
        (current_fixed_pad[2] + current_moving_pad[2]) * 0.5,
    ];
    let current_horizontal_error = ((current_pad_midpoint[0] - center[0]).powi(2)
        + (current_pad_midpoint[1] - center[1]).powi(2))
    .sqrt();
    let mut orientation_offsets = Vec::new();
    for axis in object_axes {
        if axis[2].abs() > 0.35 {
            continue;
        }
        for direction in [axis, [-axis[0], -axis[1], -axis[2]]] {
            // tilt sweep: 0, +/-30, +/-60, +/-90, +/-120, +/-150, 180 degrees
            let tilts = std::iter::once(0.0)
                .chain((1..=5).flat_map(|k| {
                    let a = f64::from(k) * std::f64::consts::FRAC_PI_6;
                    [a, -a]
                }))
                .chain(std::iter::once(std::f64::consts::PI));
            for tilt in tilts {
                if let Some(offset) =
                    chain.offset_for_top_down_closing_axis(&current, direction, tilt)
                {
                    orientation_offsets.push(offset);
                }
            }
        }
    }
    let candidates = [0.06_f64, 0.08, 0.10]
        .into_iter()
        .enumerate()
        .map(|(rank, clearance)| {
            let approach = [center[0], center[1], center[2] + clearance];
            let mut approach_ik_count = 0usize;
            let mut grasp_ik_count = 0usize;
            let mut geometry_feasible_count = 0usize;
            let mut pair = None;
            for offset in orientation_offsets.iter().copied() {
                let approach_solutions = chain
                    .solve_position_candidates_with_reference(approach, &current, &current, offset);
                approach_ik_count += approach_solutions.len();
                for approach_solution in approach_solutions {
                    let seed = names
                        .iter()
                        .map(|name| {
                            approach_solution
                                .joints
                                .iter()
                                .find(|(joint, _)| joint == name)
                                .map(|(_, value)| *value)
                        })
                        .collect::<Option<Vec<_>>>();
                    let Some(seed) = seed else { continue };
                    let grasp_solutions = chain
                        .solve_position_candidates_with_reference(center, &seed, &current, offset);
                    grasp_ik_count += grasp_solutions.len();
                    for grasp_solution in grasp_solutions {
                        let metrics = chain.grasp_metrics(
                            &grasp_solution,
                            center,
                            object_axes,
                            object_extents,
                        );
                        if !metrics.feasible {
                            continue;
                        }
                        geometry_feasible_count += 1;
                        let total = approach_solution.score + grasp_solution.score;
                        if pair
                            .as_ref()
                            .map(|(score, _, _)| total < *score)
                            .unwrap_or(true)
                        {
                            pair = Some((total, approach_solution, grasp_solution));
                            break;
                        }
                    }
                }
            }
            let (approach_ik, grasp_ik) = match pair {
                Some((_, approach_solution, grasp_solution)) => {
                    (Some(approach_solution), Some(grasp_solution))
                }
                None => (None, None),
            };
            let grasp_metrics = grasp_ik
                .as_ref()
                .map(|solution| chain.grasp_metrics(solution, center, object_axes, object_extents));
            let grasp_feasible = grasp_metrics.as_ref().map(|m| m.feasible).unwrap_or(false);
            json!({
                "rank": rank + 1,
                "strategy": "closing_axis_aligned_free_roll",
                "approach_position_m": approach,
                "grasp_position_m": center,
                "clearance_m": clearance,
                "approach_ik": approach_ik.as_ref().map(crate::kinematics::solution_json),
                "grasp_ik": grasp_ik.as_ref().map(crate::kinematics::solution_json),
                "ik_feasible": approach_ik.is_some() && grasp_ik.is_some(),
                "diagnostics": {
                    "orientation_candidates": orientation_offsets.len(),
                    "approach_ik_count": approach_ik_count,
                    "grasp_ik_count": grasp_ik_count,
                    "geometry_feasible_count": geometry_feasible_count
                },
                "grasp_geometry": grasp_metrics.as_ref().map(crate::kinematics::grasp_metrics_json),
                "grasp_feasible": grasp_feasible,
                "uses_previous_absolute_pose": false
            })
        })
        .collect::<Vec<_>>();
    println!(
        "{}",
        serde_json::to_string(&json!({
            "ok": true,
            "mode": "online_plan_dry_run",
            "observation_frame_id": state.frame_id,
            "sensor_stamp_ns": state.sensor_stamp_ns,
            "camera_frame": state.tf.source_frame,
            "target_frame": state.tf.target_frame,
            "yellow_pixels": points.len(),
            "geometry_points": robust_points.len(),
            "current_joints_rad": names.iter().cloned().zip(current.iter().copied()).collect::<HashMap<_,_>>(),
            "current_tool_geometry": {
                "fixed_pad_inner_base_m": current_fixed_pad,
                "moving_pad_inner_base_m": current_moving_pad,
                "pad_midpoint_base_m": current_pad_midpoint,
                "horizontal_error_to_object_m": current_horizontal_error,
                "height_above_object_m": current_pad_midpoint[2] - center[2]
            },
            "pixel_center_uv": pixel_center,
            "object_center_m": center,
            "object_axes_base": object_axes,
            "object_extents_m": object_extents,
            "candidates": candidates
        }))
        .map_err(|e| e.to_string())?
    );
    Ok(())
}

fn yellow_component_mask(rgb: &image::RgbImage) -> Vec<bool> {
    let width = rgb.width() as usize;
    let height = rgb.height() as usize;
    let broad: Vec<bool> = rgb
        .pixels()
        .map(|pixel| {
            let [r, g, b] = pixel.0.map(|value| value as f64);
            r.max(g) >= 30.0 && r.min(g) - b >= 8.0 && (r - g).abs() <= 0.50 * r.max(g)
        })
        .collect();
    let mut visited = vec![false; broad.len()];
    let mut selected = vec![false; broad.len()];
    for start in 0..broad.len() {
        if visited[start] || !broad[start] {
            continue;
        }
        let mut queue = VecDeque::from([start]);
        visited[start] = true;
        let mut component = Vec::new();
        let mut sum_r = 0.0;
        let mut sum_g = 0.0;
        let mut sum_chroma = 0.0;
        while let Some(index) = queue.pop_front() {
            component.push(index);
            let [r, g, b] = rgb
                .get_pixel((index % width) as u32, (index / width) as u32)
                .0;
            sum_r += r as f64;
            sum_g += g as f64;
            sum_chroma += r.min(g).saturating_sub(b) as f64;
            let x = index % width;
            let y = index / width;
            for ny in y.saturating_sub(1)..=(y + 1).min(height - 1) {
                for nx in x.saturating_sub(1)..=(x + 1).min(width - 1) {
                    let neighbor = ny * width + nx;
                    if !visited[neighbor] && broad[neighbor] {
                        visited[neighbor] = true;
                        queue.push_back(neighbor);
                    }
                }
            }
        }
        let area = component.len();
        let mean_chroma = sum_chroma / area as f64;
        // R/G separates our yellow block from the other coloured things in frame,
        // and the window has to be TWO-SIDED. The lower bound was measured against
        // the green cube on frame 20260818-112803: yellow block 0.9762, green cube
        // 0.6424. The old 0.98 threshold sat ABOVE the block's own ratio, so
        // detection flipped between 1973 and 0 pixels on nothing but frame-to-frame
        // sensor noise (frame 20260818-112115 read 0.9833, the next frame 0.9762).
        // 0.85 sits in the middle of that gap: 13% margin on yellow, 21% on green.
        //
        // The upper bound was missing, and orange walks straight through a one-sided
        // r >= 0.85g test because orange has MORE red than green, not less. The
        // gripper's own pads are orange, so once the arm entered the frame the mask
        // merged pads into the object: frame 20260818-170043 planned a grasp on a
        // "91.4 x 39.6 mm object" from 4050 px, against a 51 x 19 x 29 mm block.
        // Measured on that frame over chromatic pixels only: yellow block 0.9881,
        // upper pad 1.7542, lower pad 1.3212. 1.15 sits in the gap with 16% margin
        // on yellow and 15% below the nearer pad. This is a TIGHTENING -- it can
        // only shrink the candidate set, never admit something the old test refused.
        if (20..=5000).contains(&area)
            && sum_r >= 0.85 * sum_g
            && sum_r <= 1.15 * sum_g
            && mean_chroma >= 10.0
        {
            for index in component {
                selected[index] = true;
            }
        }
    }
    selected
}

fn depth_at_or_neighbor(
    depth: &[f32],
    width: usize,
    height: usize,
    u: usize,
    v: usize,
) -> Option<f64> {
    let direct = depth[v * width + u] as f64;
    if direct.is_finite() && (0.20..=2.0).contains(&direct) {
        return Some(direct);
    }
    let mut nearby = Vec::new();
    for y in v.saturating_sub(2)..=(v + 2).min(height - 1) {
        for x in u.saturating_sub(2)..=(u + 2).min(width - 1) {
            let value = depth[y * width + x] as f64;
            if value.is_finite() && (0.20..=2.0).contains(&value) {
                nearby.push(value);
            }
        }
    }
    if nearby.len() < 3 {
        return None;
    }
    nearby.sort_by(|a, b| a.partial_cmp(b).unwrap_or(Ordering::Equal));
    if nearby[nearby.len() - 1] - nearby[0] > 0.040 {
        return None;
    }
    Some(nearby[nearby.len() / 2])
}

/// Object frame for a TOP-DOWN grasp, taken from the HORIZONTAL FOOTPRINT.
///
/// The closing axis of a top-down grasp is horizontal, so only the footprint
/// carries information about it. A 3D covariance of a single-view point set does
/// not give a horizontal axis: an upright block shows a small top face plus two
/// side faces, so every eigenvector comes out tilted. Frame 20260818-160425
/// measured |axis.z| = 0.790 / 0.445 / 0.422 with two near-equal extents (33.6
/// and 34.7 mm) -- a degenerate covariance, not a measurement of the box. The
/// best object_axis_angle_rad a horizontal closing axis can reach against such
/// an axis is asin(|axis.z|), i.e. 25 deg at best, so the <= 0.35 rad gate could
/// never be satisfied and the planner emitted zero orientation candidates.
///
/// Projecting to the xy plane first makes both in-plane axes horizontal by
/// construction, so that gate measures the in-plane misalignment it was written
/// to measure instead of the viewpoint's tilt. Third axis is the vertical, whose
/// extent is the height span; both downstream filters skip it on |z|.
/// Authorised by the operator 2026-08-18. No gate threshold changed.
fn object_obb(points: &[[f64; 3]], center: [f64; 3]) -> ([[f64; 3]; 3], [f64; 3]) {
    let center = Vector3::new(center[0], center[1], center[2]);
    let mut covariance = Matrix2::zeros();
    for point in points {
        let delta = Vector2::new(point[0] - center[0], point[1] - center[1]);
        covariance += delta * delta.transpose();
    }
    covariance /= points.len() as f64;
    let eigen = covariance.symmetric_eigen();
    // symmetric_eigen does not order eigenvalues, so pick the major axis
    // explicitly. A near-circular footprint makes the choice arbitrary, which is
    // correct: either axis is then an equally good closing axis.
    let major = if eigen.eigenvalues[0] >= eigen.eigenvalues[1] {
        0
    } else {
        1
    };
    let column = eigen.eigenvectors.column(major);
    let long = Vector3::new(column[0], column[1], 0.0).normalize();
    let short = Vector3::new(-long.y, long.x, 0.0);
    let frame = [long, short, Vector3::z()];
    let mut axes = [[0.0; 3]; 3];
    let mut extents = [0.0; 3];
    for (index, axis) in frame.iter().enumerate() {
        axes[index] = (*axis).into();
        let mut projections: Vec<f64> = points
            .iter()
            .map(|point| axis.dot(&(Vector3::new(point[0], point[1], point[2]) - center)))
            .collect();
        projections.sort_by(|a, b| a.partial_cmp(b).unwrap_or(Ordering::Equal));
        let low = projections[(projections.len() as f64 * 0.05) as usize];
        let high =
            projections[((projections.len() as f64 * 0.95) as usize).min(projections.len() - 1)];
        extents[index] = high - low;
    }
    (axes, extents)
}

fn robust_object_points(points: &[[f64; 3]], center: [f64; 3]) -> Vec<[f64; 3]> {
    let mut deviations = [Vec::new(), Vec::new(), Vec::new()];
    for point in points {
        for axis in 0..3 {
            deviations[axis].push((point[axis] - center[axis]).abs());
        }
    }
    let limits = deviations.map(|values| (median(values) * 4.0).clamp(0.012, 0.080));
    points
        .iter()
        .copied()
        .filter(|point| (0..3).all(|axis| (point[axis] - center[axis]).abs() <= limits[axis]))
        .collect()
}

fn in_manipulation_workspace(point: [f64; 3]) -> bool {
    (0.15..=0.80).contains(&point[0])
        && (-0.60..=0.60).contains(&point[1])
        && (0.76..=0.86).contains(&point[2])
}

fn read_json<T: for<'de> Deserialize<'de>>(path: &Path) -> Result<T, String> {
    let data = fs::read(path).map_err(|e| format!("{}: {e}", path.display()))?;
    serde_json::from_slice(&data).map_err(|e| format!("{}: {e}", path.display()))
}

fn transform_point(tf: &Transform, p: [f64; 3]) -> Result<[f64; 3], String> {
    if tf.translation_m.len() != 3 || tf.rotation_xyzw.len() != 4 {
        return Err("invalid TF dimensions".into());
    }
    let [x, y, z, w] = [
        tf.rotation_xyzw[0],
        tf.rotation_xyzw[1],
        tf.rotation_xyzw[2],
        tf.rotation_xyzw[3],
    ];
    let norm = (x * x + y * y + z * z + w * w).sqrt();
    if norm < 1e-9 {
        return Err("invalid zero quaternion".into());
    }
    let (x, y, z, w) = (x / norm, y / norm, z / norm, w / norm);
    let rotated = [
        (1.0 - 2.0 * (y * y + z * z)) * p[0]
            + 2.0 * (x * y - z * w) * p[1]
            + 2.0 * (x * z + y * w) * p[2],
        2.0 * (x * y + z * w) * p[0]
            + (1.0 - 2.0 * (x * x + z * z)) * p[1]
            + 2.0 * (y * z - x * w) * p[2],
        2.0 * (x * z - y * w) * p[0]
            + 2.0 * (y * z + x * w) * p[1]
            + (1.0 - 2.0 * (x * x + y * y)) * p[2],
    ];
    Ok([
        rotated[0] + tf.translation_m[0],
        rotated[1] + tf.translation_m[1],
        rotated[2] + tf.translation_m[2],
    ])
}

fn median(mut values: Vec<f64>) -> f64 {
    values.sort_by(|a, b| a.partial_cmp(b).unwrap_or(Ordering::Equal));
    values[values.len() / 2]
}

fn read_npy_f32(path: &Path, expected: usize) -> Result<Vec<f32>, String> {
    let data = fs::read(path).map_err(|e| format!("{}: {e}", path.display()))?;
    if data.len() < 12 || &data[..6] != b"\x93NUMPY" {
        return Err("invalid NPY file".into());
    }
    let major = data[6];
    let (header_len, start) = match major {
        1 => (u16::from_le_bytes([data[8], data[9]]) as usize, 10),
        2 | 3 => (
            u32::from_le_bytes([data[8], data[9], data[10], data[11]]) as usize,
            12,
        ),
        _ => return Err(format!("unsupported NPY version {major}")),
    };
    let payload = start + header_len;
    if payload > data.len() || (data.len() - payload) / 4 != expected {
        return Err("NPY shape or payload size mismatch".into());
    }
    Ok(data[payload..]
        .chunks_exact(4)
        .map(|v| f32::from_le_bytes([v[0], v[1], v[2], v[3]]))
        .collect())
}

#[cfg(test)]
mod tests {
    /// The block and the cube as actually measured on the table, so a regression in
    /// the R/G threshold fails here instead of silently reporting "0 valid pixels".
    #[test]
    fn mask_keeps_the_yellow_block_and_drops_the_green_cube() {
        let patch = |rgb: [u8; 3]| {
            let mut image = image::RgbImage::new(40, 40);
            for pixel in image.pixels_mut() {
                *pixel = image::Rgb(rgb);
            }
            super::yellow_component_mask(&image)
                .iter()
                .filter(|kept| **kept)
                .count()
        };
        assert!(patch([179, 184, 81]) > 0, "yellow block must be detected");
        assert_eq!(patch([96, 149, 69]), 0, "green cube must be rejected");
    }

    /// The exp-20 failure, as a unit test: a block standing on end, seen from one
    /// shallow viewpoint, so only the top face and two side faces have points.
    /// The old 3D covariance returned three tilted axes on this input and the
    /// planner emitted zero orientation candidates; the footprint OBB must return
    /// two horizontal axes and recover the 30 x 18 mm footprint.
    #[test]
    fn footprint_axes_stay_horizontal_for_a_block_standing_on_end() {
        // 30 x 18 mm footprint, 50 mm tall, long side along +x, on a 0.79 table.
        let (hx, hy, h) = (0.015, 0.009, 0.050);
        let mut points = Vec::new();
        for i in 0..20 {
            let t = i as f64 / 19.0;
            for j in 0..20 {
                let s = j as f64 / 19.0;
                // top face
                points.push([-hx + 2.0 * hx * t, -hy + 2.0 * hy * s, 0.79 + h]);
                // the two side faces a camera above and to one side can see
                points.push([-hx + 2.0 * hx * t, hy, 0.79 + h * s]);
                points.push([hx, -hy + 2.0 * hy * t, 0.79 + h * s]);
            }
        }
        let center = [
            super::median(points.iter().map(|p| p[0]).collect()),
            super::median(points.iter().map(|p| p[1]).collect()),
            super::median(points.iter().map(|p| p[2]).collect()),
        ];
        let (axes, extents) = super::object_obb(&points, center);
        // Both in-plane axes horizontal: this is what the 0.35 generator filter
        // and the 0.35 object_axis_angle_rad gate need, and what the 3D
        // covariance could not deliver (it gave |z| = 0.79 / 0.45 / 0.42).
        assert!(
            axes[0][2].abs() < 1e-12,
            "long axis must be horizontal: {axes:?}"
        );
        assert!(
            axes[1][2].abs() < 1e-12,
            "short axis must be horizontal: {axes:?}"
        );
        assert!(
            (axes[2][2] - 1.0).abs() < 1e-12,
            "third axis must be vertical"
        );
        // Orthonormal in-plane pair.
        let dot = axes[0][0] * axes[1][0] + axes[0][1] * axes[1][1];
        assert!(dot.abs() < 1e-12, "in-plane axes must be orthogonal: {dot}");
        // Long axis found the 30 mm side, not the 18 mm one.
        assert!(
            axes[0][0].abs() > 0.99,
            "long axis should lie along x: {axes:?}"
        );
        // 5/95 percentiles clip the ends, so extents read slightly under true size.
        assert!(
            (0.024..=0.030).contains(&extents[0]),
            "footprint length {}",
            extents[0]
        );
        assert!(
            (0.014..=0.018).contains(&extents[1]),
            "footprint width {}",
            extents[1]
        );
        assert!(
            (0.040..=0.050).contains(&extents[2]),
            "height span {}",
            extents[2]
        );
    }
    #[test]
    fn orange_pads_are_not_yellow_but_the_block_still_is() {
        // Means measured on frame 20260818-170043 over chromatic pixels only.
        // White gutters keep the three patches separate components, the way the
        // white table separates the block from the pads in a real frame.
        let mut image = image::RgbImage::new(70, 20);
        for y in 0..20 {
            for x in 0..70 {
                image.put_pixel(x, y, image::Rgb([230, 230, 230]));
            }
            for x in 0..20 {
                image.put_pixel(x, y, image::Rgb([176, 178, 41])); // yellow block
            }
            for x in 25..40 {
                image.put_pixel(x, y, image::Rgb([160, 91, 24])); // upper orange pad
            }
            for x in 45..60 {
                image.put_pixel(x, y, image::Rgb([161, 122, 91])); // lower orange pad
            }
        }
        let mask = super::yellow_component_mask(&image);
        let count = |x0: u32, x1: u32| {
            (x0..x1)
                .flat_map(|x| (0..20u32).map(move |y| (x, y)))
                .filter(|(x, y)| mask[(*y as usize) * 70 + *x as usize])
                .count()
        };
        assert_eq!(count(0, 20), 400, "the yellow block must still be selected");
        assert_eq!(count(25, 40), 0, "the upper orange pad must be rejected");
        assert_eq!(count(45, 60), 0, "the lower orange pad must be rejected");
    }
}
