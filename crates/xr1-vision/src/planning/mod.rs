mod search;
mod types;

pub use types::{ForwardKinematicsReport, GraspCandidate, PlanReport};

use crate::kinematics::{Chain, PLANNING_MIN_TIP_Z_M};
use crate::perception::PerceptionFrame;
use crate::proposal::VisionHarnessProposal;
use std::path::Path;
use types::CurrentToolGeometry;

pub fn forward_kinematics_report(
    chain: &Chain,
    joints: &[f64],
) -> Result<ForwardKinematicsReport, String> {
    let names = chain.names();
    if joints.len() != names.len() {
        return Err(format!(
            "expected {} joint values, got {}",
            names.len(),
            joints.len()
        ));
    }
    if !joints.iter().all(|value| value.is_finite()) {
        return Err("joint vector contains non-finite values".into());
    }
    let (fixed, moving) = chain.pad_inner_points(joints);
    let midpoint = midpoint(fixed, moving);
    let tool = chain.fk(joints);
    let rotation = tool.rotation.to_rotation_matrix();
    let rows =
        std::array::from_fn(|row| [rotation[(row, 0)], rotation[(row, 1)], rotation[(row, 2)]]);
    Ok(ForwardKinematicsReport {
        ok: true,
        joints_rad: names.into_iter().zip(joints.iter().copied()).collect(),
        fixed_pad_inner_base_m: fixed,
        moving_pad_inner_base_m: moving,
        pad_midpoint_base_m: midpoint,
        tcp_origin_base_m: [tool.translation.x, tool.translation.y, tool.translation.z],
        tool_rotation_base_rowmajor: rows,
        floor_gate_threshold_m: PLANNING_MIN_TIP_Z_M,
        clears_floor_gate: midpoint[2] > PLANNING_MIN_TIP_Z_M,
    })
}

pub fn plan(
    proposal: VisionHarnessProposal,
    frame: PerceptionFrame,
    urdf_path: &Path,
) -> Result<PlanReport, String> {
    proposal.validate()?;
    let chain = Chain::from_urdf(urdf_path, "right_tcp_link")?;
    let names = chain.names();
    let current = names
        .iter()
        .map(|name| {
            frame
                .state
                .joint_state
                .positions_rad
                .get(name)
                .and_then(|value| *value)
                .ok_or_else(|| format!("missing live joint {name}"))
        })
        .collect::<Result<Vec<_>, _>>()?;
    let current_joints_rad = names.iter().cloned().zip(current.iter().copied()).collect();
    let (fixed_pad, moving_pad) = chain.pad_inner_points(&current);
    let pad_midpoint = midpoint(fixed_pad, moving_pad);
    let object = frame.object;
    let horizontal_error = ((pad_midpoint[0] - object.center_base_m[0]).powi(2)
        + (pad_midpoint[1] - object.center_base_m[1]).powi(2))
    .sqrt();
    let candidates = search::generate_candidates(
        &chain,
        &names,
        &current,
        &object,
        proposal.preferred_closing_axis,
    )?;

    Ok(PlanReport {
        ok: true,
        schema_version: 2,
        mode: "online_plan_dry_run".into(),
        proposal,
        observation_frame_id: frame.state.frame_id,
        sensor_stamp_ns: frame.state.sensor_stamp_ns,
        observation_received_at_ns: frame.state.received_at_ns,
        camera_frame: frame.camera_frame,
        target_frame: frame.target_frame,
        yellow_pixels: object.detected_pixels,
        geometry_points: object.geometry_points,
        current_joints_rad,
        current_tool_geometry: CurrentToolGeometry {
            fixed_pad_inner_base_m: fixed_pad,
            moving_pad_inner_base_m: moving_pad,
            pad_midpoint_base_m: pad_midpoint,
            horizontal_error_to_object_m: horizontal_error,
            height_above_object_m: pad_midpoint[2] - object.center_base_m[2],
        },
        pixel_center_uv: object.pixel_center_uv,
        object_center_m: object.center_base_m,
        object_axes_base: object.axes_base,
        object_extents_m: object.extents_m,
        object,
        candidates,
    })
}

fn midpoint(left: [f64; 3], right: [f64; 3]) -> [f64; 3] {
    std::array::from_fn(|index| (left[index] + right[index]) * 0.5)
}
