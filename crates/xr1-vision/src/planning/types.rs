use crate::perception::ObjectGeometry;
use crate::proposal::VisionHarnessProposal;
use serde::Serialize;
use std::collections::BTreeMap;

#[derive(Debug, Serialize)]
pub struct ForwardKinematicsReport {
    pub ok: bool,
    pub joints_rad: BTreeMap<String, f64>,
    pub fixed_pad_inner_base_m: [f64; 3],
    pub moving_pad_inner_base_m: [f64; 3],
    pub pad_midpoint_base_m: [f64; 3],
    pub tcp_origin_base_m: [f64; 3],
    pub tool_rotation_base_rowmajor: [[f64; 3]; 3],
    pub floor_gate_threshold_m: f64,
    pub clears_floor_gate: bool,
}

#[derive(Debug, Serialize)]
pub struct PlanReport {
    pub ok: bool,
    pub schema_version: u32,
    pub mode: String,
    pub proposal: VisionHarnessProposal,
    pub observation_frame_id: String,
    pub sensor_stamp_ns: u64,
    pub observation_received_at_ns: u64,
    pub camera_frame: String,
    pub target_frame: String,
    pub yellow_pixels: usize,
    pub geometry_points: usize,
    pub current_joints_rad: BTreeMap<String, f64>,
    pub current_tool_geometry: CurrentToolGeometry,
    pub pixel_center_uv: [f64; 2],
    pub object_center_m: [f64; 3],
    pub object_axes_base: [[f64; 3]; 3],
    pub object_extents_m: [f64; 3],
    pub object: ObjectGeometry,
    pub candidates: Vec<GraspCandidate>,
}

#[derive(Debug, Serialize)]
pub struct CurrentToolGeometry {
    pub fixed_pad_inner_base_m: [f64; 3],
    pub moving_pad_inner_base_m: [f64; 3],
    pub pad_midpoint_base_m: [f64; 3],
    pub horizontal_error_to_object_m: f64,
    pub height_above_object_m: f64,
}

#[derive(Debug, Serialize)]
pub struct GraspCandidate {
    pub rank: usize,
    pub object_id: String,
    pub strategy: String,
    pub approach_position_m: [f64; 3],
    pub grasp_position_m: [f64; 3],
    pub closing_axis_base: Option<[f64; 3]>,
    pub roll_rad: Option<f64>,
    pub approach_direction_base: [f64; 3],
    pub contacts: Option<ContactPair>,
    pub required_gripper_width_m: Option<f64>,
    pub clearance_m: f64,
    pub approach_ik: Option<SolutionReport>,
    pub grasp_ik: Option<SolutionReport>,
    pub ik_feasible: bool,
    pub grasp_feasible: bool,
    pub joint_limit_margin_rad: Option<f64>,
    pub collision_margin_m: Option<f64>,
    pub table_clearance_m: Option<f64>,
    pub contact_quality: Option<f64>,
    pub score: Option<f64>,
    pub diagnostics: CandidateDiagnostics,
    pub grasp_geometry: Option<GraspGeometryReport>,
    pub uses_previous_absolute_pose: bool,
}

#[derive(Debug, Serialize)]
pub struct ContactPair {
    pub fixed_pad_inner_base_m: [f64; 3],
    pub moving_pad_inner_base_m: [f64; 3],
}

#[derive(Debug, Serialize)]
pub struct CandidateDiagnostics {
    pub coarse_orientation_candidates: usize,
    pub fine_orientation_candidates: usize,
    pub approach_ik_count: usize,
    pub grasp_ik_count: usize,
    pub geometry_feasible_count: usize,
}

#[derive(Debug, Serialize)]
pub struct SolutionReport {
    pub residual_m: f64,
    pub orientation_residual_rad: f64,
    pub max_delta_rad: f64,
    pub score: f64,
    pub floor_clear: bool,
    pub orientation_offset_rpy_rad: [f64; 3],
    pub min_limit_margin_rad: f64,
    pub min_tip_z_m: f64,
    pub tool_tip_center_m: [f64; 3],
    pub joints_rad: BTreeMap<String, f64>,
}

#[derive(Debug, Serialize)]
pub struct GraspGeometryReport {
    pub pad_midpoint_error_m: f64,
    pub closing_axis_base: [f64; 3],
    pub object_axis_angle_rad: f64,
    pub object_width_m: f64,
    pub open_jaw_gap_m: f64,
    pub jaw_clearance_m: f64,
    pub fixed_pad_inner_base_m: [f64; 3],
    pub moving_pad_inner_base_m: [f64; 3],
    pub fixed_pad_signed_from_object_m: f64,
    pub moving_pad_signed_from_object_m: f64,
    pub pads_bracket_object: bool,
    pub feasible: bool,
}
