pub const TIP_CENTER_M: [f64; 3] = [-0.0225, 0.0, 0.0485];
pub(super) const FIXED_PAD_INNER_M: [f64; 3] = [0.0015, 0.0, 0.0485];
pub(super) const MOVING_PAD_INNER_OPEN_M: [f64; 3] = [-0.0450, 0.0, 0.0485];
pub const OPEN_JAW_GAP_M: f64 = 0.0465;
pub const PLANNING_MIN_TIP_Z_M: f64 = 0.785;
pub const PLANNING_MIN_LIMIT_MARGIN_RAD: f64 = 0.05;

#[derive(Clone)]
pub struct Solution {
    pub joints: Vec<(String, f64)>,
    pub residual_m: f64,
    pub orientation_residual_rad: f64,
    pub max_delta_rad: f64,
    pub score: f64,
    pub floor_clear: bool,
    pub orientation_offset_rpy_rad: [f64; 3],
    pub min_limit_margin_rad: f64,
    pub min_tip_z_m: f64,
}

pub struct GraspMetrics {
    pub pad_midpoint_error_m: f64,
    pub closing_axis: [f64; 3],
    pub object_axis_angle_rad: f64,
    pub object_width_m: f64,
    pub jaw_clearance_m: f64,
    pub fixed_pad_inner_m: [f64; 3],
    pub moving_pad_inner_m: [f64; 3],
    pub fixed_pad_signed_m: f64,
    pub moving_pad_signed_m: f64,
    pub pads_bracket_object: bool,
    pub feasible: bool,
}

#[derive(Debug)]
pub struct MotionEnvelope {
    pub max_joint_delta_rad: f64,
    pub min_joint_limit_margin_rad: f64,
    pub min_tip_z_m: f64,
    pub joint_limits_ok: bool,
}
