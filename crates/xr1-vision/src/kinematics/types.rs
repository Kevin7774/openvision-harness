// Step 1 finding #2: these are measured facts about one robot at one
// workstation, not software defaults. They now have a home in
// `harness_contracts::RobotProfile`. The constants remain the compiled default
// so no behaviour changes today, and `profile_equivalence` below asserts the two
// can never silently diverge — edit one without the other and the test fails.
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

#[cfg(test)]
mod tests {
    use super::*;
    use harness_contracts::RobotProfile;

    /// The reference profile must reproduce the compiled constants exactly.
    /// Migrating the core to read from a profile is therefore a no-op refactor,
    /// and this test is the tripwire that keeps the two representations honest.
    #[test]
    fn profile_equivalence() {
        let profile = RobotProfile::xr1_thor_reference();
        assert_eq!(profile.tool.tip_center_m, TIP_CENTER_M);
        assert_eq!(profile.tool.fixed_pad_inner_m, FIXED_PAD_INNER_M);
        assert_eq!(
            profile.tool.moving_pad_inner_open_m,
            MOVING_PAD_INNER_OPEN_M
        );
        assert_eq!(profile.tool.open_jaw_gap_m, OPEN_JAW_GAP_M);
        assert_eq!(profile.planning.min_tip_z_m, PLANNING_MIN_TIP_Z_M);
        assert_eq!(
            profile.planning.min_limit_margin_rad,
            PLANNING_MIN_LIMIT_MARGIN_RAD
        );
    }
}
