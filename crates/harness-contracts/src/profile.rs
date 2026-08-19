//! Per-robot geometry and planning limits.
//!
//! Step 1 finding #2: `kinematics/types.rs` pins the current gripper, fingertip,
//! and table geometry as compile-time constants. `PLANNING_MIN_TIP_Z_M = 0.785`
//! is a *table height* — a measured fact about one robot at one workstation, not
//! a software default. Shipped unchanged to a robot at a different table, the
//! geometry gate silently lies.
//!
//! `RobotProfile` is where those facts move. The core reads them from a loaded
//! profile instead of a `const`. The reference profile
//! ([`RobotProfile::xr1_thor_reference`]) reproduces today's constants exactly,
//! so migrating to it changes no behaviour — proven by an equivalence test in
//! `xr1-vision`.

use serde::{Deserialize, Serialize};
use std::path::Path;

/// The tool-frame and planning geometry for one physical robot. Values that were
/// `const` in the core now live here, loaded per robot.
#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct RobotProfile {
    /// Robot model family (e.g. "xr1"). A profile for a different family is not
    /// interchangeable even if the numbers look similar.
    pub model: String,
    /// Specific robot this profile was authored for (e.g. "xr1-thor-001").
    pub robot_id: String,
    pub tool: ToolGeometry,
    pub planning: PlanningLimits,
}

/// Gripper / fingertip geometry in the tool frame, in metres.
#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct ToolGeometry {
    /// Tool-frame tip centre offset [x, y, z].
    pub tip_center_m: [f64; 3],
    /// Fixed-pad inner face [x, y, z].
    pub fixed_pad_inner_m: [f64; 3],
    /// Moving-pad inner face at the open position [x, y, z].
    pub moving_pad_inner_open_m: [f64; 3],
    /// Jaw gap when open, in metres.
    pub open_jaw_gap_m: f64,
}

/// Workstation-and-arm planning limits, in metres / radians.
#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct PlanningLimits {
    /// Minimum allowed tool-tip Z. This is a **table height** — station-bound.
    pub min_tip_z_m: f64,
    /// Minimum joint-limit margin a solution must keep.
    pub min_limit_margin_rad: f64,
}

impl RobotProfile {
    /// The reference XR1/Thor profile. Every value here is byte-for-byte the
    /// constant it replaces in `xr1-vision/src/kinematics/types.rs`, so adopting
    /// the profile is a no-op refactor, not a behaviour change.
    pub fn xr1_thor_reference() -> Self {
        Self {
            model: "xr1".into(),
            robot_id: "xr1-thor-reference".into(),
            tool: ToolGeometry {
                tip_center_m: [-0.0225, 0.0, 0.0485],
                fixed_pad_inner_m: [0.0015, 0.0, 0.0485],
                moving_pad_inner_open_m: [-0.0450, 0.0, 0.0485],
                open_jaw_gap_m: 0.0465,
            },
            planning: PlanningLimits {
                min_tip_z_m: 0.785,
                min_limit_margin_rad: 0.05,
            },
        }
    }

    pub fn read(path: &Path) -> Result<Self, String> {
        let bytes =
            std::fs::read(path).map_err(|error| format!("{}: {error}", path.display()))?;
        let profile: RobotProfile = serde_json::from_slice(&bytes)
            .map_err(|error| format!("invalid robot profile {}: {error}", path.display()))?;
        profile.validate()?;
        Ok(profile)
    }

    /// Reject profiles that cannot be a real robot. A profile that passes this is
    /// self-consistent; it is *not* thereby proven to match the physical robot —
    /// that is what a [`crate::calibration::CalibrationManifest`] binds.
    pub fn validate(&self) -> Result<(), String> {
        if self.model.trim().is_empty() {
            return Err("robot profile model must not be empty".into());
        }
        if self.robot_id.trim().is_empty() {
            return Err("robot profile robot_id must not be empty".into());
        }
        let finite3 = |v: [f64; 3]| v.iter().all(|value| value.is_finite());
        if !finite3(self.tool.tip_center_m)
            || !finite3(self.tool.fixed_pad_inner_m)
            || !finite3(self.tool.moving_pad_inner_open_m)
        {
            return Err("tool geometry contains non-finite values".into());
        }
        if !(self.tool.open_jaw_gap_m.is_finite() && self.tool.open_jaw_gap_m > 0.0) {
            return Err("open_jaw_gap_m must be finite and positive".into());
        }
        if !(self.planning.min_tip_z_m.is_finite() && self.planning.min_tip_z_m > 0.0) {
            return Err("min_tip_z_m must be finite and positive".into());
        }
        if !(self.planning.min_limit_margin_rad.is_finite()
            && self.planning.min_limit_margin_rad >= 0.0)
        {
            return Err("min_limit_margin_rad must be finite and non-negative".into());
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reference_profile_is_valid() {
        assert!(RobotProfile::xr1_thor_reference().validate().is_ok());
    }

    #[test]
    fn reference_profile_round_trips_through_json() {
        let profile = RobotProfile::xr1_thor_reference();
        let json = serde_json::to_string(&profile).unwrap();
        let parsed: RobotProfile = serde_json::from_str(&json).unwrap();
        assert_eq!(profile, parsed);
    }

    #[test]
    fn a_table_height_change_is_expressible_without_touching_source() {
        let mut profile = RobotProfile::xr1_thor_reference();
        profile.planning.min_tip_z_m = 0.712; // a different workstation's table
        assert!(profile.validate().is_ok());
        assert_ne!(
            profile.planning.min_tip_z_m,
            RobotProfile::xr1_thor_reference().planning.min_tip_z_m
        );
    }

    #[test]
    fn non_finite_geometry_is_rejected() {
        let mut profile = RobotProfile::xr1_thor_reference();
        profile.tool.tip_center_m = [f64::NAN, 0.0, 0.0];
        assert!(profile.validate().is_err());
    }
}
