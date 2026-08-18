use super::model::Chain;
use super::types::{GraspMetrics, Solution, OPEN_JAW_GAP_M};
use nalgebra::Vector3;
use std::collections::HashMap;

impl Chain {
    pub fn grasp_metrics(
        &self,
        solution: &Solution,
        object_center: [f64; 3],
        object_axes: [[f64; 3]; 3],
        object_extents: [f64; 3],
    ) -> GraspMetrics {
        let by_name = solution
            .joints
            .iter()
            .map(|(name, value)| (name.as_str(), *value))
            .collect::<HashMap<_, _>>();
        let joints = self
            .names()
            .iter()
            .map(|name| by_name[name.as_str()])
            .collect::<Vec<_>>();
        let tcp = self.fk(&joints);
        let (fixed_array, moving_array) = self.pad_inner_points(&joints);
        let fixed = nalgebra::Point3::from(fixed_array);
        let moving = nalgebra::Point3::from(moving_array);
        let midpoint = nalgebra::Point3::from((fixed.coords + moving.coords) * 0.5);
        let center = Vector3::new(object_center[0], object_center[1], object_center[2]);
        let center_error = (midpoint.coords - center).norm();
        let closing = tcp.rotation * Vector3::x();
        let mut best_angle = f64::INFINITY;
        let mut best_width = f64::INFINITY;
        for (axis, width) in object_axes.iter().zip(object_extents) {
            let axis = Vector3::new(axis[0], axis[1], axis[2]);
            if axis.z.abs() > 0.75 {
                continue;
            }
            let angle = closing.dot(&axis).abs().clamp(-1.0, 1.0).acos();
            if angle < best_angle {
                best_angle = angle;
                best_width = width;
            }
        }
        let clearance = OPEN_JAW_GAP_M - best_width;
        let fixed_signed = (fixed.coords - center).dot(&closing);
        let moving_signed = (moving.coords - center).dot(&closing);
        let half_width = best_width * 0.5;
        let pads_bracket_object = fixed_signed.max(moving_signed) >= half_width
            && fixed_signed.min(moving_signed) <= -half_width;
        let feasible = center_error <= 0.008
            && best_angle <= 0.35
            && best_width >= 0.005
            && clearance >= 0.004
            && pads_bracket_object;
        GraspMetrics {
            pad_midpoint_error_m: center_error,
            closing_axis: closing.into(),
            object_axis_angle_rad: best_angle,
            object_width_m: best_width,
            jaw_clearance_m: clearance,
            fixed_pad_inner_m: fixed.coords.into(),
            moving_pad_inner_m: moving.coords.into(),
            fixed_pad_signed_m: fixed_signed,
            moving_pad_signed_m: moving_signed,
            pads_bracket_object,
            feasible,
        }
    }
}
