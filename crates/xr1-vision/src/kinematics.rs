use nalgebra::{
    DMatrix, DVector, Isometry3, Matrix3, Rotation3, Translation3, Unit, UnitQuaternion, Vector3,
};
use roxmltree::Document;
use serde_json::{json, Value};
use std::collections::HashMap;
use std::fs;
use std::path::Path;

const TIP_CENTER_M: [f64; 3] = [-0.0225, 0.0, 0.0485];
const FIXED_PAD_INNER_M: [f64; 3] = [0.0015, 0.0, 0.0485];
const MOVING_PAD_INNER_OPEN_M: [f64; 3] = [-0.0450, 0.0, 0.0485];
const OPEN_JAW_GAP_M: f64 = 0.0465;
pub const PLANNING_MIN_TIP_Z_M: f64 = 0.785;
pub const PLANNING_MIN_LIMIT_MARGIN_RAD: f64 = 0.05;

#[derive(Clone)]
struct Joint {
    name: String,
    parent: String,
    origin: Isometry3<f64>,
    axis: Vector3<f64>,
    lower: f64,
    upper: f64,
    movable: bool,
}

pub struct Chain {
    joints: Vec<Joint>,
    active: Vec<usize>,
}

pub struct Solution {
    pub joints: Vec<(String, f64)>,
    pub residual_m: f64,
    pub orientation_residual_rad: f64,
    pub max_delta_rad: f64,
    pub score: f64,
    pub floor_clear: bool,
    pub orientation_offset_rpy_rad: [f64; 3],
    pub min_limit_margin_rad: f64,
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

impl Chain {
    pub fn from_urdf(path: &Path, tip: &str) -> Result<Self, String> {
        let xml = fs::read_to_string(path).map_err(|e| e.to_string())?;
        Self::from_urdf_xml(&xml, tip)
    }

    pub(crate) fn from_urdf_xml(xml: &str, tip: &str) -> Result<Self, String> {
        let doc = Document::parse(xml).map_err(|e| e.to_string())?;
        let mut by_child = HashMap::new();
        for node in doc.descendants().filter(|n| n.has_tag_name("joint")) {
            let name = node.attribute("name").unwrap_or("").to_string();
            let kind = node.attribute("type").unwrap_or("fixed");
            let parent = child_attr(&node, "parent", "link")?;
            let child = child_attr(&node, "child", "link")?;
            let origin_node = node.children().find(|n| n.has_tag_name("origin"));
            let xyz = parse_vec(
                origin_node
                    .and_then(|n| n.attribute("xyz"))
                    .unwrap_or("0 0 0"),
            )?;
            let rpy = parse_vec(
                origin_node
                    .and_then(|n| n.attribute("rpy"))
                    .unwrap_or("0 0 0"),
            )?;
            let origin = Isometry3::from_parts(
                Translation3::new(xyz[0], xyz[1], xyz[2]),
                UnitQuaternion::from_euler_angles(rpy[0], rpy[1], rpy[2]),
            );
            let axis_node = node.children().find(|n| n.has_tag_name("axis"));
            let axis_v = parse_vec(
                axis_node
                    .and_then(|n| n.attribute("xyz"))
                    .unwrap_or("0 0 1"),
            )?;
            let limit_node = node.children().find(|n| n.has_tag_name("limit"));
            let lower = limit_node
                .and_then(|n| n.attribute("lower"))
                .and_then(|v| v.parse().ok())
                .unwrap_or(0.0);
            let upper = limit_node
                .and_then(|n| n.attribute("upper"))
                .and_then(|v| v.parse().ok())
                .unwrap_or(0.0);
            by_child.insert(
                child.clone(),
                Joint {
                    name,
                    parent,
                    origin,
                    axis: Vector3::new(axis_v[0], axis_v[1], axis_v[2]),
                    lower,
                    upper,
                    movable: matches!(kind, "revolute" | "continuous"),
                },
            );
        }
        let mut joints = Vec::new();
        let mut link = tip.to_string();
        while link != "base_link" {
            let joint = by_child
                .get(&link)
                .ok_or_else(|| format!("no parent joint for {link}"))?
                .clone();
            link = joint.parent.clone();
            joints.push(joint);
        }
        joints.reverse();
        let active = joints
            .iter()
            .enumerate()
            .filter_map(|(i, j)| j.movable.then_some(i))
            .collect();
        Ok(Self { joints, active })
    }

    pub fn names(&self) -> Vec<String> {
        self.active
            .iter()
            .map(|&i| self.joints[i].name.clone())
            .collect()
    }

    pub fn pad_inner_points(&self, q: &[f64]) -> ([f64; 3], [f64; 3]) {
        let tcp = self.fk(q);
        let fixed = tcp
            * nalgebra::Point3::new(
                FIXED_PAD_INNER_M[0],
                FIXED_PAD_INNER_M[1],
                FIXED_PAD_INNER_M[2],
            );
        let moving = tcp
            * nalgebra::Point3::new(
                MOVING_PAD_INNER_OPEN_M[0],
                MOVING_PAD_INNER_OPEN_M[1],
                MOVING_PAD_INNER_OPEN_M[2],
            );
        (fixed.coords.into(), moving.coords.into())
    }

    pub fn offset_for_top_down_closing_axis(
        &self,
        current: &[f64],
        object_axis: [f64; 3],
        tilt_rad: f64,
    ) -> Option<[f64; 3]> {
        let mut x = Vector3::new(object_axis[0], object_axis[1], 0.0);
        if x.norm() < 1e-6 {
            return None;
        }
        x.normalize_mut();
        let z = Vector3::new(0.0, 0.0, -1.0);
        let mut y = z.cross(&x);
        y.normalize_mut();
        x = y.cross(&z).normalize();
        let desired_matrix = Matrix3::from_columns(&[x, y, z]);
        let desired =
            UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(desired_matrix))
                * UnitQuaternion::from_axis_angle(&Vector3::x_axis(), tilt_rad);
        let current_rotation = self.fk(current).rotation;
        let delta = current_rotation.inverse() * desired;
        let (roll, pitch, yaw) = delta.euler_angles();
        Some([roll, pitch, yaw])
    }

    pub fn grasp_metrics(
        &self,
        solution: &Solution,
        object_center: [f64; 3],
        object_axes: [[f64; 3]; 3],
        object_extents: [f64; 3],
    ) -> GraspMetrics {
        let by_name: HashMap<&str, f64> = solution
            .joints
            .iter()
            .map(|(name, value)| (name.as_str(), *value))
            .collect();
        let q: Vec<f64> = self
            .names()
            .iter()
            .map(|name| by_name[name.as_str()])
            .collect();
        let tcp = self.fk(&q);
        let (fixed_pad_array, moving_pad_array) = self.pad_inner_points(&q);
        let fixed_pad = nalgebra::Point3::from(fixed_pad_array);
        let moving_pad = nalgebra::Point3::from(moving_pad_array);
        let midpoint = nalgebra::Point3::from((fixed_pad.coords + moving_pad.coords) * 0.5);
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
        let fixed_signed = (fixed_pad.coords - center).dot(&closing);
        let moving_signed = (moving_pad.coords - center).dot(&closing);
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
            fixed_pad_inner_m: fixed_pad.coords.into(),
            moving_pad_inner_m: moving_pad.coords.into(),
            fixed_pad_signed_m: fixed_signed,
            moving_pad_signed_m: moving_signed,
            pads_bracket_object,
            feasible,
        }
    }

    pub(crate) fn fk(&self, q: &[f64]) -> Isometry3<f64> {
        let mut pose = Isometry3::identity();
        let mut qi = 0;
        for joint in &self.joints {
            pose *= joint.origin;
            if joint.movable {
                let axis = Unit::new_normalize(joint.axis);
                pose *= Isometry3::from_parts(
                    Translation3::identity(),
                    UnitQuaternion::from_axis_angle(&axis, q[qi]),
                );
                qi += 1;
            }
        }
        pose
    }

    fn contact_pose(&self, q: &[f64]) -> Isometry3<f64> {
        self.fk(q) * Isometry3::translation(TIP_CENTER_M[0], TIP_CENTER_M[1], TIP_CENTER_M[2])
    }

    fn clamp(&self, q: &mut [f64]) {
        for (value, &index) in q.iter_mut().zip(&self.active) {
            let joint = &self.joints[index];
            *value = value.clamp(joint.lower, joint.upper);
        }
    }

    pub fn motion_envelope(
        &self,
        start: &[f64],
        target: &[f64],
        samples: usize,
    ) -> Result<MotionEnvelope, String> {
        let expected = self.active.len();
        if start.len() != expected || target.len() != expected {
            return Err(format!(
                "motion envelope expected {expected} joints, got start={} target={}",
                start.len(),
                target.len()
            ));
        }
        if samples == 0 {
            return Err("motion envelope requires at least one path sample".into());
        }
        if !start.iter().chain(target).all(|value| value.is_finite()) {
            return Err("motion envelope contains non-finite joint values".into());
        }

        let max_joint_delta_rad = start
            .iter()
            .zip(target)
            .map(|(a, b)| (a - b).abs())
            .fold(0.0, f64::max);
        let mut min_joint_limit_margin_rad = f64::INFINITY;
        let mut joint_limits_ok = true;
        for ((start_value, target_value), &index) in start.iter().zip(target).zip(&self.active) {
            let joint = &self.joints[index];
            joint_limits_ok &= *start_value >= joint.lower
                && *start_value <= joint.upper
                && *target_value >= joint.lower
                && *target_value <= joint.upper;
            min_joint_limit_margin_rad = min_joint_limit_margin_rad
                .min(*target_value - joint.lower)
                .min(joint.upper - *target_value);
        }
        let min_tip_z_m = (0..=samples)
            .map(|step| {
                let t = step as f64 / samples as f64;
                let sample = start
                    .iter()
                    .zip(target)
                    .map(|(a, b)| a + t * (b - a))
                    .collect::<Vec<_>>();
                self.contact_pose(&sample).translation.vector.z
            })
            .fold(f64::INFINITY, f64::min);

        Ok(MotionEnvelope {
            max_joint_delta_rad,
            min_joint_limit_margin_rad,
            min_tip_z_m,
            joint_limits_ok,
        })
    }

    /// Return every safe IK branch found by the multi-seed search, ordered by
    /// score. Pair planning must use this instead of committing to the single
    /// best pregrasp branch before checking whether its descent branch exists.
    pub fn solve_position_candidates_with_reference(
        &self,
        target: [f64; 3],
        seed: &[f64],
        reference: &[f64],
        orientation_offset: [f64; 3],
    ) -> Vec<Solution> {
        let target_rotation = self.fk(reference).rotation
            * UnitQuaternion::from_euler_angles(
                orientation_offset[0],
                orientation_offset[1],
                orientation_offset[2],
            );
        self.solve_pose_candidates(
            Vector3::new(target[0], target[1], target[2]),
            target_rotation,
            seed,
            orientation_offset,
        )
    }

    fn solve_pose_candidates(
        &self,
        target: Vector3<f64>,
        target_rotation: UnitQuaternion<f64>,
        current: &[f64],
        orientation_offset: [f64; 3],
    ) -> Vec<Solution> {
        let midpoint: Vec<f64> = self
            .active
            .iter()
            .map(|&i| {
                let j = &self.joints[i];
                (j.lower + j.upper) * 0.5
            })
            .collect();
        let blended: Vec<f64> = current
            .iter()
            .zip(&midpoint)
            .map(|(a, b)| 0.7 * a + 0.3 * b)
            .collect();
        let mut seeds = vec![current.to_vec(), midpoint.clone(), blended.clone()];
        for index in 0..current.len() {
            for delta in [-0.45, 0.45] {
                let mut seed = blended.clone();
                seed[index] += delta;
                self.clamp(&mut seed);
                seeds.push(seed);
            }
        }
        let mut accepted = Vec::new();
        let mut best_rejected: Option<Solution> = None;
        for mut q in seeds {
            self.clamp(&mut q);
            for _ in 0..160 {
                let tcp_pose = self.fk(&q);
                let contact_pose = self.contact_pose(&q);
                let position_error = target - contact_pose.translation.vector;
                let orientation_error =
                    (target_rotation * tcp_pose.rotation.inverse()).scaled_axis();
                if position_error.norm() < 0.004 && orientation_error.norm() < 0.04 {
                    break;
                }
                let mut jac = DMatrix::zeros(6, q.len());
                let eps = 1e-4;
                for column in 0..q.len() {
                    let mut shifted = q.clone();
                    shifted[column] += eps;
                    let shifted_tcp = self.fk(&shifted);
                    let shifted_contact = self.contact_pose(&shifted);
                    let position_delta = (shifted_contact.translation.vector
                        - contact_pose.translation.vector)
                        / eps;
                    let rotation_delta =
                        (shifted_tcp.rotation * tcp_pose.rotation.inverse()).scaled_axis() / eps;
                    for row in 0..3 {
                        jac[(row, column)] = position_delta[row];
                    }
                    for row in 0..3 {
                        jac[(row + 3, column)] = rotation_delta[row];
                    }
                }
                let e = DVector::from_column_slice(&[
                    position_error[0],
                    position_error[1],
                    position_error[2],
                    orientation_error[0],
                    orientation_error[1],
                    orientation_error[2],
                ]);
                let lhs = &jac * jac.transpose() + DMatrix::identity(6, 6) * 1e-4;
                let Some(step6) = lhs.lu().solve(&e) else {
                    break;
                };
                let step = jac.transpose() * step6;
                for (value, delta) in q.iter_mut().zip(step.iter()) {
                    *value += delta.clamp(-0.08, 0.08);
                }
                self.clamp(&mut q);
            }
            let final_tcp = self.fk(&q);
            let final_contact = self.contact_pose(&q);
            let residual = (target - final_contact.translation.vector).norm();
            let orientation_residual = (target_rotation * final_tcp.rotation.inverse())
                .scaled_axis()
                .norm();
            let Ok(envelope) = self.motion_envelope(current, &q, 20) else {
                continue;
            };
            let max_delta = envelope.max_joint_delta_rad;
            let min_limit_margin = envelope.min_joint_limit_margin_rad;
            let floor_clear = envelope.min_tip_z_m > PLANNING_MIN_TIP_Z_M;
            let limit_penalty = if min_limit_margin < 0.12 {
                (0.12 - min_limit_margin) * 50.0
            } else {
                0.0
            };
            let score = residual * 1000.0
                + orientation_residual * 20.0
                + max_delta * 4.0
                + limit_penalty
                + if floor_clear { 0.0 } else { 1000.0 };
            let solution = Solution {
                joints: self.names().into_iter().zip(q).collect(),
                residual_m: residual,
                orientation_residual_rad: orientation_residual,
                max_delta_rad: max_delta,
                score,
                floor_clear,
                orientation_offset_rpy_rad: orientation_offset,
                min_limit_margin_rad: min_limit_margin,
            };
            if solution.residual_m <= 0.015
                && solution.orientation_residual_rad <= 0.10
                && solution.floor_clear
                && solution.min_limit_margin_rad >= PLANNING_MIN_LIMIT_MARGIN_RAD
            {
                accepted.push(solution);
            } else if best_rejected
                .as_ref()
                .map(|candidate| solution.score < candidate.score)
                .unwrap_or(true)
            {
                best_rejected = Some(solution);
            }
        }
        accepted.sort_by(|a, b| a.score.total_cmp(&b.score));
        if accepted.is_empty() {
            if let Some(solution) = &best_rejected {
                eprintln!(
                    "IK_REJECT target={:.4},{:.4},{:.4} offset={:.3},{:.3},{:.3} residual_m={:.5} orientation_rad={:.5} floor_clear={} limit_margin_rad={:.5}",
                    target[0], target[1], target[2], orientation_offset[0],
                    orientation_offset[1], orientation_offset[2], solution.residual_m,
                    solution.orientation_residual_rad, solution.floor_clear,
                    solution.min_limit_margin_rad
                );
            }
        }
        accepted
    }
}

pub fn solution_json(solution: &Solution) -> Value {
    json!({
        "residual_m": solution.residual_m,
        "orientation_residual_rad": solution.orientation_residual_rad,
        "max_delta_rad": solution.max_delta_rad,
        "score": solution.score,
        "floor_clear": solution.floor_clear,
        "orientation_offset_rpy_rad": solution.orientation_offset_rpy_rad,
        "min_limit_margin_rad": solution.min_limit_margin_rad,
        "tool_tip_center_m": TIP_CENTER_M,
        "joints_rad": solution.joints.iter().cloned().collect::<HashMap<_,_>>()
    })
}

pub fn grasp_metrics_json(metrics: &GraspMetrics) -> Value {
    json!({
        "pad_midpoint_error_m": metrics.pad_midpoint_error_m,
        "closing_axis_base": metrics.closing_axis,
        "object_axis_angle_rad": metrics.object_axis_angle_rad,
        "object_width_m": metrics.object_width_m,
        "open_jaw_gap_m": OPEN_JAW_GAP_M,
        "jaw_clearance_m": metrics.jaw_clearance_m,
        "fixed_pad_inner_base_m": metrics.fixed_pad_inner_m,
        "moving_pad_inner_base_m": metrics.moving_pad_inner_m,
        "fixed_pad_signed_from_object_m": metrics.fixed_pad_signed_m,
        "moving_pad_signed_from_object_m": metrics.moving_pad_signed_m,
        "pads_bracket_object": metrics.pads_bracket_object,
        "feasible": metrics.feasible,
    })
}

fn child_attr(node: &roxmltree::Node<'_, '_>, tag: &str, attr: &str) -> Result<String, String> {
    node.children()
        .find(|n| n.has_tag_name(tag))
        .and_then(|n| n.attribute(attr))
        .map(str::to_string)
        .ok_or_else(|| format!("joint missing {tag}/{attr}"))
}

fn parse_vec(value: &str) -> Result<[f64; 3], String> {
    let values = value
        .split_whitespace()
        .map(str::parse::<f64>)
        .collect::<Result<Vec<_>, _>>()
        .map_err(|e| e.to_string())?;
    if values.len() != 3 {
        return Err(format!("expected 3-vector: {value}"));
    }
    Ok([values[0], values[1], values[2]])
}

#[cfg(test)]
mod tests {
    use super::*;

    const ONE_JOINT_URDF: &str = r#"
        <robot name="test">
          <link name="base_link"/>
          <link name="right_tcp_link"/>
          <joint name="right_arm_1_joint" type="revolute">
            <parent link="base_link"/>
            <child link="right_tcp_link"/>
            <origin xyz="0 0 1" rpy="0 0 0"/>
            <axis xyz="0 0 1"/>
            <limit lower="-1" upper="1"/>
          </joint>
        </robot>
    "#;

    #[test]
    fn motion_envelope_reports_delta_margin_and_floor() {
        let chain = Chain::from_urdf_xml(ONE_JOINT_URDF, "right_tcp_link");
        assert!(chain.is_ok());
        let Some(chain) = chain.ok() else { return };
        let envelope = chain.motion_envelope(&[0.0], &[0.2], 20);
        assert!(envelope.is_ok());
        let Some(envelope) = envelope.ok() else {
            return;
        };
        assert!((envelope.max_joint_delta_rad - 0.2).abs() < 1e-12);
        assert!((envelope.min_joint_limit_margin_rad - 0.8).abs() < 1e-12);
        assert!(envelope.min_tip_z_m > 1.0);
        assert!(envelope.joint_limits_ok);
    }

    #[test]
    fn motion_envelope_marks_out_of_limit_target() {
        let chain = Chain::from_urdf_xml(ONE_JOINT_URDF, "right_tcp_link");
        assert!(chain.is_ok());
        let Some(chain) = chain.ok() else { return };
        let envelope = chain.motion_envelope(&[0.0], &[1.2], 20);
        assert!(envelope.is_ok());
        let Some(envelope) = envelope.ok() else {
            return;
        };
        assert!(!envelope.joint_limits_ok);
        assert!(envelope.min_joint_limit_margin_rad < 0.0);
    }
}
