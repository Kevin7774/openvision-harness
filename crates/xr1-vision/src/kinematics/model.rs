use super::types::{MotionEnvelope, FIXED_PAD_INNER_M, MOVING_PAD_INNER_OPEN_M, TIP_CENTER_M};
use nalgebra::{Isometry3, Matrix3, Rotation3, Translation3, Unit, UnitQuaternion, Vector3};
use roxmltree::Document;
use std::collections::HashMap;
use std::f64::consts::PI;
use std::fs;
use std::path::Path;

#[derive(Clone)]
pub(super) struct Joint {
    pub(super) name: String,
    pub(super) parent: String,
    pub(super) origin: Isometry3<f64>,
    pub(super) axis: Vector3<f64>,
    pub(super) lower: f64,
    pub(super) upper: f64,
    pub(super) movable: bool,
}

pub struct Chain {
    pub(super) joints: Vec<Joint>,
    pub(super) active: Vec<usize>,
}

impl Chain {
    pub fn from_urdf(path: &Path, tip: &str) -> Result<Self, String> {
        let xml = fs::read_to_string(path).map_err(|error| error.to_string())?;
        Self::from_urdf_xml(&xml, tip)
    }

    pub(crate) fn from_urdf_xml(xml: &str, tip: &str) -> Result<Self, String> {
        let document = Document::parse(xml).map_err(|error| error.to_string())?;
        let mut by_child = HashMap::new();
        for node in document
            .descendants()
            .filter(|node| node.has_tag_name("joint"))
        {
            let name = node.attribute("name").unwrap_or("").to_string();
            let kind = node.attribute("type").unwrap_or("fixed");
            let parent = child_attr(&node, "parent", "link")?;
            let child = child_attr(&node, "child", "link")?;
            let origin_node = node.children().find(|child| child.has_tag_name("origin"));
            let xyz = parse_vec(
                origin_node
                    .and_then(|origin| origin.attribute("xyz"))
                    .unwrap_or("0 0 0"),
            )?;
            let rpy = parse_vec(
                origin_node
                    .and_then(|origin| origin.attribute("rpy"))
                    .unwrap_or("0 0 0"),
            )?;
            let origin = Isometry3::from_parts(
                Translation3::new(xyz[0], xyz[1], xyz[2]),
                UnitQuaternion::from_euler_angles(rpy[0], rpy[1], rpy[2]),
            );
            let axis_node = node.children().find(|child| child.has_tag_name("axis"));
            let axis_value = parse_vec(
                axis_node
                    .and_then(|axis| axis.attribute("xyz"))
                    .unwrap_or("0 0 1"),
            )?;
            let axis = Vector3::new(axis_value[0], axis_value[1], axis_value[2]);
            let movable = matches!(kind, "revolute" | "continuous");
            if movable && axis.norm() < 1e-12 {
                return Err(format!("joint {name} has a zero rotation axis"));
            }
            let (lower, upper) = joint_limits(&node, &name, kind)?;
            by_child.insert(
                child,
                Joint {
                    name,
                    parent,
                    origin,
                    axis,
                    lower,
                    upper,
                    movable,
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
            .filter_map(|(index, joint)| joint.movable.then_some(index))
            .collect();
        Ok(Self { joints, active })
    }

    pub fn names(&self) -> Vec<String> {
        self.active
            .iter()
            .map(|&index| self.joints[index].name.clone())
            .collect()
    }

    pub fn pad_inner_points(&self, joints: &[f64]) -> ([f64; 3], [f64; 3]) {
        let tcp = self.fk(joints);
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
        roll_rad: f64,
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
                * UnitQuaternion::from_axis_angle(&Vector3::x_axis(), roll_rad);
        let delta = self.fk(current).rotation.inverse() * desired;
        let (roll, pitch, yaw) = delta.euler_angles();
        Some([roll, pitch, yaw])
    }

    pub(crate) fn fk(&self, joints: &[f64]) -> Isometry3<f64> {
        let mut pose = Isometry3::identity();
        let mut active_index = 0;
        for joint in &self.joints {
            pose *= joint.origin;
            if joint.movable {
                let axis = Unit::new_normalize(joint.axis);
                pose *= Isometry3::from_parts(
                    Translation3::identity(),
                    UnitQuaternion::from_axis_angle(&axis, joints[active_index]),
                );
                active_index += 1;
            }
        }
        pose
    }

    pub(super) fn contact_pose(&self, joints: &[f64]) -> Isometry3<f64> {
        self.fk(joints) * Isometry3::translation(TIP_CENTER_M[0], TIP_CENTER_M[1], TIP_CENTER_M[2])
    }

    pub(super) fn clamp(&self, joints: &mut [f64]) {
        for (value, &index) in joints.iter_mut().zip(&self.active) {
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
            .map(|(left, right)| (left - right).abs())
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
                let ratio = step as f64 / samples as f64;
                let sample = start
                    .iter()
                    .zip(target)
                    .map(|(left, right)| left + ratio * (right - left))
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
}

fn joint_limits(
    node: &roxmltree::Node<'_, '_>,
    name: &str,
    kind: &str,
) -> Result<(f64, f64), String> {
    if kind == "continuous" {
        return Ok((-PI, PI));
    }
    if kind != "revolute" {
        return Ok((0.0, 0.0));
    }
    let limit = node
        .children()
        .find(|child| child.has_tag_name("limit"))
        .ok_or_else(|| format!("joint {name} is missing limit"))?;
    let lower = limit
        .attribute("lower")
        .ok_or_else(|| format!("joint {name} is missing lower limit"))?
        .parse::<f64>()
        .map_err(|error| format!("joint {name} lower limit: {error}"))?;
    let upper = limit
        .attribute("upper")
        .ok_or_else(|| format!("joint {name} is missing upper limit"))?
        .parse::<f64>()
        .map_err(|error| format!("joint {name} upper limit: {error}"))?;
    if !lower.is_finite() || !upper.is_finite() || lower >= upper {
        return Err(format!("joint {name} has invalid limits {lower}..{upper}"));
    }
    Ok((lower, upper))
}

fn child_attr(node: &roxmltree::Node<'_, '_>, tag: &str, attr: &str) -> Result<String, String> {
    node.children()
        .find(|child| child.has_tag_name(tag))
        .and_then(|child| child.attribute(attr))
        .map(str::to_string)
        .ok_or_else(|| format!("joint missing {tag}/{attr}"))
}

fn parse_vec(value: &str) -> Result<[f64; 3], String> {
    let values = value
        .split_whitespace()
        .map(str::parse::<f64>)
        .collect::<Result<Vec<_>, _>>()
        .map_err(|error| error.to_string())?;
    if values.len() != 3 || !values.iter().all(|value| value.is_finite()) {
        return Err(format!("expected finite 3-vector: {value}"));
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

    #[test]
    fn malformed_revolute_limits_are_rejected() {
        let xml = ONE_JOINT_URDF.replace("upper=\"1\"", "upper=\"bad\"");
        assert!(Chain::from_urdf_xml(&xml, "right_tcp_link").is_err());
    }
}
