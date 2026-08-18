use nalgebra::{Matrix3, Vector3};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, HashSet};

const MAX_CONDITION_NUMBER: f64 = 100.0;

#[derive(Clone, Debug, Default, Deserialize, Serialize)]
pub struct SensorRequirements {
    #[serde(default)]
    pub d405: bool,
    #[serde(default)]
    pub tactile: bool,
}

#[derive(Debug, Deserialize)]
pub struct ServoInput {
    pub schema_version: u32,
    pub observation_frame_id: String,
    /// The three named joints corresponding to the columns of `jacobian`.
    pub controlled_joints: Vec<String>,
    /// Row-major ds/dq. Signal is [u_px, v_px, depth_m].
    pub jacobian: [[f64; 3]; 3],
    /// Target minus current signal: [du_px, dv_px, dz_m].
    pub error: [f64; 3],
    #[serde(default = "default_damping")]
    pub damping: f64,
    #[serde(default)]
    pub requires: SensorRequirements,
}

#[derive(Debug, Serialize)]
pub struct ServoProposal {
    pub controlled_joints: Vec<String>,
    pub joint_delta_rad: BTreeMap<String, f64>,
    pub predicted_signal_delta: [f64; 3],
    pub predicted_remaining_error: [f64; 3],
    pub condition_number: f64,
    pub applied_scale: f64,
    pub max_joint_step_rad: f64,
}

pub fn propose(input: &ServoInput, max_joint_step_rad: f64) -> Result<ServoProposal, String> {
    validate(input, max_joint_step_rad)?;
    let flat = input.jacobian.iter().flatten().copied().collect::<Vec<_>>();
    let jacobian = Matrix3::from_row_slice(&flat);
    // The first two signal rows are pixels/rad while the third is metres/rad.
    // Row-normalise only for the observability check, then solve using the
    // original measured gains so units and predictions remain physical.
    let mut normalized = jacobian;
    for row in 0..3 {
        let norm = (0..3)
            .map(|column| normalized[(row, column)].powi(2))
            .sum::<f64>()
            .sqrt();
        if norm <= f64::EPSILON {
            return Err(format!(
                "visual-servo Jacobian signal row {row} is unobservable; re-measure at this pose"
            ));
        }
        for column in 0..3 {
            normalized[(row, column)] /= norm;
        }
    }
    let singular = normalized.svd(false, false).singular_values;
    let largest = singular.max();
    let smallest = singular.min();
    if smallest <= f64::EPSILON {
        return Err("visual-servo Jacobian is singular; re-measure at this pose".into());
    }
    let condition_number = largest / smallest;
    if condition_number > MAX_CONDITION_NUMBER {
        return Err(format!(
            "visual-servo Jacobian condition number {condition_number:.3} exceeds {MAX_CONDITION_NUMBER:.3}; re-measure at this pose"
        ));
    }

    let error = Vector3::from(input.error);
    let Some(inverse) = jacobian.try_inverse() else {
        return Err("visual-servo Jacobian cannot be inverted; re-measure at this pose".into());
    };
    let raw_delta = inverse * error * input.damping;
    let largest_step = raw_delta.abs().max();
    let applied_scale = if largest_step > max_joint_step_rad {
        max_joint_step_rad / largest_step
    } else {
        1.0
    };
    let joint_delta = raw_delta * applied_scale;
    let predicted_delta = jacobian * joint_delta;
    let remaining = error - predicted_delta;
    let joint_delta_array: [f64; 3] = joint_delta.into();

    Ok(ServoProposal {
        controlled_joints: input.controlled_joints.clone(),
        joint_delta_rad: input
            .controlled_joints
            .iter()
            .cloned()
            .zip(joint_delta_array)
            .collect(),
        predicted_signal_delta: predicted_delta.into(),
        predicted_remaining_error: remaining.into(),
        condition_number,
        applied_scale,
        max_joint_step_rad,
    })
}

fn validate(input: &ServoInput, max_joint_step_rad: f64) -> Result<(), String> {
    if input.schema_version != 1 {
        return Err(format!(
            "unsupported visual-servo schema version {}",
            input.schema_version
        ));
    }
    if input.observation_frame_id.trim().is_empty() {
        return Err("observation_frame_id must not be empty".into());
    }
    if input.controlled_joints.len() != 3 {
        return Err(format!(
            "visual servo requires exactly three controlled joints, got {}",
            input.controlled_joints.len()
        ));
    }
    let unique = input.controlled_joints.iter().collect::<HashSet<_>>();
    if unique.len() != input.controlled_joints.len() {
        return Err("controlled_joints must be unique".into());
    }
    if !input
        .jacobian
        .iter()
        .flatten()
        .chain(input.error.iter())
        .all(|value| value.is_finite())
    {
        return Err("visual-servo input contains non-finite values".into());
    }
    if !input.damping.is_finite() || !(0.0..=1.0).contains(&input.damping) {
        return Err("damping must be finite and within [0, 1]".into());
    }
    if !max_joint_step_rad.is_finite() || max_joint_step_rad <= 0.0 {
        return Err("max_joint_step_rad must be finite and positive".into());
    }
    Ok(())
}

fn default_damping() -> f64 {
    0.5
}

#[cfg(test)]
mod tests {
    use super::*;

    fn input(jacobian: [[f64; 3]; 3], error: [f64; 3]) -> ServoInput {
        ServoInput {
            schema_version: 1,
            observation_frame_id: "frame-1".into(),
            controlled_joints: vec![
                "right_arm_2_joint".into(),
                "right_arm_4_joint".into(),
                "right_arm_6_joint".into(),
            ],
            jacobian,
            error,
            damping: 0.5,
            requires: SensorRequirements::default(),
        }
    }

    #[test]
    fn square_exact_solve_halves_every_error_component() {
        let result = propose(
            &input(
                [[100.0, 20.0, 0.0], [0.0, 80.0, 10.0], [0.0, 0.0, 0.5]],
                [20.0, -10.0, 0.04],
            ),
            100.0,
        );
        assert!(result.is_ok());
        let Some(proposal) = result.ok() else { return };
        for (remaining, original) in proposal
            .predicted_remaining_error
            .iter()
            .zip([20.0, -10.0, 0.04])
        {
            assert!((remaining - original * 0.5).abs() < 1e-10);
        }
    }

    #[test]
    fn hard_step_ceiling_scales_the_whole_solution() {
        let result = propose(
            &input(
                [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
                [1.0, 0.5, 0.25],
            ),
            0.05,
        );
        assert!(result.is_ok());
        let Some(proposal) = result.ok() else { return };
        let largest = proposal
            .joint_delta_rad
            .values()
            .map(|value| value.abs())
            .fold(0.0, f64::max);
        assert!((largest - 0.05).abs() < 1e-12);
        assert!(proposal.applied_scale < 1.0);
    }

    #[test]
    fn zero_error_proposes_zero_motion() {
        let result = propose(
            &input(
                [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
                [0.0; 3],
            ),
            0.05,
        );
        assert!(result.is_ok());
        let Some(proposal) = result.ok() else { return };
        assert!(proposal.joint_delta_rad.values().all(|value| *value == 0.0));
    }

    #[test]
    fn duplicate_control_joint_is_rejected() {
        let mut request = input([[1.0, 0.0, 0.0]; 3], [1.0; 3]);
        request.controlled_joints[2] = request.controlled_joints[0].clone();
        assert!(propose(&request, 0.05).is_err());
    }

    #[test]
    fn singular_measurement_is_rejected() {
        assert!(propose(
            &input([[1.0, 0.0, 0.0], [0.0; 3], [0.0; 3]], [1.0; 3]),
            0.05
        )
        .is_err());
    }
}
