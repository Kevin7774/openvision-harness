use nalgebra::{Matrix3, Vector3};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, HashSet};

use crate::perception::ServoSignalSample;

const MAX_CONDITION_NUMBER: f64 = 100.0;
const STALL_REDUCTION_FRACTION: f64 = 0.10;
const STALL_STEP_COUNT: usize = 3;

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

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct ServoProposal {
    pub controlled_joints: Vec<String>,
    pub joint_delta_rad: BTreeMap<String, f64>,
    pub predicted_signal_delta: [f64; 3],
    pub predicted_remaining_error: [f64; 3],
    pub condition_number: f64,
    pub applied_scale: f64,
    pub max_joint_step_rad: f64,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct JacobianPerturbationPair {
    pub joint: String,
    pub negative: ServoSignalSample,
    pub positive: ServoSignalSample,
}

#[derive(Debug, Deserialize)]
pub struct JacobianMeasurementInput {
    pub schema_version: u32,
    pub controlled_joints: Vec<String>,
    pub center: ServoSignalSample,
    pub pairs: Vec<JacobianPerturbationPair>,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct JacobianCalibration {
    pub schema_version: u32,
    pub reference_frame_id: String,
    pub controlled_joints: Vec<String>,
    pub reference_joints_rad: BTreeMap<String, f64>,
    pub reference_signal: [f64; 3],
    pub jacobian: [[f64; 3]; 3],
    pub perturbation_span_rad: BTreeMap<String, f64>,
    pub max_background_joint_drift_rad: f64,
    pub sample_frame_ids: Vec<String>,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct ServoTarget {
    pub schema_version: u32,
    pub source_frame_id: String,
    pub signal: [f64; 3],
    /// Per-axis acceptance band [u_px, v_px, optical_depth_m].
    pub tolerance: [f64; 3],
}

#[derive(Debug, Deserialize)]
pub struct CalibratedServoRequest {
    pub schema_version: u32,
    pub calibration: JacobianCalibration,
    pub target: ServoTarget,
    pub current: ServoSignalSample,
    #[serde(default = "default_damping")]
    pub damping: f64,
    #[serde(default)]
    pub requires: SensorRequirements,
}

#[derive(Debug, Deserialize)]
pub struct ReconciliationInput {
    pub schema_version: u32,
    pub target: ServoTarget,
    pub before: ServoSignalSample,
    pub after: ServoSignalSample,
    pub proposal: ServoProposal,
    #[serde(default)]
    pub prior_improvement_ratios: Vec<f64>,
}

#[derive(Debug, Serialize)]
pub struct ServoReconciliation {
    pub ok: bool,
    pub schema_version: u32,
    pub before_frame_id: String,
    pub after_frame_id: String,
    pub predicted_signal_delta: [f64; 3],
    pub actual_signal_delta: [f64; 3],
    pub prediction_residual: [f64; 3],
    pub predicted_actual_gain: [Option<f64>; 3],
    pub direction_match: bool,
    pub prediction_within_tolerance: bool,
    pub prediction_match: bool,
    pub before_error: [f64; 3],
    pub after_error: [f64; 3],
    pub normalized_error_before: f64,
    pub normalized_error_after: f64,
    pub improvement_ratio: f64,
    pub converged: bool,
    pub stalled: bool,
    pub continue_servo: bool,
    pub stop_reason: Option<String>,
}

pub fn measure_jacobian(input: &JacobianMeasurementInput) -> Result<JacobianCalibration, String> {
    if input.schema_version != 1 {
        return Err(format!(
            "unsupported Jacobian measurement schema {}",
            input.schema_version
        ));
    }
    validate_controlled_joints(&input.controlled_joints)?;
    validate_sample(&input.center)?;
    if input.pairs.len() != input.controlled_joints.len() {
        return Err(format!(
            "Jacobian measurement requires one +/- pair per controlled joint, got {}",
            input.pairs.len()
        ));
    }

    let mut pairs = BTreeMap::new();
    let mut frame_ids = HashSet::from([input.center.frame_id.clone()]);
    for pair in &input.pairs {
        if !input.controlled_joints.contains(&pair.joint) {
            return Err(format!(
                "perturbation joint {} is not controlled",
                pair.joint
            ));
        }
        if pairs.insert(pair.joint.clone(), pair).is_some() {
            return Err(format!("duplicate perturbation pair for {}", pair.joint));
        }
        validate_sample(&pair.negative)?;
        validate_sample(&pair.positive)?;
        if !frame_ids.insert(pair.negative.frame_id.clone())
            || !frame_ids.insert(pair.positive.frame_id.clone())
        {
            return Err("Jacobian measurement frame_ids must be unique".into());
        }
    }

    let mut jacobian = [[0.0; 3]; 3];
    let mut spans = BTreeMap::new();
    let mut max_background_drift = 0.0_f64;
    let mut sample_frame_ids = vec![input.center.frame_id.clone()];
    for (column, joint) in input.controlled_joints.iter().enumerate() {
        let pair = pairs
            .get(joint)
            .ok_or_else(|| format!("missing perturbation pair for {joint}"))?;
        let negative_joint = sample_joint(&pair.negative, joint)?;
        let positive_joint = sample_joint(&pair.positive, joint)?;
        let center_joint = sample_joint(&input.center, joint)?;
        let span = positive_joint - negative_joint;
        if !span.is_finite() || span.abs() <= 1e-9 {
            return Err(format!("perturbation span for {joint} is zero"));
        }
        if center_joint < negative_joint.min(positive_joint)
            || center_joint > negative_joint.max(positive_joint)
        {
            return Err(format!(
                "center joint {joint} is not bracketed by its +/- samples"
            ));
        }
        spans.insert(joint.clone(), span);
        for (row, values) in jacobian.iter_mut().enumerate() {
            values[column] = (pair.positive.signal[row] - pair.negative.signal[row]) / span;
        }
        for other in &input.controlled_joints {
            if other == joint {
                continue;
            }
            let center = sample_joint(&input.center, other)?;
            max_background_drift = max_background_drift
                .max((sample_joint(&pair.negative, other)? - center).abs())
                .max((sample_joint(&pair.positive, other)? - center).abs());
        }
        sample_frame_ids.push(pair.negative.frame_id.clone());
        sample_frame_ids.push(pair.positive.frame_id.clone());
    }

    let probe = ServoInput {
        schema_version: 1,
        observation_frame_id: input.center.frame_id.clone(),
        controlled_joints: input.controlled_joints.clone(),
        jacobian,
        error: [0.0; 3],
        damping: 0.5,
        requires: SensorRequirements::default(),
    };
    propose(&probe, 1.0)?;
    Ok(JacobianCalibration {
        schema_version: 1,
        reference_frame_id: input.center.frame_id.clone(),
        controlled_joints: input.controlled_joints.clone(),
        reference_joints_rad: input.center.joints_rad.clone(),
        reference_signal: input.center.signal,
        jacobian,
        perturbation_span_rad: spans,
        max_background_joint_drift_rad: max_background_drift,
        sample_frame_ids,
    })
}

pub fn input_from_calibration(
    calibration: &JacobianCalibration,
    target: &ServoTarget,
    current: &ServoSignalSample,
    damping: f64,
    requires: SensorRequirements,
) -> Result<ServoInput, String> {
    if calibration.schema_version != 1 || target.schema_version != 1 {
        return Err("servo calibration and target must use schema_version=1".into());
    }
    validate_controlled_joints(&calibration.controlled_joints)?;
    validate_target(target)?;
    validate_sample(current)?;
    for joint in &calibration.controlled_joints {
        sample_joint(current, joint)?;
    }
    let input = ServoInput {
        schema_version: 1,
        observation_frame_id: current.frame_id.clone(),
        controlled_joints: calibration.controlled_joints.clone(),
        jacobian: calibration.jacobian,
        error: std::array::from_fn(|axis| target.signal[axis] - current.signal[axis]),
        damping,
        requires,
    };
    validate(&input, crate::safety::MAX_SERVO_STEP_RAD)?;
    Ok(input)
}

pub fn input_from_request(request: &CalibratedServoRequest) -> Result<ServoInput, String> {
    if request.schema_version != 1 {
        return Err(format!(
            "unsupported calibrated servo request schema {}",
            request.schema_version
        ));
    }
    input_from_calibration(
        &request.calibration,
        &request.target,
        &request.current,
        request.damping,
        request.requires.clone(),
    )
}

pub fn reconcile(input: &ReconciliationInput) -> Result<ServoReconciliation, String> {
    if input.schema_version != 1 {
        return Err(format!(
            "unsupported servo reconciliation schema {}",
            input.schema_version
        ));
    }
    validate_target(&input.target)?;
    validate_sample(&input.before)?;
    validate_sample(&input.after)?;
    if input.before.frame_id == input.after.frame_id
        || input.after.received_at_ns <= input.before.received_at_ns
    {
        return Err("reconciliation requires a newer, distinct post-action frame".into());
    }
    if input.proposal.controlled_joints.is_empty()
        || input.proposal.joint_delta_rad.len() != input.proposal.controlled_joints.len()
    {
        return Err("reconciliation proposal has inconsistent named deltas".into());
    }
    if !input
        .prior_improvement_ratios
        .iter()
        .all(|value| value.is_finite())
    {
        return Err("prior improvement ratios must be finite".into());
    }

    let actual_signal_delta =
        std::array::from_fn(|axis| input.after.signal[axis] - input.before.signal[axis]);
    let prediction_residual = std::array::from_fn(|axis| {
        actual_signal_delta[axis] - input.proposal.predicted_signal_delta[axis]
    });
    let predicted_actual_gain = std::array::from_fn(|axis| {
        let predicted = input.proposal.predicted_signal_delta[axis];
        (predicted.abs() > 1e-9).then_some(actual_signal_delta[axis] / predicted)
    });
    let direction_match = (0..3).all(|axis| {
        let predicted = input.proposal.predicted_signal_delta[axis];
        predicted.abs() <= 1e-9
            || actual_signal_delta[axis].abs() <= 1e-9
            || predicted.signum() == actual_signal_delta[axis].signum()
    });
    let prediction_within_tolerance =
        (0..3).all(|axis| prediction_residual[axis].abs() <= input.target.tolerance[axis]);
    let before_error =
        std::array::from_fn(|axis| input.target.signal[axis] - input.before.signal[axis]);
    let after_error =
        std::array::from_fn(|axis| input.target.signal[axis] - input.after.signal[axis]);
    let normalized_error_before = normalized_error(before_error, input.target.tolerance);
    let normalized_error_after = normalized_error(after_error, input.target.tolerance);
    let improvement_ratio = if normalized_error_before <= f64::EPSILON {
        0.0
    } else {
        1.0 - normalized_error_after / normalized_error_before
    };
    let converged = (0..3).all(|axis| after_error[axis].abs() <= input.target.tolerance[axis]);
    let low_improvement_count = input
        .prior_improvement_ratios
        .iter()
        .rev()
        .take(STALL_STEP_COUNT - 1)
        .take_while(|value| **value < STALL_REDUCTION_FRACTION)
        .count()
        + usize::from(improvement_ratio < STALL_REDUCTION_FRACTION);
    let stalled = low_improvement_count >= STALL_STEP_COUNT;
    let prediction_match = direction_match && prediction_within_tolerance;
    let stop_reason = if converged {
        Some("converged".into())
    } else if !direction_match {
        Some("predicted and actual signal directions disagree; re-measure Jacobian".into())
    } else if !prediction_within_tolerance {
        Some(
            "predicted and actual signal deltas differ beyond tolerance; diagnose the local model"
                .into(),
        )
    } else if stalled {
        Some("error reduction stayed below 10% for three consecutive steps".into())
    } else {
        None
    };
    Ok(ServoReconciliation {
        ok: true,
        schema_version: 1,
        before_frame_id: input.before.frame_id.clone(),
        after_frame_id: input.after.frame_id.clone(),
        predicted_signal_delta: input.proposal.predicted_signal_delta,
        actual_signal_delta,
        prediction_residual,
        predicted_actual_gain,
        direction_match,
        prediction_within_tolerance,
        prediction_match,
        before_error,
        after_error,
        normalized_error_before,
        normalized_error_after,
        improvement_ratio,
        converged,
        stalled,
        continue_servo: stop_reason.is_none(),
        stop_reason,
    })
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
    validate_controlled_joints(&input.controlled_joints)?;
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

fn validate_controlled_joints(controlled_joints: &[String]) -> Result<(), String> {
    if controlled_joints.len() != 3 {
        return Err(format!(
            "visual servo requires exactly three controlled joints, got {}",
            controlled_joints.len()
        ));
    }
    if controlled_joints
        .iter()
        .any(|joint| joint.trim().is_empty())
    {
        return Err("controlled joint names must not be empty".into());
    }
    let unique = controlled_joints.iter().collect::<HashSet<_>>();
    if unique.len() != controlled_joints.len() {
        return Err("controlled_joints must be unique".into());
    }
    Ok(())
}

fn validate_sample(sample: &ServoSignalSample) -> Result<(), String> {
    if sample.schema_version != 1 {
        return Err(format!(
            "unsupported servo signal sample schema {}",
            sample.schema_version
        ));
    }
    if sample.frame_id.trim().is_empty() || sample.received_at_ns == 0 {
        return Err("servo signal sample requires frame_id and received_at_ns".into());
    }
    if !sample.signal.iter().all(|value| value.is_finite())
        || !sample.joints_rad.values().all(|value| value.is_finite())
    {
        return Err("servo signal sample contains non-finite values".into());
    }
    Ok(())
}

fn validate_target(target: &ServoTarget) -> Result<(), String> {
    if target.schema_version != 1 {
        return Err(format!(
            "unsupported servo target schema {}",
            target.schema_version
        ));
    }
    if target.source_frame_id.trim().is_empty() {
        return Err("servo target source_frame_id must not be empty".into());
    }
    if !target.signal.iter().all(|value| value.is_finite())
        || !target
            .tolerance
            .iter()
            .all(|value| value.is_finite() && *value > 0.0)
    {
        return Err("servo target signal/tolerance must be finite and tolerance positive".into());
    }
    Ok(())
}

fn sample_joint(sample: &ServoSignalSample, joint: &str) -> Result<f64, String> {
    sample
        .joints_rad
        .get(joint)
        .copied()
        .ok_or_else(|| format!("sample {} is missing joint {joint}", sample.frame_id))
}

fn normalized_error(error: [f64; 3], tolerance: [f64; 3]) -> f64 {
    error
        .iter()
        .zip(tolerance)
        .map(|(value, scale)| (value / scale).powi(2))
        .sum::<f64>()
        .sqrt()
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

    fn sample(
        frame_id: &str,
        received_at_ns: u64,
        joints: [f64; 3],
        signal: [f64; 3],
    ) -> ServoSignalSample {
        ServoSignalSample {
            schema_version: 1,
            frame_id: frame_id.into(),
            received_at_ns,
            joints_rad: [
                ("right_arm_2_joint".into(), joints[0]),
                ("right_arm_4_joint".into(), joints[1]),
                ("right_arm_6_joint".into(), joints[2]),
            ]
            .into_iter()
            .collect(),
            signal,
        }
    }

    fn calibration_input() -> JacobianMeasurementInput {
        let controlled: Vec<String> = vec![
            "right_arm_2_joint".into(),
            "right_arm_4_joint".into(),
            "right_arm_6_joint".into(),
        ];
        let center_signal = [700.0, 400.0, 0.5];
        let jacobian = [[100.0, 20.0, 0.0], [0.0, 80.0, 10.0], [0.0, 0.0, 0.5]];
        let pairs = controlled
            .iter()
            .enumerate()
            .map(|(column, joint)| {
                let negative_signal =
                    std::array::from_fn(|row| center_signal[row] - jacobian[row][column] * 0.02);
                let positive_signal =
                    std::array::from_fn(|row| center_signal[row] + jacobian[row][column] * 0.02);
                let mut negative_joints = [0.0; 3];
                let mut positive_joints = [0.0; 3];
                negative_joints[column] = -0.02;
                positive_joints[column] = 0.02;
                JacobianPerturbationPair {
                    joint: joint.clone(),
                    negative: sample(
                        &format!("negative-{column}"),
                        2 + column as u64 * 2,
                        negative_joints,
                        negative_signal,
                    ),
                    positive: sample(
                        &format!("positive-{column}"),
                        3 + column as u64 * 2,
                        positive_joints,
                        positive_signal,
                    ),
                }
            })
            .collect();
        JacobianMeasurementInput {
            schema_version: 1,
            controlled_joints: controlled,
            center: sample("center", 1, [0.0; 3], center_signal),
            pairs,
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

    #[test]
    fn central_difference_recovers_the_measured_jacobian() {
        let calibration = measure_jacobian(&calibration_input());
        assert!(calibration.is_ok());
        let Some(calibration) = calibration.ok() else {
            return;
        };
        let expected = [[100.0, 20.0, 0.0], [0.0, 80.0, 10.0], [0.0, 0.0, 0.5]];
        for (actual_row, expected_row) in calibration.jacobian.iter().zip(expected) {
            for (actual, expected) in actual_row.iter().zip(expected_row) {
                assert!((actual - expected).abs() < 1e-10);
            }
        }
        assert_eq!(calibration.sample_frame_ids.len(), 7);
    }

    #[test]
    fn reconciliation_accepts_matching_motion_and_rejects_a_sign_flip() {
        let target = ServoTarget {
            schema_version: 1,
            source_frame_id: "target".into(),
            signal: [710.0, 396.0, 0.52],
            tolerance: [2.0, 2.0, 0.003],
        };
        let proposal = ServoProposal {
            controlled_joints: vec![
                "right_arm_2_joint".into(),
                "right_arm_4_joint".into(),
                "right_arm_6_joint".into(),
            ],
            joint_delta_rad: [
                ("right_arm_2_joint".into(), 0.01),
                ("right_arm_4_joint".into(), -0.01),
                ("right_arm_6_joint".into(), 0.01),
            ]
            .into_iter()
            .collect(),
            predicted_signal_delta: [5.0, -2.0, 0.01],
            predicted_remaining_error: [5.0, -2.0, 0.01],
            condition_number: 1.0,
            applied_scale: 1.0,
            max_joint_step_rad: 0.05,
        };
        let matching = ReconciliationInput {
            schema_version: 1,
            target: target.clone(),
            before: sample("before", 10, [0.0; 3], [700.0, 400.0, 0.5]),
            after: sample("after", 20, [0.01, -0.01, 0.01], [704.8, 397.9, 0.5095]),
            proposal: proposal.clone(),
            prior_improvement_ratios: Vec::new(),
        };
        let report = reconcile(&matching);
        assert!(report.is_ok());
        let Some(report) = report.ok() else { return };
        assert!(report.prediction_match);
        assert!(report.improvement_ratio > 0.0);
        assert!(report.continue_servo);

        let sign_flip = ReconciliationInput {
            after: sample(
                "wrong-way",
                30,
                [-0.01, -0.01, 0.01],
                [695.0, 397.9, 0.5095],
            ),
            ..matching
        };
        let report = reconcile(&sign_flip);
        assert!(report.is_ok());
        let Some(report) = report.ok() else { return };
        assert!(!report.direction_match);
        assert!(!report.continue_servo);
    }

    #[test]
    fn reconciliation_stops_when_prediction_magnitude_misses_tolerance() {
        let input = ReconciliationInput {
            schema_version: 1,
            target: ServoTarget {
                schema_version: 1,
                source_frame_id: "target".into(),
                signal: [20.0, 20.0, 1.0],
                tolerance: [1.0, 1.0, 0.01],
            },
            before: sample("before", 10, [0.0; 3], [0.0, 0.0, 0.5]),
            after: sample("after", 20, [0.01; 3], [5.0, 5.0, 0.6]),
            proposal: ServoProposal {
                controlled_joints: vec!["a".into(), "b".into(), "c".into()],
                joint_delta_rad: [("a".into(), 0.01), ("b".into(), 0.01), ("c".into(), 0.01)]
                    .into_iter()
                    .collect(),
                predicted_signal_delta: [1.0, 1.0, 0.01],
                predicted_remaining_error: [19.0, 19.0, 0.49],
                condition_number: 1.0,
                applied_scale: 1.0,
                max_joint_step_rad: 0.05,
            },
            prior_improvement_ratios: Vec::new(),
        };
        let report = reconcile(&input).unwrap();
        assert!(report.direction_match);
        assert!(!report.prediction_within_tolerance);
        assert!(!report.prediction_match);
        assert!(!report.continue_servo);
        assert!(report.stop_reason.is_some());
    }

    #[test]
    fn three_low_improvement_steps_stop_the_loop() {
        let input = ReconciliationInput {
            schema_version: 1,
            target: ServoTarget {
                schema_version: 1,
                source_frame_id: "target".into(),
                signal: [10.0, 10.0, 1.0],
                tolerance: [1.0, 1.0, 0.1],
            },
            before: sample("before", 10, [0.0; 3], [0.0, 0.0, 0.0]),
            after: sample("after", 20, [0.01; 3], [0.1, 0.1, 0.01]),
            proposal: ServoProposal {
                controlled_joints: vec!["a".into(), "b".into(), "c".into()],
                joint_delta_rad: [("a".into(), 0.01), ("b".into(), 0.01), ("c".into(), 0.01)]
                    .into_iter()
                    .collect(),
                predicted_signal_delta: [0.1, 0.1, 0.01],
                predicted_remaining_error: [9.9, 9.9, 0.99],
                condition_number: 1.0,
                applied_scale: 1.0,
                max_joint_step_rad: 0.05,
            },
            prior_improvement_ratios: vec![0.05, 0.09],
        };
        let report = reconcile(&input);
        assert!(report.is_ok());
        assert!(report.ok().map(|value| value.stalled).unwrap_or(false));
    }
}
