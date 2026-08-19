use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

pub const DEFAULT_MAX_TACTILE_AGE_MS: f64 = 500.0;
pub const MAX_TACTILE_AGE_MS: f64 = 1_000.0;
pub const MIN_PAD_SAMPLES: usize = 5;

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct TactilePadSample {
    pub id: String,
    pub raw: f64,
    pub median_abs_deviation: f64,
    pub sample_count: usize,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct TactileObservation {
    pub schema_version: u32,
    pub sample_id: String,
    pub sensor_stamp_ns: u64,
    pub received_at_ns: u64,
    pub pads: Vec<TactilePadSample>,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct TactileCalibration {
    pub schema_version: u32,
    pub pad_ids: [String; 2],
    pub baseline_raw: [f64; 2],
    pub polarity: [f64; 2],
    pub max_baseline_drift: f64,
    pub contact_delta: f64,
    pub max_balance_delta: f64,
    pub pressure_ceiling_delta: f64,
    pub retention_delta: f64,
    pub max_retention_drop: f64,
    pub max_median_abs_deviation: f64,
    #[serde(default = "default_max_tactile_age_ms")]
    pub max_age_ms: f64,
}

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum TactileDecision {
    CloseIncrement,
    Hold,
    CorrectPadImbalance,
    ReleaseIncrement,
    Retained,
    Slipping,
}

#[derive(Clone, Debug, Serialize)]
pub struct TactileAssessment {
    pub ok: bool,
    pub sample_id: String,
    pub age_ms: f64,
    pub pad_delta: [f64; 2],
    pub contact: [bool; 2],
    pub balanced: bool,
    pub overpressure: bool,
    pub decision: TactileDecision,
    pub reason: String,
}

pub fn assess_baseline(
    observation: &TactileObservation,
    calibration: &TactileCalibration,
    now_ns: u64,
) -> Result<TactileAssessment, String> {
    let mut assessment = assess(observation, calibration, now_ns)?;
    let drift_ok = assessment
        .pad_delta
        .iter()
        .all(|delta| delta.abs() <= calibration.max_baseline_drift);
    if !drift_ok {
        return Err(format!(
            "open-gripper tactile baseline drift {:?} exceeds {:.6}",
            assessment.pad_delta, calibration.max_baseline_drift
        ));
    }
    assessment.decision = TactileDecision::Hold;
    assessment.reason = "fresh two-pad open baseline is within the calibrated drift band".into();
    Ok(assessment)
}

pub fn assess_closure(
    observation: &TactileObservation,
    calibration: &TactileCalibration,
    now_ns: u64,
) -> Result<TactileAssessment, String> {
    let mut assessment = assess(observation, calibration, now_ns)?;
    let contact = assessment.contact;
    let balance_error = (assessment.pad_delta[0] - assessment.pad_delta[1]).abs();
    assessment.balanced = contact == [true, true] && balance_error <= calibration.max_balance_delta;

    let (decision, reason) = if assessment.overpressure {
        (
            TactileDecision::ReleaseIncrement,
            format!(
                "pressure ceiling reached; deltas={:?} ceiling={:.6}",
                assessment.pad_delta, calibration.pressure_ceiling_delta
            ),
        )
    } else if contact == [false, false] {
        (
            TactileDecision::CloseIncrement,
            format!(
                "no contact yet; deltas={:?} contact_delta={:.6}",
                assessment.pad_delta, calibration.contact_delta
            ),
        )
    } else if assessment.balanced {
        (
            TactileDecision::Hold,
            format!("both pads have balanced contact; balance_error={balance_error:.6}"),
        )
    } else {
        (
            TactileDecision::CorrectPadImbalance,
            format!(
                "contact is one-sided or imbalanced; contact={contact:?} balance_error={balance_error:.6}"
            ),
        )
    };
    assessment.decision = decision;
    assessment.reason = reason;
    Ok(assessment)
}

pub fn assess_retention(
    held: &TactileObservation,
    current: &TactileObservation,
    calibration: &TactileCalibration,
    now_ns: u64,
) -> Result<TactileAssessment, String> {
    let held_assessment = assess(held, calibration, held.received_at_ns)?;
    let mut current_assessment = assess(current, calibration, now_ns)?;
    let retained = current_assessment
        .pad_delta
        .iter()
        .all(|delta| *delta >= calibration.retention_delta);
    let drops = std::array::from_fn::<_, 2, _>(|index| {
        held_assessment.pad_delta[index] - current_assessment.pad_delta[index]
    });
    let slipping = drops
        .iter()
        .any(|drop| *drop > calibration.max_retention_drop);
    if retained && !slipping {
        current_assessment.decision = TactileDecision::Retained;
        current_assessment.reason = format!(
            "both pads retain contact after lift; deltas={:?} drops={drops:?}",
            current_assessment.pad_delta
        );
    } else {
        current_assessment.decision = TactileDecision::Slipping;
        current_assessment.reason = format!(
            "grasp retention failed; retained={retained} deltas={:?} drops={drops:?}",
            current_assessment.pad_delta
        );
    }
    Ok(current_assessment)
}

fn assess(
    observation: &TactileObservation,
    calibration: &TactileCalibration,
    now_ns: u64,
) -> Result<TactileAssessment, String> {
    validate_calibration(calibration)?;
    if observation.schema_version != 1 {
        return Err(format!(
            "unsupported tactile observation schema {}",
            observation.schema_version
        ));
    }
    if observation.sample_id.trim().is_empty() {
        return Err("tactile observation has no sample_id".into());
    }
    if now_ns < observation.received_at_ns {
        return Err("tactile observation timestamp is in the future".into());
    }
    if now_ns < observation.sensor_stamp_ns {
        return Err("tactile sensor timestamp is in the future".into());
    }
    let received_age_ms = (now_ns - observation.received_at_ns) as f64 / 1_000_000.0;
    let sensor_age_ms = (now_ns - observation.sensor_stamp_ns) as f64 / 1_000_000.0;
    let age_ms = received_age_ms.max(sensor_age_ms);
    if age_ms > calibration.max_age_ms {
        return Err(format!(
            "tactile observation is stale: {age_ms:.3}ms > {:.3}ms",
            calibration.max_age_ms
        ));
    }
    if observation.sensor_stamp_ns == 0 || observation.sensor_stamp_ns > observation.received_at_ns
    {
        return Err("tactile sensor timestamp is after its receive timestamp".into());
    }

    let pads = observation
        .pads
        .iter()
        .map(|pad| (pad.id.as_str(), pad))
        .collect::<BTreeMap<_, _>>();
    if pads.len() != observation.pads.len() {
        return Err("tactile observation contains duplicate pad ids".into());
    }
    let ordered = calibration
        .pad_ids
        .iter()
        .map(|id| {
            pads.get(id.as_str())
                .copied()
                .ok_or_else(|| format!("tactile observation is missing calibrated pad {id}"))
        })
        .collect::<Result<Vec<_>, _>>()?;
    if pads.len() != ordered.len() {
        return Err("tactile observation has uncalibrated extra pads".into());
    }
    for pad in &ordered {
        if !pad.raw.is_finite() || !pad.median_abs_deviation.is_finite() {
            return Err(format!(
                "tactile pad {} contains a non-finite value",
                pad.id
            ));
        }
        if pad.sample_count < MIN_PAD_SAMPLES {
            return Err(format!(
                "tactile pad {} has {} samples; need at least {MIN_PAD_SAMPLES}",
                pad.id, pad.sample_count
            ));
        }
        if pad.median_abs_deviation > calibration.max_median_abs_deviation {
            return Err(format!(
                "tactile pad {} is noisy: MAD {:.6} > {:.6}",
                pad.id, pad.median_abs_deviation, calibration.max_median_abs_deviation
            ));
        }
    }

    let pad_delta = std::array::from_fn(|index| {
        (ordered[index].raw - calibration.baseline_raw[index]) * calibration.polarity[index]
    });
    let contact = std::array::from_fn(|index| pad_delta[index] >= calibration.contact_delta);
    let overpressure = pad_delta
        .iter()
        .any(|delta| *delta >= calibration.pressure_ceiling_delta);
    Ok(TactileAssessment {
        ok: true,
        sample_id: observation.sample_id.clone(),
        age_ms,
        pad_delta,
        contact,
        balanced: false,
        overpressure,
        decision: TactileDecision::Hold,
        reason: String::new(),
    })
}

fn validate_calibration(calibration: &TactileCalibration) -> Result<(), String> {
    if calibration.schema_version != 1 {
        return Err(format!(
            "unsupported tactile calibration schema {}",
            calibration.schema_version
        ));
    }
    if calibration.pad_ids[0].trim().is_empty()
        || calibration.pad_ids[1].trim().is_empty()
        || calibration.pad_ids[0] == calibration.pad_ids[1]
    {
        return Err("tactile calibration must name two distinct pads".into());
    }
    let values = calibration
        .baseline_raw
        .iter()
        .chain(calibration.polarity.iter())
        .copied()
        .chain([
            calibration.max_baseline_drift,
            calibration.contact_delta,
            calibration.max_balance_delta,
            calibration.pressure_ceiling_delta,
            calibration.retention_delta,
            calibration.max_retention_drop,
            calibration.max_median_abs_deviation,
            calibration.max_age_ms,
        ]);
    if !values.into_iter().all(f64::is_finite) {
        return Err("tactile calibration contains a non-finite value".into());
    }
    if calibration.polarity.iter().any(|value| value.abs() != 1.0) {
        return Err("tactile polarity values must be exactly -1 or +1".into());
    }
    if calibration.max_baseline_drift < 0.0
        || calibration.contact_delta <= calibration.max_baseline_drift
        || calibration.max_balance_delta < 0.0
        || calibration.pressure_ceiling_delta <= calibration.contact_delta
        || calibration.retention_delta <= 0.0
        || calibration.retention_delta > calibration.pressure_ceiling_delta
        || calibration.max_retention_drop < 0.0
        || calibration.max_median_abs_deviation < 0.0
        || calibration.max_age_ms <= 0.0
        || calibration.max_age_ms > MAX_TACTILE_AGE_MS
    {
        return Err("tactile calibration thresholds are inconsistent".into());
    }
    Ok(())
}

fn default_max_tactile_age_ms() -> f64 {
    DEFAULT_MAX_TACTILE_AGE_MS
}

#[cfg(test)]
mod tests {
    use super::*;

    fn calibration() -> TactileCalibration {
        TactileCalibration {
            schema_version: 1,
            pad_ids: ["fixed".into(), "moving".into()],
            baseline_raw: [100.0, 200.0],
            polarity: [1.0, -1.0],
            max_baseline_drift: 2.0,
            contact_delta: 10.0,
            max_balance_delta: 5.0,
            pressure_ceiling_delta: 50.0,
            retention_delta: 8.0,
            max_retention_drop: 8.0,
            max_median_abs_deviation: 1.0,
            max_age_ms: 500.0,
        }
    }

    fn observation(raw: [f64; 2], received_at_ns: u64) -> TactileObservation {
        TactileObservation {
            schema_version: 1,
            sample_id: format!("sample-{received_at_ns}"),
            sensor_stamp_ns: received_at_ns - 1_000_000,
            received_at_ns,
            pads: vec![
                TactilePadSample {
                    id: "fixed".into(),
                    raw: raw[0],
                    median_abs_deviation: 0.5,
                    sample_count: 10,
                },
                TactilePadSample {
                    id: "moving".into(),
                    raw: raw[1],
                    median_abs_deviation: 0.5,
                    sample_count: 10,
                },
            ],
        }
    }

    #[test]
    fn closure_advances_only_before_contact() {
        let report = assess_closure(
            &observation([105.0, 196.0], 1_000_000_000),
            &calibration(),
            1_100_000_000,
        )
        .unwrap();
        assert_eq!(report.decision, TactileDecision::CloseIncrement);
    }

    #[test]
    fn balanced_two_pad_contact_holds() {
        let report = assess_closure(
            &observation([120.0, 178.0], 1_000_000_000),
            &calibration(),
            1_100_000_000,
        )
        .unwrap();
        assert_eq!(report.decision, TactileDecision::Hold);
        assert!(report.balanced);
    }

    #[test]
    fn one_sided_contact_stops_closing() {
        let report = assess_closure(
            &observation([120.0, 196.0], 1_000_000_000),
            &calibration(),
            1_100_000_000,
        )
        .unwrap();
        assert_eq!(report.decision, TactileDecision::CorrectPadImbalance);
    }

    #[test]
    fn pressure_ceiling_requests_one_release_increment() {
        let report = assess_closure(
            &observation([151.0, 170.0], 1_000_000_000),
            &calibration(),
            1_100_000_000,
        )
        .unwrap();
        assert_eq!(report.decision, TactileDecision::ReleaseIncrement);
    }

    #[test]
    fn stale_or_noisy_samples_fail_closed() {
        assert!(assess_closure(
            &observation([120.0, 180.0], 1_000_000_000),
            &calibration(),
            1_600_000_001
        )
        .is_err());
        let mut noisy = observation([120.0, 180.0], 1_000_000_000);
        noisy.pads[0].median_abs_deviation = 1.1;
        assert!(assess_closure(&noisy, &calibration(), 1_100_000_000).is_err());
    }

    #[test]
    fn retention_detects_a_large_pad_drop() {
        let held = observation([125.0, 175.0], 1_000_000_000);
        let current = observation([112.0, 188.0], 1_200_000_000);
        let report = assess_retention(&held, &current, &calibration(), 1_250_000_000).unwrap();
        assert_eq!(report.decision, TactileDecision::Slipping);
    }
}
