//! Staleness-bound calibration.
//!
//! Step 1, failure mode #2 "made mobile": an old calibration copied to another
//! machine cannot be detected as already-invalid unless every calibration binds
//! the exact conditions it was measured under. Without that binding, the most
//! dangerous failure appears — software emits correct-looking output while the
//! physical result is wrong.
//!
//! Every [`CalibrationBinding`] therefore carries `robot_id`, sensor/tool
//! serials, URDF hash, station id, measurement time, applicable pose range,
//! sample count, and an error metric. [`CalibrationBinding::check`] compares that
//! binding against the robot the harness is *actually* running on and returns a
//! typed decision. A binding that is not [`CalibrationStatus::Valid`] must block
//! motion — this is what makes "download and use" safe for a second robot.

use serde::{Deserialize, Serialize};
use std::path::Path;

/// Identity and conditions of the robot the harness is running on *right now*.
/// Assembled at commissioning time from live discovery, not from any file that
/// could have travelled with a stale calibration.
#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct RobotIdentity {
    pub robot_id: String,
    pub station_id: String,
    pub urdf_hash: String,
    /// Live sensor/tool serials, keyed by role (e.g. "d405", "tool", "zed").
    pub serials: Vec<(String, String)>,
    /// Current wall-clock time in nanoseconds since the Unix epoch. Passed in so
    /// this crate stays clock-free and deterministically testable.
    pub now_ns: u64,
}

impl RobotIdentity {
    fn serial(&self, role: &str) -> Option<&str> {
        self.serials
            .iter()
            .find(|(key, _)| key == role)
            .map(|(_, value)| value.as_str())
    }
}

/// One calibration artifact plus the exact conditions under which it was
/// measured. The payload (extrinsics, Jacobian, thresholds) is opaque here; the
/// point of this type is the *binding*, not the numbers.
#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct CalibrationBinding {
    /// What this calibrates: "tool", "zed_extrinsics", "d405_extrinsics",
    /// "tactile", "servo", "station".
    pub kind: String,
    pub robot_id: String,
    pub station_id: String,
    pub urdf_hash: String,
    /// Roles whose serials must match the live robot for this calibration to
    /// apply (e.g. ["d405"] for a D405 extrinsic).
    pub required_serials: Vec<(String, String)>,
    pub measured_at_ns: u64,
    /// How long this calibration is trusted after measurement, in nanoseconds.
    pub valid_for_ns: u64,
    /// Applicable tool-pose range [min, max] per axis [x, y, z] in metres.
    pub pose_range_min_m: [f64; 3],
    pub pose_range_max_m: [f64; 3],
    pub sample_count: usize,
    /// Reported error metric of the fit (e.g. RMS residual in metres).
    pub error_metric: f64,
    /// Opaque calibration payload interpreted by the platform, not by this crate.
    pub payload: serde_json::Value,
}

/// The typed decision. Only `Valid` may gate motion; every other variant blocks.
#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
#[serde(tag = "status", rename_all = "snake_case")]
pub enum CalibrationStatus {
    Valid,
    /// Belongs to a different robot, station, or URDF — the copied-to-another-
    /// machine case Step 1 calls out by name.
    Mismatch { reason: String },
    /// Correct robot, but measured too long ago to trust.
    Stale { age_ns: u64, valid_for_ns: u64 },
    /// Too few samples or a non-finite / implausible error metric to trust.
    Insufficient { reason: String },
}

impl CalibrationStatus {
    /// The single question the motion gate asks.
    pub fn may_gate_motion(&self) -> bool {
        matches!(self, CalibrationStatus::Valid)
    }
}

impl CalibrationBinding {
    /// Minimum samples below which a calibration is considered untrustworthy.
    pub const MIN_SAMPLES: usize = 8;

    /// Compare this binding against the robot actually present. Ordering is
    /// deliberate: identity mismatch is reported before staleness, because a
    /// foreign-but-recent calibration is more dangerous than an old-but-own one
    /// and must never be mistaken for merely stale.
    pub fn check(&self, identity: &RobotIdentity) -> CalibrationStatus {
        if self.robot_id != identity.robot_id {
            return CalibrationStatus::Mismatch {
                reason: format!(
                    "calibration is for robot {:?}, running on {:?}",
                    self.robot_id, identity.robot_id
                ),
            };
        }
        if self.station_id != identity.station_id {
            return CalibrationStatus::Mismatch {
                reason: format!(
                    "calibration measured at station {:?}, running at {:?}",
                    self.station_id, identity.station_id
                ),
            };
        }
        if self.urdf_hash != identity.urdf_hash {
            return CalibrationStatus::Mismatch {
                reason: format!(
                    "calibration URDF hash {:?} does not match live {:?}",
                    self.urdf_hash, identity.urdf_hash
                ),
            };
        }
        for (role, expected) in &self.required_serials {
            match identity.serial(role) {
                Some(live) if live == expected => {}
                Some(live) => {
                    return CalibrationStatus::Mismatch {
                        reason: format!(
                            "{role} serial {expected:?} in calibration, but {live:?} is present"
                        ),
                    };
                }
                None => {
                    return CalibrationStatus::Mismatch {
                        reason: format!("{role} required by calibration is not present"),
                    };
                }
            }
        }
        if self.sample_count < Self::MIN_SAMPLES {
            return CalibrationStatus::Insufficient {
                reason: format!(
                    "{} samples is below the {} required",
                    self.sample_count,
                    Self::MIN_SAMPLES
                ),
            };
        }
        if !(self.error_metric.is_finite() && self.error_metric >= 0.0) {
            return CalibrationStatus::Insufficient {
                reason: "error_metric is not a finite non-negative number".into(),
            };
        }
        // Clock going backwards relative to the measurement is treated as a
        // mismatch of trust, not silently accepted.
        if identity.now_ns < self.measured_at_ns {
            return CalibrationStatus::Stale {
                age_ns: 0,
                valid_for_ns: self.valid_for_ns,
            };
        }
        let age_ns = identity.now_ns - self.measured_at_ns;
        if age_ns > self.valid_for_ns {
            return CalibrationStatus::Stale {
                age_ns,
                valid_for_ns: self.valid_for_ns,
            };
        }
        CalibrationStatus::Valid
    }
}

/// A full set of calibrations for one robot, as referenced by `platform.toml`.
#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct CalibrationManifest {
    pub robot_id: String,
    pub bindings: Vec<CalibrationBinding>,
}

impl CalibrationManifest {
    pub fn read(path: &Path) -> Result<Self, String> {
        let bytes =
            std::fs::read(path).map_err(|error| format!("{}: {error}", path.display()))?;
        serde_json::from_slice(&bytes)
            .map_err(|error| format!("invalid calibration manifest {}: {error}", path.display()))
    }

    /// The manifest as a whole gates motion only if every binding is `Valid` for
    /// the present robot. The first offending binding is returned so the operator
    /// sees exactly what blocked commissioning.
    pub fn gate_motion(
        &self,
        identity: &RobotIdentity,
    ) -> Result<(), (String, CalibrationStatus)> {
        if self.robot_id != identity.robot_id {
            return Err((
                "manifest".into(),
                CalibrationStatus::Mismatch {
                    reason: format!(
                        "manifest is for robot {:?}, running on {:?}",
                        self.robot_id, identity.robot_id
                    ),
                },
            ));
        }
        for binding in &self.bindings {
            let status = binding.check(identity);
            if !status.may_gate_motion() {
                return Err((binding.kind.clone(), status));
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn identity() -> RobotIdentity {
        RobotIdentity {
            robot_id: "xr1-thor-001".into(),
            station_id: "bench-a".into(),
            urdf_hash: "urdf-abc".into(),
            serials: vec![
                ("d405".into(), "262422270599".into()),
                ("tool".into(), "g2-right-01".into()),
            ],
            now_ns: 1_000_000_000_000,
        }
    }

    fn own_binding() -> CalibrationBinding {
        CalibrationBinding {
            kind: "d405_extrinsics".into(),
            robot_id: "xr1-thor-001".into(),
            station_id: "bench-a".into(),
            urdf_hash: "urdf-abc".into(),
            required_serials: vec![("d405".into(), "262422270599".into())],
            measured_at_ns: 999_000_000_000,
            valid_for_ns: 10_000_000_000,
            pose_range_min_m: [0.2, -0.3, 0.7],
            pose_range_max_m: [0.6, 0.3, 1.0],
            sample_count: 24,
            error_metric: 0.0018,
            payload: serde_json::json!({ "extrinsics": "..." }),
        }
    }

    #[test]
    fn own_fresh_calibration_is_valid_and_gates_motion() {
        let status = own_binding().check(&identity());
        assert_eq!(status, CalibrationStatus::Valid);
        assert!(status.may_gate_motion());
    }

    #[test]
    fn a_calibration_copied_from_another_robot_is_a_mismatch_and_blocks() {
        // The exact Step 1 scenario: same numbers, different robot.
        let mut foreign = own_binding();
        foreign.robot_id = "xr1-thor-002".into();
        let status = foreign.check(&identity());
        assert!(matches!(status, CalibrationStatus::Mismatch { .. }));
        assert!(!status.may_gate_motion());
    }

    #[test]
    fn a_calibration_from_a_different_station_blocks() {
        let mut moved = own_binding();
        moved.station_id = "bench-b".into();
        assert!(!moved.check(&identity()).may_gate_motion());
    }

    #[test]
    fn a_swapped_sensor_serial_blocks() {
        let mut swapped = own_binding();
        swapped.required_serials = vec![("d405".into(), "999999999999".into())];
        assert!(!swapped.check(&identity()).may_gate_motion());
    }

    #[test]
    fn a_changed_urdf_blocks() {
        let mut rebuilt = own_binding();
        rebuilt.urdf_hash = "urdf-xyz".into();
        assert!(!rebuilt.check(&identity()).may_gate_motion());
    }

    #[test]
    fn an_expired_calibration_is_stale_and_blocks() {
        let mut old = own_binding();
        old.measured_at_ns = 1; // long ago relative to now
        let status = old.check(&identity());
        assert!(matches!(status, CalibrationStatus::Stale { .. }));
        assert!(!status.may_gate_motion());
    }

    #[test]
    fn too_few_samples_is_insufficient_and_blocks() {
        let mut thin = own_binding();
        thin.sample_count = 1;
        assert!(matches!(
            thin.check(&identity()),
            CalibrationStatus::Insufficient { .. }
        ));
    }

    #[test]
    fn foreign_recent_calibration_reads_as_mismatch_not_stale() {
        // Ordering guarantee: identity is checked before age, so a foreign but
        // recent calibration never masquerades as merely stale.
        let mut foreign = own_binding();
        foreign.robot_id = "xr1-thor-002".into();
        foreign.measured_at_ns = identity().now_ns; // perfectly fresh
        assert!(matches!(
            foreign.check(&identity()),
            CalibrationStatus::Mismatch { .. }
        ));
    }

    #[test]
    fn manifest_blocks_on_first_bad_binding_and_names_it() {
        let mut bad = own_binding();
        bad.kind = "servo".into();
        bad.sample_count = 0;
        let manifest = CalibrationManifest {
            robot_id: "xr1-thor-001".into(),
            bindings: vec![own_binding(), bad],
        };
        let err = manifest.gate_motion(&identity()).unwrap_err();
        assert_eq!(err.0, "servo");
    }

    #[test]
    fn manifest_passes_when_every_binding_is_valid() {
        let manifest = CalibrationManifest {
            robot_id: "xr1-thor-001".into(),
            bindings: vec![own_binding()],
        };
        assert!(manifest.gate_motion(&identity()).is_ok());
    }
}
