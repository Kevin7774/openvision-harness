use crate::grasp_feedback::{
    self, TactileAssessment, TactileCalibration, TactileDecision, TactileObservation,
};
use crate::perception;
use crate::runtime::{self, RuntimePaths};
use crate::support::adapter::{json_string, parse_last_json, require_ok};
use crate::support::args::{flag, option, optional_option};
use crate::support::evidence::{canonical_file, create_new_json, read_json};
use crate::support::runlock::RobotActionLoopLock;
use crate::visual_servo::{self, ServoTarget};
use serde::{Deserialize, Serialize};
use std::fs::{self, OpenOptions};
use std::io::Write;
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

const CLOSE_INCREMENT: f64 = 0.05;
// Both G2 grippers measured 838--840 mm at close01=0.00 on 2026-08-10.
const OPEN_POSITION_MIN_MM: u32 = 830;
const DEFAULT_MAX_STEPS: usize = 20;
const DEFAULT_TIMEOUT_S: f64 = 360.0;
const MIN_STEP_BUDGET_S: f64 = 16.0;
const MAX_D405_AUTHORIZATION_AGE_MS: f64 = 3_000.0;

pub fn run(runtime: &RuntimePaths, args: Vec<String>) -> Result<(), String> {
    let config = GraspLoopConfig::parse(&args)?;
    GraspLoopSession::start(runtime, config)?.execute()
}

struct GraspLoopConfig {
    tactile_config: PathBuf,
    tactile_calibration: TactileCalibration,
    d405_target: ServoTarget,
    side: String,
    max_steps: usize,
    timeout_s: f64,
    go: bool,
}

impl GraspLoopConfig {
    fn parse(args: &[String]) -> Result<Self, String> {
        validate_command_args(
            args,
            &[
                "--tactile-config",
                "--tactile-calibration",
                "--d405-target",
                "--side",
                "--max-steps",
                "--timeout",
            ],
            &["--go"],
        )?;
        let tactile_config = canonical_file(&option(args, "--tactile-config")?)?;
        let tactile_calibration: TactileCalibration = read_json(
            Path::new(&option(args, "--tactile-calibration")?),
            "tactile calibration",
        )?;
        let d405_target: ServoTarget = read_json(
            Path::new(&option(args, "--d405-target")?),
            "D405 grasp target",
        )?;
        let side = optional_option(args, "--side")?.unwrap_or_else(|| "right".into());
        if side != "right" {
            return Err("the calibrated D405 is on the right wrist; --side must be right".into());
        }
        if !d405_target.source_frame_id.starts_with("d405-") {
            return Err("D405 grasp target must be measured from a named d405-* frame".into());
        }
        let max_steps = optional_option(args, "--max-steps")?
            .map(|value| {
                value
                    .parse::<usize>()
                    .map_err(|_| "--max-steps must be an integer".to_string())
            })
            .transpose()?
            .unwrap_or(DEFAULT_MAX_STEPS);
        if !(1..=20).contains(&max_steps) {
            return Err("--max-steps must be within 1..20".into());
        }
        let timeout_s = optional_option(args, "--timeout")?
            .map(|value| {
                value
                    .parse::<f64>()
                    .map_err(|_| "--timeout must be a number".to_string())
            })
            .transpose()?
            .unwrap_or(DEFAULT_TIMEOUT_S);
        if !timeout_s.is_finite() || !(30.0..=600.0).contains(&timeout_s) {
            return Err("--timeout must be within 30..600 seconds".into());
        }
        Ok(Self {
            tactile_config,
            tactile_calibration,
            d405_target,
            side,
            max_steps,
            timeout_s,
            go: flag(args, "--go"),
        })
    }
}

struct GraspLoopSession<'a> {
    runtime: &'a RuntimePaths,
    config: GraspLoopConfig,
    session_id: String,
    session_dir: PathBuf,
    events_path: PathBuf,
    deadline: Instant,
    _lock: RobotActionLoopLock,
}

impl<'a> GraspLoopSession<'a> {
    fn start(runtime: &'a RuntimePaths, config: GraspLoopConfig) -> Result<Self, String> {
        let started_ns = runtime::unix_time_ns()?;
        let session_id = format!("{started_ns}-{}", std::process::id());
        let lock = RobotActionLoopLock::acquire("grasp-feedback loop", &session_id)?;
        let session_dir = runtime
            .data_root()
            .join("experiments")
            .join("grasp_feedback")
            .join(&session_id);
        fs::create_dir_all(&session_dir)
            .map_err(|error| format!("{}: {error}", session_dir.display()))?;
        let events_path = session_dir.join("events.jsonl");
        let deadline = Instant::now() + Duration::from_secs_f64(config.timeout_s);
        let session = Self {
            runtime,
            config,
            session_id,
            session_dir,
            events_path,
            deadline,
            _lock: lock,
        };
        session.record(serde_json::json!({
            "event": "grasp_loop_started",
            "at_ns": started_ns,
            "session_id": session.session_id,
            "go": session.config.go,
            "side": session.config.side,
            "max_steps": session.config.max_steps,
            "timeout_s": session.config.timeout_s,
            "close_increment": CLOSE_INCREMENT,
            "service_policy": "never_start_stop_or_restart_services; bounded direct devices refuse when owned"
        }))?;
        Ok(session)
    }

    fn execute(&self) -> Result<(), String> {
        let gripper = self.observe_gripper()?;
        if gripper.position_mm < OPEN_POSITION_MIN_MM {
            return self.finish(
                "refused",
                false,
                0,
                Some("gripper is not at the measured open position; opening is a separate action"),
            );
        }
        let near_field = self.capture_d405()?;
        if !visual_servo::target_reached(&self.config.d405_target, &near_field.sample)? {
            return self.finish(
                "refused",
                false,
                0,
                Some("fresh D405 target is outside the calibrated grasp tolerance"),
            );
        }
        self.record(serde_json::json!({
            "event": "d405_grasp_alignment_verified",
            "at_ns": runtime::unix_time_ns()?,
            "sample": near_field
        }))?;
        let mut d405_sample = near_field.sample.clone();
        let baseline = self.capture_tactile()?;
        let baseline_assessment = grasp_feedback::assess_baseline(
            &baseline,
            &self.config.tactile_calibration,
            runtime::unix_time_ns()?,
        )?;
        self.record(serde_json::json!({
            "event": "open_tactile_baseline_verified",
            "at_ns": runtime::unix_time_ns()?,
            "gripper": gripper,
            "tactile": baseline,
            "assessment": baseline_assessment
        }))?;

        let mut progress = GripProgress {
            close_fraction: 0.0,
            expected_position_mm: gripper.position_mm,
        };
        let mut current = baseline;
        for step_index in 1..=self.config.max_steps {
            if self
                .deadline
                .saturating_duration_since(Instant::now())
                .as_secs_f64()
                < MIN_STEP_BUDGET_S
            {
                return self.finish(
                    "timeout",
                    self.config.go && step_index > 1,
                    step_index - 1,
                    Some("insufficient deadline budget for one jaw action and mandatory pressure/D405 observations"),
                );
            }
            let assessment = grasp_feedback::assess_closure(
                &current,
                &self.config.tactile_calibration,
                runtime::unix_time_ns()?,
            )?;
            match assessment.decision {
                TactileDecision::Hold => {
                    return self.finish("contact", self.config.go, step_index - 1, None)
                }
                TactileDecision::CorrectPadImbalance => {
                    return self.finish(
                        "imbalanced",
                        self.config.go,
                        step_index - 1,
                        Some("one-sided or imbalanced contact requires a pose correction"),
                    )
                }
                TactileDecision::CloseIncrement | TactileDecision::ReleaseIncrement => {}
                TactileDecision::Retained | TactileDecision::Slipping => {
                    return Err("retention decision is invalid during closure".into())
                }
            }

            ensure_d405_fresh(d405_sample.received_at_ns, runtime::unix_time_ns()?)?;

            let envelope = make_envelope(
                &self.config.side,
                &current,
                &assessment,
                (&self.config.d405_target, &d405_sample),
                &progress,
                runtime::unix_time_ns()?,
                self.config.tactile_calibration.max_age_ms,
            )?;
            let proposal_path = self
                .session_dir
                .join(format!("step-{step_index:02}-grip-envelope.json"));
            create_new_json(&proposal_path, &envelope)?;
            let adapter = match self.invoke_grip_adapter(
                &proposal_path,
                &current.sample_id,
                envelope["target_close_fraction"]
                    .as_f64()
                    .ok_or_else(|| "grip envelope target is not numeric".to_string())?,
                envelope["direction"]
                    .as_str()
                    .ok_or_else(|| "grip envelope direction is not a string".to_string())?,
            ) {
                Ok(report) => report,
                Err(error) if self.config.go => {
                    return self.recover_grip_adapter_failure(step_index, &error)
                }
                Err(error) => return Err(error),
            };
            self.record(serde_json::json!({
                "event": "grip_increment_completed",
                "at_ns": runtime::unix_time_ns()?,
                "step_index": step_index,
                "assessment_before": assessment,
                "envelope_path": proposal_path,
                "adapter": adapter
            }))?;
            if !self.config.go {
                return self.finish("dry_run_ready", false, 0, None);
            }

            progress.close_fraction = adapter.target_close_fraction;
            progress.expected_position_mm = adapter.after.position_mm;
            let after = match self.capture_tactile() {
                Ok(observation) => observation,
                Err(error) => {
                    self.record(serde_json::json!({
                        "event": "post_increment_tactile_failed",
                        "at_ns": runtime::unix_time_ns()?,
                        "step_index": step_index,
                        "error": error
                    }))?;
                    return self.finish(
                        "stopped",
                        true,
                        step_index,
                        Some("jaw increment completed but the mandatory pressure re-observation failed"),
                    );
                }
            };
            let after_assessment = grasp_feedback::assess_closure(
                &after,
                &self.config.tactile_calibration,
                runtime::unix_time_ns()?,
            )?;
            self.record(serde_json::json!({
                "event": "post_increment_tactile_observed",
                "at_ns": runtime::unix_time_ns()?,
                "step_index": step_index,
                "tactile": after,
                "assessment": after_assessment
            }))?;
            current = after;

            if assessment.decision == TactileDecision::ReleaseIncrement {
                if after_assessment.overpressure {
                    return self.finish(
                        "overpressure",
                        true,
                        step_index,
                        Some("pressure remains above the ceiling after the only allowed release increment"),
                    );
                }
                return self.finish(
                    "pressure_released",
                    true,
                    step_index,
                    Some("one release increment completed after the pressure ceiling"),
                );
            }
            if after_assessment.decision == TactileDecision::CloseIncrement {
                let near_field = match self.capture_d405() {
                    Ok(observation) => observation,
                    Err(error) => {
                        self.record(serde_json::json!({
                            "event": "post_increment_d405_failed",
                            "at_ns": runtime::unix_time_ns()?,
                            "step_index": step_index,
                            "error": error
                        }))?;
                        return self.finish(
                            "stopped",
                            true,
                            step_index,
                            Some("pressure permits another close, but the mandatory D405 re-observation failed"),
                        );
                    }
                };
                let aligned =
                    visual_servo::target_reached(&self.config.d405_target, &near_field.sample)?;
                self.record(serde_json::json!({
                    "event": "post_increment_d405_observed",
                    "at_ns": runtime::unix_time_ns()?,
                    "step_index": step_index,
                    "aligned": aligned,
                    "sample": near_field
                }))?;
                if !aligned {
                    return self.finish(
                        "target_moved",
                        true,
                        step_index,
                        Some("D405 target left the calibrated grasp tolerance after the jaw increment"),
                    );
                }
                d405_sample = near_field.sample.clone();
                current = match self.capture_tactile() {
                    Ok(observation) => observation,
                    Err(error) => {
                        self.record(serde_json::json!({
                            "event": "pre_increment_tactile_refresh_failed",
                            "at_ns": runtime::unix_time_ns()?,
                            "step_index": step_index + 1,
                            "error": error
                        }))?;
                        return self.finish(
                            "stopped",
                            true,
                            step_index,
                            Some("D405 remains aligned, but pressure could not be refreshed before another close"),
                        );
                    }
                };
                let refreshed_assessment = grasp_feedback::assess_closure(
                    &current,
                    &self.config.tactile_calibration,
                    runtime::unix_time_ns()?,
                )?;
                self.record(serde_json::json!({
                    "event": "pre_increment_tactile_refreshed",
                    "at_ns": runtime::unix_time_ns()?,
                    "step_index": step_index + 1,
                    "tactile": current,
                    "assessment": refreshed_assessment
                }))?;
            }
        }
        self.finish(
            "max_steps",
            self.config.go,
            self.config.max_steps,
            Some("maximum jaw increments reached without balanced two-pad contact"),
        )
    }

    fn capture_d405(&self) -> Result<perception::NearFieldSignalObservation, String> {
        let output_root = self.runtime.data_root().join("sensors/d405");
        let root = output_root.to_string_lossy().into_owned();
        let output = self
            .runtime
            .run_python_capture("d405_observe.py", &["--output-root", root.as_str()])?;
        let report = parse_last_json(&output, "D405 adapter")?;
        require_ok(&report, "D405 adapter")?;
        let latest_path = PathBuf::from(json_string(&report, "latest_path")?);
        perception::observe_near_field_signal(&latest_path)
    }

    fn capture_tactile(&self) -> Result<TactileObservation, String> {
        let config = self.config.tactile_config.to_string_lossy().into_owned();
        let output_root = self.runtime.data_root().join("sensors/tactile");
        let root = output_root.to_string_lossy().into_owned();
        let output = self.runtime.run_python_capture(
            "tactile_adapter.py",
            &["--config", config.as_str(), "--output-root", root.as_str()],
        )?;
        let report = parse_last_json(&output, "tactile adapter")?;
        require_ok(&report, "tactile adapter")?;
        serde_json::from_value(report)
            .map_err(|error| format!("invalid tactile observation: {error}"))
    }

    fn observe_gripper(&self) -> Result<GripperState, String> {
        let output = self.runtime.run_python_capture(
            "grip_adapter.py",
            &["observe", "--side", self.config.side.as_str()],
        )?;
        let report = parse_last_json(&output, "gripper adapter")?;
        require_ok(&report, "gripper adapter")?;
        let observation: GripperObservation = serde_json::from_value(report)
            .map_err(|error| format!("invalid gripper observation: {error}"))?;
        if observation.mode != "gripper_observation" || observation.schema_version != 1 {
            return Err("unsupported gripper observation contract".into());
        }
        Ok(observation.state)
    }

    fn invoke_grip_adapter(
        &self,
        proposal_path: &Path,
        tactile_sample_id: &str,
        target_close_fraction: f64,
        direction: &str,
    ) -> Result<GripAdapterReport, String> {
        let proposal = proposal_path.to_string_lossy().into_owned();
        let mut args = vec!["execute", "--proposal", proposal.as_str()];
        if self.config.go {
            args.push("--go");
        }
        let output = self.runtime.run_python_capture("grip_adapter.py", &args)?;
        let report = parse_last_json(&output, "grip adapter")?;
        require_ok(&report, "grip adapter")?;
        let report: GripAdapterReport = serde_json::from_value(report)
            .map_err(|error| format!("invalid grip adapter report: {error}"))?;
        if report.schema_version != 1
            || report.mode != "tactile_grip_increment"
            || report.executed != self.config.go
            || report.requires_reobservation != self.config.go
            || report.side != self.config.side
            || report.tactile_sample_id != tactile_sample_id
            || report.direction != direction
            || (report.target_close_fraction - target_close_fraction).abs() > 1e-12
        {
            return Err("grip adapter result does not match the requested transaction".into());
        }
        Ok(report)
    }

    fn recover_grip_adapter_failure(&self, step_index: usize, error: &str) -> Result<(), String> {
        let recovery = match self.capture_tactile() {
            Ok(observation) => serde_json::json!({
                "ok": true,
                "observation": observation,
                "assessment": grasp_feedback::assess_closure(
                    &observation,
                    &self.config.tactile_calibration,
                    runtime::unix_time_ns()?
                ).ok()
            }),
            Err(recovery_error) => serde_json::json!({
                "ok": false,
                "error": recovery_error
            }),
        };
        self.record(serde_json::json!({
            "event": "grip_adapter_failed_after_authorization",
            "at_ns": runtime::unix_time_ns()?,
            "step_index": step_index,
            "error": error,
            "recovery_tactile": recovery
        }))?;
        self.finish(
            "stopped",
            true,
            step_index,
            Some("gripper adapter failed; a mandatory recovery pressure observation was attempted"),
        )
    }

    fn record(&self, value: serde_json::Value) -> Result<(), String> {
        let mut file = OpenOptions::new()
            .create(true)
            .append(true)
            .open(&self.events_path)
            .map_err(|error| format!("{}: {error}", self.events_path.display()))?;
        serde_json::to_writer(&mut file, &value).map_err(|error| error.to_string())?;
        file.write_all(b"\n").map_err(|error| error.to_string())?;
        file.flush().map_err(|error| error.to_string())?;
        file.sync_data().map_err(|error| error.to_string())
    }

    fn finish(
        &self,
        status: &str,
        executed: bool,
        completed_steps: usize,
        reason: Option<&str>,
    ) -> Result<(), String> {
        let success = matches!(status, "contact" | "dry_run_ready" | "pressure_released");
        let summary = serde_json::json!({
            "ok": success,
            "schema_version": 1,
            "mode": "d405_tactile_grasp_loop",
            "status": status,
            "executed": executed,
            "completed_steps": completed_steps,
            "session_id": self.session_id,
            "events_path": self.events_path,
            "reason": reason,
            "services_restarted": false,
            "lift_authorized": false
        });
        self.record(serde_json::json!({
            "event": "grasp_loop_finished",
            "at_ns": runtime::unix_time_ns()?,
            "summary": summary
        }))?;
        println!(
            "{}",
            serde_json::to_string(&summary).map_err(|error| error.to_string())?
        );
        if success {
            Ok(())
        } else {
            Err(format!("grasp loop ended with status {status}"))
        }
    }
}

#[derive(Clone, Copy)]
struct GripProgress {
    close_fraction: f64,
    expected_position_mm: u32,
}

fn make_envelope(
    side: &str,
    tactile: &TactileObservation,
    assessment: &TactileAssessment,
    d405_alignment: (&ServoTarget, &perception::ServoSignalSample),
    progress: &GripProgress,
    generated_at_ns: u64,
    max_tactile_age_ms: f64,
) -> Result<serde_json::Value, String> {
    let (d405_target, d405_sample) = d405_alignment;
    let (direction, delta) = match assessment.decision {
        TactileDecision::CloseIncrement => ("close", CLOSE_INCREMENT),
        TactileDecision::ReleaseIncrement => ("release", -CLOSE_INCREMENT),
        _ => return Err("tactile assessment does not authorize a jaw increment".into()),
    };
    let target = (progress.close_fraction + delta).clamp(0.0, 1.0);
    if (target - progress.close_fraction).abs() < 1e-12 {
        return Err(format!(
            "cannot move gripper farther in {direction} direction"
        ));
    }
    let tactile_valid_ns = (max_tactile_age_ms * 1_000_000.0) as u64;
    let expires_at_ns = tactile
        .sensor_stamp_ns
        .checked_add(tactile_valid_ns)
        .ok_or_else(|| "tactile expiry timestamp overflowed".to_string())?;
    if expires_at_ns < generated_at_ns {
        return Err("tactile sample expired before the grip envelope was generated".into());
    }
    Ok(serde_json::json!({
        "ok": true,
        "schema_version": 2,
        "mode": "tactile_grip_increment",
        "generated_at_ns": generated_at_ns,
        "expires_at_ns": expires_at_ns,
        "side": side,
        "tactile_sample_id": tactile.sample_id,
        "expected_position_mm": progress.expected_position_mm,
        "previous_close_fraction": progress.close_fraction,
        "target_close_fraction": target,
        "direction": direction,
        "tactile_decision": assessment.decision,
        "d405_alignment": {
            "target": d405_target,
            "sample": d405_sample
        }
    }))
}

#[derive(Deserialize)]
struct GripperObservation {
    schema_version: u32,
    mode: String,
    #[serde(flatten)]
    state: GripperState,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
struct GripperState {
    position_mm: u32,
}

#[derive(Deserialize, Serialize)]
struct GripAdapterReport {
    schema_version: u32,
    mode: String,
    executed: bool,
    side: String,
    tactile_sample_id: String,
    direction: String,
    target_close_fraction: f64,
    after: GripperState,
    requires_reobservation: bool,
}

/// This loop's label for unknown-argument errors lives in exactly one place.
fn validate_command_args(args: &[String], options: &[&str], flags: &[&str]) -> Result<(), String> {
    crate::support::args::validate_command_args(Some("grasp-loop"), args, options, flags)
}

fn ensure_d405_fresh(received_at_ns: u64, now_ns: u64) -> Result<(), String> {
    if now_ns < received_at_ns {
        return Err("D405 observation timestamp is in the future".into());
    }
    let age_ms = (now_ns - received_at_ns) as f64 / 1_000_000.0;
    if age_ms > MAX_D405_AUTHORIZATION_AGE_MS {
        return Err(format!(
            "D405 authorization is stale: {age_ms:.3}ms > {MAX_D405_AUTHORIZATION_AGE_MS:.0}ms"
        ));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::grasp_feedback::TactilePadSample;

    fn observation() -> TactileObservation {
        TactileObservation {
            schema_version: 1,
            sample_id: "sample-1".into(),
            sensor_stamp_ns: 99,
            received_at_ns: 100,
            pads: vec![
                TactilePadSample {
                    id: "fixed".into(),
                    raw: 1.0,
                    median_abs_deviation: 0.0,
                    sample_count: 5,
                },
                TactilePadSample {
                    id: "moving".into(),
                    raw: 1.0,
                    median_abs_deviation: 0.0,
                    sample_count: 5,
                },
            ],
        }
    }

    fn assessment(decision: TactileDecision) -> TactileAssessment {
        TactileAssessment {
            ok: true,
            sample_id: "sample-1".into(),
            age_ms: 1.0,
            pad_delta: [0.0, 0.0],
            contact: [false, false],
            balanced: false,
            overpressure: false,
            decision,
            reason: "test".into(),
        }
    }

    fn d405_target() -> ServoTarget {
        ServoTarget {
            schema_version: 1,
            source_frame_id: "d405-target".into(),
            signal: [100.0, 200.0, 0.2],
            tolerance: [5.0, 5.0, 0.01],
        }
    }

    fn d405_sample() -> perception::ServoSignalSample {
        perception::ServoSignalSample {
            schema_version: 1,
            frame_id: "d405-current".into(),
            received_at_ns: 120,
            joints_rad: Default::default(),
            signal: [101.0, 198.0, 0.201],
        }
    }

    #[test]
    fn no_contact_authorizes_exactly_one_close_increment() {
        let envelope = make_envelope(
            "right",
            &observation(),
            &assessment(TactileDecision::CloseIncrement),
            (&d405_target(), &d405_sample()),
            &GripProgress {
                close_fraction: 0.20,
                expected_position_mm: 672,
            },
            123,
            500.0,
        )
        .unwrap();
        assert_eq!(envelope["target_close_fraction"], 0.25);
        assert_eq!(envelope["tactile_sample_id"], "sample-1");
        assert_eq!(
            envelope["d405_alignment"]["sample"]["frame_id"],
            "d405-current"
        );
    }

    #[test]
    fn pressure_ceiling_authorizes_only_one_release_increment() {
        let envelope = make_envelope(
            "right",
            &observation(),
            &assessment(TactileDecision::ReleaseIncrement),
            (&d405_target(), &d405_sample()),
            &GripProgress {
                close_fraction: 0.50,
                expected_position_mm: 420,
            },
            123,
            500.0,
        )
        .unwrap();
        assert_eq!(envelope["target_close_fraction"], 0.45);
        assert_eq!(envelope["direction"], "release");
    }

    #[test]
    fn hold_and_imbalance_cannot_create_motion_envelopes() {
        for decision in [TactileDecision::Hold, TactileDecision::CorrectPadImbalance] {
            assert!(make_envelope(
                "right",
                &observation(),
                &assessment(decision),
                (&d405_target(), &d405_sample()),
                &GripProgress {
                    close_fraction: 0.50,
                    expected_position_mm: 420,
                },
                123,
                500.0,
            )
            .is_err());
        }
    }

    #[test]
    fn d405_authorization_expires_before_an_old_frame_can_close() {
        assert!(ensure_d405_fresh(1_000_000_000, 3_999_999_999).is_ok());
        assert!(ensure_d405_fresh(1_000_000_000, 4_000_000_001).is_err());
        assert!(ensure_d405_fresh(2_000_000_000, 1_000_000_000).is_err());
    }
}
