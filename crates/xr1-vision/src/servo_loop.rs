use crate::hardware;
use crate::kinematics::Chain;
use crate::observation::{self, JointState};
use crate::perception;
use crate::perception::ServoSignalSample;
use crate::runtime::{self, RuntimePaths};
use crate::safety;
use crate::support::adapter::{json_string, parse_last_json};
use crate::support::args::{f64_option, flag, option, optional_option, usize_option};
use crate::support::evidence::{append_json_line, write_json};
use crate::support::runlock::RobotActionLoopLock;
use crate::visual_servo;
use std::collections::HashMap;
use std::fs;
use std::path::PathBuf;
use std::time::{Duration, Instant};

const RUN_ID: &str = "visual-servo-closed-loop";
// The historical camera loop stopped at 9.1 px (about 4.9 mm at the table).
const DEFAULT_TOLERANCE_PX: f64 = 9.0;
// ZED depth was independently validated to 2.0 mm at the teleoperated grasp;
// 5 mm keeps the acceptance band above that measured residual.
const DEFAULT_DEPTH_TOLERANCE_M: f64 = 0.005;
// Reuse the measured-plan start-drift gate already enforced by motion_adapter.py.
const MAX_CALIBRATION_START_DRIFT_RAD: f64 = 0.035;

pub fn run(runtime: &RuntimePaths, args: Vec<String>) -> Result<(), String> {
    let config = ServoLoopConfig::parse(&args)?;
    ServoLoopSession::start(runtime, config)?.execute()
}

struct ServoLoopConfig {
    calibration: visual_servo::JacobianCalibration,
    run_id: String,
    max_steps: usize,
    timeout_s: f64,
    capture_timeout_s: f64,
    tolerance_px: f64,
    depth_tolerance_m: f64,
    go: bool,
    requirements: visual_servo::SensorRequirements,
    observer: LoopObserver,
}

enum LoopObserver {
    Zed,
    D405(visual_servo::ServoTarget),
}

impl ServoLoopConfig {
    fn parse(args: &[String]) -> Result<Self, String> {
        validate_command_args(
            args,
            &[
                "--calibration",
                "--run-id",
                "--max-steps",
                "--timeout",
                "--capture-timeout",
                "--tolerance-px",
                "--depth-tolerance-m",
                "--d405-target",
            ],
            &[
                "--go",
                "--require-d405",
                "--require-tactile",
                "--require-force-feedback",
            ],
        )?;
        let calibration_path = option(args, "--calibration")?;
        let calibration = serde_json::from_slice(
            &fs::read(&calibration_path).map_err(|error| format!("{calibration_path}: {error}"))?,
        )
        .map_err(|error| format!("invalid visual-servo calibration: {error}"))?;
        let run_id = optional_option(args, "--run-id")?.unwrap_or_else(|| RUN_ID.to_string());
        validate_run_id(&run_id)?;
        let max_steps = usize_option(args, "--max-steps", 6, 1, 12)?;
        let timeout_s = f64_option(args, "--timeout", 180.0, 10.0, 600.0)?;
        let capture_timeout_s = f64_option(args, "--capture-timeout", 20.0, 1.0, 60.0)?;
        if timeout_s < capture_timeout_s + 10.0 {
            return Err(
                "--timeout must leave the capture timeout plus 10s for DDS discovery and one bounded adapter action"
                    .into(),
            );
        }
        let d405_target = optional_option(args, "--d405-target")?
            .map(|path| {
                serde_json::from_slice::<visual_servo::ServoTarget>(
                    &fs::read(&path).map_err(|error| format!("{path}: {error}"))?,
                )
                .map_err(|error| format!("invalid D405 target: {error}"))
            })
            .transpose()?;
        if d405_target.is_some()
            && args.iter().any(|argument| {
                matches!(argument.as_str(), "--tolerance-px" | "--depth-tolerance-m")
            })
        {
            return Err(
                "D405 tolerance comes from --d405-target; do not also supply loop tolerances"
                    .into(),
            );
        }
        let mut requirements = visual_servo::SensorRequirements {
            d405: flag(args, "--require-d405"),
            tactile: flag(args, "--require-tactile"),
            force_feedback: flag(args, "--require-force-feedback"),
        };
        let observer = if let Some(target) = d405_target {
            validate_d405_artifacts(&calibration, &target)?;
            requirements.d405 = true;
            LoopObserver::D405(target)
        } else {
            LoopObserver::Zed
        };
        Ok(Self {
            calibration,
            run_id,
            max_steps,
            timeout_s,
            capture_timeout_s,
            tolerance_px: f64_option(args, "--tolerance-px", DEFAULT_TOLERANCE_PX, 1.0, 50.0)?,
            depth_tolerance_m: f64_option(
                args,
                "--depth-tolerance-m",
                DEFAULT_DEPTH_TOLERANCE_M,
                0.001,
                0.05,
            )?,
            go: flag(args, "--go"),
            requirements,
            observer,
        })
    }
}

struct ServoLoopSession<'a> {
    runtime: &'a RuntimePaths,
    config: ServoLoopConfig,
    session_id: String,
    session_dir: PathBuf,
    events_path: PathBuf,
    deadline: Instant,
    _lock: RobotActionLoopLock,
}

impl<'a> ServoLoopSession<'a> {
    fn start(runtime: &'a RuntimePaths, config: ServoLoopConfig) -> Result<Self, String> {
        let started_ns = now_ns()?;
        let session_id = format!("{started_ns}-{}", std::process::id());
        let lock = RobotActionLoopLock::acquire("visual-servo loop", &session_id)?;
        let session_dir = runtime
            .data_root()
            .join("vista_runs")
            .join(&config.run_id)
            .join("servo_sessions")
            .join(&session_id);
        fs::create_dir_all(&session_dir)
            .map_err(|error| format!("{}: {error}", session_dir.display()))?;
        let events_path = session_dir.join("events.jsonl");
        append_json_line(
            &events_path,
            &serde_json::json!({
                "event": "servo_loop_started",
                "at_ns": started_ns,
                "session_id": session_id,
                "run_id": config.run_id,
                "go": config.go,
                "max_steps": config.max_steps,
                "timeout_s": config.timeout_s,
                "service_policy": "never_start_stop_or_restart_services; use existing ZED publisher; direct D405 capture refuses when owned",
                "requirements": config.requirements,
                "limitations": [
                    "D405 observation is active only with --d405-target and a fresh validated real frame; otherwise the observer is ZED",
                    "tactile pressure is a separate grasp-loop transaction; arm joint force feedback remains unavailable",
                    "self-collision and gripper-body collision remain outside the servo gate"
                ]
            }),
        )?;
        let timeout_s = config.timeout_s;
        Ok(Self {
            runtime,
            config,
            session_id,
            session_dir,
            events_path,
            deadline: Instant::now() + Duration::from_secs_f64(timeout_s),
            _lock: lock,
        })
    }

    fn execute(&self) -> Result<(), String> {
        let mut current = self.capture_observation(0)?;
        let target = self.pin_target(&current)?;
        if visual_servo::target_reached(&target, &current.sample)? {
            return self.finish_initial_alignment(&current);
        }

        let mut improvement_ratios = Vec::new();
        let mut executed_any = false;
        for step_index in 1..=self.config.max_steps {
            if !self.has_step_budget() {
                return self.finish(
                    "timeout",
                    executed_any,
                    step_index - 1,
                    &current.sample.frame_id,
                    Some(
                        "insufficient deadline budget for one action and its mandatory re-observation",
                    ),
                );
            }

            let step = self.prepare_step(step_index, &target, &current)?;
            if !step.safety.approved {
                self.record(serde_json::json!({
                    "event": "servo_step_refused",
                    "at_ns": now_ns()?,
                    "step_index": step_index,
                    "proposal_path": step.path,
                    "safety": step.safety
                }))?;
                return self.finish(
                    "refused",
                    executed_any,
                    step_index - 1,
                    &current.sample.frame_id,
                    Some("deterministic safety or required-sensor gate refused the next microstep"),
                );
            }

            let adapter_report = match self.invoke_adapter(&step) {
                Ok(report) => report,
                Err(error) if self.config.go => {
                    return self.recover_adapter_failure(
                        step_index,
                        &target,
                        &current,
                        &step.proposal,
                        &improvement_ratios,
                        &error,
                    );
                }
                Err(error) => return Err(error),
            };
            if !self.config.go {
                self.record(serde_json::json!({
                    "event": "servo_loop_dry_run_completed",
                    "at_ns": now_ns()?,
                    "step_index": step_index,
                    "adapter": adapter_report
                }))?;
                return self.finish("dry_run_ready", false, 0, &current.sample.frame_id, None);
            }
            executed_any = true;

            let after = self.capture_observation(step_index)?;
            let reconciliation = self.reconcile_step(
                &target,
                &current,
                &after,
                &step,
                &adapter_report,
                &improvement_ratios,
            )?;
            improvement_ratios.push(reconciliation.improvement_ratio);
            current = after;

            if adapter_report.get("motion_completed") != Some(&serde_json::Value::Bool(true)) {
                return self.finish(
                    "stopped",
                    true,
                    step_index,
                    &current.sample.frame_id,
                    Some("joint endpoint missed the approved microstep by more than 0.01rad"),
                );
            }
            if reconciliation.converged {
                return self.finish(
                    "converged",
                    true,
                    step_index,
                    &current.sample.frame_id,
                    reconciliation.stop_reason.as_deref(),
                );
            }
            if !reconciliation.continue_servo {
                return self.finish(
                    "stopped",
                    true,
                    step_index,
                    &current.sample.frame_id,
                    reconciliation.stop_reason.as_deref(),
                );
            }
        }

        self.finish(
            "max_steps",
            executed_any,
            self.config.max_steps,
            &current.sample.frame_id,
            Some("maximum microstep count reached without convergence"),
        )
    }

    fn pin_target(&self, current: &LoopObservation) -> Result<visual_servo::ServoTarget, String> {
        let calibration_drift =
            visual_servo::calibration_pose_drift(&self.config.calibration, &current.sample)?;
        if calibration_drift > MAX_CALIBRATION_START_DRIFT_RAD {
            return Err(format!(
                "calibration start drift {calibration_drift:.4}rad exceeds {MAX_CALIBRATION_START_DRIFT_RAD:.3}rad; return to the measured centre pose or re-calibrate"
            ));
        }
        let target = match &self.config.observer {
            LoopObserver::Zed => visual_servo::ServoTarget {
                schema_version: 1,
                source_frame_id: current.sample.frame_id.clone(),
                signal: current.target_signal,
                tolerance: [
                    self.config.tolerance_px,
                    self.config.tolerance_px,
                    self.config.depth_tolerance_m,
                ],
            },
            LoopObserver::D405(target) => target.clone(),
        };
        self.record(serde_json::json!({
            "event": "servo_target_pinned",
            "at_ns": now_ns()?,
            "target": target,
            "calibration_start_drift_rad": calibration_drift,
            "observation": current.report
        }))?;
        Ok(target)
    }

    fn finish_initial_alignment(&self, current: &LoopObservation) -> Result<(), String> {
        let sensors = hardware::inspect()?;
        if !safety::sensor_requirements_met(&self.config.requirements, &sensors) {
            return self.finish(
                "refused",
                false,
                0,
                &current.sample.frame_id,
                Some("target is aligned, but a required sensor capability is not healthy"),
            );
        }
        self.finish("converged", false, 0, &current.sample.frame_id, None)
    }

    fn prepare_step(
        &self,
        step_index: usize,
        target: &visual_servo::ServoTarget,
        current: &LoopObservation,
    ) -> Result<PreparedStep, String> {
        let request = visual_servo::CalibratedServoRequest {
            schema_version: 1,
            calibration: self.config.calibration.clone(),
            target: target.clone(),
            current: current.sample.clone(),
            damping: 0.5,
            requires: self.config.requirements.clone(),
        };
        let input = visual_servo::input_from_request(&request)?;
        let proposal = visual_servo::propose(&input, safety::MAX_SERVO_STEP_RAD)?;
        let chain = Chain::from_urdf(
            self.runtime.arm_urdf(),
            visual_servo::controlled_arm_tip(&input.controlled_joints)?,
        )?;
        let generated_at_ns = now_ns()?;
        let safety_report = safety::evaluate_servo_live(
            &chain,
            &current.sample.frame_id,
            current.sample.received_at_ns,
            &current.joint_state,
            &input.observation_frame_id,
            &proposal,
            &self.config.requirements,
            &hardware::inspect()?,
            generated_at_ns,
        )?;
        let path = self
            .session_dir
            .join(format!("step-{step_index:02}-proposal.json"));
        write_json(
            &path,
            &serde_json::json!({
                "ok": true,
                "schema_version": 1,
                "mode": "visual_servo_proposal",
                "generated_at_ns": generated_at_ns,
                "observation_frame_id": input.observation_frame_id,
                "context": {
                    "session_id": self.session_id,
                    "step_index": step_index,
                    "calibration_reference_frame_id": self.config.calibration.reference_frame_id,
                    "target": target,
                    "before": current.sample
                },
                "proposal": proposal,
                "safety": safety_report,
                "ready_for_execution_adapter": safety_report.approved,
                "execution_authorized": false,
                "service_policy": "never_start_stop_or_restart_services; direct D405 capture refuses when owned"
            }),
        )?;
        Ok(PreparedStep {
            index: step_index,
            observation_frame_id: input.observation_frame_id,
            proposal,
            path,
            safety: safety_report,
        })
    }

    fn invoke_adapter(&self, step: &PreparedStep) -> Result<serde_json::Value, String> {
        let proposal_path = step.path.to_string_lossy().into_owned();
        let mut adapter_args = vec!["--proposal", proposal_path.as_str()];
        if self.config.go {
            adapter_args.push("--go");
        }
        self.runtime
            .run_python_capture("servo_adapter.py", &adapter_args)
            .and_then(|output| parse_last_json(&output, "servo adapter"))
            .and_then(|report| {
                validate_adapter_report(report, self.config.go, &step.observation_frame_id)
            })
    }

    fn reconcile_step(
        &self,
        target: &visual_servo::ServoTarget,
        before: &LoopObservation,
        after: &LoopObservation,
        step: &PreparedStep,
        adapter_report: &serde_json::Value,
        prior_improvement_ratios: &[f64],
    ) -> Result<visual_servo::ServoReconciliation, String> {
        let reconciliation = visual_servo::reconcile(&visual_servo::ReconciliationInput {
            schema_version: 1,
            target: target.clone(),
            before: before.sample.clone(),
            after: after.sample.clone(),
            proposal: step.proposal.clone(),
            prior_improvement_ratios: prior_improvement_ratios.to_vec(),
        })?;
        self.record(serde_json::json!({
            "event": "servo_step_reconciled",
            "at_ns": now_ns()?,
            "step_index": step.index,
            "proposal_path": step.path,
            "adapter": adapter_report,
            "after_observation": after.report,
            "reconciliation": reconciliation
        }))?;
        Ok(reconciliation)
    }

    fn recover_adapter_failure(
        &self,
        step_index: usize,
        target: &visual_servo::ServoTarget,
        before: &LoopObservation,
        proposal: &visual_servo::ServoProposal,
        prior_improvement_ratios: &[f64],
        error: &str,
    ) -> Result<(), String> {
        let after = self.capture_observation(step_index)?;
        let recovery_reconciliation = visual_servo::reconcile(&visual_servo::ReconciliationInput {
            schema_version: 1,
            target: target.clone(),
            before: before.sample.clone(),
            after: after.sample.clone(),
            proposal: proposal.clone(),
            prior_improvement_ratios: prior_improvement_ratios.to_vec(),
        })
        .ok();
        self.record(serde_json::json!({
            "event": "servo_adapter_failed_after_authorization",
            "at_ns": now_ns()?,
            "step_index": step_index,
            "error": error,
            "recovery_observation": after.report,
            "recovery_reconciliation": recovery_reconciliation
        }))?;
        self.finish(
            "stopped",
            true,
            step_index,
            &after.sample.frame_id,
            Some("hardware adapter failed; a mandatory recovery observation was captured"),
        )
    }

    fn capture_observation(&self, observation_index: usize) -> Result<LoopObservation, String> {
        let timeout = format!("{:.3}", self.config.capture_timeout_s);
        let (report, sample, target_signal, joint_state) = match &self.config.observer {
            LoopObserver::Zed => {
                let output = self.runtime.run_python_capture(
                    "vista_observe.py",
                    &[
                        "--run-id",
                        &self.config.run_id,
                        "--timeout",
                        timeout.as_str(),
                    ],
                )?;
                let report = parse_last_json(&output, "VISTA observation")?;
                validate_observation_report(&report, "VISTA")?;
                let state_path = PathBuf::from(json_string(&report, "state_path")?);
                let frame_id = json_string(&report, "frame_id")?;
                let latest_path = self.session_dir.join(format!(
                    "observation-{observation_index:02}-{frame_id}.latest.json"
                ));
                write_json(&latest_path, &report)?;
                let chain = Chain::from_urdf(self.runtime.arm_urdf(), "right_tcp_link")?;
                let signal = perception::observe_servo_signal(&latest_path, &chain)?;
                let state = observation::read_state(&state_path)?;
                (
                    report,
                    signal.sample,
                    signal.observed_target_signal,
                    state.joint_state,
                )
            }
            LoopObserver::D405(target) => {
                let output_root = self.runtime.data_root().join("sensors/d405");
                let root = output_root.to_string_lossy().into_owned();
                let output = self.runtime.run_python_capture(
                    "d405_observe.py",
                    &[
                        "--timeout",
                        timeout.as_str(),
                        "--output-root",
                        root.as_str(),
                    ],
                )?;
                let report = parse_last_json(&output, "D405 observation")?;
                validate_observation_report(&report, "D405")?;
                let latest_path = PathBuf::from(json_string(&report, "latest_path")?);
                let signal = perception::observe_near_field_signal(&latest_path)?;
                let joint_state = JointState {
                    received_at_ns: signal.joint_received_at_ns,
                    positions_rad: signal
                        .sample
                        .joints_rad
                        .iter()
                        .map(|(name, value)| (name.clone(), Some(*value)))
                        .collect::<HashMap<_, _>>(),
                };
                (report, signal.sample, target.signal, joint_state)
            }
        };
        let frame_id = sample.frame_id.as_str();
        let latest_path = self.session_dir.join(format!(
            "observation-{observation_index:02}-{frame_id}.latest.json"
        ));
        write_json(&latest_path, &report)?;
        Ok(LoopObservation {
            sample,
            target_signal,
            joint_state,
            report,
        })
    }

    fn has_step_budget(&self) -> bool {
        self.deadline
            .saturating_duration_since(Instant::now())
            .as_secs_f64()
            >= self.config.capture_timeout_s + 10.0
    }

    fn record(&self, value: serde_json::Value) -> Result<(), String> {
        append_json_line(&self.events_path, &value)
    }

    fn finish(
        &self,
        status: &str,
        executed: bool,
        completed_steps: usize,
        frame_id: &str,
        reason: Option<&str>,
    ) -> Result<(), String> {
        let success = matches!(status, "converged" | "dry_run_ready");
        let summary = serde_json::json!({
            "ok": success,
            "schema_version": 1,
            "mode": "bounded_visual_servo_loop",
            "status": status,
            "executed": executed,
            "completed_steps": completed_steps,
            "current_frame_id": frame_id,
            "session_id": self.session_id,
            "events_path": self.events_path,
            "reason": reason,
            "services_restarted": false,
            "suggested_next_action": if status == "converged" {
                "close the gripper as a separate action, then verify static obstruction and lift retention"
            } else {
                "inspect the recorded reconciliation and sensor gates before another action"
            }
        });
        self.record(serde_json::json!({
            "event": "servo_loop_finished",
            "at_ns": now_ns()?,
            "summary": summary
        }))?;
        println!(
            "{}",
            serde_json::to_string(&summary).map_err(|error| error.to_string())?
        );
        if success {
            Ok(())
        } else {
            Err(format!("visual-servo loop ended with status {status}"))
        }
    }
}

struct PreparedStep {
    index: usize,
    observation_frame_id: String,
    proposal: visual_servo::ServoProposal,
    path: PathBuf,
    safety: safety::ServoSafetyReport,
}

struct LoopObservation {
    sample: ServoSignalSample,
    target_signal: [f64; 3],
    joint_state: JointState,
    report: serde_json::Value,
}


fn validate_adapter_report(
    report: serde_json::Value,
    go: bool,
    expected_frame_id: &str,
) -> Result<serde_json::Value, String> {
    if report.get("schema_version") != Some(&serde_json::Value::from(1))
        || report.get("mode").and_then(serde_json::Value::as_str) != Some("visual_servo_microstep")
    {
        return Err("servo adapter returned an unsupported report schema or mode".into());
    }
    if report.get("executed") != Some(&serde_json::Value::Bool(go)) {
        return Err("servo adapter execution flag does not match the requested mode".into());
    }
    if report
        .get("observation_frame_id")
        .and_then(serde_json::Value::as_str)
        != Some(expected_frame_id)
    {
        return Err("servo adapter report is not bound to the proposal observation frame".into());
    }
    if !report
        .get("motion_completed")
        .is_some_and(serde_json::Value::is_boolean)
    {
        return Err("servo adapter report is missing motion_completed".into());
    }
    if go && report.get("requires_reobservation") != Some(&serde_json::Value::Bool(true)) {
        return Err("executed servo adapter did not require re-observation".into());
    }
    Ok(report)
}


fn validate_observation_report(report: &serde_json::Value, source: &str) -> Result<(), String> {
    if report.get("ok") != Some(&serde_json::Value::Bool(true)) {
        return Err(format!("{source} observation failed: {report}"));
    }
    Ok(())
}

fn validate_run_id(run_id: &str) -> Result<(), String> {
    let mut characters = run_id.chars();
    let valid_first = characters
        .next()
        .is_some_and(|value| value.is_ascii_alphanumeric());
    let valid_rest =
        characters.all(|value| value.is_ascii_alphanumeric() || matches!(value, '.' | '_' | '-'));
    if !valid_first || !valid_rest || run_id.len() > 64 {
        return Err("--run-id must match [A-Za-z0-9][A-Za-z0-9._-]{0,63}".into());
    }
    Ok(())
}

fn validate_d405_artifacts(
    calibration: &visual_servo::JacobianCalibration,
    target: &visual_servo::ServoTarget,
) -> Result<(), String> {
    let frame_is_d405 = |frame: &str| frame.starts_with("d405-");
    if !frame_is_d405(&target.source_frame_id)
        || !frame_is_d405(&calibration.reference_frame_id)
        || calibration.sample_frame_ids.is_empty()
        || !calibration
            .sample_frame_ids
            .iter()
            .all(|frame| frame_is_d405(frame))
    {
        return Err(
            "--d405-target requires a Jacobian and target measured from named d405-* frames".into(),
        );
    }
    if !calibration
        .controlled_joints
        .iter()
        .all(|joint| joint.starts_with("right_arm_") && joint.ends_with("_joint"))
    {
        return Err("the right-wrist D405 may control only named right-arm joints".into());
    }
    Ok(())
}


/// This loop's label for unknown-argument errors lives in exactly one place.
fn validate_command_args(args: &[String], options: &[&str], flags: &[&str]) -> Result<(), String> {
    crate::support::args::validate_command_args(Some("servo-loop"), args, options, flags)
}

fn now_ns() -> Result<u64, String> {
    runtime::unix_time_ns()
}


#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::BTreeMap;

    fn d405_artifacts() -> (visual_servo::JacobianCalibration, visual_servo::ServoTarget) {
        (
            visual_servo::JacobianCalibration {
                schema_version: 1,
                reference_frame_id: "d405-center".into(),
                controlled_joints: vec![
                    "right_arm_2_joint".into(),
                    "right_arm_4_joint".into(),
                    "right_arm_6_joint".into(),
                ],
                reference_joints_rad: BTreeMap::new(),
                reference_signal: [424.0, 240.0, 0.10],
                jacobian: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
                perturbation_span_rad: BTreeMap::new(),
                max_background_joint_drift_rad: 0.0,
                sample_frame_ids: vec!["d405-negative".into(), "d405-positive".into()],
            },
            visual_servo::ServoTarget {
                schema_version: 1,
                source_frame_id: "d405-target".into(),
                signal: [424.0, 240.0, 0.08],
                tolerance: [5.0, 5.0, 0.005],
            },
        )
    }

    #[test]
    fn run_id_cannot_escape_the_data_root() {
        assert!(validate_run_id("servo-session_01").is_ok());
        assert!(validate_run_id("../outside").is_err());
        assert!(validate_run_id("").is_err());
    }

    #[test]
    fn unknown_or_value_less_options_are_rejected() {
        assert!(validate_command_args(
            &["--calibration".into(), "c.json".into(), "--go".into()],
            &["--calibration"],
            &["--go"]
        )
        .is_ok());
        assert!(validate_command_args(&["--oops".into()], &[], &[]).is_err());
        assert!(validate_command_args(&["--calibration".into()], &["--calibration"], &[]).is_err());
    }

    #[test]
    fn physical_adapter_report_must_force_reobservation() {
        let report = serde_json::json!({
            "schema_version": 1,
            "mode": "visual_servo_microstep",
            "executed": true,
            "observation_frame_id": "frame-1",
            "motion_completed": true,
            "requires_reobservation": false
        });
        assert!(validate_adapter_report(report, true, "frame-1").is_err());
        let dry_run = serde_json::json!({
            "schema_version": 1,
            "mode": "visual_servo_microstep",
            "executed": false,
            "observation_frame_id": "frame-1",
            "motion_completed": true,
            "requires_reobservation": false
        });
        assert!(validate_adapter_report(dry_run, false, "frame-1").is_ok());
    }

    #[test]
    fn adapter_report_must_match_the_proposal_frame() {
        let report = serde_json::json!({
            "schema_version": 1,
            "mode": "visual_servo_microstep",
            "executed": true,
            "observation_frame_id": "wrong-frame",
            "motion_completed": true,
            "requires_reobservation": true
        });
        assert!(validate_adapter_report(report, true, "frame-1").is_err());
    }

    #[test]
    fn d405_target_refuses_a_zed_jacobian_or_left_arm() {
        let (mut calibration, target) = d405_artifacts();
        assert!(validate_d405_artifacts(&calibration, &target).is_ok());
        calibration.reference_frame_id = "zed-frame".into();
        assert!(validate_d405_artifacts(&calibration, &target).is_err());
        calibration.reference_frame_id = "d405-center".into();
        calibration.controlled_joints[0] = "left_arm_2_joint".into();
        assert!(validate_d405_artifacts(&calibration, &target).is_err());
    }
}
