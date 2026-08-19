use crate::hardware;
use crate::kinematics::Chain;
use crate::observation;
use crate::perception;
use crate::runtime::{self, RuntimePaths};
use crate::safety;
use crate::visual_servo;
use std::fs;
use std::fs::{File, OpenOptions};
use std::io::Write;
use std::path::{Path, PathBuf};
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
            requirements: visual_servo::SensorRequirements {
                d405: flag(args, "--require-d405"),
                tactile: flag(args, "--require-tactile"),
                force_feedback: flag(args, "--require-force-feedback"),
            },
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
    _lock: ServoLoopLock,
}

impl<'a> ServoLoopSession<'a> {
    fn start(runtime: &'a RuntimePaths, config: ServoLoopConfig) -> Result<Self, String> {
        let started_ns = now_ns()?;
        let session_id = format!("{started_ns}-{}", std::process::id());
        let lock = ServoLoopLock::acquire(&session_id)?;
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
                "service_policy": "use_existing_publishers_only_no_start_stop_or_restart",
                "requirements": config.requirements,
                "limitations": [
                    "D405 and tactile data are safety requirements only until their measured stream/protocol adapters are healthy",
                    "force feedback is unavailable; G2 obstruction feedback can verify a grasp only after close plus lift",
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
        if visual_servo::target_reached(&target, &current.signal.sample)? {
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
                    &current.signal.sample.frame_id,
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
                    &current.signal.sample.frame_id,
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
                return self.finish(
                    "dry_run_ready",
                    false,
                    0,
                    &current.signal.sample.frame_id,
                    None,
                );
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
                    &current.signal.sample.frame_id,
                    Some("joint endpoint missed the approved microstep by more than 0.01rad"),
                );
            }
            if reconciliation.converged {
                return self.finish(
                    "converged",
                    true,
                    step_index,
                    &current.signal.sample.frame_id,
                    reconciliation.stop_reason.as_deref(),
                );
            }
            if !reconciliation.continue_servo {
                return self.finish(
                    "stopped",
                    true,
                    step_index,
                    &current.signal.sample.frame_id,
                    reconciliation.stop_reason.as_deref(),
                );
            }
        }

        self.finish(
            "max_steps",
            executed_any,
            self.config.max_steps,
            &current.signal.sample.frame_id,
            Some("maximum microstep count reached without convergence"),
        )
    }

    fn pin_target(&self, current: &LoopObservation) -> Result<visual_servo::ServoTarget, String> {
        let calibration_drift =
            visual_servo::calibration_pose_drift(&self.config.calibration, &current.signal.sample)?;
        if calibration_drift > MAX_CALIBRATION_START_DRIFT_RAD {
            return Err(format!(
                "calibration start drift {calibration_drift:.4}rad exceeds {MAX_CALIBRATION_START_DRIFT_RAD:.3}rad; return to the measured centre pose or re-calibrate"
            ));
        }
        let target = visual_servo::ServoTarget {
            schema_version: 1,
            source_frame_id: current.signal.sample.frame_id.clone(),
            signal: current.signal.observed_target_signal,
            tolerance: [
                self.config.tolerance_px,
                self.config.tolerance_px,
                self.config.depth_tolerance_m,
            ],
        };
        self.record(serde_json::json!({
            "event": "servo_target_pinned",
            "at_ns": now_ns()?,
            "target": target,
            "calibration_start_drift_rad": calibration_drift,
            "observation": current.signal
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
                &current.signal.sample.frame_id,
                Some("target is aligned, but a required sensor capability is not healthy"),
            );
        }
        self.finish("converged", false, 0, &current.signal.sample.frame_id, None)
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
            current: current.signal.sample.clone(),
            damping: 0.5,
            requires: self.config.requirements.clone(),
        };
        let input = visual_servo::input_from_request(&request)?;
        let proposal = visual_servo::propose(&input, safety::MAX_SERVO_STEP_RAD)?;
        let state = observation::read_state(&current.state_path)?;
        let chain = Chain::from_urdf(
            self.runtime.arm_urdf(),
            visual_servo::controlled_arm_tip(&input.controlled_joints)?,
        )?;
        let generated_at_ns = now_ns()?;
        let safety_report = safety::evaluate_servo(
            &chain,
            &state,
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
                    "before": current.signal.sample
                },
                "proposal": proposal,
                "safety": safety_report,
                "ready_for_execution_adapter": safety_report.approved,
                "execution_authorized": false,
                "service_policy": "use_existing_publishers_only_no_start_stop_or_restart"
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
            before: before.signal.sample.clone(),
            after: after.signal.sample.clone(),
            proposal: step.proposal.clone(),
            prior_improvement_ratios: prior_improvement_ratios.to_vec(),
        })?;
        self.record(serde_json::json!({
            "event": "servo_step_reconciled",
            "at_ns": now_ns()?,
            "step_index": step.index,
            "proposal_path": step.path,
            "adapter": adapter_report,
            "after_observation": after.signal,
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
            before: before.signal.sample.clone(),
            after: after.signal.sample.clone(),
            proposal: proposal.clone(),
            prior_improvement_ratios: prior_improvement_ratios.to_vec(),
        })
        .ok();
        self.record(serde_json::json!({
            "event": "servo_adapter_failed_after_authorization",
            "at_ns": now_ns()?,
            "step_index": step_index,
            "error": error,
            "recovery_observation": after.signal,
            "recovery_reconciliation": recovery_reconciliation
        }))?;
        self.finish(
            "stopped",
            true,
            step_index,
            &after.signal.sample.frame_id,
            Some("hardware adapter failed; a mandatory recovery observation was captured"),
        )
    }

    fn capture_observation(&self, observation_index: usize) -> Result<LoopObservation, String> {
        let timeout = format!("{:.3}", self.config.capture_timeout_s);
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
        if report.get("ok") != Some(&serde_json::Value::Bool(true)) {
            return Err(format!("VISTA observation failed: {report}"));
        }
        let frame_id = json_string(&report, "frame_id")?;
        let state_path = PathBuf::from(json_string(&report, "state_path")?);
        let latest_path = self.session_dir.join(format!(
            "observation-{observation_index:02}-{frame_id}.latest.json"
        ));
        write_json(&latest_path, &report)?;
        let chain = Chain::from_urdf(self.runtime.arm_urdf(), "right_tcp_link")?;
        let signal = perception::observe_servo_signal(&latest_path, &chain)?;
        Ok(LoopObservation { state_path, signal })
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
    state_path: PathBuf,
    signal: perception::ServoSignalObservation,
}

struct ServoLoopLock {
    _file: File,
}

impl ServoLoopLock {
    fn acquire(session_id: &str) -> Result<Self, String> {
        let path = std::env::temp_dir().join("xr1-visual-servo-loop.lock");
        let mut file = OpenOptions::new()
            .read(true)
            .write(true)
            .create(true)
            .truncate(false)
            .open(&path)
            .map_err(|error| format!("{}: {error}", path.display()))?;
        if !try_lock_exclusive(&file)? {
            let holder = fs::read_to_string(&path).unwrap_or_else(|_| "unknown".into());
            return Err(format!(
                "another visual-servo loop is active (holder={})",
                holder.trim()
            ));
        }
        file.set_len(0)
            .map_err(|error| format!("{}: {error}", path.display()))?;
        writeln!(file, "pid={} session_id={session_id}", std::process::id())
            .map_err(|error| format!("{}: {error}", path.display()))?;
        file.flush()
            .map_err(|error| format!("{}: {error}", path.display()))?;
        Ok(Self { _file: file })
    }
}

#[cfg(unix)]
fn try_lock_exclusive(file: &File) -> Result<bool, String> {
    use std::os::fd::AsRawFd;

    const LOCK_EX: i32 = 2;
    const LOCK_NB: i32 = 4;
    extern "C" {
        fn flock(fd: i32, operation: i32) -> i32;
    }
    // SAFETY: file owns a valid descriptor for the duration of this call, and
    // flock does not retain the pointer or access Rust-managed memory.
    let result = unsafe { flock(file.as_raw_fd(), LOCK_EX | LOCK_NB) };
    if result == 0 {
        Ok(true)
    } else {
        let error = std::io::Error::last_os_error();
        match error.kind() {
            std::io::ErrorKind::WouldBlock => Ok(false),
            _ => Err(format!("cannot lock visual-servo loop: {error}")),
        }
    }
}

#[cfg(not(unix))]
fn try_lock_exclusive(_file: &File) -> Result<bool, String> {
    Err("visual-servo loop locking is unsupported on this platform".into())
}

fn parse_last_json(output: &str, source: &str) -> Result<serde_json::Value, String> {
    output
        .lines()
        .rev()
        .find_map(|line| serde_json::from_str::<serde_json::Value>(line).ok())
        .ok_or_else(|| format!("{source} produced no JSON report: {}", output.trim()))
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

fn json_string<'a>(value: &'a serde_json::Value, name: &str) -> Result<&'a str, String> {
    value
        .get(name)
        .and_then(serde_json::Value::as_str)
        .ok_or_else(|| format!("JSON report is missing string field {name}"))
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

fn validate_command_args(args: &[String], options: &[&str], flags: &[&str]) -> Result<(), String> {
    let mut index = 0;
    while index < args.len() {
        let argument = args[index].as_str();
        if flags.contains(&argument) {
            index += 1;
        } else if options.contains(&argument) {
            let Some(value) = args.get(index + 1) else {
                return Err(format!("missing value after {argument}"));
            };
            if value.starts_with("--") {
                return Err(format!("missing value after {argument}"));
            }
            index += 2;
        } else {
            return Err(format!("unsupported servo-loop argument {argument:?}"));
        }
    }
    Ok(())
}

fn write_json(path: &Path, value: &serde_json::Value) -> Result<(), String> {
    let bytes = serde_json::to_vec_pretty(value).map_err(|error| error.to_string())?;
    fs::write(path, bytes).map_err(|error| format!("{}: {error}", path.display()))
}

fn append_json_line(path: &Path, value: &serde_json::Value) -> Result<(), String> {
    let mut file = OpenOptions::new()
        .create(true)
        .append(true)
        .open(path)
        .map_err(|error| format!("{}: {error}", path.display()))?;
    serde_json::to_writer(&mut file, value).map_err(|error| error.to_string())?;
    file.write_all(b"\n")
        .map_err(|error| format!("{}: {error}", path.display()))?;
    file.flush()
        .map_err(|error| format!("{}: {error}", path.display()))?;
    file.sync_data()
        .map_err(|error| format!("{}: {error}", path.display()))
}

fn option(args: &[String], name: &str) -> Result<String, String> {
    optional_option(args, name)?.ok_or_else(|| format!("missing {name}"))
}

fn optional_option(args: &[String], name: &str) -> Result<Option<String>, String> {
    let mut values = args.windows(2).filter(|pair| pair[0] == name);
    let value = values.next().map(|pair| pair[1].clone());
    if values.next().is_some() {
        return Err(format!("{name} may only be supplied once"));
    }
    Ok(value)
}

fn usize_option(
    args: &[String],
    name: &str,
    default: usize,
    minimum: usize,
    maximum: usize,
) -> Result<usize, String> {
    let value = optional_option(args, name)?
        .map(|raw| {
            raw.parse::<usize>()
                .map_err(|_| format!("{name} must be an integer"))
        })
        .transpose()?
        .unwrap_or(default);
    if !(minimum..=maximum).contains(&value) {
        return Err(format!("{name} must be within [{minimum}, {maximum}]"));
    }
    Ok(value)
}

fn f64_option(
    args: &[String],
    name: &str,
    default: f64,
    minimum: f64,
    maximum: f64,
) -> Result<f64, String> {
    let value = optional_option(args, name)?
        .map(|raw| {
            raw.parse::<f64>()
                .map_err(|_| format!("{name} must be a number"))
        })
        .transpose()?
        .unwrap_or(default);
    if !value.is_finite() || !(minimum..=maximum).contains(&value) {
        return Err(format!(
            "{name} must be finite and within [{minimum}, {maximum}]"
        ));
    }
    Ok(value)
}

fn now_ns() -> Result<u64, String> {
    runtime::unix_time_ns()
}

fn flag(args: &[String], name: &str) -> bool {
    args.iter().any(|argument| argument == name)
}

#[cfg(test)]
mod tests {
    use super::*;

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
}
