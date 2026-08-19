use crate::experiment::ExperimentJournal;
use crate::grasp_feedback::{self, TactileCalibration, TactileObservation};
use crate::grasp_loop;
use crate::hardware;
use crate::kinematics::Chain;
use crate::observation;
use crate::perception;
use crate::planning;
use crate::proposal::TaskProposal;
use crate::runtime::{self, RuntimePaths};
use crate::safety;
use crate::servo_loop;
use crate::support::adapter::{json_string, parse_last_json};
use crate::support::args::{flag, option, optional_option};
use crate::support::evidence::read_json as read_json_file;
use crate::task::{TaskEventRecord, TaskExecutive};
use crate::visual_servo;
use std::fs;
use std::path::{Path, PathBuf};

const RUN_ID: &str = "yellow-block-harness";

pub fn run<I>(args: I) -> Result<(), String>
where
    I: IntoIterator<Item = String>,
{
    let runtime = RuntimePaths::discover();
    let journal = ExperimentJournal::new(runtime.clone());
    let mut args = args.into_iter();
    match args.next().as_deref() {
        Some("preflight") => preflight(&runtime),
        Some("observe") => observe(&runtime),
        Some("plan") => plan(&runtime, args.collect()),
        Some("validate-proposal") => validate_proposal(args.collect()),
        Some("bundle") => bundle(&runtime, args.collect()),
        Some("replay") => replay(args.collect()),
        Some("fk") => fk(&runtime, args.collect()),
        Some("begin") => {
            let args = args.collect::<Vec<_>>();
            journal.begin(&option(&args, "--purpose")?)
        }
        Some("note") => {
            let args = args.collect::<Vec<_>>();
            journal.note(&option(&args, "--section")?, &option(&args, "--text")?)
        }
        Some("grip") => {
            let args = args.collect::<Vec<_>>();
            journal.grip(&option(&args, "--side")?, &option(&args, "--state")?)
        }
        Some("end") => {
            let args = args.collect::<Vec<_>>();
            journal.end(&option(&args, "--status")?)
        }
        Some("status") => {
            journal.print_status();
            Ok(())
        }
        Some("packs") => {
            let registry = crate::taskpack::TaskPackRegistry::with_default_packs();
            println!(
                "{}",
                serde_json::to_string(&registry.skill_ids()).map_err(|e| e.to_string())?
            );
            Ok(())
        }
        Some("judge-ceiling") => judge_ceiling(args.collect()),
        Some("sensor-status") => hardware::print_status(),
        Some("d405-observe") => d405_observe(&runtime, args.collect()),
        Some("tactile-observe") => tactile_observe(&runtime, args.collect()),
        Some("tactile-assess") => tactile_assess(&runtime, args.collect()),
        Some("grasp-loop") => grasp_loop::run(&runtime, args.collect()),
        Some("servo-pads") => servo_pads(&runtime, args.collect()),
        Some("servo-observe") => servo_observe(&runtime, args.collect()),
        Some("servo-calibrate") => servo_calibrate(args.collect()),
        Some("servo-propose") => servo_propose(&runtime, args.collect()),
        Some("servo-step") => servo_step(&runtime, args.collect()),
        Some("servo-reconcile") => servo_reconcile(args.collect()),
        Some("servo-loop") => servo_loop::run(&runtime, args.collect()),
        Some("help") | Some("--help") | Some("-h") | None => {
            print_help();
            Ok(())
        }
        Some(command) => Err(format!("unknown command {command:?}; run xr1-vision help")),
    }
}

/// ADR 0005's arithmetic, made runnable: at success rate `p`, a judge with
/// systematic bias `b` stops adding information after `n* = p(1-p)/b²` episodes.
/// An operator deciding "should we collect more episodes?" needs this number,
/// because past it the answer is "no, improve the judge".
fn judge_ceiling(args: Vec<String>) -> Result<(), String> {
    validate_command_args(Some("judge-ceiling"), &args, &["--rate", "--bias"], &[])?;
    let parse = |name: &str| -> Result<f64, String> {
        option(&args, name)?
            .parse::<f64>()
            .map_err(|_| format!("{name} must be a number"))
    };
    let rate = parse("--rate")?;
    let bias = parse("--bias")?;
    let ceiling = harness_evaluation::information_ceiling(rate, bias).ok_or_else(|| {
        "--rate must be within [0, 1] and --bias must be finite and greater than zero".to_string()
    })?;
    let standard_error_at_ceiling =
        harness_evaluation::standard_error(rate, ceiling.ceil() as usize).unwrap_or(f64::NAN);
    println!(
        "{}",
        serde_json::to_string(&serde_json::json!({
            "ok": true,
            "mode": "judge_information_ceiling",
            "success_rate": rate,
            "judge_bias": bias,
            "episode_ceiling": ceiling,
            "standard_error_at_ceiling": standard_error_at_ceiling,
            "note": "past episode_ceiling, sampling noise is below the judge's systematic bias, so \
                     further episodes cannot resolve whether anything improved"
        }))
        .map_err(|error| error.to_string())?
    );
    Ok(())
}

/// The top-level CLI reports unknown arguments without a subcommand label.
fn validate_command_args(
    command: Option<&str>,
    args: &[String],
    options: &[&str],
    flags: &[&str],
) -> Result<(), String> {
    crate::support::args::validate_command_args(command, args, options, flags)
}

fn print_help() {
    println!("xr1-vision <command>");
    println!("  preflight");
    println!("  observe");
    println!(
        "  plan [--proposal FILE] [--latest FILE] [--moveit] # proposal -> validated candidates"
    );
    println!("  validate-proposal --proposal FILE # validate/upgrade TaskProposal to schema v2");
    println!("  bundle [--latest FILE] # unified ZED/robot/capability observation JSON");
    println!("  replay --proposal FILE --events FILE # deterministic task-state replay");
    println!("  fk J1 .. JN             # fingertip-pad FK + tool rotation");
    println!("  begin --purpose TEXT");
    println!("  note --section NAME --text TEXT");
    println!("  grip --side right|left --state open|close");
    println!("  end --status SUCCESS|FAILED");
    println!("  status");
    println!("  packs                   # registered task packs able to ground objects");
    println!(
        "  judge-ceiling --rate P --bias B # episodes past which a judge of bias B adds nothing"
    );
    println!("  sensor-status           # read-only physical sensor capability report");
    println!("  d405-observe [--timeout S] # capture and validate one real near-field frame");
    println!("  tactile-observe --config FILE # capture two named pressure pads without motion");
    println!(
        "  tactile-assess --mode baseline|closure|retention --observation FILE|--config FILE --calibration FILE"
    );
    println!(
        "  grasp-loop --tactile-config FILE --tactile-calibration FILE --d405-target FILE [--timeout S] [--go]"
    );
    println!("  servo-pads --frame DIR  # physical pad signal using the shared Rust detector");
    println!("  servo-observe [--latest FILE] # pad/target signal from one saved observation");
    println!("  servo-calibrate --input FILE # fit local 3x3 Jacobian from +/- samples");
    println!(
        "  servo-propose --input FILE|--request FILE --state FILE # bounded proposal + safety gate"
    );
    println!("  servo-step --proposal FILE [--go] # execute at most one approved microstep");
    println!("  servo-reconcile --input FILE # compare predicted vs observed microstep");
    println!(
        "  servo-loop --calibration FILE [--go] # bounded observe/step/reobserve loop; never restarts services"
    );
}

fn d405_observe(runtime: &RuntimePaths, args: Vec<String>) -> Result<(), String> {
    validate_command_args(None, &args, &["--timeout"], &[])?;
    let timeout = optional_option(&args, "--timeout")?.unwrap_or_else(|| "8.0".into());
    let timeout_value = timeout
        .parse::<f64>()
        .map_err(|_| "--timeout must be a number".to_string())?;
    if !timeout_value.is_finite() || !(3.0..=20.0).contains(&timeout_value) {
        return Err("--timeout must be within 3..20 seconds".into());
    }
    let output_root = runtime.data_root().join("sensors/d405");
    let root = output_root.to_string_lossy().into_owned();
    let output = runtime.run_python_capture(
        "d405_observe.py",
        &[
            "--timeout",
            timeout.as_str(),
            "--output-root",
            root.as_str(),
        ],
    )?;
    let report = parse_last_json(&output, "D405 adapter")?;
    if report.get("ok") != Some(&serde_json::Value::Bool(true)) {
        return Err(format!("D405 observation failed: {report}"));
    }
    let latest_path = PathBuf::from(json_string(&report, "latest_path")?);
    let signal = perception::observe_near_field_signal(&latest_path)?;
    println!(
        "{}",
        serde_json::to_string(&serde_json::json!({
            "ok": true,
            "schema_version": 1,
            "mode": "d405_observe",
            "capture": report,
            "signal": signal
        }))
        .map_err(|error| error.to_string())?
    );
    Ok(())
}

fn tactile_assess(runtime: &RuntimePaths, args: Vec<String>) -> Result<(), String> {
    validate_command_args(
        None,
        &args,
        &[
            "--mode",
            "--observation",
            "--config",
            "--calibration",
            "--held",
        ],
        &[],
    )?;
    let mode = option(&args, "--mode")?;
    let observation_path = optional_option(&args, "--observation")?;
    let config_path = optional_option(&args, "--config")?;
    let observation: TactileObservation = match (observation_path, config_path) {
        (Some(path), None) => read_json_file(Path::new(&path), "tactile observation")?,
        (None, Some(path)) => serde_json::from_value(capture_tactile(runtime, &path)?)
            .map_err(|error| format!("invalid tactile observation: {error}"))?,
        (Some(_), Some(_)) => return Err("use exactly one of --observation or --config".into()),
        (None, None) => return Err("missing --observation or --config".into()),
    };
    let calibration: TactileCalibration = read_json_file(
        Path::new(&option(&args, "--calibration")?),
        "tactile calibration",
    )?;
    let now_ns = runtime::unix_time_ns()?;
    let report = match mode.as_str() {
        "baseline" => grasp_feedback::assess_baseline(&observation, &calibration, now_ns)?,
        "closure" => grasp_feedback::assess_closure(&observation, &calibration, now_ns)?,
        "retention" => {
            let held_path = optional_option(&args, "--held")?
                .ok_or_else(|| "retention assessment requires --held FILE".to_string())?;
            let held: TactileObservation =
                read_json_file(Path::new(&held_path), "held tactile observation")?;
            grasp_feedback::assess_retention(&held, &observation, &calibration, now_ns)?
        }
        _ => return Err("--mode must be baseline, closure or retention".into()),
    };
    if mode != "retention" && optional_option(&args, "--held")?.is_some() {
        return Err("--held is only valid with --mode retention".into());
    }
    println!(
        "{}",
        serde_json::to_string(&serde_json::json!({
            "ok": true,
            "schema_version": 1,
            "mode": "tactile_assessment",
            "assessment_mode": mode,
            "observation": observation,
            "assessment": report
        }))
        .map_err(|error| error.to_string())?
    );
    Ok(())
}

fn tactile_observe(runtime: &RuntimePaths, args: Vec<String>) -> Result<(), String> {
    validate_command_args(None, &args, &["--config"], &[])?;
    let report = capture_tactile(runtime, &option(&args, "--config")?)?;
    println!(
        "{}",
        serde_json::to_string(&report).map_err(|error| error.to_string())?
    );
    Ok(())
}

fn capture_tactile(runtime: &RuntimePaths, config_path: &str) -> Result<serde_json::Value, String> {
    let config =
        fs::canonicalize(config_path).map_err(|error| format!("{config_path}: {error}"))?;
    let config = config.to_string_lossy().into_owned();
    let output_root = runtime.data_root().join("sensors/tactile");
    let root = output_root.to_string_lossy().into_owned();
    let output = runtime.run_python_capture(
        "tactile_adapter.py",
        &["--config", config.as_str(), "--output-root", root.as_str()],
    )?;
    let report = parse_last_json(&output, "tactile adapter")?;
    if report.get("ok") != Some(&serde_json::Value::Bool(true)) {
        return Err(format!("tactile capture failed: {report}"));
    }
    Ok(report)
}

fn servo_observe(runtime: &RuntimePaths, args: Vec<String>) -> Result<(), String> {
    let latest = optional_option(&args, "--latest")?
        .map(PathBuf::from)
        .unwrap_or_else(|| {
            runtime
                .data_root()
                .join("vista_runs")
                .join(RUN_ID)
                .join("latest.json")
        });
    let chain = Chain::from_urdf(runtime.arm_urdf(), "right_tcp_link")?;
    let observation = perception::observe_servo_signal(&latest, &chain)?;
    println!(
        "{}",
        serde_json::to_string(&observation).map_err(|error| error.to_string())?
    );
    Ok(())
}

fn servo_pads(runtime: &RuntimePaths, args: Vec<String>) -> Result<(), String> {
    let frame_dir = option(&args, "--frame")?;
    let chain = Chain::from_urdf(runtime.arm_urdf(), "right_tcp_link")?;
    let observation = perception::observe_pad_signal_from_frame(Path::new(&frame_dir), &chain)?;
    println!(
        "{}",
        serde_json::to_string(&observation).map_err(|error| error.to_string())?
    );
    Ok(())
}

fn servo_calibrate(args: Vec<String>) -> Result<(), String> {
    let path = option(&args, "--input")?;
    let input: visual_servo::JacobianMeasurementInput =
        serde_json::from_slice(&fs::read(&path).map_err(|error| format!("{path}: {error}"))?)
            .map_err(|error| format!("invalid Jacobian measurement: {error}"))?;
    let calibration = visual_servo::measure_jacobian(&input)?;
    println!(
        "{}",
        serde_json::to_string(&calibration).map_err(|error| error.to_string())?
    );
    Ok(())
}

fn servo_reconcile(args: Vec<String>) -> Result<(), String> {
    let path = option(&args, "--input")?;
    let input: visual_servo::ReconciliationInput =
        serde_json::from_slice(&fs::read(&path).map_err(|error| format!("{path}: {error}"))?)
            .map_err(|error| format!("invalid servo reconciliation: {error}"))?;
    let report = visual_servo::reconcile(&input)?;
    println!(
        "{}",
        serde_json::to_string(&report).map_err(|error| error.to_string())?
    );
    Ok(())
}

fn servo_step(runtime: &RuntimePaths, args: Vec<String>) -> Result<(), String> {
    let proposal = option(&args, "--proposal")?;
    let mut adapter_args = vec!["--proposal", proposal.as_str()];
    if flag(&args, "--go") {
        adapter_args.push("--go");
    }
    runtime.run_python("servo_adapter.py", &adapter_args)
}

fn preflight(runtime: &RuntimePaths) -> Result<(), String> {
    runtime.run_python("xr1.py", &["pose"])?;
    runtime.run_python("xr1_cam.py", &["doctor"])?;
    println!("{{\"ok\":true,\"phase\":\"preflight\"}}");
    Ok(())
}

fn observe(runtime: &RuntimePaths) -> Result<(), String> {
    runtime.run_python("vista_observe.py", &["--run-id", RUN_ID, "--timeout", "20"])
}

fn plan(runtime: &RuntimePaths, args: Vec<String>) -> Result<(), String> {
    let proposal = match optional_option(&args, "--proposal")? {
        Some(path) => TaskProposal::read(Path::new(&path))?,
        None => TaskProposal::yellow_block_grasp(),
    };
    let latest = optional_option(&args, "--latest")?
        .map(PathBuf::from)
        .unwrap_or_else(|| {
            runtime
                .data_root()
                .join("vista_runs")
                .join(RUN_ID)
                .join("latest.json")
        });
    let request = proposal.grasp_request()?;
    let frame = perception::observe_object(&latest, &request)?;
    let mut report = planning::plan(proposal, frame, runtime.arm_urdf())?;
    if flag(&args, "--moveit") {
        planning::validate_with_moveit(
            &mut report,
            runtime.moveit_validator(),
            runtime.arm_urdf(),
            runtime.moveit_srdf(),
        )?;
    }
    println!(
        "{}",
        serde_json::to_string(&report).map_err(|error| error.to_string())?
    );
    Ok(())
}

fn validate_proposal(args: Vec<String>) -> Result<(), String> {
    let path = option(&args, "--proposal")?;
    let proposal = TaskProposal::read(Path::new(&path))?;
    println!(
        "{}",
        serde_json::to_string(&proposal).map_err(|error| error.to_string())?
    );
    Ok(())
}

fn bundle(runtime: &RuntimePaths, args: Vec<String>) -> Result<(), String> {
    let latest = optional_option(&args, "--latest")?
        .map(PathBuf::from)
        .unwrap_or_else(|| {
            runtime
                .data_root()
                .join("vista_runs")
                .join(RUN_ID)
                .join("latest.json")
        });
    let report = observation::bundle_from_latest(&latest, hardware::inspect()?)?;
    println!(
        "{}",
        serde_json::to_string(&report).map_err(|error| error.to_string())?
    );
    Ok(())
}

fn replay(args: Vec<String>) -> Result<(), String> {
    let proposal_path = option(&args, "--proposal")?;
    let events_path = option(&args, "--events")?;
    let proposal = TaskProposal::read(Path::new(&proposal_path))?;
    let contents =
        fs::read_to_string(&events_path).map_err(|error| format!("{events_path}: {error}"))?;
    let mut events = Vec::new();
    for (index, line) in contents.lines().enumerate() {
        if line.trim().is_empty() {
            continue;
        }
        let record = serde_json::from_str::<TaskEventRecord>(line).map_err(|error| {
            format!("invalid task event {}:{}: {error}", events_path, index + 1)
        })?;
        events.push(record);
    }
    let snapshot = TaskExecutive::replay(proposal, events)?;
    println!(
        "{}",
        serde_json::to_string(&snapshot).map_err(|error| error.to_string())?
    );
    Ok(())
}

fn fk(runtime: &RuntimePaths, args: Vec<String>) -> Result<(), String> {
    let joints = args
        .iter()
        .map(|value| {
            value
                .parse::<f64>()
                .map_err(|_| format!("not a number: {value}"))
        })
        .collect::<Result<Vec<_>, _>>()?;
    if joints.is_empty() {
        return Err("usage: fk J1 J2 .. JN   (joint angles in radians)".into());
    }
    let chain = Chain::from_urdf(runtime.arm_urdf(), "right_tcp_link")?;
    let report = planning::forward_kinematics_report(&chain, &joints)?;
    println!(
        "{}",
        serde_json::to_string(&report).map_err(|error| error.to_string())?
    );
    Ok(())
}

fn servo_propose(runtime: &RuntimePaths, args: Vec<String>) -> Result<(), String> {
    let state_path = option(&args, "--state")?;
    let raw_path = optional_option(&args, "--input")?;
    let request_path = optional_option(&args, "--request")?;
    let (input, context) = match (raw_path, request_path) {
        (Some(path), None) => {
            let input: visual_servo::ServoInput = serde_json::from_slice(
                &fs::read(&path).map_err(|error| format!("{path}: {error}"))?,
            )
            .map_err(|error| format!("invalid servo input: {error}"))?;
            (input, None)
        }
        (None, Some(path)) => {
            let request: visual_servo::CalibratedServoRequest = serde_json::from_slice(
                &fs::read(&path).map_err(|error| format!("{path}: {error}"))?,
            )
            .map_err(|error| format!("invalid calibrated servo request: {error}"))?;
            let input = visual_servo::input_from_request(&request)?;
            let context = serde_json::json!({
                "calibration_reference_frame_id": request.calibration.reference_frame_id,
                "target": request.target,
                "before": request.current
            });
            (input, Some(context))
        }
        (Some(_), Some(_)) => return Err("use exactly one of --input or --request".into()),
        (None, None) => return Err("missing --input or --request".into()),
    };
    let state = observation::read_state(Path::new(&state_path))?;
    let tip = visual_servo::controlled_arm_tip(&input.controlled_joints)?;
    let chain = Chain::from_urdf(runtime.arm_urdf(), tip)?;
    let sensors = hardware::inspect()?;
    let proposal = visual_servo::propose(&input, safety::MAX_SERVO_STEP_RAD)?;
    let generated_at_ns = runtime::unix_time_ns()?;
    let safety_report = safety::evaluate_servo(
        &chain,
        &state,
        &input.observation_frame_id,
        &proposal,
        &input.requires,
        &sensors,
        generated_at_ns,
    )?;
    println!(
        "{}",
        serde_json::to_string(&serde_json::json!({
            "ok": true,
            "schema_version": 1,
            "mode": "visual_servo_proposal",
            "generated_at_ns": generated_at_ns,
            "observation_frame_id": input.observation_frame_id,
            "context": context,
            "proposal": proposal,
            "safety": safety_report,
            "ready_for_execution_adapter": safety_report.approved,
            "execution_authorized": false
        }))
        .map_err(|error| error.to_string())?
    );
    Ok(())
}
