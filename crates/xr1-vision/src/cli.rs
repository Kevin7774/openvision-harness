use crate::experiment::ExperimentJournal;
use crate::hardware;
use crate::kinematics::Chain;
use crate::observation;
use crate::perception;
use crate::planning;
use crate::proposal::VisionHarnessProposal;
use crate::runtime::RuntimePaths;
use crate::safety;
use crate::visual_servo;
use std::fs;
use std::path::Path;
use std::time::{SystemTime, UNIX_EPOCH};

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
        Some("sensor-status") => hardware::print_status(),
        Some("servo-propose") => servo_propose(&runtime, args.collect()),
        Some("help") | Some("--help") | Some("-h") | None => {
            print_help();
            Ok(())
        }
        Some(command) => Err(format!("unknown command {command:?}; run xr1-vision help")),
    }
}

fn print_help() {
    println!("xr1-vision <command>");
    println!("  preflight");
    println!("  observe");
    println!("  plan [--proposal FILE]  # semantic proposal -> perception -> grasp candidates");
    println!("  fk J1 .. JN             # fingertip-pad FK + tool rotation");
    println!("  begin --purpose TEXT");
    println!("  note --section NAME --text TEXT");
    println!("  grip --side right|left --state open|close");
    println!("  end --status SUCCESS|FAILED");
    println!("  status");
    println!("  sensor-status           # read-only physical sensor capability report");
    println!(
        "  servo-propose --input FILE --state FILE # proposal + deterministic gate; never executes"
    );
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
        Some(path) => VisionHarnessProposal::read(Path::new(&path))?,
        None => VisionHarnessProposal::yellow_block(),
    };
    let latest = runtime
        .data_root()
        .join("vista_runs")
        .join(RUN_ID)
        .join("latest.json");
    let frame = perception::observe_object(&latest, &proposal)?;
    let report = planning::plan(proposal, frame, runtime.arm_urdf())?;
    println!(
        "{}",
        serde_json::to_string(&report).map_err(|error| error.to_string())?
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
    let input_path = option(&args, "--input")?;
    let state_path = option(&args, "--state")?;
    let input: visual_servo::ServoInput = serde_json::from_slice(
        &fs::read(&input_path).map_err(|error| format!("{input_path}: {error}"))?,
    )
    .map_err(|error| format!("invalid servo input: {error}"))?;
    let state = observation::read_state(Path::new(&state_path))?;
    let tip = servo_tip(&input.controlled_joints)?;
    let chain = Chain::from_urdf(runtime.arm_urdf(), tip)?;
    let sensors = hardware::inspect()?;
    let proposal = visual_servo::propose(&input, safety::MAX_SERVO_STEP_RAD)?;
    let generated_at_ns = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_err(|error| format!("system clock is before Unix epoch: {error}"))?
        .as_nanos()
        .try_into()
        .map_err(|_| "system timestamp does not fit u64".to_string())?;
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
            "proposal": proposal,
            "safety": safety_report,
            "ready_for_execution_adapter": safety_report.approved,
            "execution_authorized": false
        }))
        .map_err(|error| error.to_string())?
    );
    Ok(())
}

fn servo_tip(controlled_joints: &[String]) -> Result<&'static str, String> {
    let right = controlled_joints
        .iter()
        .all(|name| name.starts_with("right_arm_") && name.ends_with("_joint"));
    let left = controlled_joints
        .iter()
        .all(|name| name.starts_with("left_arm_") && name.ends_with("_joint"));
    match (right, left) {
        (true, false) => Ok("right_tcp_link"),
        (false, true) => Ok("left_tcp_link"),
        _ => Err("controlled_joints must all belong to exactly one arm".into()),
    }
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn repeated_option_is_rejected() {
        let args = vec![
            "--proposal".into(),
            "a.json".into(),
            "--proposal".into(),
            "b.json".into(),
        ];
        assert!(optional_option(&args, "--proposal").is_err());
    }

    #[test]
    fn servo_joints_must_belong_to_one_arm() {
        assert!(servo_tip(&[
            "right_arm_2_joint".into(),
            "left_arm_4_joint".into(),
            "right_arm_6_joint".into(),
        ])
        .is_err());
    }
}
