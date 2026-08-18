use std::env;
use std::fs::{self, OpenOptions};
use std::io::Write;
use std::io::Read;
use std::path::{Path, PathBuf};
use std::process::{Command, ExitCode, Stdio};

mod perception;
mod kinematics;
mod local_model;
mod runtime;
mod geometry;
mod planning;
mod harness;
mod safety;
mod io;

const ROOT: &str = "/home/astrabot/workspace/zed_ws";
const SCRIPTS: &str = "/home/astrabot/workspace/zed_ws/scripts";
const RUN_ID: &str = "yellow-block-harness";

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("ERROR: {error}");
            ExitCode::FAILURE
        }
    }
}

fn run() -> Result<(), String> {
    let mut args = env::args().skip(1);
    match args.next().as_deref() {
        Some("preflight") => preflight(),
        Some("observe") => observe(),
        Some("plan") => plan(),
        Some("begin") => begin(args.collect()),
        Some("note") => note(args.collect()),
        Some("grip") => grip(args.collect()),
        Some("end") => end(args.collect()),
        Some("status") => status(),
        Some("runtime-evaluate") => runtime_evaluate(args.collect()),
        Some("runtime-stream") => runtime_stream(args.collect()),
        _ => {
            print_help();
            Ok(())
        }
    }
}

fn print_help() {
    println!("vision-harness <command>");
    println!("  preflight");
    println!("  observe");
    println!("  plan");
    println!("  begin --purpose TEXT");
    println!("  note --section NAME --text TEXT");
    println!("  grip --side right|left --state open|close");
    println!("  end --status SUCCESS|FAILED");
    println!("  status");
    println!("  runtime-evaluate --input FILE [--state FILE]");
    println!("  runtime-stream [--state FILE]");
}

fn runtime_stream(args: Vec<String>) -> Result<(), String> {
    let state = args
        .windows(2)
        .find(|pair| pair[0] == "--state")
        .map(|pair| fs::read(&pair[1]).map_err(|e| e.to_string()))
        .transpose()?
        .map(|bytes| serde_json::from_slice(&bytes).map_err(|e| e.to_string()))
        .transpose()?
        .unwrap_or_default();
    io::ndjson::run_stdin(runtime::RuntimeConfig::default(), state)
}

fn runtime_evaluate(args: Vec<String>) -> Result<(), String> {
    let input_path = option(&args, "--input")?;
    let mut data = Vec::new();
    if input_path == "-" {
        std::io::stdin().read_to_end(&mut data).map_err(|e| e.to_string())?;
    } else {
        data = fs::read(&input_path).map_err(|e| format!("{input_path}: {e}"))?;
    }
    let frame: runtime::SensorFrame =
        serde_json::from_slice(&data).map_err(|e| format!("invalid sensor frame: {e}"))?;
    let state = args
        .windows(2)
        .find(|pair| pair[0] == "--state")
        .map(|pair| fs::read(&pair[1]).map_err(|e| e.to_string()))
        .transpose()?
        .map(|bytes| serde_json::from_slice(&bytes).map_err(|e| e.to_string()))
        .transpose()?
        .unwrap_or_default();
    let proposal = args
        .windows(2)
        .find(|pair| pair[0] == "--proposal")
        .map(|pair| fs::read(&pair[1]).map_err(|e| e.to_string()))
        .transpose()?
        .map(|bytes| serde_json::from_slice(&bytes).map_err(|e| e.to_string()))
        .transpose()?;
    let mut harness = runtime::HarnessRuntime::new(runtime::RuntimeConfig::default(), state);
    let decision = harness.step(&frame, proposal);
    println!("{}", serde_json::to_string(&decision).map_err(|e| e.to_string())?);
    Ok(())
}

fn command(name: &str, args: &[&str]) -> Result<(), String> {
    let mut shell_command = format!(
        "source /opt/ros/jazzy/setup.bash && source /opt/ros/astrabot/setup.bash && exec {}",
        shell_quote(name)
    );
    for arg in args {
        shell_command.push(' ');
        shell_command.push_str(&shell_quote(arg));
    }
    let status = Command::new("/bin/bash")
        .args(["-lc", &shell_command])
        .current_dir(SCRIPTS)
        .env("ROS_DOMAIN_ID", "12")
        .env("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
        .stdin(Stdio::null())
        .status()
        .map_err(|error| format!("failed to start {name}: {error}"))?;
    if status.success() {
        Ok(())
    } else {
        Err(format!("{name} exited with {status}"))
    }
}

fn shell_quote(value: &str) -> String {
    format!("'{}'", value.replace('\'', "'\\''"))
}

fn preflight() -> Result<(), String> {
    command("python3", &["xr1.py", "pose"])?;
    command("python3", &["xr1_cam.py", "doctor"])?;
    println!("{{\"ok\":true,\"phase\":\"preflight\"}}");
    Ok(())
}

fn observe() -> Result<(), String> {
    command(
        "python3",
        &["vista_observe.py", "--run-id", RUN_ID, "--timeout", "20"],
    )
}

fn plan() -> Result<(), String> {
    let latest = Path::new(ROOT)
        .join("vista_runs")
        .join(RUN_ID)
        .join("latest.json");
    perception::plan(&latest)
}

fn harness_dir() -> PathBuf {
    Path::new(ROOT).join("experiments").join("harness")
}

fn active_path() -> PathBuf {
    harness_dir().join("ACTIVE")
}

fn next_experiment() -> Result<u32, String> {
    let mut maximum = 0;
    if harness_dir().exists() {
        for entry in fs::read_dir(harness_dir()).map_err(|e| e.to_string())? {
            let name = entry.map_err(|e| e.to_string())?.file_name();
            let name = name.to_string_lossy();
            if let Some(number) = name
                .strip_prefix("experiment_")
                .and_then(|tail| tail.strip_suffix(".md"))
                .and_then(|value| value.parse::<u32>().ok())
            {
                maximum = maximum.max(number);
            }
        }
    }
    Ok(maximum + 1)
}

fn option(args: &[String], name: &str) -> Result<String, String> {
    args.windows(2)
        .find(|pair| pair[0] == name)
        .map(|pair| pair[1].clone())
        .ok_or_else(|| format!("missing {name}"))
}

fn begin(args: Vec<String>) -> Result<(), String> {
    if active_path().exists() {
        return Err("an experiment is already active".into());
    }
    let purpose = option(&args, "--purpose")?;
    fs::create_dir_all(harness_dir()).map_err(|e| e.to_string())?;
    let number = next_experiment()?;
    let id = format!("{number:02}");
    let clip = format!("experiment_{id}_full");
    command("python3", &["xr1_cam.py", "start", &clip])?;

    let report = harness_dir().join(format!("experiment_{id}.md"));
    let initial = format!(
        "# Experiment {id}\n\nVideo: `{clip}.mov`\n\nParent Experiment: {}\n\nPurpose:\n{purpose}\n\nObservation:\n\nHypothesis:\n\nPrediction:\n\nAction:\n\nPost Observation:\n\nPrediction Match:\n\nError:\n\nMinimal Repair:\n\nMemory Update:\n\nNext Hypothesis:\n",
        if number > 1 { format!("{:02}", number - 1) } else { "NONE".into() }
    );
    fs::write(&report, initial).map_err(|e| e.to_string())?;
    fs::write(active_path(), format!("{id}\n")).map_err(|e| e.to_string())?;
    println!("{{\"ok\":true,\"experiment\":\"{id}\",\"video\":\"{clip}.mov\"}}");
    Ok(())
}

fn active_id() -> Result<String, String> {
    let id = fs::read_to_string(active_path())
        .map_err(|_| "no active experiment".to_string())?
        .trim()
        .to_string();
    if id.len() == 2 && id.chars().all(|c| c.is_ascii_digit()) {
        Ok(id)
    } else {
        Err("invalid ACTIVE marker".into())
    }
}

fn append_report(text: &str) -> Result<(), String> {
    let id = active_id()?;
    let report = harness_dir().join(format!("experiment_{id}.md"));
    let mut file = OpenOptions::new()
        .append(true)
        .open(report)
        .map_err(|e| e.to_string())?;
    writeln!(file, "{text}").map_err(|e| e.to_string())
}

fn note(args: Vec<String>) -> Result<(), String> {
    let section = option(&args, "--section")?;
    let text = option(&args, "--text")?;
    append_report(&format!("\n## {section}\n{text}"))
}

fn grip(args: Vec<String>) -> Result<(), String> {
    active_id()?;
    let side = option(&args, "--side")?;
    let state = option(&args, "--state")?;
    if !matches!(side.as_str(), "left" | "right") {
        return Err("side must be left or right".into());
    }
    if !matches!(state.as_str(), "open" | "close") {
        return Err("state must be open or close".into());
    }
    append_report(&format!("\nAction command: grip {side} {state}"))?;
    command("python3", &["xr1.py", "grip", &side, &state])
}

fn end(args: Vec<String>) -> Result<(), String> {
    let id = active_id()?;
    let result = option(&args, "--status")?;
    if !matches!(result.as_str(), "SUCCESS" | "FAILED") {
        return Err("status must be SUCCESS or FAILED".into());
    }
    command("python3", &["xr1_cam.py", "stop"])?;
    append_report(&format!("\nSTATUS: {result}"))?;
    fs::remove_file(active_path()).map_err(|e| e.to_string())?;
    println!("{{\"ok\":true,\"experiment\":\"{id}\",\"status\":\"{result}\"}}");
    Ok(())
}

fn status() -> Result<(), String> {
    match active_id() {
        Ok(id) => println!("{{\"ok\":true,\"active_experiment\":\"{id}\"}}"),
        Err(_) => println!("{{\"ok\":true,\"active_experiment\":null}}"),
    }
    Ok(())
}
