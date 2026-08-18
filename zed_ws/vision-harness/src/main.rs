use std::env;
use std::fs;
use std::io::Read;
use std::process::ExitCode;

mod geometry;
mod harness;
mod io;
mod kinematics;
mod perception;
mod planning;
mod policy;
mod runtime;
mod safety;

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
        Some("runtime-evaluate") => runtime_evaluate(args.collect()),
        Some("runtime-stream") => runtime_stream(args.collect()),
        Some("hardware-status") => io::hardware::print_status(),
        _ => {
            print_help();
            Ok(())
        }
    }
}

fn print_help() {
    println!("vision-harness <command>");
    println!("  hardware-status");
    println!("  runtime-evaluate --input FILE [--state FILE] [--proposal FILE]");
    println!("  runtime-stream [--state FILE]");
}

fn runtime_evaluate(args: Vec<String>) -> Result<(), String> {
    let input_path = option(&args, "--input")?;
    let mut data = Vec::new();
    if input_path == "-" {
        std::io::stdin().read_to_end(&mut data).map_err(|e| e.to_string())?;
    } else {
        data = fs::read(&input_path).map_err(|e| format!("{input_path}: {e}"))?;
    }
    let frame: runtime::SensorFrame = serde_json::from_slice(&data)
        .map_err(|e| format!("invalid sensor frame: {e}"))?;
    let state = load_optional(&args, "--state")?.unwrap_or_default();
    let proposal = load_optional(&args, "--proposal")?;
    let mut runtime = runtime::HarnessRuntime::new(runtime::RuntimeConfig::default(), state);
    let decision = runtime.step(&frame, proposal);
    println!("{}", serde_json::to_string(&decision).map_err(|e| e.to_string())?);
    Ok(())
}

fn runtime_stream(args: Vec<String>) -> Result<(), String> {
    let state = load_optional(&args, "--state")?.unwrap_or_default();
    io::ndjson::run_stdin(runtime::RuntimeConfig::default(), state)
}

fn option(args: &[String], name: &str) -> Result<String, String> {
    args.windows(2).find(|pair| pair[0] == name).map(|pair| pair[1].clone())
        .ok_or_else(|| format!("missing {name}"))
}

fn load_optional<T: serde::de::DeserializeOwned>(args: &[String], name: &str) -> Result<Option<T>, String> {
    args.windows(2).find(|pair| pair[0] == name)
        .map(|pair| fs::read(&pair[1]).map_err(|e| e.to_string()))
        .transpose()?
        .map(|bytes| serde_json::from_slice(&bytes).map_err(|e| e.to_string()))
        .transpose()
}
