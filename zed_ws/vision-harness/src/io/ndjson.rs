use std::io::{self, BufRead};
use crate::runtime::{HarnessRuntime, ModelProposal, RuntimeConfig, RuntimeState, SensorFrame};

#[derive(serde::Deserialize)]
struct Input { frame: SensorFrame, #[serde(default)] proposal: Option<ModelProposal> }

pub fn run_stdin(config: RuntimeConfig, state: RuntimeState) -> Result<(), String> {
    let mut runtime = HarnessRuntime::new(config, state);
    for line in io::stdin().lock().lines() {
        let line = line.map_err(|e| e.to_string())?;
        if line.trim().is_empty() { continue; }
        let input: Input = serde_json::from_str(&line).map_err(|e| format!("invalid input: {e}"))?;
        let decision = runtime.step(&input.frame, input.proposal);
        println!("{}", serde_json::to_string(&decision).map_err(|e| e.to_string())?);
    }
    Ok(())
}
