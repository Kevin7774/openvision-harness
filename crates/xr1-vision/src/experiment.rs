use crate::runtime::RuntimePaths;
use std::fs::{self, OpenOptions};
use std::io::Write;
use std::path::PathBuf;

pub struct ExperimentJournal {
    runtime: RuntimePaths,
}

impl ExperimentJournal {
    pub fn new(runtime: RuntimePaths) -> Self {
        Self { runtime }
    }

    pub fn begin(&self, purpose: &str) -> Result<(), String> {
        if self.active_path().exists() {
            return Err("an experiment is already active".into());
        }
        fs::create_dir_all(self.harness_dir()).map_err(|error| error.to_string())?;
        let number = self.next_experiment()?;
        let id = format!("{number:02}");
        let clip = format!("experiment_{id}_full");
        self.runtime.run_python("xr1_cam.py", &["start", &clip])?;

        let report = self.harness_dir().join(format!("experiment_{id}.md"));
        let parent = if number > 1 {
            format!("{:02}", number - 1)
        } else {
            "NONE".into()
        };
        let initial = format!(
            "# Experiment {id}\n\nVideo: `{clip}.mov`\n\nParent Experiment: {parent}\n\nPurpose:\n{purpose}\n\nObservation:\n\nHypothesis:\n\nPrediction:\n\nAction:\n\nPost Observation:\n\nPrediction Match:\n\nError:\n\nMinimal Repair:\n\nMemory Update:\n\nNext Hypothesis:\n"
        );
        fs::write(report, initial).map_err(|error| error.to_string())?;
        fs::write(self.active_path(), format!("{id}\n")).map_err(|error| error.to_string())?;
        println!("{{\"ok\":true,\"experiment\":\"{id}\",\"video\":\"{clip}.mov\"}}");
        Ok(())
    }

    pub fn note(&self, section: &str, text: &str) -> Result<(), String> {
        self.append_report(&format!("\n## {section}\n{text}"))
    }

    pub fn grip(&self, side: &str, state: &str) -> Result<(), String> {
        self.active_id()?;
        if !matches!(side, "left" | "right") {
            return Err("side must be left or right".into());
        }
        if !matches!(state, "open" | "close") {
            return Err("state must be open or close".into());
        }
        self.append_report(&format!("\nAction command: grip {side} {state}"))?;
        self.runtime.run_python("xr1.py", &["grip", side, state])
    }

    pub fn end(&self, result: &str) -> Result<(), String> {
        let id = self.active_id()?;
        if !matches!(result, "SUCCESS" | "FAILED") {
            return Err("status must be SUCCESS or FAILED".into());
        }
        self.runtime.run_python("xr1_cam.py", &["stop"])?;
        self.append_report(&format!("\nSTATUS: {result}"))?;
        fs::remove_file(self.active_path()).map_err(|error| error.to_string())?;
        println!("{{\"ok\":true,\"experiment\":\"{id}\",\"status\":\"{result}\"}}");
        Ok(())
    }

    pub fn print_status(&self) {
        match self.active_id() {
            Ok(id) => println!("{{\"ok\":true,\"active_experiment\":\"{id}\"}}"),
            Err(_) => println!("{{\"ok\":true,\"active_experiment\":null}}"),
        }
    }

    fn harness_dir(&self) -> PathBuf {
        self.runtime.data_root().join("experiments").join("harness")
    }

    fn active_path(&self) -> PathBuf {
        self.harness_dir().join("ACTIVE")
    }

    fn next_experiment(&self) -> Result<u32, String> {
        let mut maximum = 0;
        if self.harness_dir().exists() {
            for entry in fs::read_dir(self.harness_dir()).map_err(|error| error.to_string())? {
                let name = entry.map_err(|error| error.to_string())?.file_name();
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

    fn active_id(&self) -> Result<String, String> {
        let id = fs::read_to_string(self.active_path())
            .map_err(|_| "no active experiment".to_string())?
            .trim()
            .to_string();
        if id.len() == 2 && id.chars().all(|character| character.is_ascii_digit()) {
            Ok(id)
        } else {
            Err("invalid ACTIVE marker".into())
        }
    }

    fn append_report(&self, text: &str) -> Result<(), String> {
        let id = self.active_id()?;
        let report = self.harness_dir().join(format!("experiment_{id}.md"));
        let mut file = OpenOptions::new()
            .append(true)
            .open(report)
            .map_err(|error| error.to_string())?;
        writeln!(file, "{text}").map_err(|error| error.to_string())
    }
}
