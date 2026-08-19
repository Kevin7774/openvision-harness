//! Evidence storage.
//!
//! Step 1 finding #4: evidence storage was interleaved with orchestration. The
//! three write shapes the harness actually uses are separated here, because they
//! are **not** interchangeable:
//!
//! - [`write_json`] overwrites. For a "latest" pointer that is meant to move.
//! - [`create_new_json`] refuses to overwrite and fsyncs. For an append-only
//!   record that must never be silently replaced.
//! - [`append_json_line`] appends one JSONL record and fsyncs.
//!
//! `servo_loop.rs` and `grasp_loop.rs` both had a private `write_json`, but they
//! were different functions with the same name: one overwrote, the other used
//! `create_new` plus `sync_data`. Merging them into one would have changed
//! durability behaviour, so both survive here under names that say which is
//! which.

use serde::Deserialize;
use std::fs::{self, OpenOptions};
use std::io::Write;
use std::path::{Path, PathBuf};

/// Overwrite `path` with pretty JSON. Use for a pointer that is meant to move.
pub fn write_json(path: &Path, value: &serde_json::Value) -> Result<(), String> {
    let bytes = serde_json::to_vec_pretty(value).map_err(|error| error.to_string())?;
    fs::write(path, bytes).map_err(|error| format!("{}: {error}", path.display()))
}

/// Create `path`, failing if it already exists, then fsync. Use for a record that
/// must never be silently replaced — the refusal to overwrite is the guarantee.
pub fn create_new_json(path: &Path, value: &serde_json::Value) -> Result<(), String> {
    let mut file = OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(path)
        .map_err(|error| format!("{}: {error}", path.display()))?;
    serde_json::to_writer_pretty(&mut file, value).map_err(|error| error.to_string())?;
    file.write_all(b"\n").map_err(|error| error.to_string())?;
    file.flush().map_err(|error| error.to_string())?;
    file.sync_data().map_err(|error| error.to_string())
}

/// Append one JSON line and fsync, so a crash cannot lose an already-reported
/// step.
pub fn append_json_line(path: &Path, value: &serde_json::Value) -> Result<(), String> {
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

/// Read and deserialize a JSON file, naming the artifact kind in any error.
pub fn read_json<T: for<'de> Deserialize<'de>>(path: &Path, kind: &str) -> Result<T, String> {
    serde_json::from_slice(&fs::read(path).map_err(|error| format!("{}: {error}", path.display()))?)
        .map_err(|error| format!("invalid {kind} {}: {error}", path.display()))
}

/// Resolve a path that must already exist.
pub fn canonical_file(path: &str) -> Result<PathBuf, String> {
    fs::canonicalize(path).map_err(|error| format!("{path}: {error}"))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn temp_dir(name: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!("xr1-evidence-{name}-{}", std::process::id()));
        fs::create_dir_all(&dir).unwrap();
        dir
    }

    #[test]
    fn write_json_overwrites_but_create_new_json_refuses() {
        let dir = temp_dir("write");
        let path = dir.join("latest.json");
        let first = serde_json::json!({ "n": 1 });
        let second = serde_json::json!({ "n": 2 });

        write_json(&path, &first).unwrap();
        // A pointer is meant to move.
        write_json(&path, &second).unwrap();
        let back: serde_json::Value = read_json(&path, "pointer").unwrap();
        assert_eq!(back["n"], 2);

        // An append-only record must not be silently replaced.
        assert!(create_new_json(&path, &first).is_err());

        let fresh = dir.join("record.json");
        assert!(create_new_json(&fresh, &first).is_ok());
        assert!(create_new_json(&fresh, &second).is_err());
        fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn append_json_line_accumulates_one_record_per_line() {
        let dir = temp_dir("append");
        let path = dir.join("events.jsonl");
        append_json_line(&path, &serde_json::json!({ "step": 1 })).unwrap();
        append_json_line(&path, &serde_json::json!({ "step": 2 })).unwrap();
        let text = fs::read_to_string(&path).unwrap();
        let lines = text.lines().collect::<Vec<_>>();
        assert_eq!(lines.len(), 2);
        assert!(lines[1].contains("\"step\":2"));
        fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn reading_a_missing_or_invalid_file_names_the_kind() {
        let dir = temp_dir("read");
        let missing = dir.join("nope.json");
        let error = read_json::<serde_json::Value>(&missing, "calibration").unwrap_err();
        assert!(error.contains("nope.json"));

        let bad = dir.join("bad.json");
        fs::write(&bad, b"not json").unwrap();
        let error = read_json::<serde_json::Value>(&bad, "calibration").unwrap_err();
        assert!(error.contains("calibration"));
        fs::remove_dir_all(&dir).ok();
    }
}
