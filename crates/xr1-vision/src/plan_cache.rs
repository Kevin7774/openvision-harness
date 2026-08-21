use crate::observation::read_state;
use crate::perception;
use crate::planning;
use crate::proposal::TaskProposal;
use crate::runtime::RuntimePaths;
use crate::support::evidence::create_new_json;
use serde::{Deserialize, Serialize};
use serde_json::{json, Value};
use std::fs;
use std::io::Write;
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::time::Instant;

const CACHE_SCHEMA_VERSION: u32 = 1;
const SAFETY_POLICY_VERSION: &str = "production-plan-policy-v1";
const REQUIRED_FILES: [&str; 5] = [
    "snapshot.json",
    "proposal.json",
    "plan.json",
    "diagnostics.json",
    "timings.json",
];

struct MoveItInputs<'a> {
    validator: &'a Path,
    srdf: &'a Path,
}

struct PlanCacheInputs<'a> {
    attempts_root: &'a Path,
    latest: &'a Path,
    proposal: &'a TaskProposal,
    urdf: &'a Path,
    moveit: Option<MoveItInputs<'a>>,
}

#[derive(Deserialize)]
struct LatestPaths {
    rgb_path: PathBuf,
    depth_path: PathBuf,
    camera_info_path: PathBuf,
    state_path: PathBuf,
}

struct PreparedAttempt {
    directory_name: String,
    snapshot_id: String,
    cache_key: String,
    snapshot: Value,
    proposal: Value,
}

#[derive(Debug, Serialize)]
pub struct AttemptReceipt {
    pub ok: bool,
    pub schema_version: u32,
    pub mode: &'static str,
    pub cache_hit: bool,
    pub snapshot_id: String,
    pub cache_key: String,
    pub attempt_path: PathBuf,
    pub plan_path: PathBuf,
}

pub fn run(
    runtime: &RuntimePaths,
    latest: &Path,
    proposal: TaskProposal,
    use_moveit: bool,
) -> Result<AttemptReceipt, String> {
    require_release_planner(cfg!(debug_assertions))?;
    let planner_proposal = proposal.clone();
    load_or_plan(
        PlanCacheInputs {
            attempts_root: &runtime.data_root().join("attempts"),
            latest,
            proposal: &proposal,
            urdf: runtime.arm_urdf(),
            moveit: use_moveit.then_some(MoveItInputs {
                validator: runtime.moveit_validator(),
                srdf: runtime.moveit_srdf(),
            }),
        },
        || compute(runtime, latest, planner_proposal, use_moveit),
    )
}

fn compute(
    runtime: &RuntimePaths,
    latest: &Path,
    proposal: TaskProposal,
    use_moveit: bool,
) -> Result<planning::PlanReport, String> {
    let request = proposal.grasp_request()?;
    let frame = perception::observe_object(latest, &request)?;
    let mut report = planning::plan(proposal, frame, runtime.arm_urdf())?;
    if use_moveit {
        planning::validate_with_moveit(
            &mut report,
            runtime.moveit_validator(),
            runtime.arm_urdf(),
            runtime.moveit_srdf(),
        )?;
    }
    Ok(report)
}

fn require_release_planner(debug_build: bool) -> Result<(), String> {
    if debug_build {
        Err("robot planning forbids Debug builds; use bin/xr1".into())
    } else {
        Ok(())
    }
}

fn load_or_plan<T, F>(inputs: PlanCacheInputs<'_>, planner: F) -> Result<AttemptReceipt, String>
where
    T: Serialize,
    F: FnOnce() -> Result<T, String>,
{
    let started = Instant::now();
    let prepared = prepare(&inputs)?;
    let key_ms = elapsed_ms(started);
    fs::create_dir_all(inputs.attempts_root)
        .map_err(|error| format!("{}: {error}", inputs.attempts_root.display()))?;

    let attempt = inputs.attempts_root.join(&prepared.directory_name);
    if attempt.exists() {
        return load_existing(&attempt, &prepared.cache_key, &prepared.snapshot_id);
    }

    let incomplete = inputs
        .attempts_root
        .join(format!(".{}.incomplete", prepared.directory_name));
    fs::create_dir(&incomplete).map_err(|error| {
        format!(
            "cannot start immutable plan attempt {}: {error}; another planner may be running or an incomplete attempt needs inspection",
            incomplete.display()
        )
    })?;

    create_new_json(&incomplete.join("snapshot.json"), &prepared.snapshot)?;
    create_new_json(&incomplete.join("proposal.json"), &prepared.proposal)?;

    let planning_started = Instant::now();
    let result = planner();
    let planning_ms = elapsed_ms(planning_started);
    let persist_started = Instant::now();
    let (plan, planner_error) = match result {
        Ok(report) => (
            serde_json::to_value(report).map_err(|error| error.to_string())?,
            None,
        ),
        Err(error) => (
            json!({
                "ok": false,
                "schema_version": CACHE_SCHEMA_VERSION,
                "mode": "planner_error",
                "error": error
            }),
            Some(error),
        ),
    };
    let plan_hash = sha256_json(&plan)?;
    let diagnostics = match &planner_error {
        None => json!({
            "ok": true,
            "schema_version": CACHE_SCHEMA_VERSION,
            "cache_key": prepared.cache_key,
            "plan_sha256": plan_hash,
            "cache_policy": "immutable_same_snapshot"
        }),
        Some(error) => {
            json!({
                "ok": false,
                "schema_version": CACHE_SCHEMA_VERSION,
                "cache_key": prepared.cache_key,
                "plan_sha256": plan_hash,
                "cache_policy": "immutable_same_snapshot",
                "planner_error": error
            })
        }
    };
    create_new_json(&incomplete.join("plan.json"), &plan)?;
    create_new_json(&incomplete.join("diagnostics.json"), &diagnostics)?;
    create_new_json(
        &incomplete.join("timings.json"),
        &json!({
            "schema_version": CACHE_SCHEMA_VERSION,
            "key_ms": key_ms,
            "planning_ms": planning_ms,
            "persist_ms": elapsed_ms(persist_started),
            "total_ms": elapsed_ms(started)
        }),
    )?;
    fs::rename(&incomplete, &attempt).map_err(|error| {
        format!(
            "failed to publish immutable plan attempt {}: {error}",
            attempt.display()
        )
    })?;
    fs::File::open(inputs.attempts_root)
        .and_then(|directory| directory.sync_all())
        .map_err(|error| format!("failed to sync {}: {error}", inputs.attempts_root.display()))?;

    if let Some(error) = planner_error {
        return Err(format!(
            "planner failed; immutable diagnostics saved in {}: {error}",
            attempt.display()
        ));
    }
    Ok(receipt(
        &attempt,
        prepared.snapshot_id,
        prepared.cache_key,
        false,
    ))
}

fn load_existing(
    attempt: &Path,
    expected_key: &str,
    snapshot_id: &str,
) -> Result<AttemptReceipt, String> {
    for name in REQUIRED_FILES {
        if !attempt.join(name).is_file() {
            return Err(format!(
                "immutable plan attempt {} is incomplete; refusing to overwrite or re-plan",
                attempt.display()
            ));
        }
    }
    let snapshot: Value = read_value(&attempt.join("snapshot.json"), "plan snapshot")?;
    let proposal: Value = read_value(&attempt.join("proposal.json"), "plan proposal")?;
    let diagnostics: Value = read_value(&attempt.join("diagnostics.json"), "plan diagnostics")?;
    let plan: Value = read_value(&attempt.join("plan.json"), "cached plan")?;
    let _: Value = read_value(&attempt.join("timings.json"), "plan timings")?;
    if snapshot.get("cache_key").and_then(Value::as_str) != Some(expected_key)
        || diagnostics.get("cache_key").and_then(Value::as_str) != Some(expected_key)
    {
        return Err(format!(
            "immutable plan attempt {} has a mismatched cache key; refusing to overwrite or re-plan",
            attempt.display()
        ));
    }
    let proposal_hash = sha256_json(&proposal)?;
    let plan_hash = sha256_json(&plan)?;
    if snapshot.get("proposal_sha256").and_then(Value::as_str) != Some(proposal_hash.as_str())
        || diagnostics.get("plan_sha256").and_then(Value::as_str) != Some(plan_hash.as_str())
    {
        return Err(format!(
            "immutable plan attempt {} failed its artifact hash check; refusing to overwrite or re-plan",
            attempt.display()
        ));
    }
    match diagnostics.get("ok").and_then(Value::as_bool) {
        Some(true) if plan.get("ok") == Some(&Value::Bool(true)) => Ok(receipt(
            attempt,
            snapshot_id.into(),
            expected_key.into(),
            true,
        )),
        Some(false) => Err(format!(
            "cached planner failure in {}: {}",
            attempt.display(),
            diagnostics
                .get("planner_error")
                .and_then(Value::as_str)
                .unwrap_or("unknown planner error")
        )),
        _ => Err(format!(
            "immutable plan attempt {} is corrupt; refusing to overwrite or re-plan",
            attempt.display()
        )),
    }
}

fn receipt(
    attempt: &Path,
    snapshot_id: String,
    cache_key: String,
    cache_hit: bool,
) -> AttemptReceipt {
    AttemptReceipt {
        ok: true,
        schema_version: CACHE_SCHEMA_VERSION,
        mode: "immutable_plan_attempt",
        cache_hit,
        snapshot_id,
        cache_key,
        attempt_path: attempt.into(),
        plan_path: attempt.join("plan.json"),
    }
}

fn prepare(inputs: &PlanCacheInputs<'_>) -> Result<PreparedAttempt, String> {
    let latest: LatestPaths = read_json(inputs.latest, "latest observation")?;
    let state = read_state(&latest.state_path)?;
    let latest_hash = sha256_file(inputs.latest)?;
    let rgb_hash = sha256_file(&latest.rgb_path)?;
    let depth_hash = sha256_file(&latest.depth_path)?;
    let camera_info_hash = sha256_file(&latest.camera_info_path)?;
    let state_hash = sha256_file(&latest.state_path)?;
    let joint_state_id = sha256_json(&state.joint_state)?;
    let tf_snapshot_id = sha256_json(&state.tf)?;
    let urdf_hash = sha256_file(inputs.urdf)?;
    let proposal = serde_json::to_value(inputs.proposal).map_err(|error| error.to_string())?;
    let proposal_hash = sha256_json(&proposal)?;
    let planner_revision = env!("XR1_PLANNER_REVISION");
    let safety_policy_hash = sha256_bytes(SAFETY_POLICY_VERSION.as_bytes())?;
    let calibration_hash = sha256_json(&json!({
        "camera_info_sha256": camera_info_hash,
        "tf_snapshot_id": tf_snapshot_id
    }))?;
    let moveit = match &inputs.moveit {
        Some(paths) => json!({
            "enabled": true,
            "validator_sha256": sha256_file(paths.validator)?,
            "srdf_sha256": sha256_file(paths.srdf)?
        }),
        None => json!({ "enabled": false }),
    };
    let key_material = json!({
        "schema_version": CACHE_SCHEMA_VERSION,
        "snapshot_id": state.frame_id,
        "sensor_stamp_ns": state.sensor_stamp_ns,
        "received_at_ns": state.received_at_ns,
        "rgb_sha256": rgb_hash,
        "depth_sha256": depth_hash,
        "camera_info_sha256": camera_info_hash,
        "joint_state_id": joint_state_id,
        "tf_snapshot_id": tf_snapshot_id,
        "urdf_sha256": urdf_hash,
        "calibration_sha256": calibration_hash,
        "calibration_inputs": ["camera_info", "image_time_tf"],
        "proposal_sha256": proposal_hash,
        "planner_revision": planner_revision,
        "safety_policy_sha256": safety_policy_hash,
        "moveit": moveit
    });
    let cache_key = sha256_json(&key_material)?;
    let snapshot = json!({
        "schema_version": CACHE_SCHEMA_VERSION,
        "cache_key": cache_key,
        "snapshot_id": state.frame_id,
        "joint_state_id": joint_state_id,
        "tf_snapshot_id": tf_snapshot_id,
        "urdf_sha256": urdf_hash,
        "calibration_sha256": calibration_hash,
        "proposal_sha256": proposal_hash,
        "planner_revision": planner_revision,
        "safety_policy_sha256": safety_policy_hash,
        "moveit": moveit,
        "files": {
            "latest": { "path": inputs.latest, "sha256": latest_hash },
            "rgb": { "path": latest.rgb_path, "sha256": rgb_hash },
            "depth": { "path": latest.depth_path, "sha256": depth_hash },
            "camera_info": { "path": latest.camera_info_path, "sha256": camera_info_hash },
            "state": { "path": latest.state_path, "sha256": state_hash },
            "urdf": { "path": inputs.urdf, "sha256": urdf_hash }
        }
    });
    let frame_id = safe_id(&state.frame_id);
    let prefix = cache_key.chars().take(16).collect::<String>();
    Ok(PreparedAttempt {
        directory_name: format!("attempt_{frame_id}_{prefix}"),
        snapshot_id: state.frame_id,
        cache_key,
        snapshot,
        proposal,
    })
}

fn safe_id(value: &str) -> String {
    let id = value
        .chars()
        .take(80)
        .map(|character| match character {
            'a'..='z' | 'A'..='Z' | '0'..='9' | '-' | '_' => character,
            _ => '_',
        })
        .collect::<String>();
    if id.is_empty() {
        "unnamed".into()
    } else {
        id
    }
}

fn sha256_json<T: Serialize>(value: &T) -> Result<String, String> {
    sha256_bytes(&serde_json::to_vec(value).map_err(|error| error.to_string())?)
}

fn sha256_file(path: &Path) -> Result<String, String> {
    let output = Command::new("openssl")
        .args(["dgst", "-sha256", "-r"])
        .arg(path)
        .output()
        .map_err(|error| format!("failed to hash {}: {error}", path.display()))?;
    digest_from_output(output, path)
}

fn sha256_bytes(bytes: &[u8]) -> Result<String, String> {
    let mut child = Command::new("openssl")
        .args(["dgst", "-sha256", "-r"])
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .map_err(|error| format!("failed to start openssl sha256: {error}"))?;
    child
        .stdin
        .as_mut()
        .ok_or_else(|| "openssl sha256 stdin is unavailable".to_string())?
        .write_all(bytes)
        .map_err(|error| format!("failed to write openssl sha256 input: {error}"))?;
    let output = child
        .wait_with_output()
        .map_err(|error| format!("failed to wait for openssl sha256: {error}"))?;
    digest_from_output(output, Path::new("<memory>"))
}

fn digest_from_output(output: std::process::Output, source: &Path) -> Result<String, String> {
    if !output.status.success() {
        return Err(format!(
            "failed to hash {}: {}",
            source.display(),
            String::from_utf8_lossy(&output.stderr).trim()
        ));
    }
    let stdout = String::from_utf8_lossy(&output.stdout);
    let digest = stdout
        .split_whitespace()
        .next()
        .ok_or_else(|| format!("openssl returned no hash for {}", source.display()))?;
    if digest.len() != 64 || !digest.bytes().all(|byte| byte.is_ascii_hexdigit()) {
        return Err(format!(
            "openssl returned an invalid hash for {}",
            source.display()
        ));
    }
    Ok(digest.to_ascii_lowercase())
}

fn read_json<T: for<'de> Deserialize<'de>>(path: &Path, kind: &str) -> Result<T, String> {
    serde_json::from_slice(&fs::read(path).map_err(|error| format!("{}: {error}", path.display()))?)
        .map_err(|error| format!("invalid {kind} {}: {error}", path.display()))
}

fn read_value(path: &Path, kind: &str) -> Result<Value, String> {
    read_json(path, kind)
}

fn elapsed_ms(started: Instant) -> f64 {
    started.elapsed().as_secs_f64() * 1_000.0
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::cell::Cell;

    fn fixture(name: &str) -> (PathBuf, PathBuf, PathBuf) {
        let root =
            std::env::temp_dir().join(format!("xr1-plan-cache-{name}-{}", std::process::id()));
        fs::create_dir_all(&root).unwrap();
        let rgb = root.join("rgb.png");
        let depth = root.join("depth.npy");
        let camera = root.join("camera_info.json");
        let state = root.join("state.json");
        let latest = root.join("latest.json");
        let urdf = root.join("robot.urdf");
        fs::write(&rgb, b"rgb").unwrap();
        fs::write(&depth, b"depth").unwrap();
        fs::write(&camera, b"{}").unwrap();
        write_state(&state, 0.1, 1.0);
        fs::write(&urdf, b"<robot/>").unwrap();
        fs::write(
            &latest,
            serde_json::to_vec(&json!({
                "rgb_path": rgb,
                "depth_path": depth,
                "camera_info_path": camera,
                "state_path": state
            }))
            .unwrap(),
        )
        .unwrap();
        (root, latest, urdf)
    }

    fn write_state(path: &Path, joint: f64, tf_x: f64) {
        fs::write(
            path,
            serde_json::to_vec(&json!({
                "frame_id": "frame-1",
                "sensor_stamp_ns": 10,
                "received_at_ns": 11,
                "joint_state": {
                    "received_at_ns": 11,
                    "positions_rad": { "right_arm_1_joint": joint }
                },
                "tf": {
                    "target_frame": "base_link",
                    "source_frame": "camera",
                    "translation_m": [tf_x, 0.0, 0.0],
                    "rotation_xyzw": [0.0, 0.0, 0.0, 1.0]
                }
            }))
            .unwrap(),
        )
        .unwrap();
    }

    fn inputs<'a>(
        root: &'a Path,
        latest: &'a Path,
        urdf: &'a Path,
        proposal: &'a TaskProposal,
    ) -> PlanCacheInputs<'a> {
        PlanCacheInputs {
            attempts_root: root,
            latest,
            proposal,
            urdf,
            moveit: None,
        }
    }

    #[test]
    fn same_snapshot_reads_plan_without_calling_planner_again() {
        let (root, latest, urdf) = fixture("reuse");
        let proposal = TaskProposal::yellow_block_grasp();
        let calls = Cell::new(0);
        let first = load_or_plan(inputs(&root, &latest, &urdf, &proposal), || {
            calls.set(calls.get() + 1);
            Ok(json!({ "ok": true, "candidates": [1] }))
        })
        .unwrap();
        let second = load_or_plan(inputs(&root, &latest, &urdf, &proposal), || {
            calls.set(calls.get() + 1);
            Ok(json!({ "ok": true, "candidates": [2] }))
        })
        .unwrap();
        assert_eq!(calls.get(), 1);
        assert!(!first.cache_hit);
        assert!(second.cache_hit);
        assert_eq!(first.plan_path, second.plan_path);

        let attempt = fs::read_dir(&root)
            .unwrap()
            .filter_map(Result::ok)
            .map(|entry| entry.path())
            .find(|path| {
                path.is_dir()
                    && path
                        .file_name()
                        .and_then(|name| name.to_str())
                        .is_some_and(|name| name.starts_with("attempt_"))
            })
            .unwrap();
        let mut snapshot: Value = read_value(&attempt.join("snapshot.json"), "snapshot").unwrap();
        snapshot["cache_key"] = Value::String("wrong".into());
        fs::write(
            attempt.join("snapshot.json"),
            serde_json::to_vec(&snapshot).unwrap(),
        )
        .unwrap();
        assert!(load_or_plan(inputs(&root, &latest, &urdf, &proposal), || {
            calls.set(calls.get() + 1);
            Ok(json!({ "ok": true }))
        })
        .is_err());
        assert_eq!(calls.get(), 1);
        fs::remove_dir_all(root).ok();
    }

    #[test]
    fn debug_planner_is_forbidden() {
        assert!(require_release_planner(true).is_err());
        assert!(require_release_planner(false).is_ok());
    }

    #[test]
    fn state_model_and_proposal_changes_invalidate_the_key() {
        let (root, latest, urdf) = fixture("key");
        let proposal = TaskProposal::yellow_block_grasp();
        let base = prepare(&inputs(&root, &latest, &urdf, &proposal))
            .unwrap()
            .cache_key;
        let state = root.join("state.json");

        write_state(&state, 0.2, 1.0);
        assert_ne!(
            base,
            prepare(&inputs(&root, &latest, &urdf, &proposal))
                .unwrap()
                .cache_key
        );
        write_state(&state, 0.1, 2.0);
        assert_ne!(
            base,
            prepare(&inputs(&root, &latest, &urdf, &proposal))
                .unwrap()
                .cache_key
        );
        write_state(&state, 0.1, 1.0);
        fs::write(&urdf, b"<robot name=\"changed\"/>").unwrap();
        assert_ne!(
            base,
            prepare(&inputs(&root, &latest, &urdf, &proposal))
                .unwrap()
                .cache_key
        );
        fs::write(&urdf, b"<robot/>").unwrap();
        let mut changed_proposal = proposal.clone();
        changed_proposal.command.push_str(" now");
        assert_ne!(
            base,
            prepare(&inputs(&root, &latest, &urdf, &changed_proposal))
                .unwrap()
                .cache_key
        );
        fs::remove_dir_all(root).ok();
    }
}
