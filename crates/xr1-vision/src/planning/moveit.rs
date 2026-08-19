use super::types::{MoveItValidationSummary, PlanReport};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, HashMap, HashSet};
use std::io::Write;
use std::path::Path;
use std::process::{Command, Stdio};

const MOVEIT_REQUEST_SCHEMA_VERSION: u32 = 1;
const MOVEIT_PATH_SAMPLES: usize = 40;

// Measured table top is 0.790 m (2026-08-14 status record). The box spans the
// manipulation workspace only, so the mobile base is not introduced as a
// permanent table collision.
const TABLE_BOX_CENTER_M: [f64; 3] = [0.50, 0.0, 0.740];
const TABLE_BOX_SIZE_M: [f64; 3] = [0.80, 1.20, 0.100];

#[derive(Serialize)]
struct ValidationRequest<'a> {
    schema_version: u32,
    urdf_path: &'a Path,
    srdf_path: &'a Path,
    group_name: &'static str,
    path_samples: usize,
    current_joints_rad: &'a BTreeMap<String, f64>,
    world_boxes: Vec<WorldBox>,
    candidates: Vec<CandidateRequest<'a>>,
}

#[derive(Serialize)]
struct WorldBox {
    object_id: &'static str,
    frame_id: &'static str,
    center_m: [f64; 3],
    size_m: [f64; 3],
}

#[derive(Serialize)]
struct CandidateRequest<'a> {
    rank: usize,
    approach_joints_rad: &'a BTreeMap<String, f64>,
    grasp_joints_rad: &'a BTreeMap<String, f64>,
}

#[derive(Deserialize)]
struct ValidationResponse {
    ok: bool,
    schema_version: u32,
    model_name: String,
    group_name: String,
    path_samples: usize,
    world_object_count: usize,
    candidates: Vec<CandidateResponse>,
}

#[derive(Deserialize)]
struct CandidateResponse {
    rank: usize,
    bounds_ok: bool,
    approach_self_collision_free: bool,
    grasp_self_collision_free: bool,
    path_collision_free: bool,
    minimum_self_distance_m: f64,
    minimum_world_distance_m: f64,
}

pub fn validate_with_moveit(
    report: &mut PlanReport,
    validator_path: &Path,
    urdf_path: &Path,
    srdf_path: &Path,
) -> Result<(), String> {
    let candidates = report
        .candidates
        .iter()
        .filter_map(|candidate| {
            Some(CandidateRequest {
                rank: candidate.rank,
                approach_joints_rad: &candidate.approach_ik.as_ref()?.joints_rad,
                grasp_joints_rad: &candidate.grasp_ik.as_ref()?.joints_rad,
            })
        })
        .collect::<Vec<_>>();
    if candidates.is_empty() {
        return Err("MoveIt validation requires at least one IK-complete candidate".into());
    }
    let request = ValidationRequest {
        schema_version: MOVEIT_REQUEST_SCHEMA_VERSION,
        urdf_path,
        srdf_path,
        group_name: "right_arm",
        path_samples: MOVEIT_PATH_SAMPLES,
        current_joints_rad: &report.current_joints_rad,
        world_boxes: vec![WorldBox {
            object_id: "table",
            frame_id: "base_link",
            center_m: TABLE_BOX_CENTER_M,
            size_m: TABLE_BOX_SIZE_M,
        }],
        candidates,
    };
    let input = serde_json::to_vec(&request).map_err(|error| error.to_string())?;
    let mut child = Command::new(validator_path)
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .map_err(|error| {
            format!(
                "failed to start MoveIt validator {}: {error}",
                validator_path.display()
            )
        })?;
    child
        .stdin
        .as_mut()
        .ok_or_else(|| "MoveIt validator stdin is unavailable".to_string())?
        .write_all(&input)
        .map_err(|error| format!("failed to write MoveIt request: {error}"))?;
    let output = child
        .wait_with_output()
        .map_err(|error| format!("failed to wait for MoveIt validator: {error}"))?;
    if !output.status.success() {
        let stderr = String::from_utf8_lossy(&output.stderr);
        return Err(format!(
            "MoveIt validator exited with {}: {}",
            output.status,
            stderr.trim()
        ));
    }
    let response: ValidationResponse = serde_json::from_slice(&output.stdout)
        .map_err(|error| format!("invalid MoveIt validator response: {error}"))?;
    validate_response(&response, report)?;

    let by_rank = response
        .candidates
        .into_iter()
        .map(|candidate| (candidate.rank, candidate))
        .collect::<HashMap<_, _>>();
    for candidate in &mut report.candidates {
        let Some(result) = by_rank.get(&candidate.rank) else {
            continue;
        };
        candidate.moveit_validated = true;
        candidate.self_collision_free =
            Some(result.approach_self_collision_free && result.grasp_self_collision_free);
        candidate.path_collision_free = Some(result.path_collision_free);
        candidate.collision_margin_m = Some(result.minimum_self_distance_m);
        candidate.table_clearance_m = Some(result.minimum_world_distance_m);
        candidate.ik_feasible &= result.bounds_ok;
    }
    report.moveit_validation = Some(MoveItValidationSummary {
        backend: "moveit2_planning_scene".into(),
        model_name: response.model_name,
        group_name: response.group_name,
        path_samples: response.path_samples,
        world_object_count: response.world_object_count,
    });
    Ok(())
}

fn validate_response(response: &ValidationResponse, report: &PlanReport) -> Result<(), String> {
    if !response.ok {
        return Err("MoveIt validator returned ok=false".into());
    }
    if response.schema_version != MOVEIT_REQUEST_SCHEMA_VERSION {
        return Err(format!(
            "unsupported MoveIt response schema {}",
            response.schema_version
        ));
    }
    if response.group_name != "right_arm" || response.path_samples != MOVEIT_PATH_SAMPLES {
        return Err("MoveIt validator response does not match the request".into());
    }
    let expected = report
        .candidates
        .iter()
        .filter(|candidate| candidate.approach_ik.is_some() && candidate.grasp_ik.is_some())
        .map(|candidate| candidate.rank)
        .collect::<HashSet<_>>();
    let actual = response
        .candidates
        .iter()
        .map(|candidate| candidate.rank)
        .collect::<HashSet<_>>();
    if expected != actual || actual.len() != response.candidates.len() {
        return Err(
            "MoveIt validator response candidate ranks are incomplete or duplicated".into(),
        );
    }
    if response.candidates.iter().any(|candidate| {
        !candidate.minimum_self_distance_m.is_finite()
            || !candidate.minimum_world_distance_m.is_finite()
    }) {
        return Err("MoveIt validator returned non-finite collision distance".into());
    }
    Ok(())
}
