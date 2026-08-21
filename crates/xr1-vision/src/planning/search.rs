use super::types::{
    CandidateDiagnostics, ContactPair, GraspCandidate, GraspGeometryReport, SolutionReport,
};
use crate::kinematics::{Chain, GraspMetrics, Solution, OPEN_JAW_GAP_M, TIP_CENTER_M};
use crate::perception::ObjectGeometry;
use std::collections::HashSet;
use std::f64::consts::{PI, TAU};

const APPROACH_CLEARANCES_M: [f64; 3] = [0.06, 0.08, 0.10];
const COARSE_ROLL_STEP_RAD: f64 = PI / 6.0;
const FINE_ROLL_STEP_RAD: f64 = PI / 90.0;
const FINE_ROLL_RADIUS_STEPS: i32 = 3;
const FINE_SEED_COUNT: usize = 2;
const MAX_APPROACH_BRANCHES_PER_ORIENTATION: usize = 4;

#[derive(Clone, Copy)]
struct OrientationSample {
    family: usize,
    closing_axis: [f64; 3],
    roll_rad: f64,
    offset_rpy_rad: [f64; 3],
    preference_penalty: f64,
}

struct CandidatePair {
    score: f64,
    sample: OrientationSample,
    approach: Solution,
    grasp: Solution,
    metrics: GraspMetrics,
}

#[derive(Default)]
struct SearchStats {
    approach_ik_count: usize,
    grasp_ik_count: usize,
    geometry_feasible_count: usize,
    fine_orientation_candidates: usize,
}

pub(super) fn generate_candidates(
    chain: &Chain,
    names: &[String],
    current: &[f64],
    object: &ObjectGeometry,
    preferred_axis: Option<[f64; 3]>,
) -> Result<Vec<GraspCandidate>, String> {
    let orientations = coarse_orientation_samples(chain, current, object.axes_base, preferred_axis);
    if orientations.is_empty() {
        return Err("object geometry produced no horizontal grasp orientations".into());
    }
    let mut candidates = std::thread::scope(|scope| {
        let orientations = &orientations;
        APPROACH_CLEARANCES_M
            .map(|clearance| {
                scope.spawn(move || {
                    candidate_for_clearance(chain, names, current, object, orientations, clearance)
                })
            })
            .into_iter()
            .map(|worker| {
                worker
                    .join()
                    .map_err(|_| "planner clearance worker panicked".to_owned())
            })
            .collect::<Result<Vec<_>, _>>()
    })?;
    candidates.sort_by(|left, right| {
        left.score
            .unwrap_or(f64::INFINITY)
            .total_cmp(&right.score.unwrap_or(f64::INFINITY))
    });
    for (index, candidate) in candidates.iter_mut().enumerate() {
        candidate.rank = index + 1;
    }
    Ok(candidates)
}

fn candidate_for_clearance(
    chain: &Chain,
    names: &[String],
    current: &[f64],
    object: &ObjectGeometry,
    coarse_samples: &[OrientationSample],
    clearance_m: f64,
) -> GraspCandidate {
    let approach_position_m = [
        object.center_base_m[0],
        object.center_base_m[1],
        object.center_base_m[2] + clearance_m,
    ];
    let mut stats = SearchStats::default();
    let mut pairs = coarse_samples
        .iter()
        .filter_map(|sample| {
            evaluate_orientation(
                chain,
                names,
                current,
                object,
                approach_position_m,
                *sample,
                &mut stats,
            )
        })
        .collect::<Vec<_>>();
    pairs.sort_by(|left, right| left.score.total_cmp(&right.score));

    let fine_seeds = pairs
        .iter()
        .take(FINE_SEED_COUNT)
        .map(|pair| pair.sample)
        .collect::<Vec<_>>();
    let mut seen = coarse_samples
        .iter()
        .map(orientation_key)
        .collect::<HashSet<_>>();
    for seed in fine_seeds {
        for step in -FINE_ROLL_RADIUS_STEPS..=FINE_ROLL_RADIUS_STEPS {
            if step == 0 {
                continue;
            }
            let roll_rad = normalize_roll(seed.roll_rad + f64::from(step) * FINE_ROLL_STEP_RAD);
            let Some(offset_rpy_rad) =
                chain.offset_for_top_down_closing_axis(current, seed.closing_axis, roll_rad)
            else {
                continue;
            };
            let sample = OrientationSample {
                roll_rad,
                offset_rpy_rad,
                ..seed
            };
            if !seen.insert(orientation_key(&sample)) {
                continue;
            }
            stats.fine_orientation_candidates += 1;
            if let Some(pair) = evaluate_orientation(
                chain,
                names,
                current,
                object,
                approach_position_m,
                sample,
                &mut stats,
            ) {
                pairs.push(pair);
            }
        }
    }
    pairs.sort_by(|left, right| left.score.total_cmp(&right.score));
    build_candidate(
        object,
        approach_position_m,
        clearance_m,
        coarse_samples.len(),
        stats,
        pairs.into_iter().next(),
    )
}

fn evaluate_orientation(
    chain: &Chain,
    names: &[String],
    current: &[f64],
    object: &ObjectGeometry,
    approach_position_m: [f64; 3],
    sample: OrientationSample,
    stats: &mut SearchStats,
) -> Option<CandidatePair> {
    let approach_solutions = chain.solve_position_candidates_with_reference(
        approach_position_m,
        current,
        current,
        sample.offset_rpy_rad,
    );
    stats.approach_ik_count += approach_solutions.len();
    let mut best = None;
    for approach in approach_solutions
        .into_iter()
        .take(MAX_APPROACH_BRANCHES_PER_ORIENTATION)
    {
        let Some(seed) = solution_vector(names, &approach) else {
            continue;
        };
        let grasp_solutions = chain.solve_position_candidates_with_reference(
            object.center_base_m,
            &seed,
            current,
            sample.offset_rpy_rad,
        );
        stats.grasp_ik_count += grasp_solutions.len();
        for grasp in grasp_solutions {
            let metrics = chain.grasp_metrics(
                &grasp,
                object.center_base_m,
                object.axes_base,
                object.extents_m,
            );
            if !metrics.feasible {
                continue;
            }
            stats.geometry_feasible_count += 1;
            let score = approach.score + grasp.score + sample.preference_penalty;
            if best
                .as_ref()
                .map(|pair: &CandidatePair| score < pair.score)
                .unwrap_or(true)
            {
                best = Some(CandidatePair {
                    score,
                    sample,
                    approach: approach.clone(),
                    grasp,
                    metrics,
                });
            }
            break;
        }
    }
    best
}

fn build_candidate(
    object: &ObjectGeometry,
    approach_position_m: [f64; 3],
    clearance_m: f64,
    coarse_orientation_candidates: usize,
    stats: SearchStats,
    pair: Option<CandidatePair>,
) -> GraspCandidate {
    let diagnostics = CandidateDiagnostics {
        coarse_orientation_candidates,
        fine_orientation_candidates: stats.fine_orientation_candidates,
        approach_ik_count: stats.approach_ik_count,
        grasp_ik_count: stats.grasp_ik_count,
        geometry_feasible_count: stats.geometry_feasible_count,
    };
    let Some(pair) = pair else {
        return empty_candidate(object, approach_position_m, clearance_m, diagnostics);
    };
    let quality = contact_quality(&pair.metrics);
    GraspCandidate {
        rank: 0,
        object_id: object.object_id.clone(),
        strategy: "closing_axis_full_roll_coarse_to_fine".into(),
        approach_position_m,
        grasp_position_m: object.center_base_m,
        closing_axis_base: Some(pair.metrics.closing_axis),
        roll_rad: Some(pair.sample.roll_rad),
        approach_direction_base: [0.0, 0.0, -1.0],
        contacts: Some(ContactPair {
            fixed_pad_inner_base_m: pair.metrics.fixed_pad_inner_m,
            moving_pad_inner_base_m: pair.metrics.moving_pad_inner_m,
        }),
        required_gripper_width_m: Some(pair.metrics.object_width_m),
        clearance_m,
        approach_ik: Some(solution_report(&pair.approach)),
        grasp_ik: Some(solution_report(&pair.grasp)),
        ik_feasible: true,
        grasp_feasible: pair.metrics.feasible,
        joint_limit_margin_rad: Some(
            pair.approach
                .min_limit_margin_rad
                .min(pair.grasp.min_limit_margin_rad),
        ),
        collision_margin_m: None,
        table_clearance_m: None,
        moveit_validated: false,
        self_collision_free: None,
        path_collision_free: None,
        contact_quality: Some(quality),
        score: Some(pair.score - quality),
        diagnostics,
        grasp_geometry: Some(grasp_geometry_report(&pair.metrics)),
        uses_previous_absolute_pose: false,
    }
}

fn empty_candidate(
    object: &ObjectGeometry,
    approach_position_m: [f64; 3],
    clearance_m: f64,
    diagnostics: CandidateDiagnostics,
) -> GraspCandidate {
    GraspCandidate {
        rank: 0,
        object_id: object.object_id.clone(),
        strategy: "closing_axis_full_roll_coarse_to_fine".into(),
        approach_position_m,
        grasp_position_m: object.center_base_m,
        closing_axis_base: None,
        roll_rad: None,
        approach_direction_base: [0.0, 0.0, -1.0],
        contacts: None,
        required_gripper_width_m: None,
        clearance_m,
        approach_ik: None,
        grasp_ik: None,
        ik_feasible: false,
        grasp_feasible: false,
        joint_limit_margin_rad: None,
        collision_margin_m: None,
        table_clearance_m: None,
        moveit_validated: false,
        self_collision_free: None,
        path_collision_free: None,
        contact_quality: None,
        score: None,
        diagnostics,
        grasp_geometry: None,
        uses_previous_absolute_pose: false,
    }
}

fn coarse_orientation_samples(
    chain: &Chain,
    current: &[f64],
    object_axes: [[f64; 3]; 3],
    preferred: Option<[f64; 3]>,
) -> Vec<OrientationSample> {
    let mut samples = Vec::new();
    let mut family = 0;
    for axis in object_axes {
        if axis[2].abs() > 0.35 {
            continue;
        }
        for closing_axis in [axis, [-axis[0], -axis[1], -axis[2]]] {
            let preference_penalty = preferred
                .map(|value| closing_axis_penalty(closing_axis, value))
                .unwrap_or(0.0);
            for step in 0..12 {
                let roll_rad = f64::from(step) * COARSE_ROLL_STEP_RAD;
                if let Some(offset_rpy_rad) =
                    chain.offset_for_top_down_closing_axis(current, closing_axis, roll_rad)
                {
                    samples.push(OrientationSample {
                        family,
                        closing_axis,
                        roll_rad,
                        offset_rpy_rad,
                        preference_penalty,
                    });
                }
            }
            family += 1;
        }
    }
    samples.sort_by(|left, right| {
        left.preference_penalty
            .total_cmp(&right.preference_penalty)
            .then_with(|| left.family.cmp(&right.family))
            .then_with(|| left.roll_rad.total_cmp(&right.roll_rad))
    });
    samples
}

fn closing_axis_penalty(axis: [f64; 3], preferred: [f64; 3]) -> f64 {
    let axis_norm = (axis[0] * axis[0] + axis[1] * axis[1]).sqrt();
    let preferred_norm = (preferred[0] * preferred[0] + preferred[1] * preferred[1]).sqrt();
    let dot = (axis[0] * preferred[0] + axis[1] * preferred[1]) / (axis_norm * preferred_norm);
    dot.clamp(-1.0, 1.0).acos() * 2.0
}

fn solution_vector(names: &[String], solution: &Solution) -> Option<Vec<f64>> {
    names
        .iter()
        .map(|name| {
            solution
                .joints
                .iter()
                .find(|(joint, _)| joint == name)
                .map(|(_, value)| *value)
        })
        .collect()
}

fn solution_report(solution: &Solution) -> SolutionReport {
    SolutionReport {
        residual_m: solution.residual_m,
        orientation_residual_rad: solution.orientation_residual_rad,
        max_delta_rad: solution.max_delta_rad,
        score: solution.score,
        floor_clear: solution.floor_clear,
        orientation_offset_rpy_rad: solution.orientation_offset_rpy_rad,
        min_limit_margin_rad: solution.min_limit_margin_rad,
        min_tip_z_m: solution.min_tip_z_m,
        tool_tip_center_m: TIP_CENTER_M,
        joints_rad: solution.joints.iter().cloned().collect(),
    }
}

fn grasp_geometry_report(metrics: &GraspMetrics) -> GraspGeometryReport {
    GraspGeometryReport {
        pad_midpoint_error_m: metrics.pad_midpoint_error_m,
        closing_axis_base: metrics.closing_axis,
        object_axis_angle_rad: metrics.object_axis_angle_rad,
        object_width_m: metrics.object_width_m,
        open_jaw_gap_m: OPEN_JAW_GAP_M,
        jaw_clearance_m: metrics.jaw_clearance_m,
        fixed_pad_inner_base_m: metrics.fixed_pad_inner_m,
        moving_pad_inner_base_m: metrics.moving_pad_inner_m,
        fixed_pad_signed_from_object_m: metrics.fixed_pad_signed_m,
        moving_pad_signed_from_object_m: metrics.moving_pad_signed_m,
        pads_bracket_object: metrics.pads_bracket_object,
        feasible: metrics.feasible,
    }
}

fn contact_quality(metrics: &GraspMetrics) -> f64 {
    let position = (1.0 - metrics.pad_midpoint_error_m / 0.008).clamp(0.0, 1.0);
    let alignment = (1.0 - metrics.object_axis_angle_rad / 0.35).clamp(0.0, 1.0);
    let clearance = (metrics.jaw_clearance_m / 0.020).clamp(0.0, 1.0);
    let bracket = if metrics.pads_bracket_object {
        1.0
    } else {
        0.0
    };
    (position + alignment + clearance + bracket) * 0.25
}

fn normalize_roll(value: f64) -> f64 {
    value.rem_euclid(TAU)
}

fn orientation_key(sample: &OrientationSample) -> (usize, i64) {
    (
        sample.family,
        (normalize_roll(sample.roll_rad) * 1e9).round() as i64,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn roll_normalization_covers_one_period() {
        assert!((normalize_roll(-FINE_ROLL_STEP_RAD) - (TAU - FINE_ROLL_STEP_RAD)).abs() < 1e-12);
        assert!((normalize_roll(TAU + 0.25) - 0.25).abs() < 1e-12);
    }

    #[test]
    fn matching_closing_axis_has_no_preference_penalty() {
        assert!(closing_axis_penalty([1.0, 0.0, 0.0], [2.0, 0.0, 0.0]) < 1e-12);
        assert!(closing_axis_penalty([-1.0, 0.0, 0.0], [1.0, 0.0, 0.0]) > 6.0);
    }
}
