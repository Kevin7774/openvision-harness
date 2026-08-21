#!/usr/bin/env python3
"""Validate and execute one phase from an immutable Rust plan attempt."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time


PHASE_MAX_AGE_S = {
    "approach": 15.0,
    "grasp": 300.0,
    "return": 1800.0,
}
ATTEMPT_FILES = (
    "snapshot.json",
    "proposal.json",
    "plan.json",
    "diagnostics.json",
    "timings.json",
)


def load_plan(attempt: Path) -> dict:
    if not attempt.is_dir() or not all((attempt / name).is_file() for name in ATTEMPT_FILES):
        raise ValueError("incomplete immutable plan attempt")
    plan = json.loads((attempt / "plan.json").read_text(encoding="utf-8"))
    if not plan.get("ok") or plan.get("mode") != "online_plan_dry_run":
        raise ValueError("invalid online plan")
    if plan.get("schema_version", 1) not in (1, 2):
        raise ValueError(f"unsupported plan schema {plan.get('schema_version')!r}")
    if not isinstance(plan.get("candidates"), list):
        raise ValueError("plan candidates must be a list")
    return plan


def plan_age_seconds(plan: dict, path: Path, now_s: float | None = None) -> float:
    now_s = time.time() if now_s is None else now_s
    received_at_ns = plan.get("observation_received_at_ns")
    if isinstance(received_at_ns, int) and received_at_ns > 0:
        return now_s - received_at_ns / 1e9
    return now_s - path.stat().st_mtime


def candidate_score(candidate: dict) -> float:
    score = candidate.get("score")
    if isinstance(score, (int, float)):
        return float(score)
    return float(candidate["approach_ik"]["score"]) + float(candidate["grasp_ik"]["score"])


def select_candidate(plan: dict) -> dict:
    moveit_required = plan.get("moveit_validation") is not None
    feasible = [
        candidate
        for candidate in plan["candidates"]
        if candidate.get("ik_feasible") and candidate.get("grasp_feasible")
        and (
            not moveit_required
            or (
                candidate.get("moveit_validated")
                and candidate.get("self_collision_free")
                and candidate.get("path_collision_free")
            )
        )
    ]
    if not feasible:
        requirement = "IK, grasp geometry and MoveIt collision" if moveit_required else "IK and grasp geometry"
        raise ValueError(f"no candidate passes {requirement}")
    return min(feasible, key=candidate_score)


def phase_solution(candidate: dict, phase: str) -> dict:
    key = "approach_ik" if phase == "return" else f"{phase}_ik"
    solution = candidate.get(key)
    if not isinstance(solution, dict):
        raise ValueError(f"candidate has no {key}")
    if solution["residual_m"] > 0.005:
        raise ValueError("Cartesian residual exceeds 5mm")
    if solution["orientation_residual_rad"] > 0.05:
        raise ValueError("orientation residual exceeds 0.05rad")
    if solution["min_limit_margin_rad"] < 0.10:
        raise ValueError("joint limit margin below 0.10rad")
    return solution


def execute(attempt: Path, phase: str, go: bool) -> dict:
    try:
        plan = load_plan(attempt)
        plan_path = attempt / "plan.json"
        age = plan_age_seconds(plan, plan_path)
        max_age = PHASE_MAX_AGE_S[phase]
        if age < 0.0 or age > max_age:
            raise ValueError(f"plan is stale ({age:.1f}s, allowed 0..{max_age:.0f}s)")
        candidate = select_candidate(plan)
        solution = phase_solution(candidate, phase)
    except (KeyError, TypeError, ValueError, OSError, json.JSONDecodeError) as exc:
        raise SystemExit(f"REFUSED: {exc}") from exc

    import astra_arm

    planned_start = (
        plan["current_joints_rad"]
        if phase == "approach"
        else candidate["approach_ik"]["joints_rad"]
    )
    speed = 0.12 if phase in ("grasp", "return") else 0.20
    robot = astra_arm.Robot()
    try:
        current = robot.joints()
        drift = max(abs(current[name] - value) for name, value in planned_start.items())
        if drift > 0.035:
            raise SystemExit(f"REFUSED: joint state drift {drift:.4f}rad > 0.035rad")
        target = plan["current_joints_rad"] if phase == "return" else solution["joints_rad"]
        robot.move(target, speed=speed, dry_run=not go)
    finally:
        robot.close()
    return {
        "ok": True,
        "executed": go,
        "phase": phase,
        "observation_frame_id": plan["observation_frame_id"],
        "candidate_rank": candidate["rank"],
        "plan_age_s": age,
        "joint_drift_rad": drift,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--attempt", required=True)
    parser.add_argument("--phase", required=True, choices=tuple(PHASE_MAX_AGE_S))
    parser.add_argument("--go", action="store_true")
    args = parser.parse_args()
    print(json.dumps(execute(Path(args.attempt), args.phase, args.go)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
