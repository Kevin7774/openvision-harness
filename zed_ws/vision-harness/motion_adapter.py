#!/usr/bin/env python3
"""Execute one fresh, online-generated arm phase through astra_arm."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys
import time

sys.path.insert(0, "/home/astrabot/tools")

import astra_arm


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--plan", required=True)
    parser.add_argument("--phase", required=True, choices=("approach", "grasp", "return"))
    parser.add_argument("--go", action="store_true")
    args = parser.parse_args()

    plan_path = Path(args.plan)
    age = time.time() - plan_path.stat().st_mtime
    max_age = 1800.0 if args.phase == "return" else (300.0 if args.phase == "grasp" else 15.0)
    if age < 0.0 or age > max_age:
        raise SystemExit(f"REFUSED: plan is stale ({age:.1f}s > {max_age:.0f}s)")
    plan = json.loads(plan_path.read_text())
    if not plan.get("ok") or plan.get("mode") != "online_plan_dry_run":
        raise SystemExit("REFUSED: invalid online plan")
    feasible = [
        candidate
        for candidate in plan["candidates"]
        if candidate.get("ik_feasible")
        and candidate.get("grasp_feasible")
        and candidate.get("grasp_geometry", {}).get("feasible")
    ]
    if not feasible:
        raise SystemExit("REFUSED: no feasible candidate")
    candidate = min(
        feasible,
        key=lambda item: item["approach_ik"]["score"] + item["grasp_ik"]["score"],
    )
    geometry = candidate["grasp_geometry"]
    if geometry["pad_midpoint_error_m"] > 0.008:
        raise SystemExit("REFUSED: pad midpoint error exceeds 8mm")
    if geometry["object_axis_angle_rad"] > 0.35:
        raise SystemExit("REFUSED: closing axis is not aligned with object")
    if geometry["jaw_clearance_m"] < 0.004:
        raise SystemExit("REFUSED: jaw clearance is below 4mm")
    if not geometry.get("pads_bracket_object"):
        raise SystemExit("REFUSED: pad inner faces do not bracket the object")
    if geometry["fixed_pad_signed_from_object_m"] * geometry[
        "moving_pad_signed_from_object_m"
    ] >= 0.0:
        raise SystemExit("REFUSED: both pad inner faces are on the same object side")
    solution = candidate["approach_ik" if args.phase == "return" else f"{args.phase}_ik"]
    if solution["residual_m"] > 0.005:
        raise SystemExit("REFUSED: Cartesian residual exceeds 5mm")
    if solution["orientation_residual_rad"] > 0.05:
        raise SystemExit("REFUSED: orientation residual exceeds 0.05rad")
    if solution["min_limit_margin_rad"] < 0.10:
        raise SystemExit("REFUSED: joint limit margin below 0.10rad")
    speed = 0.08 if args.phase == "approach" else 0.12
    planned_start = (
        plan["current_joints_rad"]
        if args.phase == "approach"
        else candidate["approach_ik"]["joints_rad"]
    )
    robot = astra_arm.Robot()
    try:
        current = robot.joints()
        drift = max(abs(current[name] - value) for name, value in planned_start.items())
        if drift > 0.035:
            raise SystemExit(f"REFUSED: joint state drift {drift:.4f}rad > 0.035rad")
        target = plan["current_joints_rad"] if args.phase == "return" else solution["joints_rad"]
        robot.move(target, speed=speed, dry_run=not args.go)
    finally:
        robot.close()
    print(json.dumps({
        "ok": True,
        "executed": args.go,
        "phase": args.phase,
        "observation_frame_id": plan["observation_frame_id"],
        "candidate_rank": candidate["rank"],
        "plan_age_s": age,
        "joint_drift_rad": drift,
    }))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
