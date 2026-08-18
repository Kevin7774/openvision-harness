#!/usr/bin/env python3
"""Execute one fresh, online-generated arm phase through astra_arm."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import astra_arm


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--plan", required=True)
    parser.add_argument("--phase", required=True, choices=("approach", "grasp", "return"))
    parser.add_argument("--go", action="store_true")
    args = parser.parse_args()

    plan_path = Path(args.plan)
    age = time.time() - plan_path.stat().st_mtime
    max_age = 1800.0 if args.phase == "return" else (15.0 if args.phase == "approach" else 300.0)
    if age > max_age:
        raise SystemExit(f"REFUSED: plan is stale ({age:.1f}s > {max_age:.0f}s)")
    plan = json.loads(plan_path.read_text())
    if not plan.get("ok") or plan.get("mode") != "online_plan_dry_run":
        raise SystemExit("REFUSED: invalid online plan")
    feasible = [candidate for candidate in plan["candidates"] if candidate["ik_feasible"]]
    if not feasible:
        raise SystemExit("REFUSED: no feasible candidate")
    candidate = min(
        feasible,
        key=lambda item: item["approach_ik"]["score"] + item["grasp_ik"]["score"],
    )
    solution = candidate["approach_ik" if args.phase == "return" else f"{args.phase}_ik"]
    if solution["residual_m"] > 0.005:
        raise SystemExit("REFUSED: Cartesian residual exceeds 5mm")
    if solution["orientation_residual_rad"] > 0.05:
        raise SystemExit("REFUSED: orientation residual exceeds 0.05rad")
    if solution["min_limit_margin_rad"] < 0.10:
        raise SystemExit("REFUSED: joint limit margin below 0.10rad")
    speed = 0.12 if args.phase in ("grasp", "return") else 0.20
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
