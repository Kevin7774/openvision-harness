#!/usr/bin/env python3
"""Execute exactly one Rust-approved visual-servo joint microstep."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import time


MAX_ENVELOPE_AGE_S = 3.0
MAX_START_DRIFT_RAD = 0.01
MAX_SERVO_STEP_RAD = 0.05
SERVO_SPEED_RAD_S = 0.10
# Match the live-start drift bound: a larger endpoint miss means the requested
# microstep was not physically established and the loop must stop after seeing
# a new frame. This is stricter than astra_arm's generic 0.05 rad warning.
MAX_ARRIVAL_ERROR_RAD = 0.01


class ServoRefused(ValueError):
    """The Rust envelope or live start state cannot authorize one microstep."""


def load_envelope(path: Path) -> dict:
    try:
        envelope = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ServoRefused(f"cannot read servo envelope: {exc}") from exc
    validate_envelope(envelope)
    return envelope


def validate_envelope(envelope: dict, now_ns: int | None = None) -> None:
    if envelope.get("ok") is not True or envelope.get("mode") != "visual_servo_proposal":
        raise ServoRefused("not a Rust visual-servo proposal")
    if envelope.get("schema_version") != 1:
        raise ServoRefused(f"unsupported servo schema {envelope.get('schema_version')!r}")
    if envelope.get("ready_for_execution_adapter") is not True:
        raise ServoRefused("Rust safety gate did not approve this proposal")
    if envelope.get("execution_authorized") is not False:
        raise ServoRefused("proposal must leave physical authorization to this adapter")

    generated_at_ns = envelope.get("generated_at_ns")
    if not isinstance(generated_at_ns, int) or generated_at_ns <= 0:
        raise ServoRefused("proposal has no valid generated_at_ns")
    now_ns = time.time_ns() if now_ns is None else now_ns
    age_s = (now_ns - generated_at_ns) / 1e9
    if not 0.0 <= age_s <= MAX_ENVELOPE_AGE_S:
        raise ServoRefused(
            f"servo proposal is stale ({age_s:.3f}s, allowed 0..{MAX_ENVELOPE_AGE_S:.0f}s)"
        )

    safety = envelope.get("safety")
    proposal = envelope.get("proposal")
    if not isinstance(safety, dict) or safety.get("approved") is not True:
        raise ServoRefused("missing approved Rust safety report")
    if not isinstance(proposal, dict):
        raise ServoRefused("missing Rust servo proposal")
    current = safety.get("current_joints_rad")
    target = safety.get("target_joints_rad")
    deltas = proposal.get("joint_delta_rad")
    controlled = proposal.get("controlled_joints")
    if not all(isinstance(value, dict) for value in (current, target, deltas)):
        raise ServoRefused("servo joint maps are missing")
    if not isinstance(controlled, list) or len(controlled) != 3 or len(set(controlled)) != 3:
        raise ServoRefused("servo proposal must name exactly three unique joints")
    if set(current) != set(target) or set(deltas) != set(controlled):
        raise ServoRefused("servo joint maps are inconsistent")
    unknown_controlled = sorted(set(controlled) - set(current))
    if unknown_controlled:
        raise ServoRefused(
            f"controlled joints are absent from the approved arm: {unknown_controlled}"
        )
    if not all(
        isinstance(value, (int, float)) and math.isfinite(value)
        for mapping in (current, target, deltas)
        for value in mapping.values()
    ):
        raise ServoRefused("servo joint maps contain non-finite values")

    for joint in current:
        expected_delta = deltas.get(joint, 0.0)
        actual_delta = target[joint] - current[joint]
        if abs(actual_delta - expected_delta) > 1e-9:
            raise ServoRefused(f"target for {joint} does not match the approved delta")
        if abs(actual_delta) > MAX_SERVO_STEP_RAD + 1e-12:
            raise ServoRefused(f"target for {joint} exceeds the 0.05rad microstep ceiling")


def execute(
    envelope: dict,
    go: bool,
    *,
    robot_factory=None,
    now_ns: int | None = None,
) -> dict:
    validate_envelope(envelope, now_ns=now_ns)
    if robot_factory is None:
        import astra_arm

        robot_factory = astra_arm.Robot

    safety = envelope["safety"]
    planned_start = safety["current_joints_rad"]
    target = safety["target_joints_rad"]
    robot = robot_factory()
    try:
        current = robot.joints()
        missing = sorted(set(planned_start) - set(current))
        if missing:
            raise ServoRefused(f"live joint state is missing {missing}")
        drift = max(abs(current[name] - value) for name, value in planned_start.items())
        if drift > MAX_START_DRIFT_RAD:
            raise ServoRefused(
                f"joint state drift {drift:.4f}rad exceeds {MAX_START_DRIFT_RAD:.2f}rad"
            )
        achieved = robot.move(target, speed=SERVO_SPEED_RAD_S, dry_run=not go)
    finally:
        robot.close()

    achieved_delta = {
        joint: achieved[joint] - current[joint]
        for joint in envelope["proposal"]["controlled_joints"]
    }
    requested_delta = envelope["proposal"]["joint_delta_rad"]
    arrival_error = max(
        abs(achieved_delta[joint] - requested_delta[joint])
        for joint in envelope["proposal"]["controlled_joints"]
    )
    motion_completed = not go or arrival_error <= MAX_ARRIVAL_ERROR_RAD
    return {
        "ok": motion_completed,
        "schema_version": 1,
        "mode": "visual_servo_microstep",
        "executed": go,
        "observation_frame_id": envelope["observation_frame_id"],
        "joint_start_drift_rad": drift,
        "requested_joint_delta_rad": requested_delta,
        "achieved_joint_delta_rad": achieved_delta,
        "max_joint_arrival_error_rad": arrival_error,
        "motion_completed": motion_completed,
        "requires_reobservation": go,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--proposal", required=True)
    parser.add_argument("--go", action="store_true")
    args = parser.parse_args()
    try:
        report = execute(load_envelope(Path(args.proposal)), args.go)
    except ServoRefused as exc:
        raise SystemExit(f"REFUSED: {exc}") from exc
    print(json.dumps(report))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
