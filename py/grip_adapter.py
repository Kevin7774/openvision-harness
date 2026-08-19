#!/usr/bin/env python3
"""Observe a G2 gripper or execute exactly one Rust-approved jaw increment."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import time


GRIP_CMD = "/rm_{}/rm_driver/teleop_gripper_float"
GRIP_STATE = "/qg_robot/gripper_{}_state"
MAX_ENVELOPE_AGE_S = 1.0
MAX_POSITION_AGE_S = 0.5
MAX_POSITION_DRIFT_MM = 40
MAX_CLOSE_INCREMENT = 0.05
MAX_COMMAND_SETTLE_S = 5.0
MIN_COMMAND_MOTION_MM = 1


class GripRefused(ValueError):
    """The live gripper state cannot authorize the requested increment."""


def load_envelope(path: Path) -> dict:
    try:
        envelope = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise GripRefused(f"cannot read grip envelope: {exc}") from exc
    validate_envelope(envelope)
    return envelope


def validate_envelope(envelope: dict, now_ns: int | None = None) -> None:
    if envelope.get("ok") is not True or envelope.get("mode") != "tactile_grip_increment":
        raise GripRefused("not a tactile grip increment envelope")
    if envelope.get("schema_version") != 1:
        raise GripRefused(f"unsupported grip schema {envelope.get('schema_version')!r}")
    generated_at_ns = envelope.get("generated_at_ns")
    if not isinstance(generated_at_ns, int) or generated_at_ns <= 0:
        raise GripRefused("grip envelope has no valid generated_at_ns")
    now_ns = time.time_ns() if now_ns is None else now_ns
    age_s = (now_ns - generated_at_ns) / 1e9
    if not 0.0 <= age_s <= MAX_ENVELOPE_AGE_S:
        raise GripRefused(
            f"grip envelope is stale ({age_s:.3f}s, allowed 0..{MAX_ENVELOPE_AGE_S:.0f}s)"
        )
    expires_at_ns = envelope.get("expires_at_ns")
    if not isinstance(expires_at_ns, int) or expires_at_ns < generated_at_ns:
        raise GripRefused("grip envelope has no valid tactile expiry")
    if now_ns > expires_at_ns:
        raise GripRefused("grip envelope's tactile sample has expired")
    if envelope.get("side") not in ("left", "right"):
        raise GripRefused("grip side must be left or right")
    sample_id = envelope.get("tactile_sample_id")
    if not isinstance(sample_id, str) or not sample_id:
        raise GripRefused("grip envelope is not bound to a tactile sample")
    expected_position_mm = envelope.get("expected_position_mm")
    if not isinstance(expected_position_mm, int) or not 0 <= expected_position_mm <= 900:
        raise GripRefused("expected gripper position must be an integer in 0..900mm")
    previous = envelope.get("previous_close_fraction")
    target = envelope.get("target_close_fraction")
    if not all(
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(value)
        and 0.0 <= value <= 1.0
        for value in (previous, target)
    ):
        raise GripRefused("grip close fractions must be finite values in 0..1")
    delta = target - previous
    if abs(delta) > MAX_CLOSE_INCREMENT + 1e-12 or abs(delta) < 1e-9:
        raise GripRefused("grip increment must be non-zero and no larger than 0.05")
    direction = envelope.get("direction")
    decision = envelope.get("tactile_decision")
    allowed = {
        ("close", "close_increment"),
        ("release", "release_increment"),
    }
    if (direction, decision) not in allowed:
        raise GripRefused("grip direction does not match the tactile decision")
    if direction == "close" and delta <= 0.0:
        raise GripRefused("close increment must increase the close fraction")
    if direction == "release" and delta >= 0.0:
        raise GripRefused("release increment must decrease the close fraction")


def validate_live_state(state: dict, side: str, now_ns: int | None = None) -> None:
    now_ns = time.time_ns() if now_ns is None else now_ns
    if state.get("side") != side:
        raise GripRefused("gripper state side mismatch")
    position = state.get("position_mm")
    if not isinstance(position, int) or not 0 <= position <= 900:
        raise GripRefused("gripper position feedback is missing or invalid")
    received_at_ns = state.get("received_at_ns")
    if not isinstance(received_at_ns, int):
        raise GripRefused("gripper feedback has no receive timestamp")
    age_s = (now_ns - received_at_ns) / 1e9
    if not 0.0 <= age_s <= MAX_POSITION_AGE_S:
        raise GripRefused(f"gripper feedback is stale ({age_s:.3f}s)")
    if state.get("command_subscribers", 0) < 1:
        raise GripRefused("gripper command topic has no subscriber")
    error = state.get("error")
    if error not in (None, 0):
        raise GripRefused(f"gripper reports error {error}")


class RosGripBoundary:
    def __init__(self, side: str):
        import rclpy
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from std_msgs.msg import Float64, UInt32MultiArray

        self._rclpy = rclpy
        self._Float64 = Float64
        rclpy.init()
        self.node = rclpy.create_node(f"xr1_grip_adapter_{side}")
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.publisher = self.node.create_publisher(Float64, GRIP_CMD.format(side), qos)
        self.side = side
        self.latest = None
        self.received_at_ns = 0
        self.subscription = self.node.create_subscription(
            UInt32MultiArray, GRIP_STATE.format(side), self._receive, qos
        )

    def _receive(self, message) -> None:
        self.latest = list(message.data)
        self.received_at_ns = time.time_ns()

    def observe(self, timeout_s: float = 3.0) -> dict:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self._rclpy.spin_once(self.node, timeout_sec=0.05)
            if self.latest:
                values = self.latest
                return {
                    "side": self.side,
                    "received_at_ns": self.received_at_ns,
                    "position_mm": int(values[0]),
                    "running": int(values[1]) if len(values) > 1 else None,
                    "temperature": int(values[2]) if len(values) > 2 else None,
                    "error": int(values[3]) if len(values) > 3 else None,
                    "command_subscribers": self.publisher.get_subscription_count(),
                }
        raise GripRefused(f"no {self.side} gripper feedback within {timeout_s:.1f}s")

    def command(self, close_fraction: float) -> dict:
        before_stamp = self.received_at_ns
        self.publisher.publish(self._Float64(data=close_fraction))
        deadline = time.monotonic() + MAX_COMMAND_SETTLE_S
        last_position = None
        stable_samples = 0
        while time.monotonic() < deadline:
            self._rclpy.spin_once(self.node, timeout_sec=0.05)
            if self.received_at_ns <= before_stamp or not self.latest:
                continue
            position = int(self.latest[0])
            if last_position is not None and abs(position - last_position) <= 1:
                stable_samples += 1
            else:
                stable_samples = 0
            last_position = position
            running = int(self.latest[1]) if len(self.latest) > 1 else None
            if stable_samples >= 3 and running in (None, 0):
                return {
                    "side": self.side,
                    "received_at_ns": self.received_at_ns,
                    "position_mm": position,
                    "running": running,
                    "temperature": int(self.latest[2]) if len(self.latest) > 2 else None,
                    "error": int(self.latest[3]) if len(self.latest) > 3 else None,
                    "command_subscribers": self.publisher.get_subscription_count(),
                }
        raise GripRefused(
            f"gripper command did not settle on fresh feedback within {MAX_COMMAND_SETTLE_S:.1f}s"
        )

    def close(self) -> None:
        self.node.destroy_node()
        self._rclpy.shutdown()


def observe(side: str, *, boundary_factory=RosGripBoundary, now_ns: int | None = None) -> dict:
    boundary = boundary_factory(side)
    try:
        state = boundary.observe()
        validate_live_state(state, side, now_ns=now_ns)
        return {"ok": True, "schema_version": 1, "mode": "gripper_observation", **state}
    finally:
        boundary.close()


def execute(
    envelope: dict,
    go: bool,
    *,
    boundary_factory=RosGripBoundary,
    now_ns: int | None = None,
) -> dict:
    validate_envelope(envelope, now_ns=now_ns)
    side = envelope["side"]
    boundary = boundary_factory(side)
    try:
        before = boundary.observe()
        validate_live_state(before, side, now_ns=now_ns)
        drift = abs(before["position_mm"] - envelope["expected_position_mm"])
        if drift > MAX_POSITION_DRIFT_MM:
            raise GripRefused(
                f"gripper position drift {drift}mm exceeds {MAX_POSITION_DRIFT_MM}mm"
            )
        after = boundary.command(envelope["target_close_fraction"]) if go else before
        validate_live_state(after, side, now_ns=now_ns)
        motion_delta_mm = after["position_mm"] - before["position_mm"]
        if go and envelope["direction"] == "close" and motion_delta_mm > -MIN_COMMAND_MOTION_MM:
            raise GripRefused("close command produced no measured closing motion")
        if go and envelope["direction"] == "release" and motion_delta_mm < MIN_COMMAND_MOTION_MM:
            raise GripRefused("release command produced no measured opening motion")
    finally:
        boundary.close()
    return {
        "ok": True,
        "schema_version": 1,
        "mode": "tactile_grip_increment",
        "executed": go,
        "side": side,
        "tactile_sample_id": envelope["tactile_sample_id"],
        "direction": envelope["direction"],
        "target_close_fraction": envelope["target_close_fraction"],
        "position_drift_mm": drift,
        "motion_delta_mm": motion_delta_mm,
        "before": before,
        "after": after,
        "requires_reobservation": go,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest="command", required=True)
    observe_parser = subparsers.add_parser("observe")
    observe_parser.add_argument("--side", required=True, choices=("left", "right"))
    execute_parser = subparsers.add_parser("execute")
    execute_parser.add_argument("--proposal", required=True)
    execute_parser.add_argument("--go", action="store_true")
    args = parser.parse_args()
    try:
        if args.command == "observe":
            report = observe(args.side)
        else:
            report = execute(load_envelope(Path(args.proposal)), args.go)
    except GripRefused as exc:
        raise SystemExit(f"REFUSED: {exc}") from exc
    print(json.dumps(report, ensure_ascii=False, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
