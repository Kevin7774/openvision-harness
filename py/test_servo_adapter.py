#!/usr/bin/env python3

import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))

import servo_adapter


JOINTS = [f"right_arm_{index}_joint" for index in range(1, 8)]
CONTROLLED = ["right_arm_2_joint", "right_arm_4_joint", "right_arm_6_joint"]


def envelope(generated_at_ns=1_000_000_000):
    current = {joint: 0.0 for joint in JOINTS}
    delta = {joint: value for joint, value in zip(CONTROLLED, (0.01, -0.02, 0.03))}
    target = {joint: current[joint] + delta.get(joint, 0.0) for joint in JOINTS}
    return {
        "ok": True,
        "schema_version": 1,
        "mode": "visual_servo_proposal",
        "generated_at_ns": generated_at_ns,
        "observation_frame_id": "frame-before",
        "proposal": {
            "controlled_joints": list(CONTROLLED),
            "joint_delta_rad": delta,
        },
        "safety": {
            "approved": True,
            "current_joints_rad": current,
            "target_joints_rad": target,
        },
        "ready_for_execution_adapter": True,
        "execution_authorized": False,
    }


class FakeRobot:
    instances = []

    def __init__(self):
        self.state = {joint: 0.0 for joint in JOINTS}
        self.move_args = None
        self.closed = False
        self.__class__.instances.append(self)

    def joints(self):
        return dict(self.state)

    def move(self, target, speed, dry_run):
        self.move_args = (target, speed, dry_run)
        if not dry_run:
            self.state.update(target)
        return dict(self.state)

    def close(self):
        self.closed = True


class ShortMoveRobot(FakeRobot):
    def move(self, target, speed, dry_run):
        self.move_args = (target, speed, dry_run)
        if not dry_run:
            self.state.update(
                {joint: value * 0.5 for joint, value in target.items()}
            )
        return dict(self.state)


class ServoAdapterTest(unittest.TestCase):
    def setUp(self):
        FakeRobot.instances.clear()

    def test_dry_run_consumes_one_approved_envelope_without_motion(self):
        report = servo_adapter.execute(
            envelope(), False, robot_factory=FakeRobot, now_ns=1_500_000_000
        )
        robot = FakeRobot.instances[-1]
        self.assertFalse(report["executed"])
        self.assertFalse(report["requires_reobservation"])
        self.assertTrue(robot.move_args[2])
        self.assertTrue(robot.closed)

    def test_go_reports_reobservation_required(self):
        report = servo_adapter.execute(
            envelope(), True, robot_factory=FakeRobot, now_ns=1_500_000_000
        )
        self.assertTrue(report["executed"])
        self.assertTrue(report["requires_reobservation"])
        self.assertTrue(report["motion_completed"])
        self.assertAlmostEqual(report["achieved_joint_delta_rad"][CONTROLLED[2]], 0.03)

    def test_short_physical_move_still_requires_reobservation_and_stops(self):
        report = servo_adapter.execute(
            envelope(), True, robot_factory=ShortMoveRobot, now_ns=1_500_000_000
        )
        self.assertFalse(report["ok"])
        self.assertFalse(report["motion_completed"])
        self.assertTrue(report["requires_reobservation"])
        self.assertGreater(report["max_joint_arrival_error_rad"], 0.01)

    def test_stale_or_unapproved_proposal_is_refused(self):
        with self.assertRaisesRegex(servo_adapter.ServoRefused, "stale"):
            servo_adapter.execute(
                envelope(), False, robot_factory=FakeRobot, now_ns=5_000_000_000
            )
        unsafe = envelope()
        unsafe["safety"]["approved"] = False
        with self.assertRaisesRegex(servo_adapter.ServoRefused, "safety"):
            servo_adapter.execute(
                unsafe, False, robot_factory=FakeRobot, now_ns=1_500_000_000
            )

    def test_tampered_target_is_refused(self):
        changed = envelope()
        changed["safety"]["target_joints_rad"][CONTROLLED[0]] += 0.01
        with self.assertRaisesRegex(servo_adapter.ServoRefused, "approved delta"):
            servo_adapter.execute(
                changed, False, robot_factory=FakeRobot, now_ns=1_500_000_000
            )

    def test_unknown_controlled_joint_is_refused(self):
        changed = envelope()
        original_joint = CONTROLLED[0]
        changed["proposal"]["controlled_joints"][0] = "unknown_joint"
        changed["proposal"]["joint_delta_rad"]["unknown_joint"] = changed["proposal"][
            "joint_delta_rad"
        ].pop(original_joint)
        with self.assertRaisesRegex(servo_adapter.ServoRefused, "absent"):
            servo_adapter.execute(
                changed, False, robot_factory=FakeRobot, now_ns=1_500_000_000
            )


if __name__ == "__main__":
    unittest.main()
