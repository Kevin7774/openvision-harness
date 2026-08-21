#!/usr/bin/env python3

import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))

import grip_adapter


def envelope(generated_at_ns=1_000_000_000):
    return {
        "ok": True,
        "schema_version": 2,
        "mode": "tactile_grip_increment",
        "generated_at_ns": generated_at_ns,
        "expires_at_ns": 1_600_000_000,
        "side": "right",
        "tactile_sample_id": "sample-1",
        "expected_position_mm": 840,
        "previous_close_fraction": 0.0,
        "target_close_fraction": 0.05,
        "direction": "close",
        "tactile_decision": "close_increment",
        "d405_alignment": {
            "target": {
                "source_frame_id": "d405-target",
                "signal": [100.0, 200.0, 0.2],
                "tolerance": [5.0, 5.0, 0.01],
            },
            "sample": {
                "frame_id": "d405-current",
                "received_at_ns": 1_400_000_000,
                "signal": [101.0, 198.0, 0.201],
            },
        },
    }


class FakeBoundary:
    instances = []

    def __init__(self, side):
        self.side = side
        self.closed = False
        self.commands = []
        self.__class__.instances.append(self)

    def state(self, position=840):
        return {
            "side": self.side,
            "received_at_ns": 1_400_000_000,
            "position_mm": position,
            "running": 0,
            "temperature": 0,
            "error": 0,
            "command_subscribers": 1,
        }

    def observe(self):
        return self.state()

    def command(self, close_fraction):
        self.commands.append(close_fraction)
        result = self.state(800)
        result["received_at_ns"] = 1_500_000_000
        return result

    def close(self):
        self.closed = True


class DroppedCommandBoundary(FakeBoundary):
    def command(self, close_fraction):
        self.commands.append(close_fraction)
        return self.state()


class GripAdapterTest(unittest.TestCase):
    def setUp(self):
        FakeBoundary.instances.clear()

    def test_close_requires_fresh_converged_d405_alignment(self):
        missing = envelope()
        del missing["d405_alignment"]
        stale = envelope(generated_at_ns=5_000_000_000)
        stale["expires_at_ns"] = 6_000_000_000
        stale["d405_alignment"]["sample"]["received_at_ns"] = 1_000_000_000
        wrong_camera = envelope()
        wrong_camera["d405_alignment"]["sample"]["frame_id"] = "zed-current"
        misaligned = envelope()
        misaligned["d405_alignment"]["sample"]["signal"][0] = 106.0

        for candidate, now_ns in (
            (missing, 1_500_000_000),
            (stale, 5_500_000_000),
            (wrong_camera, 1_500_000_000),
            (misaligned, 1_500_000_000),
        ):
            with self.subTest(candidate=candidate):
                with self.assertRaisesRegex(grip_adapter.GripRefused, "D405 alignment"):
                    grip_adapter.execute(
                        candidate,
                        True,
                        boundary_factory=FakeBoundary,
                        now_ns=now_ns,
                    )
                self.assertEqual(FakeBoundary.instances, [])

    def test_dry_run_validates_without_publishing(self):
        report = grip_adapter.execute(
            envelope(), False, boundary_factory=FakeBoundary, now_ns=1_500_000_000
        )
        boundary = FakeBoundary.instances[-1]
        self.assertFalse(report["executed"])
        self.assertEqual(boundary.commands, [])
        self.assertTrue(boundary.closed)

    def test_go_executes_one_increment_and_requires_reobservation(self):
        report = grip_adapter.execute(
            envelope(), True, boundary_factory=FakeBoundary, now_ns=1_500_000_000
        )
        boundary = FakeBoundary.instances[-1]
        self.assertEqual(boundary.commands, [0.05])
        self.assertTrue(report["requires_reobservation"])
        self.assertEqual(report["after"]["position_mm"], 800)

    def test_dropped_command_is_not_reported_as_an_executed_increment(self):
        with self.assertRaisesRegex(grip_adapter.GripRefused, "no measured closing motion"):
            grip_adapter.execute(
                envelope(), True, boundary_factory=DroppedCommandBoundary, now_ns=1_500_000_000
            )

    def test_stale_or_large_increment_is_refused(self):
        with self.assertRaisesRegex(grip_adapter.GripRefused, "stale"):
            grip_adapter.execute(
                envelope(), False, boundary_factory=FakeBoundary, now_ns=3_000_000_001
            )
        unsafe = envelope()
        unsafe["target_close_fraction"] = 0.2
        with self.assertRaisesRegex(grip_adapter.GripRefused, "0.05"):
            grip_adapter.validate_envelope(unsafe, now_ns=1_500_000_000)

    def test_expired_pressure_sample_is_refused(self):
        expired = envelope()
        expired["expires_at_ns"] = 1_400_000_000
        with self.assertRaisesRegex(grip_adapter.GripRefused, "tactile sample has expired"):
            grip_adapter.validate_envelope(expired, now_ns=1_500_000_000)

    def test_release_requires_matching_pressure_decision(self):
        release = envelope()
        release.update(
            {
                "previous_close_fraction": 0.5,
                "target_close_fraction": 0.45,
                "direction": "release",
                "tactile_decision": "release_increment",
            }
        )
        del release["d405_alignment"]
        grip_adapter.validate_envelope(release, now_ns=1_500_000_000)
        release["tactile_decision"] = "close_increment"
        with self.assertRaisesRegex(grip_adapter.GripRefused, "decision"):
            grip_adapter.validate_envelope(release, now_ns=1_500_000_000)


if __name__ == "__main__":
    unittest.main()
