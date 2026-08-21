#!/usr/bin/env python3

import json
import sys
from pathlib import Path
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))

import motion_adapter


def candidate(rank, score, *, ik=True, grasp=True):
    solution = {
        "score": score / 2,
        "residual_m": 0.001,
        "orientation_residual_rad": 0.01,
        "min_limit_margin_rad": 0.2,
        "joints_rad": {"right_arm_1_joint": 0.0},
    }
    return {
        "rank": rank,
        "score": score,
        "ik_feasible": ik,
        "grasp_feasible": grasp,
        "approach_ik": solution,
        "grasp_ik": solution,
    }


class MotionAdapterTest(unittest.TestCase):
    def test_load_plan_requires_complete_attempt_directory(self):
        plan = {
            "ok": True,
            "mode": "online_plan_dry_run",
            "schema_version": 2,
            "candidates": [],
        }
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            raw_plan = root / "raw-plan.json"
            raw_plan.write_text(json.dumps(plan), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "immutable plan attempt"):
                motion_adapter.load_plan(raw_plan)

            attempt = root / "attempt"
            attempt.mkdir()
            for name in motion_adapter.ATTEMPT_FILES:
                (attempt / name).write_text(
                    json.dumps(plan if name == "plan.json" else {}),
                    encoding="utf-8",
                )
            self.assertEqual(motion_adapter.load_plan(attempt), plan)

    def test_selects_lowest_scored_fully_feasible_candidate(self):
        plan = {"candidates": [candidate(1, 8.0), candidate(2, 3.0)]}
        self.assertEqual(motion_adapter.select_candidate(plan)["rank"], 2)

    def test_rejects_ik_only_candidate(self):
        plan = {"candidates": [candidate(1, 1.0, grasp=False)]}
        with self.assertRaisesRegex(ValueError, "IK and grasp geometry"):
            motion_adapter.select_candidate(plan)

    def test_moveit_plan_requires_collision_validated_candidate(self):
        plan = {
            "moveit_validation": {"backend": "moveit2_planning_scene"},
            "candidates": [candidate(1, 1.0)],
        }
        with self.assertRaisesRegex(ValueError, "MoveIt collision"):
            motion_adapter.select_candidate(plan)

        plan["candidates"][0].update({
            "moveit_validated": True,
            "self_collision_free": True,
            "path_collision_free": True,
        })
        self.assertEqual(motion_adapter.select_candidate(plan)["rank"], 1)

    def test_uses_observation_receive_time_for_schema_two(self):
        age = motion_adapter.plan_age_seconds(
            {"observation_received_at_ns": 10_000_000_000},
            Path("unused"),
            now_s=12.5,
        )
        self.assertEqual(age, 2.5)

    def test_return_reverses_only_the_validated_grasp_to_approach_leg(self):
        plan = {"current_joints_rad": {"right_arm_1_joint": -1.0}}
        selected = candidate(1, 1.0)
        selected["approach_ik"] = {
            **selected["approach_ik"],
            "joints_rad": {"right_arm_1_joint": -1.2},
        }
        selected["grasp_ik"] = {
            **selected["grasp_ik"],
            "joints_rad": {"right_arm_1_joint": -1.4},
        }

        solution = motion_adapter.phase_solution(selected, "return")
        start, target = motion_adapter.phase_joints(plan, selected, "return", solution)

        self.assertEqual(start, selected["grasp_ik"]["joints_rad"])
        self.assertEqual(target, selected["approach_ik"]["joints_rad"])

if __name__ == "__main__":
    unittest.main()
