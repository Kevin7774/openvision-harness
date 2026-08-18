#!/usr/bin/env python3

import sys
from pathlib import Path
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
    def test_selects_lowest_scored_fully_feasible_candidate(self):
        plan = {"candidates": [candidate(1, 8.0), candidate(2, 3.0)]}
        self.assertEqual(motion_adapter.select_candidate(plan)["rank"], 2)

    def test_rejects_ik_only_candidate(self):
        plan = {"candidates": [candidate(1, 1.0, grasp=False)]}
        with self.assertRaisesRegex(ValueError, "IK and grasp geometry"):
            motion_adapter.select_candidate(plan)

    def test_uses_observation_receive_time_for_schema_two(self):
        age = motion_adapter.plan_age_seconds(
            {"observation_received_at_ns": 10_000_000_000},
            Path("unused"),
            now_s=12.5,
        )
        self.assertEqual(age, 2.5)


if __name__ == "__main__":
    unittest.main()
