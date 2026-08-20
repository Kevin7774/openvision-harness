#!/usr/bin/env python3

import unittest

try:
    from astra_arm import ARM_JOINTS, MAX_STEP_RAD, Robot
except ModuleNotFoundError as exc:
    if exc.name == "rclpy":
        ARM_JOINTS = MAX_STEP_RAD = Robot = None
    else:
        raise


@unittest.skipIf(Robot is None, "ROS 2 Python is only installed on the robot")
class AstraArmTest(unittest.TestCase):
    def test_home_splits_large_displacement_without_widening_step_limit(self):
        state = dict(zip(
            ARM_JOINTS,
            [0.0] * 7 + [-1.258, -2.638, 0.805, -1.036, 1.132, 0.678, -0.627],
        ))
        robot = Robot.__new__(Robot)
        robot.joints = lambda: state
        robot.move = lambda *args, **kwargs: self.fail("expected segmented move")
        calls = []
        robot.move_through = lambda waypoints, speed: calls.append(waypoints) or state

        robot.home("both", speed=0.5)

        self.assertEqual(len(calls[0]), 2)
        points = [state, *calls[0]]
        self.assertTrue(all(
            max(abs(b[j] - a[j]) for j in ARM_JOINTS) <= MAX_STEP_RAD
            for a, b in zip(points, points[1:])
        ))
        self.assertTrue(all(calls[0][-1][j] == 0.0 for j in ARM_JOINTS))


if __name__ == "__main__":
    unittest.main()
