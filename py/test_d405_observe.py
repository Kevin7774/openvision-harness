#!/usr/bin/env python3

import sys
from pathlib import Path
import unittest

try:
    import numpy as np
except ImportError:
    np = None

sys.path.insert(0, str(Path(__file__).resolve().parent))

import d405_observe


class D405ObserveTest(unittest.TestCase):
    def test_sustained_stream_requires_monotonic_frames_and_rate(self):
        frames = [
            {"received_at_ns": 1_000_000_000 + index * 66_666_667, "frame_number": index}
            for index in range(20)
        ]
        report = d405_observe.assess_stream(frames, 15)
        self.assertTrue(report["sustained_stream_verified"])
        self.assertGreater(report["observed_fps"], 14.0)

        frames[10]["frame_number"] = frames[9]["frame_number"]
        with self.assertRaisesRegex(d405_observe.D405CaptureError, "frame numbers"):
            d405_observe.assess_stream(frames, 15)

    def test_sustained_stream_rejects_too_few_or_slow_frames(self):
        with self.assertRaisesRegex(d405_observe.D405CaptureError, "at least 15"):
            d405_observe.assess_stream([], 15)
        slow = [
            {"received_at_ns": 1_000_000_000 + index * 200_000_000, "frame_number": index}
            for index in range(20)
        ]
        with self.assertRaisesRegex(d405_observe.D405CaptureError, "rate"):
            d405_observe.assess_stream(slow, 15)

    @unittest.skipIf(np is None, "NumPy is not installed in this development Python")
    def test_depth_statistics_ignore_zero_and_non_finite_pixels(self):
        depth = np.array([[0.0, 0.1], [np.nan, 0.3]], dtype=np.float32)
        report = d405_observe.depth_statistics(depth)
        self.assertEqual(report["valid_pixels"], 2)
        self.assertEqual(report["valid_ratio"], 0.5)
        self.assertAlmostEqual(report["median_m"], 0.2)


if __name__ == "__main__":
    unittest.main()
