#!/usr/bin/env python3

import json
import sys
from pathlib import Path
import tempfile
import unittest
from unittest import mock

try:
    import numpy as np
except ImportError:
    np = None

sys.path.insert(0, str(Path(__file__).resolve().parent))

import d405_observe


class D405ObserveTest(unittest.TestCase):
    def test_keyframes_keep_first_middle_and_last_frames(self):
        self.assertEqual(d405_observe.keyframe_indices(20), [0, 10, 19])
        self.assertEqual(d405_observe.keyframe_indices(15), [0, 7, 14])
        self.assertEqual(d405_observe.keyframe_indices(0), [])

    def test_frame_wait_retries_timeouts_but_not_backend_errors(self):
        class Pipeline:
            def __init__(self, error):
                self.error = error

            def wait_for_frames(self, _timeout_ms):
                raise self.error

        self.assertIsNone(
            d405_observe.wait_for_frames(
                Pipeline(RuntimeError("Frame didn't arrive within 1000")), 1000
            )
        )
        with self.assertRaisesRegex(d405_observe.D405CaptureError, "device disconnected"):
            d405_observe.wait_for_frames(
                Pipeline(RuntimeError("device disconnected")), 1000
            )

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

    @unittest.skipIf(np is None, "NumPy is not installed in this development Python")
    def test_capture_persists_stage_timings(self):
        class Sampler:
            def close(self):
                pass

        rgb = np.zeros((2, 2, 3), dtype=np.uint8)
        depth = np.ones((2, 2), dtype=np.uint16)
        metadata = {"received_at_ns": 1_000_000_000, "frame_number": 1}
        captured = (
            rgb,
            depth,
            0.001,
            {"serial": "test", "width": 2, "height": 2},
            [metadata],
            {"frame_count": 20, "sustained_stream_verified": True},
            metadata["received_at_ns"],
            {"received_at_ns": metadata["received_at_ns"], "positions_rad": {"joint": 0.0}},
            0,
            [("end", 0, metadata, rgb, depth)],
        )
        with tempfile.TemporaryDirectory() as directory, mock.patch.object(
            d405_observe, "JointStateSampler", return_value=Sampler()
        ), mock.patch.object(d405_observe, "capture_frames", return_value=captured):
            result = d405_observe.capture("test", Path(directory), 848, 480, 15, 8.0)
            event = json.loads((Path(directory) / "events.jsonl").read_text())

        self.assertIn("capture_frames", result["timings_ms"])
        self.assertEqual(event["timings_ms"], result["timings_ms"])

        failed_capture = list(captured)
        failed_capture[1] = np.zeros((2, 2), dtype=np.uint16)
        with tempfile.TemporaryDirectory() as directory, mock.patch.object(
            d405_observe, "JointStateSampler", return_value=Sampler()
        ), mock.patch.object(d405_observe, "capture_frames", return_value=failed_capture):
            with self.assertRaisesRegex(d405_observe.D405CaptureError, "valid ratio"):
                d405_observe.capture("test", Path(directory), 848, 480, 15, 8.0)
            event = json.loads((Path(directory) / "events.jsonl").read_text())
            self.assertFalse((Path(directory) / "latest.json").exists())

        self.assertFalse(event["ok"])
        self.assertIn("total", event["timings_ms"])


if __name__ == "__main__":
    unittest.main()
