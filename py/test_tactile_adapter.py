#!/usr/bin/env python3

import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))

import tactile_adapter


class FakeReader:
    def __init__(self, lines):
        self.lines = iter(lines)
        self.closed = False

    def readline(self, _deadline):
        return next(self.lines, None)

    def close(self):
        self.closed = True


def config():
    return {
        "schema_version": 1,
        "sample_window_ms": 100,
        "minimum_samples": 5,
        "sources": [
            {
                "id": "right_pads",
                "transport": "serial",
                "device": "/dev/pressure-right",
                "baud": 115200,
                "pads": {"fixed": 0, "moving": 1},
            }
        ],
    }


class TactileAdapterTest(unittest.TestCase):
    def test_parses_json_csv_and_labelled_scalar_frames(self):
        self.assertEqual(tactile_adapter.parse_frame('{"left": 12.5}'), {"left": 12.5})
        self.assertEqual(tactile_adapter.parse_frame("1.5, 2"), [1.5, 2.0])
        self.assertEqual(tactile_adapter.parse_frame("pressure=31"), [31.0])

    def test_rejects_gripper_and_neck_serial_devices(self):
        for device in tactile_adapter.RESERVED_SERIAL_DEVICES:
            invalid = config()
            invalid["sources"][0]["device"] = device
            with self.assertRaisesRegex(tactile_adapter.TactileCaptureError, "reserved"):
                tactile_adapter.validate_config(invalid)

    def test_capture_reports_median_mad_and_both_pad_ids(self):
        lines = [f"{100 + i % 2},{200 - i % 2}\n".encode() for i in range(20)]
        readers = []

        def factory(_source):
            reader = FakeReader(lines)
            readers.append(reader)
            return reader

        report = tactile_adapter.capture(config(), reader_factory=factory)
        self.assertTrue(report["ok"])
        self.assertEqual(report["mode"], "tactile_observation")
        self.assertEqual([pad["id"] for pad in report["pads"]], ["fixed", "moving"])
        self.assertEqual(report["pads"][0]["raw"], 100.5)
        self.assertEqual(report["pads"][0]["median_abs_deviation"], 0.5)
        self.assertEqual(report["sources"][0]["endpoint"], "/dev/pressure-right")
        self.assertTrue(readers[0].closed)

    def test_requires_at_least_five_valid_frames(self):
        def factory(_source):
            return FakeReader([b"1,2\n"] * 4)

        with self.assertRaisesRegex(tactile_adapter.TactileCaptureError, "need 5"):
            tactile_adapter.capture(config(), reader_factory=factory)

    def test_rejects_mapping_both_pads_to_one_field(self):
        invalid = config()
        invalid["sources"][0]["pads"]["moving"] = 0
        with self.assertRaisesRegex(tactile_adapter.TactileCaptureError, "one field"):
            tactile_adapter.validate_config(invalid)


if __name__ == "__main__":
    unittest.main()
