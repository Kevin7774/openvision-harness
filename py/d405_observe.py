#!/usr/bin/env python3
"""Capture a bounded D405 RGB/depth observation without managing services."""

from __future__ import annotations

import argparse
import fcntl
import json
import math
import os
from pathlib import Path
import shutil
import tempfile
import time


DEFAULT_SERIAL = "262422270599"
DEFAULT_ROOT = Path("/home/astrabot/workspace/data/sensors/d405")
MIN_DEPTH_VALID_RATIO = 0.50
MAX_JOINT_AGE_NS = 500_000_000
MAX_FRAME_JOINT_DELTA_NS = 500_000_000


class D405CaptureError(RuntimeError):
    """The D405 capture did not satisfy the near-field data contract."""


def keyframe_indices(frame_count: int) -> list[int]:
    if frame_count <= 0:
        return []
    return sorted({0, frame_count // 2, frame_count - 1})


def assess_stream(frame_metadata: list[dict], requested_fps: int) -> dict:
    if len(frame_metadata) < 15:
        raise D405CaptureError(f"D405 produced {len(frame_metadata)} frames; need at least 15")
    received = [frame["received_at_ns"] for frame in frame_metadata]
    numbers = [frame["frame_number"] for frame in frame_metadata]
    if any(not isinstance(value, int) for value in received + numbers):
        raise D405CaptureError("D405 frame metadata is not integral")
    if any(after <= before for before, after in zip(received, received[1:])):
        raise D405CaptureError("D405 host receive timestamps are not increasing")
    if any(after <= before for before, after in zip(numbers, numbers[1:])):
        raise D405CaptureError("D405 frame numbers are not increasing")
    gaps_ms = [(after - before) / 1e6 for before, after in zip(received, received[1:])]
    duration_s = (received[-1] - received[0]) / 1e9
    observed_fps = (len(received) - 1) / duration_s if duration_s > 0.0 else 0.0
    maximum_gap_ms = max(gaps_ms)
    if observed_fps < requested_fps * 0.70:
        raise D405CaptureError(
            f"D405 sustained rate {observed_fps:.2f}Hz is below 70% of {requested_fps}Hz"
        )
    if maximum_gap_ms > 3000.0 / requested_fps:
        raise D405CaptureError(
            f"D405 maximum frame gap {maximum_gap_ms:.1f}ms exceeds "
            f"{3000.0 / requested_fps:.1f}ms"
        )
    return {
        "frame_count": len(frame_metadata),
        "observed_fps": observed_fps,
        "maximum_gap_ms": maximum_gap_ms,
        "sustained_stream_verified": True,
    }


def depth_statistics(depth_m) -> dict:
    import numpy as np

    valid = depth_m[np.isfinite(depth_m) & (depth_m > 0.0)]
    ratio = float(valid.size / depth_m.size) if depth_m.size else 0.0
    if valid.size == 0:
        return {
            "unit": "m",
            "valid_ratio": 0.0,
            "valid_pixels": 0,
            "minimum_m": None,
            "median_m": None,
            "maximum_m": None,
        }
    return {
        "unit": "m",
        "valid_ratio": ratio,
        "valid_pixels": int(valid.size),
        "minimum_m": float(np.min(valid)),
        "median_m": float(np.median(valid)),
        "maximum_m": float(np.max(valid)),
    }


class JointStateSampler:
    def __init__(self) -> None:
        import rclpy
        from sensor_msgs.msg import JointState

        self.rclpy = rclpy
        self.latest = {}
        rclpy.init()
        self.node = rclpy.create_node("xr1_d405_joint_snapshot")

        def receive(message) -> None:
            self.latest["message"] = message
            self.latest["received_at_ns"] = time.time_ns()

        self.subscription = self.node.create_subscription(
            JointState, "/joint_states", receive, 10
        )

    def poll(self, timeout_s: float = 0.0) -> None:
        self.rclpy.spin_once(self.node, timeout_sec=timeout_s)

    def sample(self, timeout_s: float = 2.0) -> dict:
        deadline = time.monotonic() + timeout_s
        while "message" not in self.latest and time.monotonic() < deadline:
            self.poll(0.05)
        if "message" not in self.latest:
            raise D405CaptureError(f"no joint state within {timeout_s:.1f}s")
        message = self.latest["message"]
        positions = {
            name: float(message.position[index])
            for index, name in enumerate(message.name)
            if index < len(message.position) and math.isfinite(message.position[index])
        }
        if not positions:
            raise D405CaptureError("joint state contains no finite position")
        return {
            "received_at_ns": self.latest["received_at_ns"],
            "positions_rad": positions,
        }

    def close(self) -> None:
        self.node.destroy_subscription(self.subscription)
        self.node.destroy_node()
        self.rclpy.shutdown()


def wait_for_frames(pipeline, timeout_ms: int):
    try:
        return pipeline.wait_for_frames(timeout_ms)
    except Exception as exc:  # noqa: BLE001 - librealsense exposes backend-specific errors.
        if "Frame didn't arrive within" in str(exc):
            return None
        raise D405CaptureError(f"D405 frame wait failed: {exc}") from exc


def capture_frames(
    serial: str, width: int, height: int, fps: int, timeout_s: float, joint_sampler
):
    try:
        import numpy as np
        import pyrealsense2 as rs
    except ImportError as exc:
        raise D405CaptureError("NumPy or pyrealsense2 is not installed") from exc

    context = rs.context()
    matching = [device for device in context.query_devices() if device.get_info(rs.camera_info.serial_number) == serial]
    if len(matching) != 1:
        raise D405CaptureError(f"expected one D405 serial {serial}, found {len(matching)}")
    product = matching[0].get_info(rs.camera_info.name)
    if "D405" not in product:
        raise D405CaptureError(f"serial {serial} identifies {product}, not a D405")

    pipeline = rs.pipeline(context)
    config = rs.config()
    config.enable_device(serial)
    config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
    config.enable_stream(rs.stream.color, width, height, rs.format.rgb8, fps)
    try:
        profile = pipeline.start(config)
    except Exception as exc:  # noqa: BLE001 - librealsense exposes backend-specific errors.
        raise D405CaptureError(
            f"cannot start D405 stream (a running owner is left untouched): {exc}"
        ) from exc
    align = rs.align(rs.stream.color)
    metadata = []
    captured = []
    deadline = time.monotonic() + timeout_s
    try:
        while len(metadata) < 20 and time.monotonic() < deadline:
            remaining_ms = max(1, int((deadline - time.monotonic()) * 1000))
            frames = wait_for_frames(pipeline, min(1000, remaining_ms))
            if frames is None:
                continue
            aligned = align.process(frames)
            color = aligned.get_color_frame()
            depth = aligned.get_depth_frame()
            if not color or not depth:
                continue
            received_at_ns = time.time_ns()
            frame_metadata = {
                "received_at_ns": received_at_ns,
                "frame_number": int(color.get_frame_number()),
                "device_timestamp_ms": float(color.get_timestamp()),
            }
            metadata.append(frame_metadata)
            captured.append(
                (
                    frame_metadata,
                    np.asanyarray(color.get_data()).copy(),
                    np.asanyarray(depth.get_data()).copy(),
                )
            )
            joint_sampler.poll()
        if not captured:
            raise D405CaptureError("D405 produced no aligned RGB/depth frame")
        joints = joint_sampler.sample()
        joint_age_ns = time.time_ns() - joints["received_at_ns"]
        if not 0 <= joint_age_ns <= MAX_JOINT_AGE_NS:
            raise D405CaptureError(
                f"joint state age {joint_age_ns / 1e6:.1f}ms exceeds 500ms"
            )
        received_at_ns = captured[-1][0]["received_at_ns"]
        frame_joint_delta_ns = abs(joints["received_at_ns"] - received_at_ns)
        if frame_joint_delta_ns > MAX_FRAME_JOINT_DELTA_NS:
            raise D405CaptureError(
                f"D405/joint receive delta {frame_joint_delta_ns / 1e6:.1f}ms exceeds 500ms"
            )
        quality = assess_stream(metadata, fps)
        scale = profile.get_device().first_depth_sensor().get_depth_scale()
        intrinsics = color.profile.as_video_stream_profile().intrinsics
        camera = {
            "serial": serial,
            "frame_id": f"d405_{serial}_color_optical_frame",
            "width": intrinsics.width,
            "height": intrinsics.height,
            "distortion_model": str(intrinsics.model),
            "coeffs": list(intrinsics.coeffs),
            "k": [
                intrinsics.fx,
                0.0,
                intrinsics.ppx,
                0.0,
                intrinsics.fy,
                intrinsics.ppy,
                0.0,
                0.0,
                1.0,
            ],
        }
        keyframes = [
            (label, index, *captured[index])
            for label, index in zip(
                ("start", "middle", "end"), keyframe_indices(len(captured))
            )
        ]
        return (
            captured[-1][1],
            captured[-1][2],
            float(scale),
            camera,
            metadata,
            quality,
            received_at_ns,
            joints,
            frame_joint_delta_ns,
            keyframes,
        )
    finally:
        pipeline.stop()


def write_json(path: Path, value: dict) -> None:
    with path.open("w", encoding="utf-8") as file:
        json.dump(value, file, ensure_ascii=False, allow_nan=False, indent=2)
        file.write("\n")
        file.flush()
        os.fsync(file.fileno())


def capture(serial: str, root: Path, width: int, height: int, fps: int, timeout_s: float) -> dict:
    import cv2
    import numpy as np

    started_at_ns = time.time_ns()
    joint_sampler = JointStateSampler()
    try:
        (
            color_rgb,
            depth_raw,
            depth_scale,
            camera,
            metadata,
            stream,
            received_at_ns,
            joints,
            frame_joint_delta_ns,
            keyframes,
        ) = capture_frames(serial, width, height, fps, timeout_s, joint_sampler)
    finally:
        joint_sampler.close()
    depth_m = depth_raw.astype(np.float32)
    depth_m *= depth_scale
    depth = depth_statistics(depth_m)
    if depth["valid_ratio"] < MIN_DEPTH_VALID_RATIO:
        raise D405CaptureError(
            f"D405 depth valid ratio {depth['valid_ratio']:.3f} is below {MIN_DEPTH_VALID_RATIO:.2f}"
        )

    frame_id = "d405-" + time.strftime("%Y%m%d-%H%M%S", time.localtime(received_at_ns / 1e9))
    frame_id += f"-{received_at_ns % 1_000_000_000:09d}-{os.getpid()}"
    observations = root / "observations"
    observations.mkdir(parents=True, exist_ok=True)
    temporary = Path(tempfile.mkdtemp(prefix=".observe-", dir=observations))
    final = observations / frame_id
    try:
        rgb_path = temporary / "rgb.png"
        depth_path = temporary / "depth.npy"
        camera_path = temporary / "camera_info.json"
        state_path = temporary / "state.json"
        keyframes_path = temporary / "keyframes.json"
        if not cv2.imwrite(str(rgb_path), color_rgb[:, :, ::-1]):
            raise D405CaptureError("failed to write D405 rgb.png")
        np.save(depth_path, depth_m, allow_pickle=False)
        write_json(camera_path, camera)
        keyframe_records = []
        for label, index, frame_metadata, frame_rgb, frame_depth_raw in keyframes:
            if label == "end":
                frame_rgb_path = final / "rgb.png"
                frame_depth_path = final / "depth.npy"
            else:
                frame_rgb_path = final / f"keyframe-{label}-rgb.png"
                frame_depth_path = final / f"keyframe-{label}-depth.npy"
                if not cv2.imwrite(
                    str(temporary / frame_rgb_path.name), frame_rgb[:, :, ::-1]
                ):
                    raise D405CaptureError(
                        f"failed to write D405 {label} keyframe RGB"
                    )
                frame_depth_m = frame_depth_raw.astype(np.float32)
                frame_depth_m *= depth_scale
                np.save(
                    temporary / frame_depth_path.name,
                    frame_depth_m,
                    allow_pickle=False,
                )
            keyframe_records.append(
                {
                    "label": label,
                    "capture_index": index,
                    **frame_metadata,
                    "rgb_path": str(frame_rgb_path),
                    "depth_path": str(frame_depth_path),
                    "depth_unit": "m",
                }
            )
        write_json(
            keyframes_path,
            {
                "schema_version": 1,
                "frame_id": frame_id,
                "selection": "first_middle_last",
                "safe_for_motion_authorization": False,
                "frames": keyframe_records,
            },
        )
        state = {
            "schema_version": 1,
            "frame_id": frame_id,
            "serial": serial,
            "sensor_stamp_ns": received_at_ns,
            "received_at_ns": received_at_ns,
            "capture_started_at_ns": started_at_ns,
            "frame_joint_delta_ms": frame_joint_delta_ns / 1e6,
            "joint_state": joints,
            "depth": depth,
            "stream": stream,
            "device_frames": metadata,
        }
        write_json(state_path, state)
        os.replace(temporary, final)
    except Exception:
        shutil.rmtree(temporary, ignore_errors=True)
        raise

    result = {
        "ok": True,
        "schema_version": 1,
        "mode": "d405_near_field_observation",
        "frame_id": frame_id,
        "serial": serial,
        "sensor_stamp_ns": received_at_ns,
        "received_at_ns": received_at_ns,
        "frame_joint_delta_ms": frame_joint_delta_ns / 1e6,
        "rgb_path": str(final / "rgb.png"),
        "depth_path": str(final / "depth.npy"),
        "camera_info_path": str(final / "camera_info.json"),
        "state_path": str(final / "state.json"),
        "keyframes_path": str(final / "keyframes.json"),
        "keyframe_count": len(keyframes),
        "latest_path": str(root / "latest.json"),
        "depth_valid_ratio": depth["valid_ratio"],
        **stream,
    }
    latest_tmp = root / f".latest-{os.getpid()}.json"
    write_json(latest_tmp, result)
    os.replace(latest_tmp, root / "latest.json")
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--serial", default=DEFAULT_SERIAL)
    parser.add_argument("--output-root", default=str(DEFAULT_ROOT))
    parser.add_argument("--width", type=int, default=848)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=10)
    parser.add_argument("--timeout", type=float, default=8.0)
    args = parser.parse_args()
    if (args.width, args.height, args.fps) != (848, 480, 10):
        parser.error("only the previously exercised 848x480@10 profile is allowed")
    if not 3.0 <= args.timeout <= 20.0:
        parser.error("--timeout must be within 3..20 seconds")
    try:
        with Path("/tmp/xr1-d405-capture.lock").open("a+") as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            result = capture(
                args.serial,
                Path(args.output_root),
                args.width,
                args.height,
                args.fps,
                args.timeout,
            )
        print(json.dumps(result, ensure_ascii=False, allow_nan=False), flush=True)
        return 0
    except (BlockingIOError, D405CaptureError, OSError) as exc:
        print(json.dumps({"ok": False, "error": str(exc)}, ensure_ascii=False), flush=True)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
