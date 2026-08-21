#!/usr/bin/env python3
"""Capture one lossless, read-only ZED observation for the VISTA harness."""

from __future__ import annotations

import time

PROCESS_STARTED = time.monotonic_ns()

import argparse
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import sys
import tempfile
import threading

os.environ.setdefault("FASTDDS_BUILTIN_TRANSPORTS", "UDPv4")

import cv2
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import UInt32MultiArray
from tf2_ros import Buffer, TransformException, TransformListener

RGB_TOPIC = "/zed/zed_node/rgb/color/rect/image"
CAMERA_INFO_TOPIC = "/zed/zed_node/rgb/color/rect/camera_info"
DEPTH_TOPIC = "/zed/zed_node/depth/depth_registered"
JOINT_TOPIC = "/joint_states"
GRIPPER_TOPICS = {
    "left": "/qg_robot/gripper_left_state",
    "right": "/qg_robot/gripper_right_state",
}
RUN_ROOT = Path("/home/astrabot/workspace/data/vista_runs")
RUN_ID_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,63}$")
MAX_CLOCK_OFFSET_NS = 2_000_000_000
MAX_RGB_DEPTH_DELTA_NS = 50_000_000
MAX_JOINT_AGE_NS = 500_000_000


def stamp_ns(message) -> int:
    stamp = message.header.stamp
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class ObserveNode(Node):
    def __init__(self) -> None:
        super().__init__("vista_observe")
        self.latest = {}
        self.received_at_ns = {}
        self.received_counts = {}
        self.first_received_at_ns = {}
        self.first_sensor_stamp_ns = {}
        self.last_sensor_stamp_ns = {}
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._subscriptions = [
            self.create_subscription(Image, RGB_TOPIC, self._callback("rgb"), qos),
            self.create_subscription(
                CameraInfo, CAMERA_INFO_TOPIC, self._callback("camera_info"), qos),
            self.create_subscription(Image, DEPTH_TOPIC, self._callback("depth"), qos),
            self.create_subscription(
                JointState, JOINT_TOPIC, self._callback("joint_states"), qos),
            *[
                self.create_subscription(
                    UInt32MultiArray,
                    topic,
                    self._callback(f"gripper_{side}"),
                    qos,
                )
                for side, topic in GRIPPER_TOPICS.items()
            ],
        ]
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)

    def _callback(self, name):
        def receive(msg) -> None:
            received_at_ns = time.time_ns()
            self.latest[name] = msg
            self.received_at_ns[name] = received_at_ns
            self.received_counts[name] = self.received_counts.get(name, 0) + 1
            self.first_received_at_ns.setdefault(name, received_at_ns)
            if hasattr(msg, "header"):
                sensor_stamp_ns = stamp_ns(msg)
                self.first_sensor_stamp_ns.setdefault(name, sensor_stamp_ns)
                self.last_sensor_stamp_ns[name] = sensor_stamp_ns

        return receive


def finite_or_none(value):
    value = float(value)
    return value if math.isfinite(value) else None


def received_fps(node: ObserveNode, name: str):
    count = node.received_counts.get(name, 0)
    elapsed_ns = (
        node.last_sensor_stamp_ns.get(name, 0)
        - node.first_sensor_stamp_ns.get(name, 0)
    )
    return (count - 1) * 1e9 / elapsed_ns if count > 1 and elapsed_ns > 0 else None


def decode_rgb(msg: Image) -> np.ndarray:
    if msg.encoding != "bgra8" or msg.step < msg.width * 4:
        raise RuntimeError(
            f"RGB contract drift: {msg.width}x{msg.height} {msg.encoding} step={msg.step}")
    rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    bgra = rows[:, :msg.width * 4].reshape(msg.height, msg.width, 4)
    return np.ascontiguousarray(bgra[:, :, :3])


def decode_depth(msg: Image) -> np.ndarray:
    if msg.encoding != "32FC1" or msg.step < msg.width * 4:
        raise RuntimeError(
            f"depth contract drift: {msg.width}x{msg.height} {msg.encoding} step={msg.step}")
    dtype = ">f4" if msg.is_bigendian else "<f4"
    rows = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.step // 4)
    return np.array(rows[:, :msg.width], dtype=np.float32, copy=True)


def depth_stats(msg: Image) -> dict:
    depth = decode_depth(msg)
    valid = depth[np.isfinite(depth) & (depth > 0.0)]
    total = int(depth.size)
    count = int(valid.size)
    if count == 0:
        return {
            "unit": "m",
            "valid_ratio": 0.0,
            "valid_pixels": 0,
            "min_m": None,
            "median_m": None,
            "p95_m": None,
            "max_m": None,
        }
    return {
        "unit": "m",
        "valid_ratio": round(count / total, 6),
        "valid_pixels": count,
        "min_m": round(float(np.min(valid)), 6),
        "median_m": round(float(np.median(valid)), 6),
        "p95_m": round(float(np.percentile(valid, 95)), 6),
        "max_m": round(float(np.max(valid)), 6),
    }


def wait_for_observation(node: ObserveNode, timeout_s: float):
    deadline = time.monotonic() + timeout_s
    reason = "waiting for topics"
    required = ("rgb", "camera_info", "depth", "joint_states")
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        missing = tuple(name for name in required if name not in node.latest)
        if missing:
            reason = "missing " + ", ".join(missing)
            continue

        rgb = node.latest["rgb"]
        depth = node.latest["depth"]
        joint = node.latest["joint_states"]
        rgb_rx_ns = node.received_at_ns["rgb"]
        depth_rx_ns = node.received_at_ns["depth"]
        joint_rx_ns = node.received_at_ns["joint_states"]
        rgb_stamp_ns = stamp_ns(rgb)
        depth_stamp_ns = stamp_ns(depth)
        sync_ns = abs(rgb_stamp_ns - depth_stamp_ns)
        if sync_ns > MAX_RGB_DEPTH_DELTA_NS:
            reason = f"RGB/depth timestamp delta {sync_ns / 1e6:.3f}ms"
            continue
        if rgb.header.frame_id != depth.header.frame_id:
            reason = "RGB/depth frame_id mismatch"
            continue
        if abs(rgb_rx_ns - rgb_stamp_ns) > MAX_CLOCK_OFFSET_NS:
            reason = "RGB sensor clock offset exceeds 2s"
            continue
        if abs(depth_rx_ns - depth_stamp_ns) > MAX_CLOCK_OFFSET_NS:
            reason = "depth sensor clock offset exceeds 2s"
            continue
        joint_age_ns = abs(joint_rx_ns - stamp_ns(joint))
        if joint_age_ns > MAX_JOINT_AGE_NS:
            reason = f"joint state age {joint_age_ns / 1e6:.3f}ms"
            continue
        if (rgb.width, rgb.height) != (depth.width, depth.height):
            reason = "RGB/depth dimensions differ"
            continue
        camera_info = node.latest["camera_info"]
        if (camera_info.width, camera_info.height) != (rgb.width, rgb.height):
            reason = "camera_info dimensions differ from RGB"
            continue
        if camera_info.header.frame_id != rgb.header.frame_id:
            reason = "camera_info/RGB frame_id mismatch"
            continue

        image_time = Time(nanoseconds=rgb_stamp_ns)
        try:
            transform = node.tf_buffer.lookup_transform(
                "base_link",
                rgb.header.frame_id,
                image_time,
                timeout=Duration(seconds=0.0),
            )
        except TransformException as exc:
            reason = f"TF unavailable at image time: {exc}"
            continue
        return rgb, depth, camera_info, joint, transform

    raise RuntimeError(f"observe timeout after {timeout_s:.1f}s: {reason}")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as file:
        for chunk in iter(lambda: file.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def write_json(path: Path, value) -> None:
    with path.open("w", encoding="utf-8") as file:
        json.dump(value, file, indent=2, ensure_ascii=False, allow_nan=False)
        file.write("\n")


def joint_positions(msg: JointState) -> dict:
    return {
        name: finite_or_none(msg.position[index]) if index < len(msg.position) else None
        for index, name in enumerate(msg.name)
    }


def transform_report(transform) -> dict:
    t = transform.transform.translation
    q = transform.transform.rotation
    return {
        "target_frame": transform.header.frame_id,
        "source_frame": transform.child_frame_id,
        "stamp_ns": stamp_ns(transform),
        "translation_m": [t.x, t.y, t.z],
        "rotation_xyzw": [q.x, q.y, q.z, q.w],
    }


def gripper_reports(node: ObserveNode) -> dict:
    reports = {}
    for side in GRIPPER_TOPICS:
        name = f"gripper_{side}"
        message = node.latest.get(name)
        if message is None or not message.data:
            continue
        values = list(message.data)
        reports[side] = {
            "received_at_ns": node.received_at_ns[name],
            "position_mm": int(values[0]),
            "running": int(values[1]) if len(values) > 1 else None,
            "temperature": int(values[2]) if len(values) > 2 else None,
            "error": int(values[3]) if len(values) > 3 else None,
        }
    return reports


def capture(run_id: str, timeout_s: float) -> dict:
    if not RUN_ID_RE.fullmatch(run_id):
        raise ValueError("run-id must match [A-Za-z0-9][A-Za-z0-9._-]{0,63}")

    capture_started_at_ns = time.time_ns()
    capture_started = time.monotonic_ns()
    rclpy.init()
    rclpy_initialized = time.monotonic_ns()
    node = ObserveNode()
    node_initialized = time.monotonic_ns()
    try:
        rgb, depth, camera_info, joint, transform = wait_for_observation(node, timeout_s)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    observation_ready = time.monotonic_ns()

    rgb_image = decode_rgb(rgb)
    depth_image = decode_depth(depth)
    decoded = time.monotonic_ns()
    rgb_stamp_ns = stamp_ns(rgb)
    depth_stamp_ns = stamp_ns(depth)
    received_at_ns = max(
        node.received_at_ns["rgb"], node.received_at_ns["depth"])
    frame_id = time.strftime("%Y%m%d-%H%M%S", time.localtime(received_at_ns / 1e9))
    frame_id += f"-{received_at_ns % 1_000_000_000:09d}-{os.getpid()}"

    run_dir = RUN_ROOT / run_id
    observations_dir = run_dir / "observations"
    observations_dir.mkdir(parents=True, exist_ok=True)
    temporary_dir = Path(tempfile.mkdtemp(prefix=".observe-", dir=observations_dir))
    final_dir = observations_dir / frame_id
    persist_started = time.monotonic_ns()
    try:
        rgb_path = temporary_dir / "rgb.png"
        depth_path = temporary_dir / "depth.npy"
        camera_info_path = temporary_dir / "camera_info.json"
        state_path = temporary_dir / "state.json"

        if not cv2.imwrite(
                str(rgb_path), rgb_image, [cv2.IMWRITE_PNG_COMPRESSION, 3]):
            raise RuntimeError("failed to write rgb.png")
        np.save(depth_path, depth_image, allow_pickle=False)
        write_json(camera_info_path, {
            "frame_id": camera_info.header.frame_id,
            "sensor_stamp_ns": stamp_ns(camera_info),
            "width": camera_info.width,
            "height": camera_info.height,
            "distortion_model": camera_info.distortion_model,
            "d": list(camera_info.d),
            "k": list(camera_info.k),
            "r": list(camera_info.r),
            "p": list(camera_info.p),
        })
        state = {
            "schema_version": 1,
            "run_id": run_id,
            "frame_id": frame_id,
            "sensor_stamp_ns": rgb_stamp_ns,
            "received_at_ns": received_at_ns,
            "rgb_depth_delta_ms": abs(rgb_stamp_ns - depth_stamp_ns) / 1e6,
            "clock_offset_ms": (received_at_ns - rgb_stamp_ns) / 1e6,
            "rgb": {
                "topic": RGB_TOPIC,
                "frame_id": rgb.header.frame_id,
                "width": rgb.width,
                "height": rgb.height,
                "source_encoding": rgb.encoding,
                "stored_encoding": "png",
                "source_step": rgb.step,
            },
            "depth": {
                "topic": DEPTH_TOPIC,
                "frame_id": depth.header.frame_id,
                "width": depth.width,
                "height": depth.height,
                "source_encoding": depth.encoding,
                "stored_dtype": "float32",
                "unit": "m",
                "source_step": depth.step,
                "statistics": depth_stats(depth),
            },
            "joint_state": {
                "topic": JOINT_TOPIC,
                "sensor_stamp_ns": stamp_ns(joint),
                "received_at_ns": node.received_at_ns["joint_states"],
                "positions_rad": joint_positions(joint),
            },
            "grippers": gripper_reports(node),
            "tf": transform_report(transform),
        }
        write_json(state_path, state)
        state["files"] = {
            "rgb_sha256": sha256(rgb_path),
            "depth_sha256": sha256(depth_path),
            "camera_info_sha256": sha256(camera_info_path),
        }
        write_json(state_path, state)
        os.replace(temporary_dir, final_dir)
    except Exception:
        shutil.rmtree(temporary_dir, ignore_errors=True)
        raise
    persisted = time.monotonic_ns()

    camera = {
        "rgb_received": node.received_counts.get("rgb", 0),
        "depth_received": node.received_counts.get("depth", 0),
        "camera_info_received": node.received_counts.get("camera_info", 0),
        "rgb_received_fps": received_fps(node, "rgb"),
        "depth_received_fps": received_fps(node, "depth"),
        "first_rgb_ms": (
            node.first_received_at_ns["rgb"] - capture_started_at_ns) / 1e6,
        "consumed_frames": 1,
        "dropped_frames": None,
    }
    timings_ms = {
        "module_load": (capture_started - PROCESS_STARTED) / 1e6,
        "rclpy_init": (rclpy_initialized - capture_started) / 1e6,
        "node_init": (node_initialized - rclpy_initialized) / 1e6,
        "wait_for_observation": (observation_ready - node_initialized) / 1e6,
        "decode": (decoded - observation_ready) / 1e6,
        "persist": (persisted - persist_started) / 1e6,
    }

    result = {
        "ok": True,
        "tool": "observe",
        "run_id": run_id,
        "frame_id": frame_id,
        "rgb_path": str(final_dir / "rgb.png"),
        "depth_path": str(final_dir / "depth.npy"),
        "camera_info_path": str(final_dir / "camera_info.json"),
        "state_path": str(final_dir / "state.json"),
        "sensor_stamp_ns": rgb_stamp_ns,
        "received_at_ns": received_at_ns,
        "rgb_depth_delta_ms": abs(rgb_stamp_ns - depth_stamp_ns) / 1e6,
        "clock_offset_ms": (received_at_ns - rgb_stamp_ns) / 1e6,
        "tf_ok": True,
        "depth_valid_ratio": state["depth"]["statistics"]["valid_ratio"],
        "camera": camera,
        "fastdds_builtin_transports": os.environ.get("FASTDDS_BUILTIN_TRANSPORTS"),
        "timings_ms": timings_ms,
    }

    event = {
        "event": "observe",
        "frame_id": frame_id,
        "sensor_stamp_ns": rgb_stamp_ns,
        "received_at_ns": received_at_ns,
        "rgb_path": result["rgb_path"],
        "state_path": result["state_path"],
    }
    events_path = run_dir / "events.jsonl"
    with events_path.open("a", encoding="utf-8") as file:
        fcntl.flock(file, fcntl.LOCK_EX)
        file.write(json.dumps(event, ensure_ascii=False, allow_nan=False) + "\n")
        file.flush()
        os.fsync(file.fileno())
        fcntl.flock(file, fcntl.LOCK_UN)

    timings_ms["total"] = (time.monotonic_ns() - capture_started) / 1e6
    latest_tmp = run_dir / f".latest-{os.getpid()}.json"
    write_json(latest_tmp, result)
    os.replace(latest_tmp, run_dir / "latest.json")
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-id", default="shared")
    parser.add_argument("--timeout", type=float, default=20.0)
    args = parser.parse_args()
    if not 1.0 <= args.timeout <= 60.0:
        parser.error("--timeout must be between 1 and 60 seconds")

    finished = threading.Event()

    def watchdog() -> None:
        if not finished.wait(args.timeout + 8.0):
            print('{"ok":false,"error":"observe watchdog timeout"}', flush=True)
            os._exit(9)

    threading.Thread(target=watchdog, daemon=True).start()
    try:
        result = capture(args.run_id, args.timeout)
        print(json.dumps(result, ensure_ascii=False, allow_nan=False), flush=True)
        return 0
    except Exception as exc:  # noqa: BLE001
        print(json.dumps({"ok": False, "error": str(exc)}, ensure_ascii=False), flush=True)
        return 2
    finally:
        finished.set()


if __name__ == "__main__":
    sys.exit(main())
