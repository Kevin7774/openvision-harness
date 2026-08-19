#!/usr/bin/env python3
"""Capture two calibrated pressure channels without making control decisions."""

from __future__ import annotations

import argparse
import fcntl
import json
import math
import os
from pathlib import Path
import re
import select
import statistics
import termios
import threading
import time


RESERVED_SERIAL_DEVICES = {
    "/dev/ttyUSB0",  # Right G2 gripper, CP2102N at 2 Mbaud.
    "/dev/ttyAMA5",  # Left G2 gripper.
    "/dev/ttyAMA10",  # Neck bus.
}
SUPPORTED_BAUD = {115200: termios.B115200}
NUMBER_RE = re.compile(r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?")


class TactileCaptureError(RuntimeError):
    """A pressure capture cannot satisfy its explicit data contract."""


def load_config(path: Path) -> dict:
    try:
        config = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise TactileCaptureError(f"cannot read tactile config {path}: {exc}") from exc
    validate_config(config)
    return config


def validate_config(config: dict) -> None:
    if config.get("schema_version") != 1:
        raise TactileCaptureError("tactile config schema_version must be 1")
    sources = config.get("sources")
    if not isinstance(sources, list) or not sources:
        raise TactileCaptureError("tactile config requires at least one source")
    pad_ids = []
    source_ids = set()
    endpoints = set()
    for source in sources:
        if not isinstance(source, dict):
            raise TactileCaptureError("each tactile source must be an object")
        source_id = source.get("id")
        if not isinstance(source_id, str) or not source_id or source_id in source_ids:
            raise TactileCaptureError("tactile source ids must be non-empty and unique")
        source_ids.add(source_id)
        transport = source.get("transport")
        if transport == "serial":
            device = source.get("device")
            if not isinstance(device, str) or not device.startswith("/dev/"):
                raise TactileCaptureError(f"source {source_id} needs an absolute /dev path")
            if device in RESERVED_SERIAL_DEVICES:
                raise TactileCaptureError(f"source {source_id} selects reserved device {device}")
            if source.get("baud", 115200) not in SUPPORTED_BAUD:
                raise TactileCaptureError(f"source {source_id} supports only 115200 baud")
            endpoint = device
        elif transport == "pyusb_ch340":
            usb_path = source.get("usb_path")
            if not isinstance(usb_path, str) or not re.fullmatch(r"\d+-\d+(?:\.\d+)*", usb_path):
                raise TactileCaptureError(f"source {source_id} needs usb_path such as 1-3.4.3")
            if source.get("baud", 115200) != 115200:
                raise TactileCaptureError("raw CH340 transport currently supports only 115200 baud")
            endpoint = usb_path
        else:
            raise TactileCaptureError(f"source {source_id} has unsupported transport {transport!r}")
        if endpoint in endpoints:
            raise TactileCaptureError(f"multiple tactile sources select endpoint {endpoint}")
        endpoints.add(endpoint)
        pads = source.get("pads")
        if not isinstance(pads, dict) or not pads:
            raise TactileCaptureError(f"source {source_id} requires an explicit pads mapping")
        for pad_id, field in pads.items():
            if not isinstance(pad_id, str) or not pad_id:
                raise TactileCaptureError("pad ids must be non-empty strings")
            if not isinstance(field, (str, int)) or isinstance(field, bool):
                raise TactileCaptureError(f"pad {pad_id} field must be a JSON key or value index")
            pad_ids.append(pad_id)
        fields = list(pads.values())
        if len(fields) != len({(type(field).__name__, field) for field in fields}):
            raise TactileCaptureError(f"source {source_id} maps multiple pads to one field")
    if len(pad_ids) != 2 or len(set(pad_ids)) != 2:
        raise TactileCaptureError("tactile config must map exactly two distinct pressure pads")
    window_ms = config.get("sample_window_ms", 300)
    minimum = config.get("minimum_samples", 5)
    if not isinstance(window_ms, int) or not 100 <= window_ms <= 2000:
        raise TactileCaptureError("sample_window_ms must be an integer in 100..2000")
    if not isinstance(minimum, int) or not 5 <= minimum <= 100:
        raise TactileCaptureError("minimum_samples must be an integer in 5..100")


def parse_frame(line: bytes | str):
    if isinstance(line, bytes):
        try:
            text = line.decode("utf-8")
        except UnicodeDecodeError as exc:
            raise TactileCaptureError("pressure frame is not UTF-8") from exc
    else:
        text = line
    text = text.strip()
    if not text:
        raise TactileCaptureError("pressure frame is empty")
    if text[0] in "[{":
        try:
            value = json.loads(text)
        except json.JSONDecodeError as exc:
            raise TactileCaptureError(f"invalid pressure JSON frame: {exc}") from exc
        if not isinstance(value, (list, dict)):
            raise TactileCaptureError("pressure JSON frame must be an array or object")
        return value
    values = [float(value) for value in NUMBER_RE.findall(text)]
    if not values:
        raise TactileCaptureError("pressure frame contains no numeric value")
    return values


def extract_pad_values(frame, pads: dict) -> dict[str, float]:
    values = {}
    for pad_id, field in pads.items():
        try:
            value = frame[field]
        except (IndexError, KeyError, TypeError) as exc:
            raise TactileCaptureError(f"pressure frame has no field {field!r} for pad {pad_id}") from exc
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise TactileCaptureError(f"pressure field {field!r} for pad {pad_id} is not numeric")
        value = float(value)
        if not math.isfinite(value):
            raise TactileCaptureError(f"pressure field {field!r} for pad {pad_id} is non-finite")
        values[pad_id] = value
    return values


class SerialLineReader:
    def __init__(self, source: dict):
        self._fd = os.open(source["device"], os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        attrs = termios.tcgetattr(self._fd)
        attrs[0] = 0
        attrs[1] = 0
        attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        attrs[3] = 0
        attrs[4] = SUPPORTED_BAUD[source.get("baud", 115200)]
        attrs[5] = attrs[4]
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 1
        termios.tcsetattr(self._fd, termios.TCSANOW, attrs)
        termios.tcflush(self._fd, termios.TCIFLUSH)
        self._buffer = bytearray()

    def readline(self, deadline: float) -> bytes | None:
        while time.monotonic() < deadline:
            newline = self._buffer.find(b"\n")
            if newline >= 0:
                line = bytes(self._buffer[:newline])
                del self._buffer[: newline + 1]
                return line.rstrip(b"\r")
            ready, _, _ = select.select([self._fd], [], [], min(0.05, deadline - time.monotonic()))
            if not ready:
                continue
            chunk = os.read(self._fd, 4096)
            if chunk:
                self._buffer.extend(chunk)
                if len(self._buffer) > 65536:
                    raise TactileCaptureError("pressure serial frame exceeds 64 KiB")
        return None

    def close(self) -> None:
        os.close(self._fd)


class PyUsbCh340LineReader:
    """Small user-space CH340 boundary for kernels without ch341.ko."""

    def __init__(self, source: dict):
        try:
            import usb.core
            import usb.util
        except ImportError as exc:
            raise TactileCaptureError(
                "pyusb_ch340 transport requires the python3-usb/PyUSB package"
            ) from exc
        self._usb_util = usb.util
        self._timeout_type = usb.core.USBTimeoutError
        requested_path = source["usb_path"]
        devices = usb.core.find(find_all=True, idVendor=0x1A86, idProduct=0x7523)
        self._device = next(
            (device for device in devices if self._path(device) == requested_path), None
        )
        if self._device is None:
            raise TactileCaptureError(f"CH340 {requested_path} is not present")
        if self._device.is_kernel_driver_active(0):
            raise TactileCaptureError(
                f"CH340 {requested_path} already has a kernel owner; use its tty instead"
            )
        self._device.set_configuration()
        interface = self._device.get_active_configuration()[(0, 0)]
        self._endpoint = next(
            endpoint
            for endpoint in interface
            if usb.util.endpoint_direction(endpoint.bEndpointAddress) == usb.util.ENDPOINT_IN
            and usb.util.endpoint_type(endpoint.bmAttributes) == usb.util.ENDPOINT_TYPE_BULK
        )
        # CH340 vendor register values for 115200 8N1, matching ch341-compatible
        # implementations. Any failed transfer aborts before a pressure sample is trusted.
        self._device.ctrl_transfer(0x40, 0xA1, 0x0000, 0x0000)
        self._device.ctrl_transfer(0x40, 0x9A, 0x1312, 0xCC83)
        self._device.ctrl_transfer(0x40, 0x9A, 0x0F2C, 0x0008)
        self._device.ctrl_transfer(0x40, 0x9A, 0x2518, 0x00C3)
        self._device.ctrl_transfer(0x40, 0xA4, 0x00FF, 0x0000)
        self._buffer = bytearray()

    @staticmethod
    def _path(device) -> str:
        ports = ".".join(str(value) for value in device.port_numbers)
        return f"{device.bus}-{ports}"

    def readline(self, deadline: float) -> bytes | None:
        while time.monotonic() < deadline:
            newline = self._buffer.find(b"\n")
            if newline >= 0:
                line = bytes(self._buffer[:newline])
                del self._buffer[: newline + 1]
                return line.rstrip(b"\r")
            try:
                self._buffer.extend(bytes(self._endpoint.read(4096, timeout=50)))
            except self._timeout_type:
                continue
            except Exception as exc:  # noqa: BLE001 - report backend-specific USB errors.
                raise TactileCaptureError(f"CH340 bulk read failed: {exc}") from exc
            if len(self._buffer) > 65536:
                raise TactileCaptureError("pressure USB frame exceeds 64 KiB")
        return None

    def close(self) -> None:
        self._usb_util.dispose_resources(self._device)


def open_reader(source: dict):
    if source["transport"] == "serial":
        return SerialLineReader(source)
    return PyUsbCh340LineReader(source)


def capture_source(source: dict, deadline: float, minimum_samples: int, reader_factory) -> dict:
    reader = reader_factory(source)
    collected = {pad_id: [] for pad_id in source["pads"]}
    last_stamp_ns = 0
    parse_errors = 0
    try:
        while time.monotonic() < deadline:
            line = reader.readline(deadline)
            if line is None:
                break
            try:
                values = extract_pad_values(parse_frame(line), source["pads"])
            except TactileCaptureError:
                parse_errors += 1
                continue
            last_stamp_ns = time.time_ns()
            for pad_id, value in values.items():
                collected[pad_id].append(value)
    finally:
        reader.close()
    for pad_id, values in collected.items():
        if len(values) < minimum_samples:
            raise TactileCaptureError(
                f"source {source['id']} pad {pad_id} produced {len(values)} valid samples; "
                f"need {minimum_samples} (parse_errors={parse_errors})"
            )
    return {"values": collected, "sensor_stamp_ns": last_stamp_ns, "parse_errors": parse_errors}


def capture(config: dict, *, reader_factory=open_reader) -> dict:
    validate_config(config)
    received_start_ns = time.time_ns()
    deadline = time.monotonic() + config.get("sample_window_ms", 300) / 1000.0
    minimum = config.get("minimum_samples", 5)
    results = {}
    errors = []

    def run_source(source: dict) -> None:
        try:
            results[source["id"]] = capture_source(source, deadline, minimum, reader_factory)
        except Exception as exc:  # noqa: BLE001 - all sources must join before reporting.
            errors.append(f"{source['id']}: {exc}")

    threads = [threading.Thread(target=run_source, args=(source,)) for source in config["sources"]]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()
    if errors:
        raise TactileCaptureError("; ".join(errors))

    pads = []
    sensor_stamps_ns = []
    for source in config["sources"]:
        result = results[source["id"]]
        sensor_stamps_ns.append(result["sensor_stamp_ns"])
        for pad_id in source["pads"]:
            values = result["values"][pad_id]
            median = statistics.median(values)
            mad = statistics.median(abs(value - median) for value in values)
            pads.append(
                {
                    "id": pad_id,
                    "raw": median,
                    "median_abs_deviation": mad,
                    "sample_count": len(values),
                    "source_id": source["id"],
                    "parse_errors": result["parse_errors"],
                }
            )
    received_at_ns = time.time_ns()
    sensor_stamp_ns = min(sensor_stamps_ns)
    sample_id = time.strftime("%Y%m%d-%H%M%S", time.localtime(received_at_ns / 1e9))
    sample_id += f"-{received_at_ns % 1_000_000_000:09d}-{os.getpid()}"
    return {
        "ok": True,
        "schema_version": 1,
        "mode": "tactile_observation",
        "sample_id": sample_id,
        "sensor_stamp_ns": sensor_stamp_ns,
        "received_at_ns": received_at_ns,
        "capture_duration_ms": (received_at_ns - received_start_ns) / 1e6,
        "sources": [
            {
                "id": source["id"],
                "transport": source["transport"],
                "endpoint": source.get("usb_path", source.get("device")),
            }
            for source in config["sources"]
        ],
        "pads": pads,
    }


def store_observation(root: Path, observation: dict) -> Path:
    observations = root / "observations"
    observations.mkdir(parents=True, exist_ok=True)
    destination = observations / f"{observation['sample_id']}.json"
    payload = json.dumps(observation, ensure_ascii=False, allow_nan=False, indent=2) + "\n"
    with destination.open("x", encoding="utf-8") as file:
        file.write(payload)
        file.flush()
        os.fsync(file.fileno())
    latest_tmp = root / f".latest-{os.getpid()}.json"
    latest_tmp.write_text(payload, encoding="utf-8")
    os.replace(latest_tmp, root / "latest.json")
    return destination


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--output-root", default="/home/astrabot/workspace/data/sensors/tactile")
    args = parser.parse_args()
    lock_path = Path("/tmp/xr1-tactile-capture.lock")
    try:
        with lock_path.open("a+") as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            observation = capture(load_config(Path(args.config)))
            observation["path"] = str(store_observation(Path(args.output_root), observation))
        print(json.dumps(observation, ensure_ascii=False, allow_nan=False), flush=True)
        return 0
    except (BlockingIOError, TactileCaptureError, OSError) as exc:
        print(json.dumps({"ok": False, "error": str(exc)}, ensure_ascii=False), flush=True)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
