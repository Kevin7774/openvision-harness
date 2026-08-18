# ADR 0006: restore real D405 and tactile capabilities

## Status

Accepted, 2026-08-18. Supersedes the hardware-existence conclusion in ADR 0001;
it does not restore the deleted fake runtime.

## Evidence

- `lsusb`: `8086:0b5b Intel RealSense Depth Camera 405`.
- `rs-enumerate-devices -s`: D405 serial `262422270599`, firmware `5.15.1.55`.
- librealsense `2.58.1` and ROS driver `4.58.1` are installed.
- `lsusb -t`: D405 is on a 480M USB 2.0 path; a streaming attempt opened
  `848x480@10` then produced protocol errors, disconnects and re-enumeration.
- Two `1a86:7523` CH340 devices are present. The operator identifies tactile
  pads at 115200, but this kernel has no `ch341` tty driver and the frame
  contract/left-right mapping remain unverified.
- `/dev/ttyUSB0` is the CP2102N right G2 gripper at 2 Mbaud, not tactile.

## Decision

Represent D405 and tactile as real capabilities with explicit health:

- D405: `DEGRADED` on USB 2.0, `HEALTHY` only after USB 3.x sustained-stream verification.
- Tactile: `UNAVAILABLE` until a tty driver and verified query/response decoder exist.
- Never fabricate depth, force or contact to satisfy an interface.
- Near-field or contact-dependent actions fail closed when the required capability is unavailable.

The Rust main line owns capability state and safety decisions. ROS/librealsense and serial access may
remain thin hardware-boundary adapters under ADR 0002.
