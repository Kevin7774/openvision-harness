# ADR 0002 --- Python stays at the ROS 2 / hardware boundary

- Date: 2026-08-18
- Status: Accepted

## Context

The mandate is Rust-first. `py/` is ~2500 lines of Python that talks to the
robot: `xr1.py` (the live API: status, bringup, home, pose, grip),
`astra_arm.py` (the motion safety layer), `vista_observe.py` (ZED snapshot),
`xr1_cam.py` (the Mac recorder), `pad_offset_measure.py`.

What it depends on:

- **`rclpy`** --- for `/joint_states`, TF lookups at image time, and the vendor
  command topics. The Rust DDS crates do not speak the vendor's Fast-DDS profile
  on this platform, and the vendor overlay is **binary-only** (33 ELF, 0 Python),
  so there is no message definition to regenerate from.
- **`pyzed`** --- the ZED SDK's only supported binding here.
- The vendor **`astra_arm`** SDK, which is Python.

Rewriting this in Rust would mean re-deriving a DDS profile and an SDK ABI by
observation, against a stack that self-heals in 100 ms and cannot be stepped
through. That moves risk; it does not remove it.

## Decision

Python is permanent in exactly one role: the ROS 2 / hardware boundary. It stays
thin. **No decisions live there** --- no thresholds, no grasp scoring, no
geometry. Those are in `crates/`.

New logic does not go in `py/`. If you need a number computed, compute it in
Rust and pass it down.

## Consequences

- Two languages, and the seam between them is a file: Rust writes and reads
  JSON/PNG/NPY under `data/`, Python does the I/O. That seam is also the
  evidence trail, which is why it is worth having.
- `py/astra_arm.py` is the one Python file that *is* safety-critical (it ramps
  from the measured pose, caps velocity, clamps to the live URDF and refuses on
  stale `/joint_states`). It is exempt from "thin", and changes to it need the
  same care as a Rust safety gate.
- If the vendor ever ships message definitions, revisit.
