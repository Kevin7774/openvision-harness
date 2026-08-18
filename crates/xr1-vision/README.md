# Vision Harness

Rust-first, fail-closed manipulation runtime for Thor.

The runtime consumes synchronized ZED, D405, tactile and robot-state observations.
Perception and local policy may propose targets or bounded Cartesian corrections;
deterministic planning and safety gates retain final authority.

## Commands

```bash
vision-harness hardware-status
vision-harness runtime-evaluate --input examples/sensor_frame.json --proposal examples/model_proposal.json
vision-harness runtime-stream
```

`runtime-stream` accepts one JSON object per line containing `frame` and an optional
`proposal`. Sensor timestamps are nanoseconds in the robot clock domain. Stale,
non-finite, collision-rejected or out-of-envelope input fails closed.

## Hardware

- ZED: ROS2 RGB, depth, point cloud, camera info and TF.
- D405: librealsense 2.58.1 / realsense2_camera 4.58.1, serial `262422270599`.
- Tactile: CP2102N serial aggregator at `/dev/ttyUSB0`, 115200 baud. Payload framing
  remains fail-closed until a verified query/response contract is available.
