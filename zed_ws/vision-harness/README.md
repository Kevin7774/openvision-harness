# Vision Harness

Rust runtime for low-latency manipulation on Thor. Existing `observe` and `plan`
commands remain the global vision path. `runtime-evaluate` is the stable local
boundary for synchronized stereo, D405, tactile, robot-state and local-policy
outputs.

## Runtime evaluation

```bash
vision-harness runtime-evaluate \
  --input examples/sensor_frame.json \
  --proposal examples/model_proposal.json
```

The runtime is fail-closed. It stops on stale sensor data, collision rejection,
low joint-limit margin, excessive tactile shear, non-finite model output, or a
servo step outside the configured Cartesian/yaw envelope. The local model only
proposes bounded end-effector deltas; it never bypasses deterministic safety
checks.

Sensor timestamps use nanoseconds in the robot clock domain. Production capture
must synchronize stereo, D405, tactile and joint state before constructing a
`SensorFrame`.
