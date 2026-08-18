# Visual servo --- specification for the highest-value missing piece

**Status: proposal and deterministic gate implemented; live loop incomplete.**
The deleted Python loop remains historical context in
[ADR 0003](../decisions/0003-lost-python-pipeline.md). Rust now owns the named
3×3 solve, conditioning check, 0.05 rad hard step ceiling, observation binding,
freshness, URDF limit/margin, fingertip-floor path and sensor-capability gates.
No command is published by `servo-propose`.

Run the boundary against a newly captured state:

```bash
xr1-vision servo-propose \
  --input examples/servo_request.json \
  --state data/vista_runs/<run>/observations/<frame>/state.json
```

The output keeps `execution_authorized=false`. A true
`ready_for_execution_adapter` means only that the Rust checks passed; the
hardware boundary must still re-check live `/joint_states` freshness and command
channel idleness. Self-collision and gripper-body collision are still unmodelled
and are reported explicitly as limitations.

## Why this and not more calibration

Every gate in this workspace compares FK to FK, so all of them agree with each
other and none of them sees the tool-frame error
([ADR 0004](../decisions/0004-tool-frame-error-is-still-open.md)). A pixel-space
loop closes on what the camera sees, and **differencing cancels any constant
extrinsic error** --- it does not need the constant to be right. When it ran, it
took pad↔block error from 106.9 px to 9.1 px (≈ 4.9 mm) in four steps, with a
predicted/actual gain ratio of 0.94--0.96, using **none** of the three
calibration constants.

## Two parts already exist

**One column of the image Jacobian has been measured.** Rotating `arm_6` by −30°
alone moved the pads 252.2 px and `tcp` 140.7 px --- ratio 1.79. That measurement
was originally a *diagnosis* (a ratio ≠ 1.0 means the offset hangs rigidly off
the wrist, so `TIP_CENTER` is wrong). The same procedure over three joints is a
3×3 gain matrix. `py/pad_offset_measure.py` performs the swing.

**Pad detection is already too good.** `perception` reliably finds the two orange
pads as block candidates: 53.1--53.3 mm apart (real blocks on this table are
> 100 mm apart), consistent across observations, and merging into one blob once
closed. In the geometry chain that is a false positive; in the vision chain it is
the reference. Do not write a new detector --- invert the sign of the existing
self-filter.

## `measure`: fit the gain in place

Perturb three joints by ±a small angle around the current pose, one frame each,
and difference:

```
s = [u_block, v_block, depth_at_block]     # signal
q = [three joint angles]                   # control
J = ds/dq                                  # per joint, average both directions
```

Valid only near the pose family it was measured at --- re-measure after moving.
Six small motions, tens of seconds; three orders of magnitude cheaper than a
calibration.

## `step`: one iteration

```
1. observe
2. detect pads + block; e = s_target - s_now
3. dq = 0.5 * J^-1 * e                     # damping 0.5; do not take the whole step
4. propose to the existing deterministic gates -- freshness, step, URDF margin,
   fingertip floor path and required sensor capability.
   The servo proposes; the gates dispose.
5. execute, re-observe, print predicted dpx vs actual dpx
6. converged when |e_px| < threshold and depth is in the grip band -> suggest close
```

Three things that are not optional:

- **`J` must be square.** A 2×3 Jacobian is degenerate in z and `pinv` on it
  *lifts the arm*: seven steps once undid 70 mm of hard-won descent, then put it
  back. Put `dz` in the error vector; the z row comes from numerical
  differentiation of the existing FK, so it introduces no new constant.
- **Pin the target pixel.** The pads occlude the block from above, so the visible
  centroid creeps ~3 px per step until the target is lost. Judge "did it move"
  from the top edge, which is never occluded, not from the centroid.
- **Stop on stall**: three consecutive steps with < 10 % reduction in `|e|` ⇒
  stop and report. The timeout goes *inside* the loop --- `timeout N python3` does
  not bound an rclpy process (it once ran 2 h 25 m past its deadline and ignored
  both SIGINT and SIGTERM).

Keep the gripper **open** throughout; `close` is the last action. Two blobs are
what makes the midpoint computable.

`damp = 0.5` halving each of the three error components *exactly* is the
fingerprint of an exact solve --- it is the cheapest acceptance check that the
square system is wired up correctly.

Two approaches that look reasonable and are not:

- **Alternate descend and servo.** With a 2×3 `J` this is a staircase that walks
  back down every step it climbs. Descent and pixel correction are one action.
- **Pre-compensate the parallax with a bias, then descend blind.** The bias
  vector *is* the image signature of the z change, so the servo obediently undoes
  the descent. It is self-referential; it was tried and it failed.

## How it will fail

| Risk | Symptom | Response |
|---|---|---|
| ZED views the workspace obliquely at pitch +40°, low pixel SNR | ill-conditioned `J`, non-monotonic `\|e\|` | use ZED for coarse alignment only; do not expect mm convergence from it |
| pads merge into one blob | separation outside 35--95 mm | stay open until the final close |
| `J` is only locally valid | predicted and actual signs disagree | damping 0.5 plus a per-step angle cap; on a sign flip, stop and re-measure `J` |
| the servo asks for a pose that is geometrically unreachable | a gate refuses | that refusal is real information --- report it and ask for the block to be moved |
| IK is minutes per call | --- | measure single-step cost first. If > 30 s/step, add `dq` in joint space directly and use the gates only to verify |

## Offline self-check

Run the detector and the solve over frames already on disk, no hardware: assert
that two pad blobs are found on a known frame with separation inside the window,
that `J^-1 J ≈ I`, that `e = 0` yields `dq = 0`, and that the safety layer was
actually called. Seconds, not minutes.

## Then: stacking is easier than grasping

"Put A on B" needs two world coordinates in the geometry chain --- two extrinsic
errors. In pixel space it is a single alignment: drive A's centroid above B's,
match depth, release. Both blocks in one frame means the shared camera error
cancels. Same `step`, different target signal.
