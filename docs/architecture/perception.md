# Perception

`crates/xr1-vision/src/perception.rs`. Input: one observation directory written
by `py/vista_observe.py`. Output: one JSON document on stdout. No state, no
hardware, no network --- so any observation can be re-planned later and the two
answers compared.

## Pipeline

1. **Read** `latest.json` and follow it to `rgb.png`, `depth.npy`,
   `camera_info.json` and `state.json`. Reject the frame if the intrinsics are
   malformed or the image size disagrees with them, rather than producing
   plausible garbage.
2. **Mask** the block by colour --- see below.
3. **Project** each masked pixel through the intrinsics at its depth, with a
   small neighbour search when the exact pixel has no return, then transform into
   `base_link` using the TF that was captured *with the image* (not the TF at
   plan time).
4. **Reject** anything outside the manipulation box
   (x 0.15..0.80, y -0.60..0.60, z 0.76..0.86 m). This is what keeps the floor,
   the wall and the operator out of the object.
5. **Require 40 valid points.** Fewer is reported as
   `yellow target not reliable: N valid pixels` --- an explicit failure, never a
   guess from 5 pixels.
6. **Fit** a median centre, discard outliers, and fit an oriented bounding box
   whose two horizontal axes are forced horizontal. A block standing on end must
   not yield a tilted footprint (this is a test).
7. **Plan** grasps: for each near-horizontal object axis, both directions, and a
   tilt sweep of 0, ±30, ±60, ±90, ±120, ±150, 180°, ask the kinematics for a
   wrist orientation whose closing axis is top-down and aligned. Then for three
   approach clearances (60, 80, 100 mm) solve approach IK, seed the grasp IK from
   it, and keep the pair with the best combined score that also passes the
   grasp-geometry check.

The output carries the diagnostics (`orientation_candidates`,
`approach_ik_count`, `grasp_ik_count`, `geometry_feasible_count`) because "no
plan" has several very different causes and they need to be distinguishable
without a rebuild.

`uses_previous_absolute_pose: false` is in the output on purpose: each cycle must
re-observe. Reusing a coordinate from the previous cycle is how two sessions
once spent an afternoon reconciling numbers taken 67 mm apart.

## The colour mask

`yellow_component_mask()` is the one piece of this code base that is pure
calibration, and it is deliberately not a tunable config value --- every
threshold has a measured frame behind it in a comment.

It is two stages:

1. A **broad** per-pixel test (`max(r,g) >= 30`, `min(r,g) - b >= 8`,
   `|r-g| <= 0.50 max(r,g)`) that only has to be permissive.
2. **Connected components** over that mask, each accepted or rejected as a whole
   on its aggregate: area in 20..5000 px, mean chroma >= 10, and a **two-sided**
   R/G window `0.85 <= sum_r/sum_g <= 1.15`.

Both sides of that window were paid for:

- **Lower bound 0.85.** The previous 0.98 threshold sat *above* the block's own
  measured ratio, so detection flipped between 1973 px and 0 px on sensor noise
  alone (frame `20260818-112115` read 0.9833, the very next frame 0.9762).
  Against the green cube on `20260818-112803`: block 0.9762, cube 0.6424.
- **Upper bound 1.15.** Orange walks straight through a one-sided `r >= 0.85g`
  test, because orange has *more* red than green, not less. The gripper's own
  pads are orange, so once the arm entered frame the mask merged pads into the
  object: `20260818-170043` planned a grasp on a "91.4 x 39.6 mm object" from
  4050 px, against a block that is 51 x 19 x 29 mm. Measured on that frame over
  chromatic pixels only: block 0.9881, upper pad 1.7542, lower pad 1.3212.

Deciding per *component* rather than per *pixel* is the load-bearing choice. A
per-pixel rule cannot express "these 4050 pixels are two pads plus a block"; an
aggregate over a component can.

The three tests in that file assert exactly these numbers. If you widen a
threshold, they fail, which is the point.

## Deliberate non-goals

- **No fusion.** A previous version fused ZED, a wrist D405 and tactile shear
  force. Two of those three sensors do not exist on this robot
  ([ADR 0001](../decisions/0001-delete-tactile-d405-runtime.md)).
- **No tracking.** One frame in, one plan out. The world may have been changed by
  a human or another session between frames, so continuity would be a lie.
- **No learned model.** Nothing here loads weights. The block is found by
  arithmetic that can be audited on a single frame.
