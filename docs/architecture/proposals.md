# Task, observation and candidate contracts

The agent-facing boundary is `TaskProposal`, not a robot pose. Schema version 2
keeps the natural-language command, the grounded target, an optional semantic
destination, constraints and explicit success predicates:

```json
{
  "schema_version": 2,
  "task": "pick_place",
  "command": "把黄色方块放到绿色托盘里面",
  "target": {
    "object_id": "yellow_block",
    "description": "黄色方块"
  },
  "destination": {
    "relation": "inside",
    "reference": {
      "object_id": "green_tray",
      "description": "绿色托盘"
    }
  },
  "grasp": {
    "intent": "top_down",
    "preferred_closing_axis": null
  },
  "constraints": {
    "object_visible": true,
    "avoid_table_collision": true
  },
  "success_predicates": [
    {"type": "target_held"},
    {"type": "target_at_destination"}
  ],
  "reasoning": "...",
  "prediction": "..."
}
```

Validate it with:

```bash
xr1-vision validate-proposal --proposal examples/pick_place_proposal.json
```

Schema version 1 grasp files are still accepted and are normalised to version
2 at this boundary. A proposal may remain unresolved while an agent is
reasoning, but geometry planning requires `target.object_id`. The current
measured perception backend only grounds `yellow_block`; another detector can
be added behind the same query without changing task or execution types.

`TaskProposal::grasp_request()` is the downward boundary. It strips away the
destination and reasoning text before perception and planning. The model never
supplies Cartesian coordinates, joint angles or a TCP orientation.

## ObservationBundle

`xr1-vision bundle` reads one immutable `latest.json` and emits a single typed
bundle containing:

- ZED RGB, aligned depth and camera-intrinsic artifact paths;
- image and receive timestamps plus the image-time TF;
- named joint positions and any gripper readings captured in that frame;
- current D405 and tactile capability health.

The bundle rejects frame/timestamp mismatches, RGB/depth skew above 50 ms,
clock offset above 2 s, malformed TF and an observation without image-time TF.
D405 and tactile health are present even while their frame streams are not.

## Deterministic task replay

The task executive consumes timestamped events and owns this order:

```text
OBSERVE -> LOCK_TARGET -> GEOMETRY -> GENERATE_GRASP -> VALIDATE_GRASP
-> APPROACH -> SERVO -> GRASP -> LIFT -> VERIFY_GRASP
-> LOCK_DESTINATION -> PLACE_GEOMETRY -> GENERATE_PLACE -> VALIDATE_PLACE
-> PLACE -> VERIFY_PLACE -> COMPLETE
```

Zero feasible candidates or failed evidence enters `DIAGNOSE`. A diagnosis
must name a minimal repair and may resume only at an evidence-producing stage;
events cannot skip directly to physical actions. Replay a recorded event stream:

```bash
xr1-vision replay \
  --proposal examples/pick_place_proposal.json \
  --events examples/task_events.jsonl
```

This command is pure replay and does not move hardware. Live orchestration will
write the same event contract one action and one post-action observation at a
time.

Visual servoing advances through `ServoStepReconciled`, not through an
unobserved motion-complete event. Each record binds the executive's current
`before_frame_id` to a distinct `after_frame_id` and stores the prediction
match and improvement ratio. A matched, improving step remains in `SERVO`;
convergence advances to `GRASP`; a prediction mismatch or three-step stall
enters `DIAGNOSE`. The snapshot carries the latest frame and reconciliation
result so a replay cannot silently reuse a pre-action observation.

## Grasp candidates

`preferred_closing_axis` is only a ranking preference. Rust still generates the
closing-axis families, searches roll over `[0, 2π)`, computes contact geometry,
solves IK and ranks the result. Closing axis and roll remain separate fields.

Every `GraspCandidate` carries semantic identity, approach/grasp positions,
contacts, jaw width, approach/grasp IK, joint margin, contact quality, score and
diagnostics. `plan --moveit` sends all IK-complete candidates through the XR1
MoveIt PlanningScene bridge and fills self/path collision plus table-clearance
measurements before an execution adapter may select one.

`ServoInput` is a different measured-control contract. It binds a named 3×3
image Jacobian and signal error to one observation frame and produces a bounded
joint delta. `JacobianMeasurementInput` requires one +/- sample pair per named
joint; `ReconciliationInput` binds that proposal to distinct before/after
signals. None of these are agent task proposals.
