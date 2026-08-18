# Proposal and candidate contracts

The agent-facing boundary is `VisionHarnessProposal`, not a robot pose. The
current schema is deliberately semantic:

```json
{
  "schema_version": 1,
  "task": "grasp",
  "object_id": "yellow_block",
  "intent": "top_down",
  "preferred_closing_axis": null,
  "constraints": {
    "object_visible": true,
    "avoid_table_collision": true
  },
  "reasoning": "...",
  "prediction": "..."
}
```

Run it with:

```bash
xr1-vision plan --proposal examples/grasp_proposal.json
```

`preferred_closing_axis` is only a ranking preference. Rust still generates the
closing-axis families, searches roll over `[0, 2π)`, computes contact geometry,
solves IK and ranks the result. The model never supplies joint angles or a TCP
pose through this contract.

The output schema is version 2. Every `GraspCandidate` carries:

- semantic identity and search strategy;
- separate closing axis and roll;
- approach and grasp positions;
- optional contact pair and required jaw width;
- approach/grasp IK reports;
- feasibility, joint margin, contact quality and score;
- explicit `null` collision/table margins when those models do not exist.

`ServoInput` is a different, lower-level measured-control contract. It binds a
named 3×3 image Jacobian and signal error to one observation frame and produces
a bounded joint delta. It is not an agent task proposal and must not be used as
one.
