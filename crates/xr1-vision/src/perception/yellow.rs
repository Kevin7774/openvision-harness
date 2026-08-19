//! Yellow-block detector — now owned by the `yellow-block-pick-place` task pack.
//!
//! Step 1 finding #3: the measured colour thresholds were welded into the core.
//! They moved to `task-packs/yellow-block-pick-place/src/detector.rs`. This file
//! is a thin re-export so the three in-core call sites (`observe_object`, the
//! servo target mask, the D405 near-field target) keep the same `yellow::…` path
//! and behaviour. The regression tests moved with the code they guard.

pub(super) use yellow_block_pick_place::detector::{component_mask, components};
