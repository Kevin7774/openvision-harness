# Vision Harness — Step 3 Task Pack + Executive Wiring

**English** | [中文](harness-step-3-taskpack.zh.md)

Step 2 ([foundation](harness-step-2-foundation.md)) built the five ports and moved
the single-machine constants out of the core. Step 3 closes **finding #3**: the
yellow-block detector and its `object_id` gate leave the core and become a task
pack, dispatched through a registry, so a second object needs no core edit.

> [!NOTE]
> Still no hardware. The detector maths is byte-for-byte unchanged and its
> measured regression tests moved with it — this is a *relocation and
> indirection* change, not a perception change. Nothing here is claimed to run on
> the robot; it is claimed to compile, test, and be extensible as software.

---

## What moved

```
task-packs/
└── yellow-block-pick-place/          # NEW crate
    ├── src/detector.rs               #  the measured detector (was perception/yellow.rs)
    └── src/lib.rs                    #  impl TaskSkill: grounds "yellow_block"
crates/xr1-vision/
├── src/perception/yellow.rs          # now a 1-line re-export of the pack
├── src/taskpack.rs                   # NEW: TaskPackRegistry
├── src/proposal.rs                   # gate replaced by registry.can_ground()
├── src/task/executive.rs            # TargetLocked grounds via the registry
└── src/cli.rs                       # NEW `packs` command reports the registry
```

### The detector's canonical home is now the pack

`perception/yellow.rs` used to hold the colour thresholds. It is now:

```rust
pub(super) use yellow_block_pick_place::detector::{component_mask, components};
```

The three in-core call sites (`observe_object`, the servo target mask, the D405
near-field target) keep the same `yellow::…` path and identical behaviour. The
three measured regression tests moved into the pack — no test was lost, they run
in `yellow-block-pick-place` now.

### The hardcoded gate is gone

Before (`proposal.rs`, the Step 1 finding, verbatim):

```rust
if object_id != "yellow_block" {
    return Err(format!(
        "object_id {object_id:?} is not supported by the current measured detector"
    ));
}
```

After — grounding is a registry lookup, and this file names no object:

```rust
if !registry.can_ground(&descriptor) {
    return Err(format!("no task pack can ground object_id {object_id:?} for this task"));
}
```

The `TaskPackRegistry` holds `TaskSkill` ports. `with_default_packs()` ships
exactly the one real pack; `empty()` grounds nothing — proving the core no longer
knows any object by itself (`empty_core_knows_no_objects_on_its_own`).

### The executive grounds through the registry

`TaskExecutive::new` now builds a registry; at `TargetLocked` it grounds the
locked object through it, so an object no pack can handle is rejected *at lock
time* rather than surfacing later as a geometry failure. `new_with_registry` lets
a caller (or a test) inject a different pack set.

---

## Extensibility, proven by test

The property Step 1 said was missing — *add an object without editing the core* —
is now a passing test (`a_registered_pack_grounds_its_object_without_a_core_edit`):

```rust
struct BlueCupPack;                       // a second pack, defined at the call site
impl TaskSkill for BlueCupPack { /* grounds "blue_cup" */ }

let mut registry = TaskPackRegistry::with_default_packs();
registry.register(BlueCupPack);

// same proposal, two registries:
assert!(proposal.grasp_request().is_err());              // default packs: blue_cup unknown
assert!(proposal.grasp_request_with(&registry).is_ok()); // with the new pack: grounds
```

No line of `proposal.rs` grounding logic, `taskpack.rs`, or the executive changed
to make `blue_cup` work — only a `register` call.

And it is observable at runtime, not just in tests:

```
$ xr1-vision packs
["yellow_block.pick_place"]
```

---

## Which Step 1 items this closes

| Step 1 item | Status | Evidence |
|---|---|---|
| #3 `object_id` is a label, not pluggable grounding | **Closed** | registry lookup replaces the hardcoded string |
| #3 yellow detector welded into the core | **Closed** | detector lives in `task-packs/yellow-block-pick-place` |
| Rebuild step 4 "move the yellow-block logic into the first task pack" | **Done** | this crate |

---

## Proof

```
cargo test --workspace
  harness-contracts:               16 passed  (14 unit + 2 examples)
  xr1-vision:                      69 passed  (63 core + 4 registry + 2 grounding)
  yellow-block-pick-place:          5 passed  (3 detector regressions + 2 skill)
  ───────────────────────────────────────────
  total:                           90 passed, 0 failed
cargo clippy --workspace:          clean
xr1-vision packs:                  ["yellow_block.pick_place"]
```

The detector's measured behaviour is preserved exactly — the same three
regression frames (`20260818-…`) still gate it, now from inside the pack.

---

## Still hardware-gated / still open

Unchanged from Step 2, minus finding #3:

- [ ] Split the four large orchestration files (finding #4) — pure software, next candidate
- [ ] `harness-core` / `harness-executive` crate split (now that ports have a real implementor, the seams are drawn from use)
- [ ] `doctor → commission → verify` state machine (needs live discovery)
- [ ] ARM64 `.deb` packaging + root-level CI (needs the target)
- [ ] Live task executive instead of replay (needs the robot)
- [ ] Episode schema, judge registry, shadow / canary / rollback (finding #5)
- [ ] Three-identical-robot "swap only the Profile" validation

---

## Why this shape

The detector move is a *relocation*, not a rewrite: the maths and its measured
tests travel together, so the perception behaviour is provably unchanged while its
ownership moves to where Step 1 said it belongs. The registry indirection is the
smallest change that removes the hardcoded object name from the core — one lookup,
one boxed port, no new object-specific match arms anywhere. Adding the next object
is now a pack, exactly as the rebuild order requires.
