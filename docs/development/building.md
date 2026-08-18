# Building and checking

## Toolchain

There is **no rustup** on this machine. Rust is the Ubuntu 24.04 distro
toolchain, and the lint tools must match its version exactly or they refuse to
run:

```
rustc 1.75.0    /usr/bin/rustc      (apt: rustc)
cargo 1.75.0    /usr/bin/cargo      (apt: cargo)
rustfmt 1.7.0   /usr/bin/rustfmt    (apt: rustfmt)
clippy 0.1.75   /usr/bin/cargo-clippy (apt: rust-clippy)
```

`edition = "2021"` and `rust-version = "1.75"` in `[workspace.package]` are the
real ceiling, not a preference. Anything needing a newer compiler needs a
toolchain decision first, and an ADR.

Offline: `~/.cargo/registry` already holds these four and their transitive tail
(`image`, `nalgebra`, `roxmltree`, `serde`/`serde_json`). Adding a fifth needs
network, which this machine may not have.

## The four gates

All four must pass. In order of how fast they fail:

```bash
cargo fmt --all -- --check
cargo check --workspace
cargo clippy --workspace --all-targets --all-features
cargo test --workspace
```

Clippy is deny-level in practice: treat every warning as a failure. Two useful
ones it already caught here --- `approx_constant` (a hand-written `1.5708` that
should have been `FRAC_PI_2`, in a list that turned out to be better generated
than written) and `items_after_test_module`.

```bash
cargo build --release       # -> target/release/xr1-vision, 0 warnings
```

The release profile is `strip`, `lto`, `codegen-units = 1`. Debug builds are
fine for tests; use release on the robot, since IK runs a multi-seed
damped-least-squares solve and the debug build is minutes slower per call.

## Tests

```bash
cargo test --workspace          # 3 tests, all in perception
```

They are small on purpose and none of them mock the robot. Each one encodes a
threshold that was **measured off a named frame**, so if someone retunes the mask
by feel, a test says which physical observation they just broke:

| Test | What breaks it |
|---|---|
| `mask_keeps_the_yellow_block_and_drops_the_green_cube` | widening the R/G window until the green cube is a candidate |
| `orange_pads_are_not_yellow_but_the_block_still_is` | the gripper's own pads becoming grasp targets (this happened) |
| `footprint_axes_stay_horizontal_for_a_block_standing_on_end` | letting the oriented box tilt, which makes the closing-axis solve meaningless |

The kinematics and the plan path have no unit tests, and pretending otherwise
would be worse than the gap: their inputs are a live URDF and a live TF tree,
and their only meaningful oracle is the robot. What guards them instead is
`xr1-vision plan`, which is a dry run by default --- it prints ranked candidates
and moves nothing. That is the check to run after touching either file.

## Not yet installed

`cargo-deny`, `cargo-audit`, `cargo-nextest`, `cargo-machete`. Each wants
network and a newer toolchain than 1.75 for its current release. With four
dependencies and one crate, `cargo tree` and rustc's own dead-code pass cover
what they would tell us today; revisit when the workspace grows a second crate
or a dependency with a transitive tail.
