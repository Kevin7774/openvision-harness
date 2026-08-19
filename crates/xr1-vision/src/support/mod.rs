//! Shared orchestration support.
//!
//! Step 1 finding #4 said the big loop files had become the new monoliths, with
//! "CLI argument parsing, JSON parsing, evidence storage, locking, and the
//! adapter subprocess protocol" not separated. These are those five concerns,
//! each in its own module and shared by `cli.rs`, `servo_loop.rs`, and
//! `grasp_loop.rs` instead of copied into all three.

pub mod adapter;
pub mod args;
pub mod evidence;
pub mod runlock;
