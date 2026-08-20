//! The single robot-action-loop lock.
//!
//! Step 1 finding #4: locking was tangled into the orchestration files. Worse,
//! `servo_loop.rs` and `grasp_loop.rs` each had their own lock type
//! (`ServoLoopLock`, `GraspLoopLock`) that opened **the same lock file** —
//! `xr1-robot-action-loop.lock` — through two slightly different
//! implementations. Only one loop may move the robot at a time, so this is a
//! safety-critical mutual exclusion primitive; two copies of it is two chances
//! for that guarantee to drift apart.
//!
//! They are now one type. Where the two versions differed, this keeps the more
//! informative behaviour: report who holds the lock, and preserve the OS error
//! text when locking fails for a reason other than contention.

use std::fs::{self, File, OpenOptions};
use std::io::Write;

/// Held for as long as a loop may command the robot. Dropping it releases the
/// advisory lock with the file descriptor.
pub struct RobotActionLoopLock {
    _file: File,
}

impl RobotActionLoopLock {
    /// Acquire the exclusive robot-action lock, or fail with who holds it.
    ///
    /// `loop_name` appears in errors ("visual-servo loop", "grasp-feedback
    /// loop"); `session_id` is recorded in the lock file so the next contender
    /// can name the holder.
    pub fn acquire(loop_name: &str, session_id: &str) -> Result<Self, String> {
        let path = std::env::temp_dir().join("xr1-robot-action-loop.lock");
        let mut file = OpenOptions::new()
            .read(true)
            .write(true)
            .create(true)
            .truncate(false)
            .open(&path)
            .map_err(|error| format!("{}: {error}", path.display()))?;
        if !try_lock_exclusive(&file, loop_name)? {
            let holder = fs::read_to_string(&path).unwrap_or_else(|_| "unknown".into());
            return Err(format!(
                "another robot action loop is active (holder={})",
                holder.trim()
            ));
        }
        file.set_len(0)
            .map_err(|error| format!("{}: {error}", path.display()))?;
        writeln!(file, "pid={} session_id={session_id}", std::process::id())
            .map_err(|error| format!("{}: {error}", path.display()))?;
        file.flush()
            .map_err(|error| format!("{}: {error}", path.display()))?;
        Ok(Self { _file: file })
    }
}

/// `Ok(false)` means another process holds the lock; `Err` means locking itself
/// failed and the caller must not proceed.
#[cfg(unix)]
fn try_lock_exclusive(file: &File, loop_name: &str) -> Result<bool, String> {
    use std::os::fd::AsRawFd;

    const LOCK_EX: i32 = 2;
    const LOCK_NB: i32 = 4;
    extern "C" {
        fn flock(fd: i32, operation: i32) -> i32;
    }
    // SAFETY: file owns a valid descriptor for the duration of this call, and
    // flock does not retain the pointer or access Rust-managed memory.
    let result = unsafe { flock(file.as_raw_fd(), LOCK_EX | LOCK_NB) };
    if result == 0 {
        Ok(true)
    } else {
        let error = std::io::Error::last_os_error();
        match error.kind() {
            std::io::ErrorKind::WouldBlock => Ok(false),
            _ => Err(format!("cannot lock {loop_name}: {error}")),
        }
    }
}

#[cfg(not(unix))]
fn try_lock_exclusive(_file: &File, loop_name: &str) -> Result<bool, String> {
    Err(format!(
        "{loop_name} locking is unsupported on this platform"
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_lock_is_exclusive_and_names_its_holder() {
        let first = RobotActionLoopLock::acquire("visual-servo loop", "session-a")
            .expect("first acquisition should succeed");
        // Both loops share one lock file, so the grasp loop must be refused while
        // the servo loop holds it — that is the whole point of the primitive.
        let second = RobotActionLoopLock::acquire("grasp-feedback loop", "session-b");
        let error = match second {
            Ok(_) => panic!("second acquisition must be refused while the first is held"),
            Err(error) => error,
        };
        assert!(error.contains("another robot action loop is active"));
        assert!(
            error.contains("session-a"),
            "holder should be named: {error}"
        );

        drop(first);
        // Released with the descriptor, so the next loop can run.
        RobotActionLoopLock::acquire("grasp-feedback loop", "session-c")
            .expect("lock should be free after drop");
    }
}
