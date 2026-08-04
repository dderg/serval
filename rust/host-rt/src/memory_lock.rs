#![allow(unsafe_code)]

//! Pin the host's pages in RAM before the motion pipeline threads exist.
//!
//! Under memory pressure the kernel happily evicts pages belonging to an idle
//! planner or pump thread. Faulting them back in has been measured at 0.3-1.9 s
//! on a Voron 0 host, which is orders of magnitude past the pipeline's
//! deadlines: a move that was queued with a 100 ms lead wakes up with its start
//! time already in the past and the print aborts. `mlockall` removes the
//! failure mode outright rather than widening the lead.
//!
//! The lock covers the whole process — every pipeline thread lives in it — and
//! is engaged exactly once. It is fatal on failure: an unlocked host is not a
//! host we are willing to stream motion from.

use std::ffi::c_int;
use std::fmt;
use std::io;
use std::sync::OnceLock;

/// `MCL_CURRENT` covers everything already mapped, including the stacks of I/O
/// threads started before the lock; `MCL_FUTURE` covers every later mapping,
/// which is what the pipeline threads' stacks and heap growth land in. Either
/// flag alone leaves a hole.
pub const MEMORY_LOCK_FLAGS: c_int = libc::MCL_CURRENT | libc::MCL_FUTURE;

/// `mlockall` seam: returns the raw `errno` on failure. Injectable so tests can
/// exercise the policy without locking the test runner's address space.
pub type MlockAll = fn(flags: c_int) -> Result<(), i32>;

fn mlockall(flags: c_int) -> Result<(), i32> {
    if unsafe { libc::mlockall(flags) } == 0 {
        Ok(())
    } else {
        Err(io::Error::last_os_error().raw_os_error().unwrap_or(0))
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MemoryLockDenied {
    pub errno: i32,
}

impl fmt::Display for MemoryLockDenied {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "mlockall(MCL_CURRENT|MCL_FUTURE) failed: {} (errno {}). Motion pipeline threads must \
             never be paged out: a swap-in stall of a few hundred milliseconds pushes an \
             already-queued move's start time into the past and aborts the print. Raise the host's \
             memory-lock budget (RLIMIT_MEMLOCK): set 'LimitMEMLOCK=infinity' in the [Service] \
             section of the klipper systemd unit and run 'systemctl daemon-reload', or 'ulimit -l \
             unlimited' before launching klippy by hand. \
             'AmbientCapabilities=CAP_IPC_LOCK' in the same unit lifts the limit outright — \
             EtherCAT hosts already carry it for the endpoint's own mlockall.",
            io::Error::from_raw_os_error(self.errno),
            self.errno
        )
    }
}

impl std::error::Error for MemoryLockDenied {}

#[derive(Debug)]
pub struct ProcessMemoryLock {
    mlockall: MlockAll,
    outcome: OnceLock<Result<(), MemoryLockDenied>>,
}

impl ProcessMemoryLock {
    pub const fn new(mlockall: MlockAll) -> Self {
        Self {
            mlockall,
            outcome: OnceLock::new(),
        }
    }

    /// Idempotent: the first outcome is the process's outcome. A klippy restart
    /// re-runs planner startup in the same process and must neither re-issue
    /// the syscall nor be handed a different verdict.
    pub fn engage(&self) -> Result<(), MemoryLockDenied> {
        *self.outcome.get_or_init(|| {
            (self.mlockall)(MEMORY_LOCK_FLAGS).map_err(|errno| MemoryLockDenied { errno })
        })
    }

    /// The only sanctioned way to bring pipeline threads up: `start` runs after
    /// the lock is held, and not at all if it cannot be taken.
    pub fn start_pipeline_threads<T>(
        &self,
        start: impl FnOnce() -> T,
    ) -> Result<T, MemoryLockDenied> {
        self.engage()?;
        Ok(start())
    }
}

pub static HOST_MEMORY_LOCK: ProcessMemoryLock = ProcessMemoryLock::new(mlockall);
