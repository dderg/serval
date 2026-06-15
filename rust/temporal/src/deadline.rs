use std::cell::Cell;
use std::time::Instant;

thread_local! {
    static SOLVE_DEADLINE: Cell<Option<Instant>> = const { Cell::new(None) };
}

/// RAII guard restoring the previous deadline on drop. The solve deadline is an
/// absolute `Instant` so it survives the multi-stage temporal call chain without
/// any layer re-subtracting elapsed time. `None` means unbounded — the default
/// outside the live planner, so every existing test keeps today's behavior.
#[must_use = "the deadline is cleared when this guard is dropped"]
pub struct DeadlineGuard {
    previous: Option<Instant>,
}

impl Drop for DeadlineGuard {
    fn drop(&mut self) {
        SOLVE_DEADLINE.with(|d| d.set(self.previous));
    }
}

/// Installs an absolute solve deadline on the current thread for the lifetime of
/// the returned guard. Fan-out workers in `parallel.rs` run on scoped threads
/// that do not inherit this thread-local, so the planner sets the deadline on
/// the worker entry as well via [`with_deadline`]; for the single-threaded path
/// the guard alone suffices.
pub fn scope(deadline: Option<Instant>) -> DeadlineGuard {
    let previous = SOLVE_DEADLINE.with(|d| d.replace(deadline));
    DeadlineGuard { previous }
}

/// The absolute deadline installed on this thread, if any.
#[must_use]
pub fn current() -> Option<Instant> {
    SOLVE_DEADLINE.with(Cell::get)
}

/// True when a deadline is installed and has already passed.
#[must_use]
pub fn expired() -> bool {
    matches!(current(), Some(d) if Instant::now() >= d)
}
