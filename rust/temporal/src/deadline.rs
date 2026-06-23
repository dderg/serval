use std::cell::Cell;
use std::time::Instant;

thread_local! {
    static SOLVE_DEADLINE: Cell<Option<Instant>> = const { Cell::new(None) };
    static DEADLINE_TRUNCATED: Cell<bool> = const { Cell::new(false) };
}

#[must_use = "the deadline is cleared when this guard is dropped"]
pub struct DeadlineGuard {
    previous: Option<Instant>,
}

impl Drop for DeadlineGuard {
    fn drop(&mut self) {
        SOLVE_DEADLINE.with(|d| d.set(self.previous));
    }
}

pub fn scope(deadline: Option<Instant>) -> DeadlineGuard {
    let previous = SOLVE_DEADLINE.with(|d| d.replace(deadline));
    DeadlineGuard { previous }
}

#[must_use]
pub fn current() -> Option<Instant> {
    SOLVE_DEADLINE.with(Cell::get)
}

#[must_use]
pub fn expired() -> bool {
    matches!(current(), Some(d) if Instant::now() >= d)
}

pub fn clear_truncation() {
    DEADLINE_TRUNCATED.with(|t| t.set(false));
}

pub fn mark_truncated() {
    DEADLINE_TRUNCATED.with(|t| t.set(true));
}

#[must_use]
pub fn truncated() -> bool {
    DEADLINE_TRUNCATED.with(Cell::get)
}
