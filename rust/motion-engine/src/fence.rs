//! In-band sequence points over the streaming pipeline.
//!
//! A fence answers "what stream time does the motion submitted so far end
//! at?" without draining the pipeline. It rides the same FIFO channel as the
//! moves, so its position in the stream is exact by construction. An armed
//! fence resolves in one of two ways:
//!
//! - **Dispatch progress** — the consumer dispatched a segment whose source
//!   line is beyond the fence's, so everything at or before the fence is
//!   committed and `dispatched_through` covers it.
//! - **Barrier** — the ingress ran a barrier (drain, flush, reset), after
//!   which everything ahead of every armed fence has been dispatched or
//!   discarded.
//!
//! Progress resolution can be late by one fitter run (a run is labeled with
//! its first move's line), never early.

use std::collections::HashMap;
use std::sync::Mutex;
use std::sync::atomic::{AtomicU64, Ordering};

#[derive(Default)]
struct Armed {
    id: u64,
    after_line: u32,
}

#[derive(Default)]
pub struct FenceRegistry {
    next_id: AtomicU64,
    inner: Mutex<Inner>,
}

#[derive(Default)]
struct Inner {
    armed: Vec<Armed>,
    /// Fence id → stream time its preceding motion ends at; `None` when the
    /// stream was reset (or nothing was ever dispatched) — the caller falls
    /// back to its command-time floor.
    resolved: HashMap<u64, Option<f64>>,
}

impl FenceRegistry {
    pub fn alloc_id(&self) -> u64 {
        self.next_id.fetch_add(1, Ordering::Relaxed)
    }

    pub fn arm(&self, id: u64, after_line: u32) {
        let mut inner = self.inner.lock().unwrap_or_else(|p| p.into_inner());
        inner.armed.push(Armed { id, after_line });
    }

    pub fn has_armed(&self) -> bool {
        !self
            .inner
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .armed
            .is_empty()
    }

    pub fn resolve(&self, id: u64, t: Option<f64>) {
        let mut inner = self.inner.lock().unwrap_or_else(|p| p.into_inner());
        inner.resolved.insert(id, t);
    }

    /// Dispatch-progress hook: a segment labeled `source_line` was dispatched
    /// and the committed timeline now reaches `t_end`.
    pub fn on_dispatch(&self, source_line: u32, t_end: f64) {
        let mut inner = self.inner.lock().unwrap_or_else(|p| p.into_inner());
        if inner.armed.iter().all(|f| f.after_line >= source_line) {
            return;
        }
        let (done, still_armed): (Vec<Armed>, Vec<Armed>) = std::mem::take(&mut inner.armed)
            .into_iter()
            .partition(|f| f.after_line < source_line);
        inner.armed = still_armed;
        for f in done {
            inner.resolved.insert(f.id, Some(t_end));
        }
    }

    /// Barrier hook: everything ahead of every armed fence has been
    /// dispatched (or discarded) through `t`.
    pub fn resolve_armed(&self, t: Option<f64>) {
        let mut inner = self.inner.lock().unwrap_or_else(|p| p.into_inner());
        let armed = std::mem::take(&mut inner.armed);
        for f in armed {
            inner.resolved.insert(f.id, t);
        }
    }

    /// Removes and returns the resolution for `id`; `None` while pending.
    pub fn take(&self, id: u64) -> Option<Option<f64>> {
        let mut inner = self.inner.lock().unwrap_or_else(|p| p.into_inner());
        inner.resolved.remove(&id)
    }
}

#[cfg(test)]
mod tests;
