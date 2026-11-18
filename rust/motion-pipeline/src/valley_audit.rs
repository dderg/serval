//! Records where the advance valleys release a ratio-ramped advance: each
//! crawl valley as laid, and the path every follower whose advance reads
//! the valleys' weight reads it below 1 over. Compiled only for the tests
//! and the snapshot review that measure them; the live host never pays
//! for it.

use std::sync::Mutex;

/// A crawl valley as [`crate::follower_projection`] lays it: its crawl, how
/// the leaders cross it (path speed and the magnitude of its rate), the
/// leaders' reach there, and where it holds the weight at zero before any
/// later crawl widens it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LaidCrawl {
    pub turn: f64,
    pub speed: f64,
    pub rate: f64,
    pub reach: f64,
    pub zero: (f64, f64),
}

/// The path the valleys release the advance over, in mm: where any valley
/// reads a weight below 1, and where a crawl valley does.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Released {
    pub any: f64,
    pub crawl: f64,
}

static LAID: Mutex<Vec<LaidCrawl>> = Mutex::new(Vec::new());
static RELEASED: Mutex<Released> = Mutex::new(Released {
    any: 0.0,
    crawl: 0.0,
});

/// Every crawl valley laid since the previous call, by any thread: two
/// audited runs in one process take each other's.
pub fn take_laid() -> Vec<LaidCrawl> {
    std::mem::take(&mut *LAID.lock().expect("valley audit poisoned"))
}

/// The path released since the previous call, by any thread.
pub fn take_released() -> Released {
    std::mem::take(&mut *RELEASED.lock().expect("valley audit poisoned"))
}

pub(crate) fn lay(crawl: LaidCrawl) {
    LAID.lock().expect("valley audit poisoned").push(crawl);
}

pub(crate) fn release(path: Released) {
    let mut released = RELEASED.lock().expect("valley audit poisoned");
    released.any += path.any;
    released.crawl += path.crawl;
}

/// The length of the union of `stretches`.
pub(crate) fn union_length(mut stretches: Vec<(f64, f64)>) -> f64 {
    stretches.sort_by(|a, b| a.0.total_cmp(&b.0));
    let mut total = 0.0;
    let mut open: Option<(f64, f64)> = None;
    for (lo, hi) in stretches {
        open = match open {
            Some((start, end)) if lo <= end => Some((start, end.max(hi))),
            Some((start, end)) => {
                total += end - start;
                Some((lo, hi))
            }
            None => Some((lo, hi)),
        };
    }
    total + open.map_or(0.0, |(start, end)| end - start)
}
