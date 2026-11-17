//! What [`RatioSmoothing`] reports to [`crate::valley_audit`].

use geometry::{AdvanceValley, ValleyKind};

use super::{Crossing, RatioSmoothing};
use crate::valley_audit::{LaidCrawl, Released, lay, release, union_length};

/// The valleys pruned so far and the furthest path laid: what the advance
/// read released is known once the smoothing is dropped.
#[derive(Debug, Default)]
pub(super) struct ValleyLog {
    pruned: Vec<AdvanceValley>,
    path_end: f64,
}

impl ValleyLog {
    pub(super) fn prune(&mut self, valleys: &[AdvanceValley], s: f64) {
        self.pruned.extend(
            valleys
                .iter()
                .filter(|valley| valley.reach_end() < s)
                .copied(),
        );
    }

    pub(super) fn lay_through(&mut self, table_end: f64) {
        self.path_end = self.path_end.max(table_end);
    }
}

pub(super) fn lay_crawl(turn: f64, crossing: Crossing, reach: f64, zero: (f64, f64)) {
    lay(LaidCrawl {
        turn,
        speed: crossing.speed,
        rate: crossing.rate,
        reach,
        zero,
    });
}

impl Drop for RatioSmoothing {
    fn drop(&mut self) {
        if !self.advance_reads_the_weight() {
            return;
        }
        let path_end = self.audit.path_end;
        let released = |kinds: &[ValleyKind]| {
            union_length(
                self.audit
                    .pruned
                    .iter()
                    .chain(&self.valleys)
                    .filter(|valley| kinds.contains(&valley.kind))
                    .map(|valley| {
                        (
                            valley.reach_start().max(0.0),
                            valley.reach_end().min(path_end),
                        )
                    })
                    .filter(|(lo, hi)| lo < hi)
                    .collect(),
            )
        };
        release(Released {
            any: released(&[ValleyKind::Crawl, ValleyKind::Rest]),
            crawl: released(&[ValleyKind::Crawl]),
        });
    }
}
