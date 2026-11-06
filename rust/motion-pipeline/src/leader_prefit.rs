//! Leader group fits made ahead of the shaper.
//!
//! A fit group's leader columns depend only on the group and the segments
//! its convolution reads ([`crate::shaper::fit_leader_group`]), so they
//! can be fitted wherever those segments are at hand. The staged pipeline
//! fits each group on a thread between the planning side and the shaper,
//! as soon as the segment that completes its read arrives, and queues the
//! fits for the shaper just before that segment: the shaper, whose thread
//! bounds the live pipeline, takes them instead of fitting the group
//! itself. A group not fitted ahead, or fitted from a different read,
//! matches no key and the shaper fits it as before, so the trajectory is
//! the same either way.
//!
//! The groups chain the way the shaper's do: from the start of an era, and
//! afresh after every item that makes the shaper flush what it holds.

use std::collections::VecDeque;
use std::sync::Arc;

use trajectory::{AxisChainSet, ContinuousSegment};

use crate::lowering::FitTol;
use crate::shaper::{
    ColumnTrack, FitBoundary, LeaderGroupKey, Lookahead, Shaper, SignalBounds, fit_group_ends,
    fit_leader_group, leader_columns, reads_within,
};
use crate::types::{BaseItem, Control};
use crate::worker_pool::WorkerPool;

pub(crate) type LeaderGroupFits = Vec<(LeaderGroupKey, Vec<ColumnTrack>)>;

pub(crate) struct LeaderPrefitter {
    chains: AxisChainSet,
    fit_tol: FitTol,
    pool: Arc<WorkerPool>,
    era: u64,
    segments: VecDeque<ContinuousSegment>,
    trimmed: bool,
    group_start: usize,
}

impl LeaderPrefitter {
    pub(crate) fn of(shaper: &Shaper) -> Self {
        Self {
            chains: shaper.chains().clone(),
            fit_tol: shaper.fit_tol(),
            pool: Arc::clone(shaper.pool()),
            era: shaper.leader_prefit_era(),
            segments: VecDeque::new(),
            trimmed: false,
            group_start: 0,
        }
    }

    /// The leader group fits `item` completes the read of, to reach the
    /// shaper before `item` does.
    pub(crate) fn prefit(&mut self, item: &BaseItem) -> LeaderGroupFits {
        match item {
            BaseItem::Seg(segment) => {
                self.segments.push_back(segment.clone());
                return self.fit_ready_groups();
            }
            BaseItem::Drain => {}
            BaseItem::Control(Control::Reset { .. }) => {
                self.segments.clear();
                self.trimmed = false;
                self.era += 1;
            }
            BaseItem::Control(Control::SetAxisChains(chains)) => {
                if chains.back_support() > self.chains.back_support() {
                    self.segments.clear();
                    self.trimmed = false;
                }
                self.chains = chains.clone();
                self.era += 1;
            }
            BaseItem::Control(
                Control::Dwell { .. } | Control::SetMesh { .. } | Control::Dispatch(_),
            ) => {}
        }
        self.group_start = self.segments.len();
        Vec::new()
    }

    fn fit_ready_groups(&mut self) -> LeaderGroupFits {
        let mut fits = Vec::new();
        loop {
            self.segments.make_contiguous();
            let (segments, _) = self.segments.as_slices();
            let Some(end) = fit_group_ends(&segments[self.group_start..]).next() else {
                break;
            };
            let group = &segments[self.group_start..self.group_start + end];
            let started = crate::timing::stopwatch();
            let Some(group_fits) = self.fit_group(group, segments) else {
                break;
            };
            crate::timing::record_stage(crate::timing::Stage::LeaderPrefit, started.elapsed_us());
            fits.extend(group_fits);
            self.group_start += end;
            self.trim();
        }
        fits
    }

    /// Every kerneled leader column's fit of `group`, or `None` while the
    /// segments at hand do not reach past every read, or where a fit fails
    /// (the shaper fits the group then, and reports the failure itself).
    fn fit_group(
        &self,
        group: &[ContinuousSegment],
        segments: &[ContinuousSegment],
    ) -> Option<LeaderGroupFits> {
        let columns = leader_columns(&self.chains, group[0].axes.len());
        let window = SignalBounds {
            first_t: segments[0].t_start,
            last_t: segments[segments.len() - 1].t_end,
            boundary: if self.trimmed {
                FitBoundary::Interior
            } else {
                FitBoundary::StreamBoundary
            },
            lookahead: Lookahead::Required,
        };
        if !reads_within(&columns, group, window) {
            return None;
        }
        if columns.is_empty() {
            return Some(Vec::new());
        }
        let window_segments: Vec<&ContinuousSegment> = segments.iter().collect();
        let pool = &*self.pool;
        pool.map_indexed(
            columns.len(),
            columns.len(),
            || (),
            |(), index| {
                fit_leader_group(
                    columns[index],
                    group,
                    &window_segments,
                    window,
                    (self.era, self.fit_tol),
                    pool,
                )
            },
        )
        .expect("leader prefit thread panicked")
        .into_iter()
        .flatten()
        .collect::<Result<_, _>>()
        .ok()
    }

    /// Drops the segments no read of a group from `group_start` on reaches,
    /// keeping, as the shaper's history does, the one a read starting on a
    /// segment's start needs before it.
    fn trim(&mut self) {
        let Some(next) = self.segments.get(self.group_start) else {
            return;
        };
        let keep_after = next.t_start - self.chains.back_support();
        while self.segments.len() >= 2 && self.segments[1].t_start < keep_after {
            self.segments.pop_front();
            self.group_start -= 1;
            self.trimmed = true;
        }
    }
}
