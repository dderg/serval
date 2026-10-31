//! Accounting for accepted work across independently reporting execution sources.
//!
//! Consumption releases input storage; retirement confirms execution. Interruption
//! abandons unfinished work without counting it as executed. All counts use the
//! same caller-defined work unit and wrap modulo `2^32`.
//!
//! This component owns no queue, clock, transport, or execution policy. The caller
//! commits acceptance after its backend accepts work and supplies absolute source
//! reports and interruption receipts.

/// Absolute consumed and executed odometers for one source.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct Progress {
    pub consumed: u32,
    pub retired: u32,
}

/// Source odometers immediately before and after discarding interrupted work.
///
/// The `after - before` jump is not credited as consumed or executed work.
/// Each receipt must be supplied exactly once.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Cut {
    pub source: usize,
    pub before: Progress,
    pub after: Progress,
}

/// Logical totals, independent of source odometer jumps.
///
/// `pushed` counts accepted work; `retired` counts executed work.
/// `abandoned` counts accepted work discarded without execution, with
/// `abandoned_unconsumed` tracking the portion discarded before consumption.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct Snapshot {
    pub pushed: u32,
    pub consumed: u32,
    pub retired: u32,
    pub abandoned_unconsumed: u32,
    pub abandoned: u32,
}

#[derive(Debug, Default, Clone, Copy)]
struct SourceCredit {
    progress: Progress,
    has_cut: bool,
}

/// Allocation-free accounting with private state and a fixed number of sources.
#[derive(Debug)]
pub struct ExecutionCredit<const SOURCES: usize> {
    totals: Snapshot,
    sources: [SourceCredit; SOURCES],
}

impl<const SOURCES: usize> Default for ExecutionCredit<SOURCES> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const SOURCES: usize> ExecutionCredit<SOURCES> {
    pub fn new() -> Self {
        Self {
            totals: Snapshot::default(),
            sources: [SourceCredit::default(); SOURCES],
        }
    }

    /// Returns a copy of the current logical totals.
    pub fn snapshot(&self) -> Snapshot {
        self.totals
    }

    /// Previews the accepted head without committing acceptance.
    pub fn accepted_head(&self, additional: u32) -> u32 {
        self.totals.pushed.wrapping_add(additional)
    }

    /// Commits accepted work and returns the new accepted head.
    pub fn accept(&mut self, count: u32) -> u32 {
        self.totals.pushed = self.accepted_head(count);
        self.totals.pushed
    }

    /// Incorporates an absolute report from `source`.
    ///
    /// Before a source's first cut, modular regressions remain observable.
    /// Afterwards, each odometer independently ignores deltas above `u32::MAX / 2`
    /// as stale reports; legitimate advances must fit within that half-range.
    ///
    /// Panics if `source >= SOURCES`.
    pub fn observe(&mut self, source: usize, progress: Progress) {
        let credit = &mut self.sources[source];
        let consumed_delta = progress.consumed.wrapping_sub(credit.progress.consumed);
        if !credit.has_cut || consumed_delta <= u32::MAX / 2 {
            self.totals.consumed = self.totals.consumed.wrapping_add(consumed_delta);
            credit.progress.consumed = progress.consumed;
        }
        let retired_delta = progress.retired.wrapping_sub(credit.progress.retired);
        if !credit.has_cut || retired_delta <= u32::MAX / 2 {
            self.totals.retired = self.totals.retired.wrapping_add(retired_delta);
            credit.progress.retired = progress.retired;
        }
    }

    /// Reconciles every receipt for one interruption, then abandons all outstanding
    /// work and returns the newly abandoned count.
    ///
    /// The caller must already have stopped/discarded the interrupted work and must
    /// supply all receipts together, including those from different sources.
    /// An empty receipt set abandons work without changing source baselines.
    /// Cut jumps advance baselines rather than replacing newer observed reports.
    ///
    /// Panics for an out-of-range source; partial changes are not rolled back.
    pub fn interrupt(&mut self, cuts: impl IntoIterator<Item = Cut>) -> u32 {
        for cut in cuts {
            self.observe(cut.source, cut.before);
            let credit = &mut self.sources[cut.source];
            credit.progress.consumed = credit
                .progress
                .consumed
                .wrapping_add(cut.after.consumed.wrapping_sub(cut.before.consumed));
            credit.progress.retired = credit
                .progress
                .retired
                .wrapping_add(cut.after.retired.wrapping_sub(cut.before.retired));
            credit.has_cut = true;
        }
        let abandoned = self.outstanding();
        self.totals.abandoned = self.totals.abandoned.wrapping_add(abandoned);
        self.totals.abandoned_unconsumed = self.totals.pushed.wrapping_sub(self.totals.consumed);
        abandoned
    }

    /// Accepted work still awaiting consumption, excluding abandoned input.
    pub fn awaiting_consumption(&self) -> u32 {
        self.totals
            .pushed
            .wrapping_sub(self.totals.consumed)
            .wrapping_sub(self.totals.abandoned_unconsumed)
    }

    /// Accepted work neither executed nor abandoned.
    pub fn outstanding(&self) -> u32 {
        self.totals
            .pushed
            .wrapping_sub(self.totals.retired)
            .wrapping_sub(self.totals.abandoned)
    }
}

#[cfg(test)]
#[path = "execution_credit_tests.rs"]
mod execution_credit_tests;
