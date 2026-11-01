//! The live pipeline with its shaper on a dedicated thread. Fitting, velocity
//! planning and lowering stay on the caller's thread; the lowerer's items
//! cross a bounded FIFO to the shaper, which emits into the caller-supplied
//! sink. Every item — segments, drains and control tokens — keeps its order,
//! so a barrier still acknowledges only after everything ahead of it has
//! been shaped and emitted, and no arithmetic moves between threads.

use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use crossbeam_channel::{Receiver, Sender, bounded};

use crate::shaper::Shaper;
use crate::{BaseItem, Pipeline, PlanningFront, StreamInput, TrajectoryItem};

/// Lowered items the shaper may lag behind the planner. Bounded so a slow
/// shaper backpressures planning instead of buffering without limit.
pub const SHAPER_INBOX_CAP: usize = 16;

/// How long dropping the pipeline waits for the shaper to finish the items
/// already queued for it before detaching it with an error.
const SHAPER_JOIN_TIMEOUT: Duration = Duration::from_secs(2);

enum ShaperInput {
    Item(BaseItem),
    Finish,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ShaperExit {
    SinkClosed,
    PlanningHungUp,
    Finished,
}

/// A planning side whose `feed`, `idle` and `finish` report `false` once the
/// shaper thread has stopped — its sink refused an item, or it panicked —
/// exactly as the synchronous [`Pipeline`] reports a refusing sink.
pub struct StagedPipeline {
    front: PlanningFront,
    shaper_inbox: Option<ShaperInbox>,
    shaper_thread: Option<JoinHandle<ShaperExit>>,
}

impl Pipeline {
    pub fn into_staged(
        self,
        sink: impl FnMut(TrajectoryItem) -> bool + Send + 'static,
    ) -> StagedPipeline {
        let Self { front, shaper } = self;
        let (sender, rx) = bounded(SHAPER_INBOX_CAP);
        let shaper_thread = std::thread::Builder::new()
            .name("kalico-shaper".into())
            .spawn(move || run_shaper(shaper, &rx, sink))
            .expect("spawn shaper thread");
        StagedPipeline {
            front,
            shaper_inbox: Some(ShaperInbox {
                sender,
                #[cfg(feature = "bench-metrics")]
                segments_sent: 0,
            }),
            shaper_thread: Some(shaper_thread),
        }
    }
}

fn run_shaper(
    mut shaper: Shaper,
    rx: &Receiver<ShaperInput>,
    mut sink: impl FnMut(TrajectoryItem) -> bool,
) -> ShaperExit {
    loop {
        let Ok(input) = rx.recv() else {
            return ShaperExit::PlanningHungUp;
        };
        #[cfg(feature = "bench-metrics")]
        let busy = crate::timing::stopwatch();
        let exit = match input {
            ShaperInput::Item(item) => {
                (!shaper.feed(item, &mut sink)).then_some(ShaperExit::SinkClosed)
            }
            ShaperInput::Finish => Some(if shaper.finish(&mut sink) {
                ShaperExit::Finished
            } else {
                ShaperExit::SinkClosed
            }),
        };
        #[cfg(feature = "bench-metrics")]
        crate::timing::record_stage(crate::timing::Stage::ShaperBusy, busy.elapsed_us());
        if let Some(exit) = exit {
            return exit;
        }
    }
}

struct ShaperInbox {
    sender: Sender<ShaperInput>,
    #[cfg(feature = "bench-metrics")]
    segments_sent: u64,
}

impl ShaperInbox {
    fn send(&mut self, item: BaseItem) -> bool {
        #[cfg(feature = "bench-metrics")]
        if matches!(item, BaseItem::Seg(_)) {
            self.segments_sent += 1;
        }
        self.send_input(ShaperInput::Item(item))
    }

    fn send_input(&self, input: ShaperInput) -> bool {
        #[cfg(feature = "bench-metrics")]
        let blocked = crate::timing::stopwatch();
        let sent = self.sender.send(input).is_ok();
        #[cfg(feature = "bench-metrics")]
        crate::timing::record_stage(
            crate::timing::Stage::PlanningBlockedOnShaper,
            blocked.elapsed_us(),
        );
        sent
    }
}

impl StagedPipeline {
    pub fn feed(&mut self, item: StreamInput) -> bool {
        let inbox = self
            .shaper_inbox
            .as_mut()
            .expect("pipeline already finished");
        self.front.feed(item, &mut |item| inbox.send(item))
    }

    pub fn idle(&mut self) -> bool {
        let inbox = self
            .shaper_inbox
            .as_mut()
            .expect("pipeline already finished");
        self.front.idle(&mut |item| inbox.send(item))
    }

    /// Segments the planning side has committed to the shaper so far.
    #[cfg(feature = "bench-metrics")]
    #[must_use]
    pub fn segments_planned(&self) -> u64 {
        self.shaper_inbox
            .as_ref()
            .expect("pipeline already finished")
            .segments_sent
    }

    /// Flush every stage into the sink and wait for the shaper to finish.
    pub fn finish(&mut self) -> bool {
        let mut inbox = self.shaper_inbox.take().expect("pipeline already finished");
        let finish_sent = self.front.finish(&mut |item| inbox.send(item))
            && inbox.send_input(ShaperInput::Finish);
        drop(inbox);
        let handle = self
            .shaper_thread
            .take()
            .expect("shaper thread already joined");
        let exit = match handle.join() {
            Ok(exit) => exit,
            Err(panic) => std::panic::resume_unwind(panic),
        };
        finish_sent && exit == ShaperExit::Finished
    }
}

/// The shaper may still be shaping what was queued when the planning side
/// goes away. Closing the inbox lets it drain those items and stop; a panic
/// it raised resurfaces here, and a shaper that cannot finish in time is
/// detached with an error rather than hanging shutdown.
impl Drop for StagedPipeline {
    fn drop(&mut self) {
        self.shaper_inbox.take();
        let Some(handle) = self.shaper_thread.take() else {
            return;
        };
        let deadline = Instant::now() + SHAPER_JOIN_TIMEOUT;
        while !handle.is_finished() && Instant::now() < deadline {
            std::thread::sleep(Duration::from_millis(1));
        }
        if !handle.is_finished() {
            tracing::error!(
                subsystem = "motion",
                event = "shaper_thread_detached",
                timeout_s = SHAPER_JOIN_TIMEOUT.as_secs_f64(),
                "shaper thread still busy after the planning side closed — detaching it"
            );
            return;
        }
        if let Err(panic) = handle.join() {
            if !std::thread::panicking() {
                std::panic::resume_unwind(panic);
            }
        }
    }
}

#[cfg(test)]
mod tests;
