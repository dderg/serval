//! The live pipeline with its shaper on a dedicated thread. Fitting, velocity
//! planning and lowering stay on the caller's thread; the lowerer's items
//! cross a bounded FIFO to the shaper, which emits into the caller-supplied
//! sink. Every item — segments, drains and control tokens — keeps its order,
//! so a barrier still acknowledges only after everything ahead of it has
//! been shaped and emitted, and no arithmetic moves between threads.

use std::thread::JoinHandle;

use crossbeam_channel::{Receiver, Sender, bounded};

use crate::shaper::Shaper;
use crate::timing::{Stage, record_stage, stopwatch};
use crate::{BaseItem, Pipeline, PlanningFront, StreamInput, TrajectoryItem};

/// Lowered items the shaper may lag behind the planner. Bounded so a slow
/// shaper backpressures planning instead of buffering without limit.
pub const SHAPER_INBOX_CAP: usize = 16;

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
        let busy = stopwatch();
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
        record_stage(Stage::ShaperBusy, busy.elapsed_us());
        if let Some(exit) = exit {
            return exit;
        }
    }
}

struct ShaperInbox {
    sender: Sender<ShaperInput>,
    segments_sent: u64,
}

impl ShaperInbox {
    fn send(&mut self, item: BaseItem) -> bool {
        if matches!(item, BaseItem::Seg(_)) {
            self.segments_sent += 1;
        }
        self.send_input(ShaperInput::Item(item))
    }

    fn send_input(&self, input: ShaperInput) -> bool {
        let blocked = stopwatch();
        let sent = self.sender.send(input).is_ok();
        record_stage(Stage::PlanningBlockedOnShaper, blocked.elapsed_us());
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

#[cfg(test)]
mod tests;
