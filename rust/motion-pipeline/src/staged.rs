//! The live pipeline with its shaper on a dedicated thread. Fitting, velocity
//! planning, lowering and the shaper's per-segment source fits stay on the
//! caller's thread; the lowerer's items cross a bounded FIFO to a prefit
//! thread, which fits each leader group ahead of the shaper
//! ([`crate::leader_prefit`]) and passes the fits and the items on through a
//! second bounded FIFO to the shaper, which emits into the caller-supplied
//! sink. Every item — segments, drains and control tokens — keeps its
//! order, so a barrier still acknowledges only after everything ahead of it
//! has been shaped and emitted, and no arithmetic moves between threads in a
//! way that changes its result.

use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use crossbeam_channel::{Receiver, Sender, bounded};

use crate::leader_prefit::{LeaderGroupFits, LeaderPrefitter};
use crate::shaper::{Shaper, SourceMaterializer};
use crate::{BaseItem, Pipeline, PlanningFront, StreamInput, TrajectoryItem};

/// Items each stage's inbox holds. Bounded so a slow shaper backpressures
/// planning instead of buffering without limit, and deep enough to carry
/// the shaper through the planning side's bursts: the planner lowers a
/// whole velocity window at once, and at 16 items the shaper and the
/// planner each idled waiting on the other. On the Pi 5 at two cores, 128
/// raised trident_kernel's worst 5 s window by 9 % and
/// p3_voron_direct_drive_kernel's by 10 %; 1024 added nothing over 128.
const STAGE_INBOX_CAP: usize = 32;

/// The most lowered items planning runs ahead of a shaper that holds one:
/// both stage inboxes full and one more in the prefit thread's hands.
pub const SHAPER_INBOX_CAP: usize = 2 * STAGE_INBOX_CAP + 1;

/// How long dropping the pipeline waits for the shaper to finish the items
/// already queued for it before detaching it with an error.
const SHAPER_JOIN_TIMEOUT: Duration = Duration::from_secs(2);

enum ShaperInput {
    Item(BaseItem),
    LeaderPrefits(LeaderGroupFits),
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
    materializer: SourceMaterializer,
    planned_through_s: f64,
    shaper_inbox: Option<ShaperInbox>,
    prefit_thread: Option<JoinHandle<()>>,
    shaper_thread: Option<JoinHandle<ShaperExit>>,
}

impl Pipeline {
    pub fn into_staged(
        self,
        sink: impl FnMut(TrajectoryItem) -> bool + Send + 'static,
    ) -> StagedPipeline {
        let Self { front, shaper } = self;
        let materializer = SourceMaterializer::of(&shaper);
        let prefitter = LeaderPrefitter::of(&shaper);
        let (sender, prefit_rx) = bounded(STAGE_INBOX_CAP);
        let (shaper_tx, shaper_rx) = bounded(STAGE_INBOX_CAP);
        let prefit_thread = std::thread::Builder::new()
            .name("kalico-prefit".into())
            .stack_size(crate::worker_pool::FAN_OUT_THREAD_STACK)
            .spawn(move || run_prefit(prefitter, &prefit_rx, &shaper_tx))
            .expect("spawn prefit thread");
        let shaper_thread = std::thread::Builder::new()
            .name("kalico-shaper".into())
            .stack_size(crate::worker_pool::FAN_OUT_THREAD_STACK)
            .spawn(move || run_shaper(shaper, &shaper_rx, sink))
            .expect("spawn shaper thread");
        StagedPipeline {
            front,
            materializer,
            planned_through_s: 0.0,
            shaper_inbox: Some(ShaperInbox {
                sender,
                #[cfg(feature = "bench-metrics")]
                segments_sent: 0,
            }),
            prefit_thread: Some(prefit_thread),
            shaper_thread: Some(shaper_thread),
        }
    }
}

/// Passes every input on to the shaper, each segment preceded by the leader
/// group fits it completes the read of. Stops once the planning side hangs
/// up, after passing `Finish` on, or once the shaper has stopped.
fn run_prefit(
    mut prefitter: LeaderPrefitter,
    rx: &Receiver<ShaperInput>,
    shaper: &Sender<ShaperInput>,
) {
    while let Ok(input) = rx.recv() {
        if let ShaperInput::Item(item) = &input {
            let fits = prefitter.prefit(item);
            if !fits.is_empty() && shaper.send(ShaperInput::LeaderPrefits(fits)).is_err() {
                return;
            }
        }
        let finish = matches!(input, ShaperInput::Finish);
        if shaper.send(input).is_err() || finish {
            return;
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
            ShaperInput::LeaderPrefits(fits) => {
                shaper.accept_leader_prefits(fits);
                None
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
        let (front, mut handoff) = self.handoff();
        front.feed(item, &mut handoff)
    }

    pub fn idle(&mut self) -> bool {
        let (front, mut handoff) = self.handoff();
        front.idle(&mut handoff)
    }

    /// The planning front and what it hands each lowered item to: the
    /// item's sources are fitted, then it is queued for the shaper. One item
    /// at a time, so the shaper starts on a planned window's first segment
    /// while the rest of the window is still being fitted.
    fn handoff(&mut self) -> (&mut PlanningFront, impl FnMut(BaseItem) -> bool + '_) {
        let inbox = self
            .shaper_inbox
            .as_mut()
            .expect("pipeline already finished");
        let materializer = &mut self.materializer;
        let planned_through_s = &mut self.planned_through_s;
        (&mut self.front, move |mut item| {
            if let BaseItem::Seg(seg) = &item {
                *planned_through_s = planned_through_s.max(seg.t_end);
            }
            materializer.materialize(std::slice::from_mut(&mut item));
            inbox.send(item)
        })
    }

    /// The stream time the planning side has committed to the shaper
    /// through.
    #[must_use]
    pub fn planned_through_s(&self) -> f64 {
        self.planned_through_s
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
        let flushed = {
            let (front, mut handoff) = self.handoff();
            front.finish(&mut handoff)
        };
        let finish_sent = flushed
            && self
                .shaper_inbox
                .as_ref()
                .expect("pipeline already finished")
                .send_input(ShaperInput::Finish);
        self.shaper_inbox.take();
        let prefit = self
            .prefit_thread
            .take()
            .expect("prefit thread already joined");
        if let Err(panic) = prefit.join() {
            std::panic::resume_unwind(panic);
        }
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
/// goes away. Closing the inbox lets the prefit thread pass on what it holds
/// and stop, and the shaper drain those items and stop; a panic either
/// raised resurfaces here, and threads that cannot finish in time are
/// detached with an error rather than hanging shutdown.
impl Drop for StagedPipeline {
    fn drop(&mut self) {
        self.shaper_inbox.take();
        let (Some(prefit), Some(shaper)) = (self.prefit_thread.take(), self.shaper_thread.take())
        else {
            return;
        };
        let deadline = Instant::now() + SHAPER_JOIN_TIMEOUT;
        while !(prefit.is_finished() && shaper.is_finished()) && Instant::now() < deadline {
            std::thread::sleep(Duration::from_millis(1));
        }
        if !(prefit.is_finished() && shaper.is_finished()) {
            tracing::error!(
                subsystem = "motion",
                event = "shaper_thread_detached",
                timeout_s = SHAPER_JOIN_TIMEOUT.as_secs_f64(),
                "shaper thread still busy after the planning side closed — detaching it"
            );
            return;
        }
        let panics = [prefit.join().err(), shaper.join().err()];
        if let Some(panic) = panics.into_iter().flatten().next() {
            if !std::thread::panicking() {
                std::panic::resume_unwind(panic);
            }
        }
    }
}

#[cfg(test)]
mod tests;
