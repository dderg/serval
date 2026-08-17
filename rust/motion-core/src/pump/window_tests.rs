// Windowed PushPieces delivery: the pump keeps up to `send_window` bundles
// in flight, commits ring bookkeeping at submit, replays a transiently-failed
// bundle byte-identically, discards stale halt outcomes across a halt epoch,
// and fails loudly when a committed bundle exhausts its replay budget.
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::mpsc;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use crossbeam_channel::unbounded;

use super::messages::{PendingSend, ResolvedSend};
use super::{
    AxisFrame, AxisKey, EnqueueMsg, MAX_LEAD_SECS, PieceSink, PumpCallbacks, PumpMsg, SendError,
    run_pump,
};
use runtime::piece_ring::PieceEntry;

fn wait_until(cond: impl Fn() -> bool, what: &str) {
    let deadline = Instant::now() + Duration::from_secs(5);
    while !cond() {
        assert!(Instant::now() < deadline, "timed out waiting for: {what}");
        std::thread::sleep(Duration::from_millis(1));
    }
}

fn make_piece(t: u64) -> (PieceEntry, f64) {
    (
        PieceEntry {
            start_time: t,
            duration: 0.001,
            ..PieceEntry::zeroed()
        },
        t as f64,
    )
}

fn make_enqueue(key: AxisKey, pieces: Vec<(PieceEntry, f64)>) -> EnqueueMsg {
    EnqueueMsg {
        epoch_freq: None,
        key,
        pieces,
        epoch: crate::anchor::StreamEpoch::Continuation,
        lead_secs: MAX_LEAD_SECS,
        source_line: u32::MAX,
        batch_end: true,
    }
}

/// One scripted outcome per submission, in submission order. `Deferred`
/// resolves on the pump's next poll; `Never` stays pending until the entry's
/// own machinery gives up (used with a window-full block).
#[derive(Clone)]
enum Script {
    Transient,
    Halted,
}

struct ScriptedPending {
    outcome: Option<Result<(), SendError>>,
    released: Arc<AtomicBool>,
}

impl PendingSend for ScriptedPending {
    fn poll(&mut self) -> Option<Result<(), SendError>> {
        if !self.released.load(Ordering::Acquire) {
            return None;
        }
        Some(self.outcome.take().unwrap_or(Ok(())))
    }

    fn wait(&mut self, _cap: Duration) -> Option<Result<(), SendError>> {
        self.poll()
    }
}

/// Sink that records every submission (slot, head, piece count) and resolves
/// each according to its script; unscripted submissions resolve `Ok`.
#[derive(Clone)]
struct WindowScriptSink {
    submissions: Arc<Mutex<Vec<(u16, u32, usize)>>>,
    bundles: Arc<Mutex<Vec<(u32, Vec<u8>)>>>,
    script: Arc<Mutex<Vec<Script>>>,
    window: usize,
    released: Arc<AtomicBool>,
}

impl WindowScriptSink {
    fn new(window: usize, script: Vec<Script>) -> Self {
        Self::build(window, script, true)
    }

    /// Outcomes stay pending until `release`, so several bundles can be held in
    /// flight at once.
    fn gated(window: usize, script: Vec<Script>) -> Self {
        Self::build(window, script, false)
    }

    fn build(window: usize, script: Vec<Script>, released: bool) -> Self {
        Self {
            submissions: Arc::new(Mutex::new(Vec::new())),
            bundles: Arc::new(Mutex::new(Vec::new())),
            script: Arc::new(Mutex::new(script)),
            window,
            released: Arc::new(AtomicBool::new(released)),
        }
    }

    fn release(&self) {
        self.released.store(true, Ordering::Release);
    }

    fn submitted(&self) -> Vec<(u16, u32, usize)> {
        self.submissions.lock().unwrap().clone()
    }

    /// One `(mcu_id, axes)` row per submitted bundle.
    fn bundles(&self) -> Vec<(u32, Vec<u8>)> {
        self.bundles.lock().unwrap().clone()
    }
}

impl PieceSink for WindowScriptSink {
    fn send_frame(
        &self,
        _key: AxisKey,
        _pieces: &[PieceEntry],
        _start_slot: u16,
        _new_head: u32,
        _room: u32,
    ) -> Result<i32, SendError> {
        unreachable!("windowed pump must route through submit_mcu_frames");
    }

    fn send_window(&self, _mcu_id: u32) -> usize {
        self.window
    }

    fn submit_mcu_frames(
        &self,
        mcu_id: u32,
        frames: &[AxisFrame],
    ) -> Result<Box<dyn PendingSend>, SendError> {
        let f = frames.first().expect("bundle is non-empty");
        self.bundles
            .lock()
            .unwrap()
            .push((mcu_id, frames.iter().map(|af| af.axis).collect()));
        self.submissions
            .lock()
            .unwrap()
            .push((f.start_slot, f.new_head, f.pieces.len()));
        let mut script = self.script.lock().unwrap();
        let outcome = if script.is_empty() {
            Ok(())
        } else {
            match script.remove(0) {
                Script::Transient => Err(SendError::Transient("scripted loss".into())),
                Script::Halted => Err(SendError::Halted("scripted endpoint halt".into())),
            }
        };
        Ok(Box::new(ScriptedPending {
            outcome: Some(outcome),
            released: Arc::clone(&self.released),
        }))
    }
}

fn spawn_pump(
    sink: WindowScriptSink,
    fatal: Arc<Mutex<Vec<AxisKey>>>,
) -> (
    crossbeam_channel::Sender<PumpMsg>,
    crossbeam_channel::Sender<EnqueueMsg>,
    std::thread::JoinHandle<()>,
) {
    const RING_DEPTH: u32 = 64;
    let (ctl, control_rx) = unbounded::<PumpMsg>();
    let (data, data_rx) = unbounded::<EnqueueMsg>();
    let callbacks = PumpCallbacks {
        ring_depth_of: Box::new(move |_| RING_DEPTH),
        mcu_clock_of: Box::new(|_| None),
        on_fatal_transport: Box::new(move |key| fatal.lock().unwrap().push(key)),
        on_abandon: Box::new(|_, _| {}),
        on_drip_stall: Box::new(|_| {}),
    };
    let handle = std::thread::spawn(move || {
        run_pump(
            control_rx,
            data_rx,
            sink,
            callbacks,
            None,
            Arc::new(crate::drain::DrainLedger::new()),
            Arc::new(AtomicU64::new(0)),
        );
    });
    (ctl, data, handle)
}

#[test]
fn windowed_sends_commit_at_submit_and_advance_slots() {
    let key = AxisKey { mcu_id: 1, axis: 0 };
    let sink = WindowScriptSink::new(4, vec![]);
    let fatal = Arc::new(Mutex::new(Vec::new()));
    let (ctl, data, handle) = spawn_pump(sink.clone(), Arc::clone(&fatal));

    data.send(make_enqueue(key, vec![make_piece(0)])).unwrap();
    wait_until(|| sink.submitted().len() == 1, "first bundle submitted");
    data.send(make_enqueue(key, vec![make_piece(1)])).unwrap();
    wait_until(|| sink.submitted().len() == 2, "second bundle submitted");

    let submitted = sink.submitted();
    assert_eq!(
        submitted[0],
        (0, 1, 1),
        "first bundle writes slot 0, head 1"
    );
    assert_eq!(
        submitted[1],
        (1, 2, 1),
        "commit-at-submit advances the cursor for the next bundle"
    );
    assert!(fatal.lock().unwrap().is_empty());

    ctl.send(PumpMsg::Shutdown).unwrap();
    handle.join().unwrap();
}

#[test]
fn transient_outcome_replays_byte_identical_bundle() {
    let key = AxisKey { mcu_id: 1, axis: 0 };
    let sink = WindowScriptSink::new(4, vec![Script::Transient]);
    let fatal = Arc::new(Mutex::new(Vec::new()));
    let (ctl, data, handle) = spawn_pump(sink.clone(), Arc::clone(&fatal));

    data.send(make_enqueue(key, vec![make_piece(0)])).unwrap();
    wait_until(
        || sink.submitted().len() >= 2,
        "lost bundle replayed after the transient outcome",
    );
    let submitted = sink.submitted();
    assert_eq!(
        submitted[0], submitted[1],
        "replay is byte-identical: same slot, head, and piece count"
    );
    assert!(
        fatal.lock().unwrap().is_empty(),
        "one replay within budget is not fatal"
    );

    ctl.send(PumpMsg::Shutdown).unwrap();
    handle.join().unwrap();
}

#[test]
fn replay_budget_exhaustion_is_fatal() {
    let key = AxisKey { mcu_id: 1, axis: 0 };
    let sink = WindowScriptSink::new(
        4,
        vec![Script::Transient, Script::Transient, Script::Transient],
    );
    let fatal = Arc::new(Mutex::new(Vec::new()));
    let (_ctl, data, handle) = spawn_pump(sink.clone(), Arc::clone(&fatal));

    data.send(make_enqueue(key, vec![make_piece(0)])).unwrap();
    wait_until(
        || !fatal.lock().unwrap().is_empty(),
        "a committed bundle undeliverable after the replay budget fails loudly",
    );
    assert_eq!(fatal.lock().unwrap()[0], key);
    handle.join().unwrap();
}

#[test]
fn halted_outcome_halts_the_bundles_axes() {
    let key = AxisKey { mcu_id: 1, axis: 0 };
    let sink = WindowScriptSink::new(4, vec![Script::Halted]);
    let fatal = Arc::new(Mutex::new(Vec::new()));
    let (ctl, data, handle) = spawn_pump(sink.clone(), Arc::clone(&fatal));

    data.send(make_enqueue(key, vec![make_piece(0)])).unwrap();
    wait_until(|| sink.submitted().len() == 1, "first bundle submitted");
    // A follow-up enqueue for the now-halted key is dropped, not submitted.
    data.send(make_enqueue(key, vec![make_piece(1)])).unwrap();
    let (ack_tx, ack_rx) = mpsc::sync_channel::<()>(1);
    ctl.send(PumpMsg::Barrier(ack_tx)).unwrap();
    ack_rx.recv().unwrap();
    assert_eq!(
        sink.submitted().len(),
        1,
        "pieces enqueued after the halt outcome must not reach the wire"
    );
    assert!(fatal.lock().unwrap().is_empty());

    ctl.send(PumpMsg::Shutdown).unwrap();
    handle.join().unwrap();
}

#[test]
fn resolved_send_reports_its_outcome_once() {
    let mut ready = ResolvedSend(Some(Err(SendError::Transient("x".into()))));
    assert!(matches!(ready.poll(), Some(Err(SendError::Transient(_)))));
}

/// One MCU carries several axis subsets, so a halt reported by one bundle's
/// response purges in-flight bundles belonging to axes that were nowhere in it.
/// Every axis whose committed bundle the purge discarded must be halted, or it
/// continues from later staged pieces across the hole the purge left.
#[test]
fn endpoint_halt_halts_every_axis_whose_bundle_it_purged() {
    let a = AxisKey { mcu_id: 1, axis: 0 };
    let b = AxisKey { mcu_id: 1, axis: 1 };
    let live = AxisKey { mcu_id: 1, axis: 2 };
    let sink = WindowScriptSink::gated(4, vec![Script::Halted, Script::Halted]);
    let fatal = Arc::new(Mutex::new(Vec::new()));
    let (ctl, data, handle) = spawn_pump(sink.clone(), Arc::clone(&fatal));

    data.send(make_enqueue(a, vec![make_piece(0)])).unwrap();
    wait_until(|| sink.bundles().len() == 1, "axis 0 bundle submitted");
    data.send(make_enqueue(b, vec![make_piece(1)])).unwrap();
    wait_until(|| sink.bundles().len() == 2, "axis 1 bundle submitted");
    assert_eq!(
        sink.bundles(),
        vec![(1, vec![0]), (1, vec![1])],
        "the two axes are in flight in separate bundles on one MCU"
    );

    // Axis 2 is unaffected by the halt, so its send drives the pass that
    // resolves axis 0's halt and purges axis 1's committed bundle.
    sink.release();
    data.send(make_enqueue(live, vec![make_piece(2)])).unwrap();
    wait_until(
        || sink.bundles().iter().any(|(_, axes)| axes == &vec![2]),
        "an unaffected axis on the same MCU keeps sending",
    );

    data.send(make_enqueue(b, vec![make_piece(3)])).unwrap();
    data.send(make_enqueue(a, vec![make_piece(4)])).unwrap();
    let (ack_tx, ack_rx) = mpsc::sync_channel::<()>(1);
    ctl.send(PumpMsg::Barrier(ack_tx)).unwrap();
    ack_rx.recv().unwrap();

    let after: Vec<(u32, Vec<u8>)> = sink.bundles().split_off(2);
    assert!(
        after.iter().all(|(_, axes)| axes == &vec![2]),
        "no axis purged by the halt may reach the wire again: {after:?}"
    );
    assert!(
        fatal.lock().unwrap().is_empty(),
        "an endpoint halt is not a transport fatality"
    );

    ctl.send(PumpMsg::Shutdown).unwrap();
    handle.join().unwrap();
}

/// A bundle is built before the drain that precedes its submission, so an
/// in-flight bundle resolving `Halted` in that drain halts the endpoint after
/// its successor's pieces already left the staging queue. The successor must be
/// abandoned, not pushed at an endpoint that just refused its predecessor.
#[test]
fn bundle_built_before_the_drain_that_halts_it_is_abandoned() {
    let key = AxisKey { mcu_id: 1, axis: 0 };
    let sink = WindowScriptSink::gated(4, vec![Script::Halted]);
    let fatal = Arc::new(Mutex::new(Vec::new()));
    let (ctl, data, handle) = spawn_pump(sink.clone(), Arc::clone(&fatal));

    data.send(make_enqueue(key, vec![make_piece(0)])).unwrap();
    wait_until(|| sink.bundles().len() == 1, "first bundle submitted");

    sink.release();
    data.send(make_enqueue(key, vec![make_piece(1)])).unwrap();
    let (ack_tx, ack_rx) = mpsc::sync_channel::<()>(1);
    ctl.send(PumpMsg::Barrier(ack_tx)).unwrap();
    ack_rx.recv().unwrap();

    assert_eq!(
        sink.bundles().len(),
        1,
        "the bundle built before the halting drain must not be submitted: {:?}",
        sink.bundles()
    );
    assert!(fatal.lock().unwrap().is_empty());

    ctl.send(PumpMsg::Shutdown).unwrap();
    handle.join().unwrap();
}

/// Response staleness is endpoint-local: a resume on one MCU says nothing
/// about a halt another MCU is reporting. Counting resumes globally made the
/// second MCU's halt look already-handled, so its refused bundle was rolled
/// back — pieces abandoned — without halting the axis, reopening the hole.
#[test]
fn resume_on_one_mcu_does_not_suppress_a_halt_on_another() {
    let halting = AxisKey { mcu_id: 1, axis: 0 };
    let elsewhere = AxisKey { mcu_id: 2, axis: 0 };
    let sink = WindowScriptSink::gated(4, vec![Script::Halted]);
    let fatal = Arc::new(Mutex::new(Vec::new()));
    let (ctl, data, handle) = spawn_pump(sink.clone(), Arc::clone(&fatal));

    data.send(make_enqueue(halting, vec![make_piece(0)]))
        .unwrap();
    wait_until(|| sink.bundles().len() == 1, "mcu 1 bundle in flight");

    ctl.send(PumpMsg::Resume(vec![elsewhere])).unwrap();
    let (ack_tx, ack_rx) = mpsc::sync_channel::<()>(1);
    ctl.send(PumpMsg::Barrier(ack_tx)).unwrap();
    ack_rx.recv().unwrap();
    sink.release();

    // This send resolves mcu 1's in-flight bundle as `Halted`; the resume on
    // mcu 2 must not classify that outcome as already-handled.
    data.send(make_enqueue(halting, vec![make_piece(1)]))
        .unwrap();
    let (ack_tx, ack_rx) = mpsc::sync_channel::<()>(1);
    ctl.send(PumpMsg::Barrier(ack_tx)).unwrap();
    ack_rx.recv().unwrap();

    assert_eq!(
        sink.bundles().len(),
        1,
        "mcu 1 halted, so nothing more may reach its wire: {:?}",
        sink.bundles()
    );
    assert!(
        fatal.lock().unwrap().is_empty(),
        "an endpoint halt is not a transport fatality"
    );

    ctl.send(PumpMsg::Shutdown).unwrap();
    handle.join().unwrap();
}

/// A resume clears the halt, so a `Halted` response from a bundle submitted
/// before it is stale and must not re-halt the fresh stream.
#[test]
fn halt_response_from_before_a_resume_does_not_rehalt() {
    let key = AxisKey { mcu_id: 1, axis: 0 };
    let sink = WindowScriptSink::gated(4, vec![Script::Halted]);
    let fatal = Arc::new(Mutex::new(Vec::new()));
    let (ctl, data, handle) = spawn_pump(sink.clone(), Arc::clone(&fatal));

    data.send(make_enqueue(key, vec![make_piece(0)])).unwrap();
    wait_until(|| sink.bundles().len() == 1, "bundle submitted");

    ctl.send(PumpMsg::Resume(vec![key])).unwrap();
    let (ack_tx, ack_rx) = mpsc::sync_channel::<()>(1);
    ctl.send(PumpMsg::Barrier(ack_tx)).unwrap();
    ack_rx.recv().unwrap();
    sink.release();

    data.send(make_enqueue(key, vec![make_piece(1)])).unwrap();
    wait_until(
        || sink.bundles().len() == 2,
        "the resumed stream keeps sending despite the stale halt outcome",
    );
    assert!(fatal.lock().unwrap().is_empty());

    ctl.send(PumpMsg::Shutdown).unwrap();
    handle.join().unwrap();
}
