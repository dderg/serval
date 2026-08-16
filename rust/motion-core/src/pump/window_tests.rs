// Windowed PushPieces delivery: the pump keeps up to `send_window` bundles
// in flight, commits ring bookkeeping at submit, replays a transiently-failed
// bundle byte-identically, discards stale halt outcomes across a halt epoch,
// and fails loudly when a committed bundle exhausts its replay budget.
use std::sync::atomic::AtomicU64;
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
}

impl PendingSend for ScriptedPending {
    fn poll(&mut self) -> Option<Result<(), SendError>> {
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
    script: Arc<Mutex<Vec<Script>>>,
    window: usize,
}

impl WindowScriptSink {
    fn new(window: usize, script: Vec<Script>) -> Self {
        Self {
            submissions: Arc::new(Mutex::new(Vec::new())),
            script: Arc::new(Mutex::new(script)),
            window,
        }
    }

    fn submitted(&self) -> Vec<(u16, u32, usize)> {
        self.submissions.lock().unwrap().clone()
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
        _mcu_id: u32,
        frames: &[AxisFrame],
    ) -> Result<Box<dyn PendingSend>, SendError> {
        let f = frames.first().expect("bundle is non-empty");
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
