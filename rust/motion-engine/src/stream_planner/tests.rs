use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use super::*;
use crate::stream::StreamConfig;
use geometry::segment::SourceRange;
use geometry::{ChainFitConfig, MoveContext, VelocityConfig, VelocityLimits, line_move};
use nurbs::eval::eval;

#[derive(Clone, Default)]
struct Capture {
    segs: Arc<Mutex<Vec<(f64, f64, f64)>>>, // (t_start, t_end, x_at_end)
    nudges: Arc<Mutex<usize>>,
}

impl Capture {
    fn dispatch(&self) -> DispatchFn {
        let segs = Arc::clone(&self.segs);
        Arc::new(move |seg: &ShapedSegment| {
            let x_end = eval(&seg.axes[0], seg.t_end);
            segs.lock().unwrap().push((seg.t_start, seg.t_end, x_end));
            Ok(())
        })
    }
    fn nudge_dispatch(&self) -> NudgeDispatchFn {
        let nudges = Arc::clone(&self.nudges);
        Arc::new(move |_mcu_id, _piece| {
            *nudges.lock().unwrap() += 1;
            Ok(())
        })
    }
    fn snapshot(&self) -> Vec<(f64, f64, f64)> {
        self.segs.lock().unwrap().clone()
    }
    fn nudge_count(&self) -> usize {
        *self.nudges.lock().unwrap()
    }
}

fn cfg() -> StreamConfig {
    cfg_cap(64)
}

fn cfg_cap(max_buffer_moves: usize) -> StreamConfig {
    StreamConfig {
        chain: ChainFitConfig::default(),
        velocity: VelocityConfig::default(),
        fit_tol_mm: 1e-3,
        max_buffer_moves,
        limits: VelocityLimits::try_new(300.0, 5000.0, 5.0).unwrap(),
    }
}

fn ctx(line_no: u32) -> MoveContext {
    MoveContext {
        extruder_axis: 3,
        feedrate_mm_s: 80.0,
        limits: VelocityLimits::try_new(300.0, 5000.0, 5.0).unwrap(),
        source: SourceRange {
            start_line: line_no,
            end_line: line_no,
        },
    }
}

fn line(line_no: u32, start: [f64; 3], end: [f64; 3]) -> geometry::Move {
    line_move(start, end, 0.0, ctx(line_no)).unwrap()
}

fn line_e(line_no: u32, feed: f64, start: [f64; 3], end: [f64; 3], e: f64) -> geometry::Move {
    let mut c = ctx(line_no);
    c.feedrate_mm_s = feed;
    line_move(start, end, e, c).unwrap()
}

// One Voron-cube perimeter loop (the print that aborted on the bench), as
// (x, y, e_delta) with the closing point returning to the start neighbourhood.
const VORON_PERIMETER: [(f64, f64, f64); 17] = [
    (102.008, 96.308, 0.14859),
    (103.2, 95.814, 0.04756),
    (121.8, 95.814, 0.68571),
    (122.992, 96.308, 0.04756),
    (128.692, 102.008, 0.29718),
    (129.186, 103.2, 0.04756),
    (129.186, 121.8, 0.68571),
    (128.692, 122.992, 0.04756),
    (122.992, 128.692, 0.29718),
    (121.8, 129.186, 0.04756),
    (103.2, 129.186, 0.68571),
    (102.008, 128.692, 0.04756),
    (96.308, 122.992, 0.29718),
    (95.814, 121.8, 0.04756),
    (95.814, 103.2, 0.68571),
    (96.308, 102.008, 0.04756),
    (99.158, 99.158, 0.14711),
];

#[test]
fn nonstop_flood_of_real_perimeter_drains_without_crashing() {
    // Drive the real planner thread with a continuous, back-to-back flood of the
    // Voron perimeter (the geometry that aborted with "head-trim geometry:
    // ZeroMotion"). The host normally throttles, but here we push as fast as the
    // channel accepts to stress the coalescing/commit path. The planner aborts
    // the process on any commit error, so reaching the flush and seeing a
    // contiguous, complete trajectory is the pass condition.
    let cap = Capture::default();
    let mut h = StreamPlannerHandle::spawn(
        cfg(),
        AxisChainSet::default(),
        vec![99.158, 99.158, 0.2, 0.0],
        cap.dispatch(),
        cap.nudge_dispatch(),
    );

    let mut prev = [99.158, 99.158, 0.2];
    let mut line_no = 1u32;
    let mut submitted = 0usize;
    for _loop_idx in 0..12 {
        for (x, y, e) in VORON_PERIMETER {
            let end = [x, y, 0.2];
            h.submit_move(line_e(line_no, 50.0, prev, end, e)).unwrap();
            prev = end;
            line_no += 1;
            submitted += 1;
        }
    }
    h.flush().unwrap();

    let segs = cap.snapshot();
    assert!(!segs.is_empty(), "flood dispatched nothing");
    // The look-ahead buffer must DRAIN: each move yields a bounded number of
    // output segments (line body plus at most its two blend halves), so total
    // dispatch is O(moves). A buffer that fails to drain re-dispatches the whole
    // accumulated path every commit, exploding to O(moves * commits). This is the
    // regression guard for the line-number drain key: a constant
    // `source.start_line` makes `front.start_line < keep_line` a no-op, so the
    // buffer never empties and a real print replays from the start, a bit further
    // each pass.
    assert!(
        segs.len() <= submitted * 6,
        "buffer did not drain: {} segments from {submitted} moves (re-dispatch explosion)",
        segs.len()
    );
    for w in segs.windows(2) {
        assert!(
            (w[1].0 - w[0].1).abs() < 1e-6,
            "time gap between committed segments: {} -> {}",
            w[0].1,
            w[1].0
        );
    }
    h.shutdown();
}

#[test]
fn streams_collinear_moves_to_a_contiguous_trajectory() {
    let cap = Capture::default();
    let mut h = StreamPlannerHandle::spawn(
        cfg(),
        AxisChainSet::default(),
        vec![0.0, 0.0, 0.0],
        cap.dispatch(),
        cap.nudge_dispatch(),
    );

    h.submit_move(line(1, [0.0, 0.0, 0.0], [30.0, 0.0, 0.0]))
        .unwrap();
    h.submit_move(line(2, [30.0, 0.0, 0.0], [60.0, 0.0, 0.0]))
        .unwrap();
    h.submit_move(line(3, [60.0, 0.0, 0.0], [90.0, 0.0, 0.0]))
        .unwrap();
    h.flush().unwrap();

    let segs = cap.snapshot();
    assert!(!segs.is_empty(), "nothing dispatched");
    assert!(
        (segs[0].0 - 0.0).abs() < 1e-9,
        "first segment starts at t=0"
    );
    for w in segs.windows(2) {
        assert!((w[1].0 - w[0].1).abs() < 1e-9, "time gap between segments");
    }
    let last = segs.last().unwrap();
    assert!(
        (last.2 - 90.0).abs() < 1e-6,
        "trajectory reaches the final x"
    );
    assert!(h.commit_fire_count() >= 1);
    h.shutdown();
}

#[test]
fn dwell_inserts_a_time_gap_then_resumes() {
    let cap = Capture::default();
    let mut h = StreamPlannerHandle::spawn(
        cfg(),
        AxisChainSet::default(),
        vec![0.0, 0.0, 0.0],
        cap.dispatch(),
        cap.nudge_dispatch(),
    );

    h.submit_move(line(1, [0.0, 0.0, 0.0], [30.0, 0.0, 0.0]))
        .unwrap();
    h.flush().unwrap();
    let pre = cap.snapshot();
    let pre_end = pre.last().unwrap().1;

    h.dwell(1.0).unwrap();
    h.submit_move(line(2, [30.0, 0.0, 0.0], [60.0, 0.0, 0.0]))
        .unwrap();
    h.flush().unwrap();

    let post = cap.snapshot();
    let resume_start = post[pre.len()].0;
    assert!(
        (resume_start - (pre_end + 1.0)).abs() < 1e-6,
        "expected a 1.0s dwell gap: resume {resume_start} vs pre_end {pre_end}"
    );
    h.shutdown();
}

#[test]
fn stream_open_restarts_the_timeline_at_zero() {
    let cap = Capture::default();
    let mut h = StreamPlannerHandle::spawn(
        cfg(),
        AxisChainSet::default(),
        vec![0.0, 0.0, 0.0],
        cap.dispatch(),
        cap.nudge_dispatch(),
    );

    h.submit_move(line(1, [0.0, 0.0, 0.0], [30.0, 0.0, 0.0]))
        .unwrap();
    h.flush().unwrap();
    let before = cap.snapshot().len();

    h.stream_open(vec![0.0, 0.0, 0.0]).unwrap();
    h.submit_move(line(2, [0.0, 0.0, 0.0], [30.0, 0.0, 0.0]))
        .unwrap();
    h.flush().unwrap();

    let post = cap.snapshot();
    assert!(post.len() > before);
    assert!(
        (post[before].0 - 0.0).abs() < 1e-9,
        "post-stream-open timeline must restart at 0, got {}",
        post[before].0
    );
    h.shutdown();
}

#[test]
fn home_drip_moves_to_the_travel_endpoint_on_the_new_pipeline() {
    let cap = Capture::default();
    let mut h = StreamPlannerHandle::spawn(
        cfg(),
        AxisChainSet::default(),
        vec![0.0, 0.0, 0.0, 0.0],
        cap.dispatch(),
        cap.nudge_dispatch(),
    );
    let (tx, rx) = crossbeam_channel::bounded(1);
    h.home_drip(HomeDripParams {
        home_pos: [0.0, 0.0, 0.0, 0.0],
        start: [0.0, 0.0, 0.0],
        axis: 0,
        direction: 1.0,
        speed_mm_s: 50.0,
        max_travel_mm: 20.0,
        cohort: 0,
        participants: Vec::new(),
        notify: tx,
    })
    .unwrap();
    assert!(rx.recv().unwrap().is_ok());
    let segs = cap.snapshot();
    assert!(!segs.is_empty(), "homing dispatched nothing");
    assert!(
        (segs.last().unwrap().2 - 20.0).abs() < 1e-6,
        "homing reaches the travel endpoint"
    );
    h.shutdown();
}

#[test]
fn nudge_dispatches_pieces_and_advances_time() {
    let cap = Capture::default();
    let mut h = StreamPlannerHandle::spawn(
        cfg(),
        AxisChainSet::default(),
        vec![0.0, 0.0, 0.0, 0.0],
        cap.dispatch(),
        cap.nudge_dispatch(),
    );
    let (tx, rx) = crossbeam_channel::bounded(1);
    h.submit_nudge(NudgeParams {
        mcu_id: 0,
        axis: 0,
        motor_mask: 0,
        delta_mm: 1.0,
        speed: 10.0,
        accel: 100.0,
        notify: tx,
    })
    .unwrap();
    assert!(rx.recv().unwrap().is_ok());
    assert!(cap.nudge_count() > 0, "no nudge pieces dispatched");
    assert!(
        h.last_move_time() > 0.0,
        "time did not advance past the nudge"
    );
    h.shutdown();
}

fn nominal_secs(m: &geometry::Move) -> f64 {
    nominal_t(m).unwrap()
}

#[test]
fn intake_tally_accrues_nominal_on_intake() {
    let atomic = AtomicU64::new(0);
    let mut tally = IntakeTally::new(&atomic);
    let a = line(1, [0.0, 0.0, 0.0], [40.0, 0.0, 0.0]);
    let b = line(2, [40.0, 0.0, 0.0], [60.0, 0.0, 0.0]);
    let expected = nominal_secs(&a) + nominal_secs(&b);

    tally.record_intake(&a);
    tally.record_intake(&b);

    assert!(expected > 0.0);
    assert!((f64::from_bits(atomic.load(Ordering::Acquire)) - expected).abs() < 1e-12);
}

#[test]
fn intake_tally_subtracts_committed_nominal() {
    let atomic = AtomicU64::new(0);
    let mut tally = IntakeTally::new(&atomic);
    let a = line(1, [0.0, 0.0, 0.0], [40.0, 0.0, 0.0]);
    let b = line(2, [40.0, 0.0, 0.0], [60.0, 0.0, 0.0]);
    let c = line(3, [60.0, 0.0, 0.0], [100.0, 0.0, 0.0]);
    tally.record_intake(&a);
    tally.record_intake(&b);
    tally.record_intake(&c);

    tally.subtract_committed(1);

    let remaining = nominal_secs(&b) + nominal_secs(&c);
    assert!((f64::from_bits(atomic.load(Ordering::Acquire)) - remaining).abs() < 1e-12);
}

#[test]
fn intake_tally_empties_to_exactly_zero_on_full_commit() {
    let atomic = AtomicU64::new(0);
    let mut tally = IntakeTally::new(&atomic);
    tally.record_intake(&line(1, [0.0, 0.0, 0.0], [40.0, 0.0, 0.0]));
    tally.record_intake(&line(2, [40.0, 0.0, 0.0], [60.0, 0.0, 0.0]));

    tally.subtract_committed(2);

    assert_eq!(f64::from_bits(atomic.load(Ordering::Acquire)), 0.0);
}

#[test]
fn intake_tally_reset_zeroes_the_signal() {
    let atomic = AtomicU64::new(0);
    let mut tally = IntakeTally::new(&atomic);
    tally.record_intake(&line(1, [0.0, 0.0, 0.0], [40.0, 0.0, 0.0]));

    tally.reset();

    assert_eq!(f64::from_bits(atomic.load(Ordering::Acquire)), 0.0);
}

#[test]
fn flushed_stream_reads_zero_uncommitted_intake() {
    let cap = Capture::default();
    let mut h = StreamPlannerHandle::spawn(
        cfg(),
        AxisChainSet::default(),
        vec![0.0, 0.0, 0.0],
        cap.dispatch(),
        cap.nudge_dispatch(),
    );
    h.submit_move(line(1, [0.0, 0.0, 0.0], [30.0, 0.0, 0.0]))
        .unwrap();
    h.submit_move(line(2, [30.0, 0.0, 0.0], [60.0, 0.0, 0.0]))
        .unwrap();
    h.flush().unwrap();

    assert_eq!(h.uncommitted_intake_secs(), 0.0);
    h.shutdown();
}

#[test]
fn partial_commit_head_trim_keeps_intake_tally_bounded() {
    let cap = Capture::default();
    let mut h = StreamPlannerHandle::spawn(
        cfg_cap(256),
        AxisChainSet::default(),
        vec![0.0, 0.0, 0.0],
        cap.dispatch(),
        cap.nudge_dispatch(),
    );

    let n: u32 = 16;
    let mut prev = [0.0, 0.0, 0.0];
    let mut total_nominal = 0.0;
    for i in 0..n {
        let x = f64::from(i + 1) * 20.0;
        let y = if i % 2 == 0 { 3.0 } else { 0.0 };
        let end = [x, y, 0.0];
        let m = line(i + 1, prev, end);
        total_nominal += nominal_secs(&m);
        h.submit_move(m).unwrap();
        prev = end;
    }

    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(5);
    while cap.snapshot().len() < 6 {
        assert!(
            std::time::Instant::now() < deadline,
            "continuity commit never dispatched the blend run"
        );
        std::thread::yield_now();
    }

    let mid = h.uncommitted_intake_secs();
    assert!(
        mid >= 0.0,
        "tally went negative through the real head-trim path: {mid}"
    );
    assert!(
        mid < total_nominal,
        "committed moves were not subtracted on the real commit path: {mid} !< {total_nominal}"
    );

    h.flush().unwrap();
    assert_eq!(
        h.uncommitted_intake_secs(),
        0.0,
        "tally must be exactly zero after flush through the partial-commit path"
    );
    h.shutdown();
}

#[test]
fn submit_move_errors_when_channel_full_instead_of_blocking() {
    let (tx, _rx) = crossbeam_channel::bounded::<StreamMsg>(1);
    tx.try_send(StreamMsg::Move(line(1, [0.0, 0.0, 0.0], [10.0, 0.0, 0.0])))
        .unwrap();

    let err = try_submit_move(&tx, line(2, [10.0, 0.0, 0.0], [20.0, 0.0, 0.0]))
        .expect_err("a full channel must error, not block");
    assert!(matches!(err, StreamPlannerError::ChannelFull));
}

#[test]
fn channel_depth_tracks_occupancy_and_refuses_overflow_at_capacity() {
    let cap = 4;
    let (tx, _rx) = crossbeam_channel::bounded::<StreamMsg>(cap);
    for i in 0..cap {
        assert_eq!(tx.len(), i);
        try_submit_move(&tx, line(i as u32, [0.0, 0.0, 0.0], [10.0, 0.0, 0.0]))
            .expect("submit below capacity must succeed");
    }
    assert_eq!(tx.len(), cap);
    let err = try_submit_move(&tx, line(cap as u32, [0.0, 0.0, 0.0], [10.0, 0.0, 0.0]))
        .expect_err("submit at capacity must refuse, not block or grow");
    assert!(matches!(err, StreamPlannerError::ChannelFull));
    assert_eq!(tx.len(), cap, "a refused submit must not grow the queue");
}

#[test]
fn continuous_blend_run_dispatches_continuously_without_flush() {
    let cap = Capture::default();
    // Generous cap so the buffer-cap backstop never fires: the continuity commit
    // alone must drain the run.
    let mut h = StreamPlannerHandle::spawn(
        cfg_cap(256),
        AxisChainSet::default(),
        vec![0.0, 0.0, 0.0],
        cap.dispatch(),
        cap.nudge_dispatch(),
    );

    // A gentle zig-zag: every vertex blends (no unblended seam). The old
    // clean-seam-only commit hung here forever; the continuity commit cuts at
    // each blend exit (zero curvature) and dispatches as moves arrive.
    let n: u32 = 40;
    let mut prev = [0.0, 0.0, 0.0];
    for i in 0..n {
        let x = f64::from(i + 1) * 20.0;
        let y = if i % 2 == 0 { 3.0 } else { 0.0 };
        let end = [x, y, 0.0];
        h.submit_move(line(i + 1, prev, end)).unwrap();
        prev = end;
    }

    // No flush, no waiting on the cap: dispatch happens via commit(false).
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(5);
    while cap.snapshot().len() < 10 {
        assert!(
            std::time::Instant::now() < deadline,
            "continuity commit never dispatched a continuous-blend run (got {})",
            cap.snapshot().len()
        );
        std::thread::yield_now();
    }

    let segs = cap.snapshot();
    assert!(
        (segs[0].0 - 0.0).abs() < 1e-9,
        "first segment starts at t=0"
    );
    for w in segs.windows(2) {
        assert!((w[1].0 - w[0].1).abs() < 1e-9, "time gap between segments");
    }
    h.shutdown();
}

fn co_move(line_no: u32, start: [f64; 3], end: [f64; 3], e_delta: f64) -> geometry::Move {
    line_move(start, end, e_delta, ctx(line_no)).unwrap()
}

#[test]
fn live_retune_pressure_advance_applies_to_plans_after_the_swap() {
    let deltas: Arc<Mutex<Vec<f64>>> = Arc::new(Mutex::new(Vec::new()));
    let cap = Arc::clone(&deltas);
    let dispatch: DispatchFn = Arc::new(move |seg: &ShapedSegment| {
        let t_mid = 0.5 * (seg.t_start + seg.t_end);
        let de = eval(&seg.axes[3], t_mid) - eval(&seg.axes[3], seg.t_start);
        cap.lock().unwrap().push(de);
        Ok(())
    });
    let noop_nudge: NudgeDispatchFn = Arc::new(|_, _| Ok(()));

    let mut h = StreamPlannerHandle::spawn(
        cfg(),
        AxisChainSet::default(),
        vec![0.0, 0.0, 0.0, 0.0],
        dispatch,
        noop_nudge,
    );

    h.submit_move(co_move(1, [0.0, 0.0, 0.0], [40.0, 0.0, 0.0], 4.0))
        .unwrap();
    h.flush().unwrap();
    let before = deltas.lock().unwrap().clone();
    assert_eq!(before.len(), 1, "first move should emit one segment");

    let mut chains = vec![trajectory::CompiledChain::default(); 4];
    chains[3] = trajectory::CompiledChain {
        stages: vec![trajectory::ChainStage::LinearPressureAdvance { k: 0.2 }],
    };
    h.update_axis_chains(AxisChainSet {
        chains,
        followers: Vec::new(),
    })
    .unwrap();

    h.submit_move(co_move(2, [40.0, 0.0, 0.0], [80.0, 0.0, 0.0], 4.0))
        .unwrap();
    h.flush().unwrap();
    let after = deltas.lock().unwrap().clone();
    assert_eq!(after.len(), 2, "second move should emit one more segment");

    let pre_swap = after[0];
    let post_swap = after[1];
    assert!((pre_swap - before[0]).abs() < 1e-12, "held output mutated");
    assert!(
        post_swap > pre_swap + 1e-2,
        "post-swap PA should push the extruder ahead at mid-move: {post_swap} vs {pre_swap}"
    );
    h.shutdown();
}

#[test]
fn flush_returns_after_commit_without_sleeping_until_playout() {
    let cap = Capture::default();
    let mut h = StreamPlannerHandle::spawn(
        cfg(),
        AxisChainSet::default(),
        vec![0.0, 0.0, 0.0, 0.0],
        cap.dispatch(),
        cap.nudge_dispatch(),
    );
    h.submit_move(line_e(1, 5.0, [0.0, 0.0, 0.0], [10.0, 0.0, 0.0], 0.0))
        .unwrap();

    let started = Instant::now();
    h.flush().unwrap();
    let elapsed = started.elapsed();

    assert!(
        !cap.snapshot().is_empty(),
        "flush committed nothing to dispatch"
    );
    assert!(
        elapsed < Duration::from_millis(500),
        "flush blocked {elapsed:?} on a ~2s move: it slept until the play-out \
         deadline instead of returning after commit and letting the caller poll \
         the drain counter"
    );
    h.shutdown();
}
