use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use super::*;
use crate::planner::{HomeDripParams, NudgeParams};
use crate::pump::{AxisKey, FrontierMsg, LOOKAHEAD_SECS};
use crate::stream::StreamConfig;
use geometry::segment::SourceRange;
use geometry::{ChainFitConfig, MoveContext, VelocityConfig, VelocityLimits, line_move};
use nurbs::eval::eval;

#[derive(Clone, Default)]
struct Capture {
    segs: Arc<Mutex<Vec<(f64, f64, f64)>>>,
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
    fn gated_dispatch(&self, draining: Arc<AtomicBool>) -> DispatchFn {
        let segs = Arc::clone(&self.segs);
        Arc::new(move |seg: &ShapedSegment| {
            if !draining.load(Ordering::Acquire) {
                return Err(DispatchError::Gated);
            }
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

fn cfg(keep_secs: f64) -> StreamConfig {
    StreamConfig {
        chain: ChainFitConfig::default(),
        velocity: VelocityConfig::default(),
        fit_tol_mm: 1e-3,
        keep_secs,
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

fn credit_inputs() -> (
    crossbeam_channel::Sender<FrontierMsg>,
    crossbeam_channel::Receiver<FrontierMsg>,
    Arc<AtomicU64>,
) {
    let (credit_tx, credit_rx) = crossbeam_channel::unbounded();
    let frontier_bits = Arc::new(AtomicU64::new(0.0f64.to_bits()));
    (credit_tx, credit_rx, frontier_bits)
}

fn default_inputs() -> (
    crossbeam_channel::Sender<FrontierMsg>,
    crossbeam_channel::Receiver<FrontierMsg>,
    Arc<AtomicU64>,
) {
    let (credit_tx, credit_rx) = crossbeam_channel::unbounded();
    let frontier_bits = Arc::new(AtomicU64::new(f64::NEG_INFINITY.to_bits()));
    (credit_tx, credit_rx, frontier_bits)
}

#[test]
fn streams_collinear_moves_to_a_contiguous_trajectory() {
    let cap = Capture::default();
    let (_credit_tx, credit_rx, frontier_bits) = default_inputs();
    let mut h = StreamPlannerHandle::spawn(
        cfg(1.0),
        vec![0.0, 0.0, 0.0],
        cap.dispatch(),
        cap.nudge_dispatch(),
        credit_rx,
        frontier_bits,
        Arc::new(AtomicBool::new(false)),
        Vec::new(),
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
    let (_credit_tx, credit_rx, frontier_bits) = default_inputs();
    let mut h = StreamPlannerHandle::spawn(
        cfg(1.0),
        vec![0.0, 0.0, 0.0],
        cap.dispatch(),
        cap.nudge_dispatch(),
        credit_rx,
        frontier_bits,
        Arc::new(AtomicBool::new(false)),
        Vec::new(),
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
    let (_credit_tx, credit_rx, frontier_bits) = default_inputs();
    let mut h = StreamPlannerHandle::spawn(
        cfg(1.0),
        vec![0.0, 0.0, 0.0],
        cap.dispatch(),
        cap.nudge_dispatch(),
        credit_rx,
        frontier_bits,
        Arc::new(AtomicBool::new(false)),
        Vec::new(),
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
    let (_credit_tx, credit_rx, frontier_bits) = default_inputs();
    let mut h = StreamPlannerHandle::spawn(
        cfg(1.0),
        vec![0.0, 0.0, 0.0, 0.0],
        cap.dispatch(),
        cap.nudge_dispatch(),
        credit_rx,
        frontier_bits,
        Arc::new(AtomicBool::new(false)),
        Vec::new(),
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
    let (_credit_tx, credit_rx, frontier_bits) = default_inputs();
    let mut h = StreamPlannerHandle::spawn(
        cfg(1.0),
        vec![0.0, 0.0, 0.0, 0.0],
        cap.dispatch(),
        cap.nudge_dispatch(),
        credit_rx,
        frontier_bits,
        Arc::new(AtomicBool::new(false)),
        Vec::new(),
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

#[test]
fn backpressure_parks_dispatch_services_flush_and_resumes_on_frontier() {
    let cap = Capture::default();
    let (credit_tx, credit_rx, frontier_bits) = credit_inputs();
    let gate_bits = Arc::clone(&frontier_bits);
    let segs = Arc::clone(&cap.segs);
    let dispatch: DispatchFn = Arc::new(move |seg: &ShapedSegment| {
        let frontier = f64::from_bits(gate_bits.load(Ordering::Acquire));
        if seg.t_start > frontier + LOOKAHEAD_SECS {
            return Err(DispatchError::Gated);
        }
        let x_end = eval(&seg.axes[0], seg.t_end);
        segs.lock().unwrap().push((seg.t_start, seg.t_end, x_end));
        Ok(())
    });
    let mut h = StreamPlannerHandle::spawn(
        cfg(1.0),
        vec![0.0, 0.0, 0.0],
        dispatch,
        cap.nudge_dispatch(),
        credit_rx,
        frontier_bits,
        Arc::new(AtomicBool::new(false)),
        vec![AxisKey { mcu_id: 1, axis: 0 }],
    );

    h.submit_move(line(1, [0.0, 0.0, 0.0], [30.0, 0.0, 0.0]))
        .unwrap();
    h.dwell(6.0).unwrap();
    h.submit_move(line(2, [30.0, 0.0, 0.0], [60.0, 0.0, 0.0]))
        .unwrap();

    std::thread::sleep(Duration::from_millis(50));
    let parked = cap.snapshot();
    assert!(!parked.is_empty(), "first segment should dispatch");
    assert!(
        parked.iter().all(|(_, _, x)| *x < 60.0),
        "second move dispatched before the frontier advanced"
    );

    let flush_rx = h.flush_start().unwrap();
    flush_rx
        .recv_timeout(Duration::from_millis(250))
        .expect("flush should be serviced while dispatch is parked");

    credit_tx
        .send(FrontierMsg {
            key: AxisKey { mcu_id: 1, axis: 0 },
            freed_time: 2.0,
        })
        .unwrap();

    for _ in 0..20 {
        if cap
            .snapshot()
            .iter()
            .any(|(_, _, x)| (*x - 60.0).abs() < 1e-6)
        {
            h.shutdown();
            return;
        }
        std::thread::sleep(Duration::from_millis(10));
    }
    h.shutdown();
    panic!("dispatch did not resume after frontier advanced");
}

#[test]
fn backpressure_frontier_is_min_across_axes() {
    let cap = Capture::default();
    let (credit_tx, credit_rx, frontier_bits) = credit_inputs();
    let mut h = StreamPlannerHandle::spawn(
        cfg(1.0),
        vec![0.0, 0.0, 0.0],
        cap.dispatch(),
        cap.nudge_dispatch(),
        credit_rx,
        Arc::clone(&frontier_bits),
        Arc::new(AtomicBool::new(false)),
        vec![
            AxisKey { mcu_id: 1, axis: 0 },
            AxisKey { mcu_id: 1, axis: 1 },
        ],
    );

    credit_tx
        .send(FrontierMsg {
            key: AxisKey { mcu_id: 1, axis: 0 },
            freed_time: 8.0,
        })
        .unwrap();
    credit_tx
        .send(FrontierMsg {
            key: AxisKey { mcu_id: 1, axis: 1 },
            freed_time: 3.0,
        })
        .unwrap();

    for _ in 0..20 {
        if (h.frontier() - 3.0).abs() < 1e-9 {
            h.shutdown();
            return;
        }
        std::thread::sleep(Duration::from_millis(10));
    }
    h.shutdown();
    panic!("frontier did not settle to bottleneck axis");
}

#[test]
fn drain_pending_flushes_gated_segments_past_a_barrier() {
    let cap = Capture::default();
    let (_credit_tx, credit_rx, frontier_bits) = credit_inputs();
    let draining = Arc::new(AtomicBool::new(false));
    let mut h = StreamPlannerHandle::spawn(
        cfg(1.0),
        vec![0.0, 0.0, 0.0],
        cap.gated_dispatch(Arc::clone(&draining)),
        cap.nudge_dispatch(),
        credit_rx,
        Arc::clone(&frontier_bits),
        Arc::clone(&draining),
        vec![AxisKey { mcu_id: 1, axis: 0 }],
    );

    h.submit_move(line(1, [0.0, 0.0, 0.0], [30.0, 0.0, 0.0]))
        .unwrap();
    h.flush().unwrap();
    assert!(
        cap.snapshot().is_empty(),
        "consumption gate must hold the committed segment back"
    );

    h.drain_pending().unwrap();
    assert!(
        !cap.snapshot().is_empty(),
        "drain_pending must force gated segments out across a homing barrier"
    );

    h.shutdown();
}
