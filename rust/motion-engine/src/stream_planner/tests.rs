use std::sync::{Arc, Mutex};

use super::*;
use crate::planner::{HomeDripParams, NudgeParams};
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

#[test]
fn streams_collinear_moves_to_a_contiguous_trajectory() {
    let cap = Capture::default();
    let mut h = StreamPlannerHandle::spawn(
        cfg(1.0),
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
        cfg(1.0),
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
        cfg(1.0),
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
        cfg(1.0),
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
        cfg(1.0),
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
        cfg(1.0),
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
        kernel: None,
        gain: 0.2,
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
