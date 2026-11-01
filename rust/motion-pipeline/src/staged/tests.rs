use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use geometry::segment::SourceRange;
use geometry::{CornerFitConfig, MoveContext, VelocityLimits, line_move};
use trajectory::{AxisChainSet, CompiledChain, PostProcessorInstance};

use crate::{
    BarrierAck, Control, DispatchCommand, Pipeline, SHAPER_INBOX_CAP, StreamConfig, StreamInput,
    TrajectoryItem,
};

fn limits() -> VelocityLimits {
    VelocityLimits::try_new(
        300.0,
        5000.0,
        geometry::corner_deviation_from_scv(5.0, 5000.0),
        f64::INFINITY,
    )
    .unwrap()
}

fn config() -> StreamConfig {
    StreamConfig {
        corner: CornerFitConfig::default(),
        integration_tol: 1e-7,
        max_extrude_only_velocity_mm_s: f64::INFINITY,
        max_extrude_only_accel_mm_s2: f64::INFINITY,
        fit_tol_mm: 1e-3,
        fit_tol_accel_mm_s2: 50.0,
        max_buffer_moves: 64,
        limits: limits(),
    }
}

fn shaped_follower_chains() -> AxisChainSet {
    let bell = |name: &str, smooth_time: f64| {
        PostProcessorInstance::new(name, &trajectory::algos::SmoothBell, vec![smooth_time])
    };
    let compile = |instances: &[PostProcessorInstance]| {
        CompiledChain::compile(instances).expect("test chain compiles")
    };
    AxisChainSet {
        chains: vec![
            compile(&[bell("is_x", 0.02)]),
            compile(&[bell("is_y", 0.025)]),
            CompiledChain::default(),
            compile(&[
                PostProcessorInstance::new(
                    "pa",
                    &trajectory::algos::LinearPressureAdvance,
                    vec![0.03],
                ),
                bell("st", 0.01),
            ]),
        ],
        followers: vec![(3, vec![0, 1, 2])],
    }
}

const HOME: [f64; 4] = [0.0; 4];

fn zigzag_moves(count: u32) -> Vec<StreamInput> {
    let mut start = [0.0, 0.0, 0.0];
    (1..=count)
        .map(|line| {
            let x = if line % 2 == 0 { 0.0 } else { 20.0 };
            let end = [x, f64::from(line), 0.0];
            let context = MoveContext {
                extruder_axis: 3,
                feedrate_mm_s: 150.0,
                limits: limits(),
                source: SourceRange {
                    start_line: line,
                    end_line: line,
                },
            };
            let m = line_move(start, end, 0.8, context).unwrap();
            start = end;
            StreamInput::Move(m)
        })
        .collect()
}

fn stream_with_a_mid_drain() -> Vec<StreamInput> {
    let mut items = zigzag_moves(30);
    items.insert(15, StreamInput::Drain);
    items
}

fn pipeline() -> Pipeline {
    Pipeline::new(config(), shaped_follower_chains(), HOME.to_vec(), 0.0)
}

fn describe(item: &TrajectoryItem) -> String {
    match item {
        TrajectoryItem::Seg(segment) => format!("Seg {segment:?}"),
        TrajectoryItem::Parked => "Parked".to_string(),
        TrajectoryItem::Control(control) => format!("Control {control:?}"),
    }
}

fn synchronous_output(items: Vec<StreamInput>) -> Vec<String> {
    let mut pipeline = pipeline();
    let mut out = Vec::new();
    let mut collect = |item: TrajectoryItem| {
        out.push(describe(&item));
        true
    };
    for item in items {
        assert!(pipeline.feed(item, &mut collect));
    }
    assert!(pipeline.finish(&mut collect));
    out
}

fn ack(reply: &crossbeam_channel::Sender<BarrierAck>) {
    reply
        .send(BarrierAck {
            dispatched_through: None,
            result: Ok(()),
        })
        .unwrap();
}

#[test]
fn the_staged_pipeline_emits_exactly_the_synchronous_stream() {
    let expected = synchronous_output(stream_with_a_mid_drain());
    let emitted = Arc::new(Mutex::new(Vec::new()));
    let sink = Arc::clone(&emitted);
    let mut staged = pipeline().into_staged(move |item| {
        sink.lock().unwrap().push(describe(&item));
        true
    });
    for item in stream_with_a_mid_drain() {
        assert!(staged.feed(item));
    }
    assert!(staged.finish());
    assert!(expected.iter().any(|item| item.starts_with("Seg")));
    assert_eq!(*emitted.lock().unwrap(), expected);
}

#[test]
fn a_barrier_is_acknowledged_only_after_the_shaper_emitted_everything_ahead_of_it() {
    let expected_segments = synchronous_output(zigzag_moves(20))
        .iter()
        .filter(|item| item.starts_with("Seg"))
        .count();
    let segments = Arc::new(AtomicUsize::new(0));
    let segments_at_barrier = Arc::new(AtomicUsize::new(usize::MAX));
    let (counted, at_barrier) = (Arc::clone(&segments), Arc::clone(&segments_at_barrier));
    let mut staged = pipeline().into_staged(move |item| {
        match item {
            TrajectoryItem::Seg(_) => {
                std::thread::sleep(Duration::from_millis(2));
                counted.fetch_add(1, Ordering::SeqCst);
            }
            TrajectoryItem::Control(Control::Dispatch(DispatchCommand::Barrier(reply))) => {
                at_barrier.store(counted.load(Ordering::SeqCst), Ordering::SeqCst);
                ack(&reply);
            }
            TrajectoryItem::Parked | TrajectoryItem::Control(_) => {}
        }
        true
    });
    for item in zigzag_moves(20) {
        assert!(staged.feed(item));
    }
    assert!(staged.feed(StreamInput::Drain));
    let (reply, acked) = crossbeam_channel::bounded(1);
    assert!(staged.feed(StreamInput::Control(Control::Dispatch(
        DispatchCommand::Barrier(reply)
    ))));
    acked.recv().expect("barrier acknowledged");
    assert_eq!(segments.load(Ordering::SeqCst), expected_segments);
    assert_eq!(
        segments_at_barrier.load(Ordering::SeqCst),
        expected_segments
    );
    assert!(staged.finish());
}

fn dwell() -> StreamInput {
    StreamInput::Control(Control::Dwell { secs: 0.01 })
}

fn feed_until_refused(staged: &mut crate::StagedPipeline) -> bool {
    std::iter::repeat_with(dwell)
        .take(4 * SHAPER_INBOX_CAP)
        .any(|item| !staged.feed(item))
}

#[test]
fn a_refusing_sink_closes_the_planning_side() {
    let mut staged = pipeline().into_staged(|_| false);
    assert!(feed_until_refused(&mut staged));
    assert!(!staged.finish());
}

#[test]
fn a_panicking_shaper_closes_the_planning_side() {
    let mut staged = pipeline().into_staged(|_| panic!("shaper sink exploded"));
    assert!(feed_until_refused(&mut staged));
}

#[test]
fn a_stalled_sink_backpressures_planning_at_the_inbox_bound() {
    let (release, gate) = crossbeam_channel::bounded::<()>(0);
    let mut staged = pipeline().into_staged(move |_| {
        let _ = gate.recv();
        true
    });
    let fed = Arc::new(AtomicUsize::new(0));
    let feeding = Arc::clone(&fed);
    let dwells = 4 * SHAPER_INBOX_CAP;
    let planner = std::thread::spawn(move || {
        for _ in 0..dwells {
            assert!(staged.feed(dwell()));
            feeding.fetch_add(1, Ordering::SeqCst);
        }
        staged
    });
    std::thread::sleep(Duration::from_millis(200));
    let fed_while_stalled = fed.load(Ordering::SeqCst);
    assert!(
        fed_while_stalled <= SHAPER_INBOX_CAP + 1,
        "planning ran {fed_while_stalled} items ahead of a stalled shaper"
    );
    drop(release);
    let mut staged = planner.join().unwrap();
    assert_eq!(fed.load(Ordering::SeqCst), dwells);
    assert!(staged.finish());
}
