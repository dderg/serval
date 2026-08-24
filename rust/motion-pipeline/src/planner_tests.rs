//! The commit horizon: how far into its window the streaming planner may cut,
//! and whether cutting there still reproduces a single-window plan.

use crossbeam_channel::unbounded;
use geometry::segment::SourceRange;
use geometry::{CornerFitConfig, MoveContext, VelocityLimits, line_move};

use crate::planner::Planner;
use crate::types::{PlannedItem, PlannedMove, StreamConfig, StreamInput};

const MAX_V: f64 = 600.0;
const ACCEL: f64 = 20_000.0;
const JERK: f64 = f64::INFINITY;
const TRAVEL_MM: f64 = 30.0;
const PRINT_MM: f64 = 0.5;
const PRINT_V: f64 = 50.0;

fn limits() -> VelocityLimits {
    VelocityLimits::try_new(MAX_V, ACCEL, 0.04, JERK).unwrap()
}

fn config(max_buffer_moves: usize) -> StreamConfig {
    StreamConfig {
        corner: CornerFitConfig::default(),
        integration_tol: 1e-4,
        max_extrude_only_velocity_mm_s: f64::INFINITY,
        max_extrude_only_accel_mm_s2: f64::INFINITY,
        fit_tol_mm: 0.005,
        fit_tol_accel_mm_s2: 50.0,
        max_buffer_moves,
        limits: limits(),
    }
}

/// Two fast travels followed by a long run of slow print moves, all collinear
/// along +X so every seam is a tangent-continuous straight-line seam the
/// planner may cut at.
fn travel_then_print(print_moves: usize) -> Vec<geometry::Move> {
    let mut moves = Vec::with_capacity(print_moves + 2);
    let mut x = 0.0;
    let push = |x: &mut f64, len: f64, feed: f64, line_no: u32, out: &mut Vec<geometry::Move>| {
        let ctx = MoveContext {
            extruder_axis: 3,
            feedrate_mm_s: feed,
            limits: limits(),
            source: SourceRange {
                start_line: line_no,
                end_line: line_no,
            },
        };
        out.push(line_move([*x, 0.0, 0.0], [*x + len, 0.0, 0.0], 0.0, ctx).unwrap());
        *x += len;
    };
    push(&mut x, TRAVEL_MM, MAX_V, 0, &mut moves);
    push(&mut x, TRAVEL_MM, MAX_V, 1, &mut moves);
    for i in 0..print_moves {
        push(&mut x, PRINT_MM, PRINT_V, 2 + i as u32, &mut moves);
    }
    moves
}

fn stopping_distance_mm(v: f64) -> f64 {
    let limits = limits();
    let speed = v.min(limits.max_velocity_mm_s);
    speed * speed / (2.0 * limits.accel_mm_s2)
}

fn stream(moves: &[geometry::Move], drain: bool) -> Vec<PlannedMove> {
    let (tx, rx) = unbounded();
    let mut planner = Planner::new(config(4096));
    for m in moves {
        assert!(
            planner.feed(StreamInput::Move(m.clone()), &tx),
            "planner output channel closed"
        );
    }
    if drain {
        assert!(planner.finish(&tx), "planner output channel closed");
    }
    drop(tx);
    rx.into_iter()
        .filter_map(|item| match item {
            PlannedItem::Move(m) => Some(m),
            PlannedItem::Drain | PlannedItem::Control(_) => None,
        })
        .collect()
}

#[test]
fn commit_horizon_is_set_by_the_open_tails_own_speed() {
    let print_moves = 200;
    let moves = travel_then_print(print_moves);
    let total_arc = 2.0 * TRAVEL_MM + print_moves as f64 * PRINT_MM;
    let fast_stopping_distance = stopping_distance_mm(MAX_V);
    let tail_stopping_distance = stopping_distance_mm(PRINT_V);
    assert!(tail_stopping_distance < PRINT_MM);
    assert!(fast_stopping_distance > tail_stopping_distance);

    let emitted = stream(&moves, false);
    let emitted_arc: f64 = emitted.iter().map(|m| m.geometry.segment.s_len()).sum();
    assert!(
        emitted_arc > total_arc - fast_stopping_distance,
        "committed only {emitted_arc:.2} mm of {total_arc:.2}; the tail's own stopping \
         distance is {tail_stopping_distance:.2} mm, so the boundary belongs near the \
         window end, not a fast-tail stopping distance of {fast_stopping_distance:.2} mm back"
    );
}

#[test]
fn streamed_commits_reproduce_a_single_window_plan() {
    let moves = travel_then_print(200);
    let streamed = stream(&moves, true);
    let whole = {
        // One window: no move is committed until the terminal rest is real,
        // so every body is planned against the true end of the stream.
        let (tx, rx) = unbounded();
        let mut planner = Planner::new(config(4096));
        for m in &moves {
            assert!(planner.feed(StreamInput::Move(m.clone()), &tx), "closed");
            // Never let the batch trigger fire: `absorb` re-plans every
            // `REPLAN_BATCH_MOVES` arrivals, and the whole point here is to
            // compare against a plan that never cut.
            planner.moves_since_plan = 0;
        }
        assert!(planner.finish(&tx), "closed");
        drop(tx);
        rx.into_iter()
            .filter_map(|item| match item {
                PlannedItem::Move(m) => Some(m),
                PlannedItem::Drain | PlannedItem::Control(_) => None,
            })
            .collect::<Vec<_>>()
    };

    assert_eq!(streamed.len(), whole.len(), "same moves, different count");
    let worst = streamed
        .iter()
        .zip(&whole)
        .map(|(a, b)| (a.velocity.exit_v - b.velocity.exit_v).abs())
        .fold(0.0_f64, f64::max);
    assert!(
        worst < 1.0,
        "streaming cut at the tail-local horizon changed a seam velocity by \
         {worst} mm/s against the single-window plan — a horizon cut short enough \
         to expose the fictional terminal rest would show the whole braking ramp, \
         tens of mm/s, not the iterative velocity stage's own residual"
    );
}

const RAMP_MM: f64 = 40.0;
const RAMP_FEEDS: [f64; 5] = [10.0, 30.0, 80.0, 200.0, 10.0];
const RAMP_E_PER_MM: f64 = 0.05;

fn ramp_limits() -> VelocityLimits {
    VelocityLimits::try_new(300.0, 3000.0, 5.0, f64::INFINITY).unwrap()
}

/// The speed_ramp shape, repeated: collinear 40 mm extruding moves whose
/// feedrate steps 10 → 30 → 80 → 200 → 10 mm/s. Collinear with one `de/ds`
/// throughout, so no seam anchors to rest and the whole stream is one run —
/// every dip below the slowest feedrate is the planner's own doing.
fn speed_ramp(cycles: usize) -> Vec<geometry::Move> {
    let mut moves = Vec::with_capacity(cycles * RAMP_FEEDS.len());
    let mut x = 0.0;
    for i in 0..cycles * RAMP_FEEDS.len() {
        let ctx = MoveContext {
            extruder_axis: 3,
            feedrate_mm_s: RAMP_FEEDS[i % RAMP_FEEDS.len()],
            limits: ramp_limits(),
            source: SourceRange {
                start_line: i as u32,
                end_line: i as u32,
            },
        };
        moves.push(
            line_move(
                [x, 0.0, 0.0],
                [x + RAMP_MM, 0.0, 0.0],
                RAMP_E_PER_MM * RAMP_MM,
                ctx,
            )
            .unwrap(),
        );
        x += RAMP_MM;
    }
    moves
}

fn ramp_config() -> StreamConfig {
    StreamConfig {
        limits: ramp_limits(),
        ..config(4096)
    }
}

fn plan_one_window(moves: &[geometry::Move]) -> Vec<PlannedMove> {
    let (tx, rx) = unbounded();
    let mut planner = Planner::new(ramp_config());
    for m in moves {
        assert!(planner.feed(StreamInput::Move(m.clone()), &tx), "closed");
        planner.moves_since_plan = 0;
    }
    assert!(planner.finish(&tx), "closed");
    drop(tx);
    rx.into_iter()
        .filter_map(|item| match item {
            PlannedItem::Move(m) => Some(m),
            PlannedItem::Drain | PlannedItem::Control(_) => None,
        })
        .collect()
}

fn stream_ramp(moves: &[geometry::Move]) -> Vec<PlannedMove> {
    let (tx, rx) = unbounded();
    let mut planner = Planner::new(ramp_config());
    for m in moves {
        assert!(planner.feed(StreamInput::Move(m.clone()), &tx), "closed");
    }
    assert!(planner.finish(&tx), "closed");
    drop(tx);
    rx.into_iter()
        .filter_map(|item| match item {
            PlannedItem::Move(m) => Some(m),
            PlannedItem::Drain | PlannedItem::Control(_) => None,
        })
        .collect()
}

/// Slowest velocity anywhere strictly inside the plan — the ramps out of and
/// into the stream's own rest anchors are the two moves at the ends.
fn interior_min_velocity(plan: &[PlannedMove]) -> f64 {
    plan[1..plan.len() - 1]
        .iter()
        .flat_map(|m| {
            m.velocity.phases.iter().flat_map(|phase| {
                let end_velocity = phase.state_at(phase.t0 + phase.dt).1;
                [phase.v0, end_velocity]
            })
        })
        .fold(f64::INFINITY, f64::min)
}

#[test]
fn streamed_speed_ramp_holds_its_plateaus() {
    // 100 moves is many commit horizons' worth, so the stream really does cut
    // and warm-start repeatedly instead of resolving as one window.
    let moves = speed_ramp(20);
    let streamed = stream_ramp(&moves);
    let whole = plan_one_window(&moves);
    assert_eq!(streamed.len(), moves.len());
    assert_eq!(whole.len(), moves.len());

    let floor = RAMP_FEEDS.iter().fold(f64::INFINITY, |m, &f| m.min(f));
    let whole_min = interior_min_velocity(&whole);
    assert!(
        whole_min >= floor - 1e-6,
        "the single-window plan itself dipped to {whole_min} mm/s between rest \
         anchors; the slowest feedrate in the ramp is {floor} mm/s"
    );
    let streamed_min = interior_min_velocity(&streamed);
    assert!(
        streamed_min >= whole_min - 1e-6,
        "streaming dipped to {streamed_min} mm/s where the single-window plan \
         held {whole_min} mm/s — a commit seam is braking toward its own \
         fictional terminal rest"
    );
}
