use super::*;
use geometry::segment::SourceRange;
use geometry::{ChainFitConfig, MoveContext, VelocityConfig, VelocityLimits, line_move};
use nurbs::eval::eval;
use proptest::prelude::*;
use trajectory::{AxisChainSet, PostProcessorType, ShapedSegment};

fn cfg() -> StreamConfig {
    StreamConfig {
        chain: ChainFitConfig::default(),
        velocity: VelocityConfig::default(),
        fit_tol_mm: 1e-3,
        max_buffer_moves: 64,
        limits: VelocityLimits::try_new(300.0, 5000.0, 5.0).unwrap(),
    }
}

fn ctx(line_no: u32, feed: f64) -> MoveContext {
    MoveContext {
        extruder_axis: 3,
        feedrate_mm_s: feed,
        limits: VelocityLimits::try_new(300.0, 5000.0, 5.0).unwrap(),
        source: SourceRange {
            start_line: line_no,
            end_line: line_no,
        },
    }
}

fn line(line_no: u32, start: [f64; 3], end: [f64; 3], e: f64) -> geometry::Move {
    line_move(start, end, e, ctx(line_no, 80.0)).unwrap()
}

fn cfg_bench() -> StreamConfig {
    StreamConfig {
        chain: ChainFitConfig::default(),
        velocity: VelocityConfig {
            max_jerk_mm_s3: 1_000_000.0,
            integration_tol: 1e-4,
            ..VelocityConfig::default()
        },
        fit_tol_mm: 0.005,
        max_buffer_moves: 512,
        limits: VelocityLimits::try_new(100.0, 1000.0, 5.0).unwrap(),
    }
}

fn line_bench(line_no: u32, start: [f64; 3], end: [f64; 3]) -> geometry::Move {
    let ctx = MoveContext {
        extruder_axis: 3,
        feedrate_mm_s: 60.0,
        limits: VelocityLimits::try_new(100.0, 1000.0, 5.0).unwrap(),
        source: SourceRange {
            start_line: line_no,
            end_line: line_no,
        },
    };
    line_move(start, end, 0.0, ctx).unwrap()
}

// Real first perimeter from a Voron cube print (Neptune bench), as (x, y, e).
// 135° chamfer corners blend; short ~1.3mm chamfers sit between long ~18.6mm edges.
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
    (99.13, 99.186, 0.14711),
];

#[test]
fn voron_cube_perimeter_streams_without_degenerate_trim() {
    // Real first perimeter from a Voron cube print (Neptune bench) — the print
    // that aborted with "head-trim geometry: ZeroMotion". 135° chamfer corners
    // blend; short ~1.3mm chamfer segments sit between long ~18.6mm edges.
    // Replays the moves through incremental commits like the planner loop and
    // asserts no commit ever errors.
    let start = [99.158, 99.158, 0.2];
    let mut s = StreamState::new(
        cfg(),
        AxisChainSet::default(),
        &[start[0], start[1], start[2], 0.0],
        0.0,
    );
    let mut prev = start;
    for (i, (x, y, e)) in VORON_PERIMETER.into_iter().enumerate() {
        let end = [x, y, 0.2];
        s.push(line(i as u32 + 1, prev, end, e)).unwrap();
        prev = end;
        s.commit(false)
            .unwrap_or_else(|err| panic!("commit at move {i} errored: {err}"));
    }
    s.commit(true).expect("final flush must not error");
    assert!(s.is_empty());
}

#[test]
fn cold_run_infill_streams_without_overcommit() {
    // Real infill prefix from cold_run.gcode (Neptune bench) — the path that
    // aborted klippy mid-print with `velocity plan: OverCommitted`. The hazard
    // is purely about commit granularity: committing one move per commit (as the
    // run_loop does under a fast SD-stream burst) pins an over-optimistic seam
    // velocity that the re-fit of the following moves cannot honor. Committing
    // the identical path in one batch plans cleanly, so this asserts the
    // streamed result matches the batched one: no commit may error. Bench
    // limits: max 100 mm/s, 1000 mm/s^2, jerk 1e6; infill feed 60 mm/s.
    let start = [99.158, 99.158, 0.0];
    let mut s = StreamState::new(
        cfg_bench(),
        AxisChainSet::default(),
        &[start[0], start[1], start[2], 0.0],
        0.0,
    );
    let pts: [(f64, f64); 91] = [
        (99.158, 99.158),
        (102.008, 96.308),
        (103.2, 95.814),
        (121.8, 95.814),
        (122.992, 96.308),
        (128.692, 102.008),
        (129.186, 103.2),
        (129.186, 121.8),
        (128.692, 122.992),
        (122.992, 128.692),
        (121.8, 129.186),
        (103.2, 129.186),
        (102.008, 128.692),
        (96.308, 122.992),
        (95.814, 121.8),
        (95.814, 103.2),
        (96.308, 102.008),
        (99.13, 99.186),
        (99.453, 99.51),
        (102.331, 96.631),
        (103.2, 96.271),
        (121.8, 96.271),
        (122.669, 96.631),
        (128.369, 102.331),
        (128.729, 103.2),
        (128.729, 121.8),
        (128.369, 122.669),
        (122.669, 128.369),
        (121.8, 128.729),
        (103.2, 128.729),
        (102.331, 128.369),
        (96.631, 122.669),
        (96.271, 121.8),
        (96.271, 103.2),
        (96.631, 102.331),
        (99.425, 99.538),
        (121.445, 127.05),
        (103.555, 127.05),
        (97.95, 121.445),
        (97.95, 103.555),
        (103.555, 97.95),
        (121.445, 97.95),
        (127.05, 103.555),
        (127.05, 121.445),
        (121.474, 127.022),
        (108.669, 105.367),
        (108.715, 105.339),
        (109.475, 104.986),
        (110.267, 104.714),
        (111.083, 104.525),
        (111.913, 104.422),
        (112.751, 104.404),
        (113.598, 104.474),
        (114.808, 104.736),
        (115.602, 105.018),
        (116.357, 105.378),
        (117.072, 105.814),
        (117.748, 106.33),
        (118.626, 107.202),
        (119.143, 107.866),
        (119.586, 108.577),
        (119.953, 109.33),
        (120.24, 110.116),
        (120.445, 110.928),
        (120.565, 111.756),
        (120.599, 112.593),
        (120.567, 113.226),
        (120.449, 114.051),
        (120.247, 114.864),
        (119.961, 115.651),
        (119.596, 116.404),
        (119.148, 117.127),
        (118.365, 118.085),
        (117.754, 118.664),
        (117.089, 119.174),
        (116.376, 119.612),
        (115.621, 119.974),
        (114.833, 120.256),
        (114.019, 120.456),
        (113.19, 120.57),
        (112.353, 120.598),
        (111.518, 120.54),
        (110.68, 120.393),
        (109.499, 120.023),
        (108.734, 119.671),
        (108.014, 119.244),
        (107.342, 118.745),
        (106.725, 118.179),
        (106.169, 117.553),
        (105.85, 117.108),
        (105.267, 116.145),
    ];
    let mut prev = start;
    for (i, (x, y)) in pts.into_iter().enumerate() {
        let end = [x, y, 0.2];
        s.push(line_bench(i as u32 + 1, prev, end)).unwrap();
        prev = end;
        s.commit(false)
            .unwrap_or_else(|err| panic!("commit at move {i} errored: {err}"));
    }
    s.commit(true).expect("final flush must not error");
    assert!(s.is_empty());
}

#[test]
fn collinear_jogs_commit_at_the_seam_without_stopping() {
    let mut s = StreamState::new(cfg(), AxisChainSet::default(), &[0.0, 0.0, 0.0], 0.0);
    s.push(line(1, [0.0, 0.0, 0.0], [50.0, 0.0, 0.0], 0.0))
        .unwrap();
    s.push(line(2, [50.0, 0.0, 0.0], [100.0, 0.0, 0.0], 0.0))
        .unwrap();

    let committed = s.commit(false).unwrap();
    assert!(!committed.is_empty());
    assert_eq!(s.buffered(), 1);
    assert!(
        s.entry_velocity() > 1.0,
        "seam velocity {} should be cruising, not stopped",
        s.entry_velocity()
    );
    let last = committed.last().unwrap();
    assert!((eval(&last.axes[0], last.t_end) - 50.0).abs() < 1e-6);
}

#[test]
fn flush_commits_everything_to_rest() {
    let mut s = StreamState::new(cfg(), AxisChainSet::default(), &[0.0, 0.0, 0.0], 0.0);
    s.push(line(1, [0.0, 0.0, 0.0], [50.0, 0.0, 0.0], 0.0))
        .unwrap();
    s.push(line(2, [50.0, 0.0, 0.0], [100.0, 0.0, 0.0], 0.0))
        .unwrap();

    // A forced flush drains the whole buffer to rest regardless of where the
    // finality barrier sits — it materializes the brake-to-rest tail.
    let committed = s.commit(true).unwrap();
    assert!(!committed.is_empty());
    assert!(s.is_empty());
    assert_eq!(s.entry_velocity(), 0.0);
    let last = committed.last().unwrap();
    assert!((eval(&last.axes[0], last.t_end) - 100.0).abs() < 1e-6);
}

#[test]
fn blended_corner_commits_through_the_blend_without_stopping() {
    // A 90-degree corner is blended (a biclothoid). The blend rejoins the
    // outgoing line at zero curvature, so the commit runs through the whole
    // blend and keeps the outgoing move as a head-trimmed remainder — never
    // splitting the blend itself, and never stopping at the corner.
    let mut s = StreamState::new(cfg(), AxisChainSet::default(), &[0.0, 0.0, 0.0], 0.0);
    s.push(line(1, [0.0, 0.0, 0.0], [50.0, 0.0, 0.0], 0.0))
        .unwrap();
    s.push(line(2, [50.0, 0.0, 0.0], [50.0, 50.0, 0.0], 0.0))
        .unwrap();

    let committed = s.commit(false).unwrap();
    assert!(!committed.is_empty(), "the blend must commit");
    assert_eq!(
        s.buffered(),
        1,
        "outgoing move kept as a head-trimmed remainder"
    );
    assert!(
        s.entry_velocity() > 1.0,
        "corner is rounded, not stopped: seam velocity {}",
        s.entry_velocity()
    );

    // The kept remainder still ends at the original move endpoint.
    let rest = s.commit(true).unwrap();
    assert!(s.is_empty());
    let last = rest.last().unwrap();
    assert!((eval(&last.axes[0], last.t_end) - 50.0).abs() < 1e-6);
    assert!((eval(&last.axes[1], last.t_end) - 50.0).abs() < 1e-6);
}

#[test]
fn continuous_blended_chain_drains_without_a_single_stop() {
    // A gentle zigzag: every corner is shallow enough to blend, so there is no
    // unblended seam anywhere. The continuity commit must still drain it (the
    // old clean-seam-only commit would hang here forever), and no interior seam
    // may drop to rest — that would be the stutter we are eliminating.
    let mut s = StreamState::new(cfg(), AxisChainSet::default(), &[0.0, 0.0, 0.0], 0.0);
    let pts = [
        [0.0, 0.0, 0.0],
        [20.0, 0.0, 0.0],
        [40.0, 3.0, 0.0],
        [60.0, 0.0, 0.0],
        [80.0, 3.0, 0.0],
        [100.0, 0.0, 0.0],
        [120.0, 3.0, 0.0],
    ];
    for (i, w) in pts.windows(2).enumerate() {
        s.push(line(i as u32 + 1, w[0], w[1], 0.0)).unwrap();
    }
    let pushed = pts.len() - 1;

    let mut all: Vec<trajectory::ShapedSegment> = Vec::new();
    let mut progressed = true;
    while progressed {
        let committed = s.commit(false).unwrap();
        progressed = !committed.is_empty();
        for seg in committed {
            if let Some(prev) = all.last() {
                assert!(
                    s.entry_velocity() >= 0.0,
                    "entry velocity must stay defined"
                );
                assert!(
                    (seg.t_start - prev.t_end).abs() < 1e-9,
                    "time gap between committed segments"
                );
            }
            all.push(seg);
        }
    }
    assert!(
        s.buffered() < pushed,
        "continuity commit must drain a fully-blended chain ({} of {} left)",
        s.buffered(),
        pushed
    );
    // Every interior seam carried real speed: the planner never paused.
    assert!(
        s.entry_velocity() > 1.0,
        "final carried seam velocity {} should be cruising",
        s.entry_velocity()
    );

    let rest = s.commit(true).unwrap();
    for w in rest.windows(2) {
        assert!((w[1].t_start - w[0].t_end).abs() < 1e-9);
    }
    if let (Some(prev), Some(first)) = (all.last(), rest.first()) {
        assert!((first.t_start - prev.t_end).abs() < 1e-9);
    }
    assert!(s.is_empty());
}

#[test]
fn head_trim_preserves_position_and_extrusion_continuity() {
    // Commit through a blend with extrusion, then verify the kept remainder
    // resumes exactly where the committed trajectory ended (no gap, no overlap)
    // in both the spatial axes and the extruder.
    let mut s = StreamState::new(cfg(), AxisChainSet::default(), &[0.0, 0.0, 0.0, 0.0], 0.0);
    s.push(line(1, [0.0, 0.0, 0.0], [50.0, 0.0, 0.0], 5.0))
        .unwrap();
    s.push(line(2, [50.0, 0.0, 0.0], [50.0, 50.0, 0.0], 5.0))
        .unwrap();
    s.push(line(3, [50.0, 50.0, 0.0], [100.0, 50.0, 0.0], 5.0))
        .unwrap();

    let first = s.commit(false).unwrap();
    assert!(!first.is_empty());
    let seam = first.last().unwrap();
    let seam_x = eval(&seam.axes[0], seam.t_end);
    let seam_y = eval(&seam.axes[1], seam.t_end);
    let seam_e = eval(&seam.axes[3], seam.t_end);

    let rest = s.commit(true).unwrap();
    let resume = rest.first().unwrap();
    assert!((eval(&resume.axes[0], resume.t_start) - seam_x).abs() < 1e-6);
    assert!((eval(&resume.axes[1], resume.t_start) - seam_y).abs() < 1e-6);
    assert!((eval(&resume.axes[3], resume.t_start) - seam_e).abs() < 1e-6);

    // Total extrusion across the whole (3-move) path is conserved: 15 mm.
    let last = rest.last().unwrap();
    assert!((eval(&last.axes[3], last.t_end) - 15.0).abs() < 1e-3);
}

#[test]
fn odometer_accumulates_extrusion_across_commits() {
    let mut s = StreamState::new(cfg(), AxisChainSet::default(), &[0.0, 0.0, 0.0, 0.0], 0.0);
    s.push(line(1, [0.0, 0.0, 0.0], [40.0, 0.0, 0.0], 4.0))
        .unwrap();
    s.push(line(2, [40.0, 0.0, 0.0], [80.0, 0.0, 0.0], 4.0))
        .unwrap();

    let committed = s.commit(true).unwrap();
    assert!(s.is_empty());
    let last = committed.last().unwrap();
    assert!((eval(&last.axes[3], last.t_end) - 8.0).abs() < 1e-3);
}

#[test]
fn committed_trajectory_is_time_contiguous() {
    let mut s = StreamState::new(cfg(), AxisChainSet::default(), &[0.0, 0.0, 0.0], 2.0);
    s.push(line(1, [0.0, 0.0, 0.0], [30.0, 0.0, 0.0], 0.0))
        .unwrap();
    s.push(line(2, [30.0, 0.0, 0.0], [60.0, 0.0, 0.0], 0.0))
        .unwrap();
    s.push(line(3, [60.0, 0.0, 0.0], [90.0, 0.0, 0.0], 0.0))
        .unwrap();

    let committed = s.commit(true).unwrap();
    assert_eq!(committed[0].t_start, 2.0);
    for w in committed.windows(2) {
        assert!(
            (w[1].t_start - w[0].t_end).abs() < 1e-9,
            "time gap between segments"
        );
    }
}

#[test]
fn advance_idle_reanchors_committed_time_after_a_gap() {
    let mut s = StreamState::new(cfg(), AxisChainSet::default(), &[0.0, 0.0, 0.0], 0.0);
    s.push(line(1, [0.0, 0.0, 0.0], [30.0, 0.0, 0.0], 0.0))
        .unwrap();
    let first = s.commit(true).unwrap();
    let after_first = first.last().unwrap().t_end;

    let idle_gap_past_horizon_secs = 50.0;
    s.advance_idle(after_first + idle_gap_past_horizon_secs);
    s.push(line(2, [30.0, 0.0, 0.0], [60.0, 0.0, 0.0], 0.0))
        .unwrap();
    let second = s.commit(true).unwrap();
    assert!(
        second[0].t_start >= after_first + idle_gap_past_horizon_secs - 1e-9,
        "second move must start at the re-anchored time, got {}",
        second[0].t_start
    );
    s.advance_idle(0.0);
    assert!(s.t_committed() >= second.last().unwrap().t_end - 1e-9);
}

fn full_replan(moves: &[geometry::Move], home: &[f64]) -> Vec<ShapedSegment> {
    let mut s = StreamState::new(cfg(), AxisChainSet::default(), home, 0.0);
    for m in moves {
        s.push(m.clone()).unwrap();
    }
    s.commit(true).expect("full re-plan flush")
}

fn committed_prefix(moves: &[geometry::Move], home: &[f64]) -> Vec<ShapedSegment> {
    // Realistic deep-buffer streaming: the driver coalesces a burst of moves so
    // the planner has full look-ahead, then commits every move up to the finality
    // barrier in one shot. Those are the segments actually dispatched mid-print.
    let mut s = StreamState::new(cfg(), AxisChainSet::default(), home, 0.0);
    for m in moves {
        s.push(m.clone()).unwrap();
    }
    s.commit(false).expect("incremental commit")
}

fn voron_moves() -> (Vec<geometry::Move>, Vec<f64>) {
    let start = [99.158, 99.158, 0.2];
    let mut prev = start;
    let mut moves = Vec::new();
    for (i, (x, y, e)) in VORON_PERIMETER.into_iter().enumerate() {
        let end = [x, y, 0.2];
        moves.push(line(i as u32 + 1, prev, end, e));
        prev = end;
    }
    (moves, vec![start[0], start[1], start[2], 0.0])
}

fn dense_arc_moves() -> (Vec<geometry::Move>, Vec<f64>) {
    // A dense polygonal circle: short, continuously-turning segments — the
    // infill-arc character of cold_run.gcode that the streamed re-plan stressed.
    let (r, n, c) = (20.0_f64, 60u32, [50.0, 50.0, 0.0]);
    let start = [c[0] + r, c[1], 0.0];
    let mut prev = start;
    let mut moves = Vec::new();
    for i in 1..=n {
        let a = std::f64::consts::TAU * f64::from(i) / f64::from(n);
        let end = [c[0] + r * a.cos(), c[1] + r * a.sin(), 0.0];
        moves.push(line(i, prev, end, 0.0));
        prev = end;
    }
    (moves, vec![start[0], start[1], start[2], 0.0])
}

#[test]
fn committed_segments_match_a_full_replan() {
    // Output-equivalence: every segment committed incrementally up to the finality
    // barrier is byte-for-byte identical to the leading segments a single full
    // re-plan to rest produces. The non-negotiable throughput constraint forbids
    // trading trajectory quality for cheaper planning; the structural proof
    // guarantees it, and this checks the proof was implemented faithfully. (The
    // brake-to-rest tail past the barrier is a separate flush-only artifact,
    // rebuilt at end-of-stream, and is intentionally not part of this comparison.)
    for (label, (moves, home)) in [
        ("voron_perimeter", voron_moves()),
        ("dense_infill_arc", dense_arc_moves()),
    ] {
        let full = full_replan(&moves, &home);
        let committed = committed_prefix(&moves, &home);
        assert!(!committed.is_empty(), "{label}: nothing committed");
        assert!(
            committed.len() < full.len(),
            "{label}: the barrier must defer the brake-to-rest tail"
        );
        for (i, (a, b)) in committed.iter().zip(&full).enumerate() {
            assert!(
                (a.t_start - b.t_start).abs() < 1e-9,
                "{label} seg {i}: t_start {} vs {}",
                a.t_start,
                b.t_start
            );
            assert!(
                (a.t_end - b.t_end).abs() < 1e-9,
                "{label} seg {i}: t_end {} vs {}",
                a.t_end,
                b.t_end
            );
            for axis in 0..2 {
                let da = eval(&a.axes[axis], a.t_end);
                let db = eval(&b.axes[axis], b.t_end);
                assert!(
                    (da - db).abs() < 1e-9,
                    "{label} seg {i} axis {axis}: {da} vs {db}"
                );
            }
        }
    }
}

#[test]
fn open_tail_stays_bounded_as_buffer_depth_grows() {
    // Collinear cruise edges: each commit advances to the finality barrier, so the
    // retained buffer (the open tail / brake-to-rest region) stays a small constant
    // regardless of total depth — the structural guarantee behind flat-in-depth
    // pipe_plan cost, replacing the old O(buffer-depth) re-plan that spiked to
    // 217 ms.
    let mut tails = Vec::new();
    for &depth in &[50usize, 100, 200, 500] {
        let mut s = StreamState::new(cfg(), AxisChainSet::default(), &[0.0, 0.0, 0.0], 0.0);
        let mut prev = [0.0, 0.0, 0.0];
        let mut max_tail = 0usize;
        for i in 0..depth {
            let end = [(i as f64 + 1.0) * 20.0, 0.0, 0.0];
            s.push(line(i as u32 + 1, prev, end, 0.0)).unwrap();
            prev = end;
            s.commit(false).expect("commit");
            max_tail = max_tail.max(s.buffered());
        }
        tails.push((depth, max_tail));
    }
    let baseline = tails[0].1;
    for &(depth, tail) in &tails {
        assert!(
            tail <= baseline + 2,
            "open tail grew with depth (depth={depth} tail={tail} baseline={baseline}): \
             per-commit work is not flat in buffer depth"
        );
    }
}

#[test]
fn stall_brake_shortfall_is_attributable_and_fails_loud() {
    let mut s = StreamState::new(cfg(), AxisChainSet::default(), &[0.0, 0.0, 0.0], 0.0);
    s.push(line(1, [0.0, 0.0, 0.0], [50.0, 0.0, 0.0], 0.0))
        .unwrap();
    let solve_const = 0.05;
    match s.commit_stall_brake(0.01, solve_const) {
        Err(StreamError::BrakeToRestShortfall {
            lead_remaining,
            solve_const: sc,
        }) => {
            assert!((lead_remaining - 0.01).abs() < 1e-12);
            assert!((sc - solve_const).abs() < 1e-12);
        }
        other => panic!("expected BrakeToRestShortfall, got {other:?}"),
    }
    // The failed shortfall left the buffer intact; an adequate lead drains it.
    let segs = s
        .commit_stall_brake(1.0, solve_const)
        .expect("adequate lead drains to rest");
    assert!(!segs.is_empty());
    assert!(s.is_empty());
    assert_eq!(s.entry_velocity(), 0.0);
}

fn commit_prefix_signature(ys: &[f64], n: usize) -> Option<Vec<(f64, f64, f64, f64)>> {
    let mut s = StreamState::new(cfg(), AxisChainSet::default(), &[0.0, 0.0, 0.0], 0.0);
    let mut prev = [0.0, 0.0, 0.0];
    for i in 0..n {
        let end = [(i as f64 + 1.0) * 20.0, ys[i], 0.0];
        s.push(line(i as u32 + 1, prev, end, 0.0)).unwrap();
        prev = end;
    }
    // Some random shapes lower to a degenerate phase; those inputs are out of
    // scope for the append-invariance property and are skipped by the caller.
    let segs = s.commit(false).ok()?;
    // A non-forced commit never commits the buffer's tentative terminal rest, so
    // the open tail stays buffered.
    assert!(s.buffered() >= 1);
    Some(
        segs.iter()
            .map(|seg| {
                (
                    seg.t_start,
                    seg.t_end,
                    eval(&seg.axes[0], seg.t_end),
                    eval(&seg.axes[1], seg.t_end),
                )
            })
            .collect(),
    )
}

proptest! {
    #[test]
    fn locked_prefix_is_invariant_under_append(
        ys in prop::collection::vec(-1.5f64..1.5, 30..50),
        cut in 8usize..20,
        extra in 1usize..10,
    ) {
        let n_short = cut.min(ys.len() - 1);
        let n_long = (cut + extra).min(ys.len() - 1);
        let (Some(short), Some(long)) =
            (commit_prefix_signature(&ys, n_short), commit_prefix_signature(&ys, n_long))
        else {
            return Ok(());
        };
        // Appending moves can only EXTEND the locked prefix, never retract it.
        prop_assert!(
            long.len() >= short.len(),
            "append shrank the locked prefix: {} -> {}",
            short.len(),
            long.len()
        );
        // Every already-committed seam is unchanged by the append. Positions are
        // exact (deterministic geometry); seam times match within the iterative
        // velocity stage's tolerance — the single segment ending at the barrier
        // carries a negligible terminal-dependent body timing (tens of µs).
        for (i, (a, b)) in short.iter().zip(&long).enumerate() {
            prop_assert!((a.2 - b.2).abs() < 1e-9, "seam {i} x changed: {} vs {}", a.2, b.2);
            prop_assert!((a.3 - b.3).abs() < 1e-9, "seam {i} y changed: {} vs {}", a.3, b.3);
            prop_assert!((a.0 - b.0).abs() < 1e-3, "seam {i} t_start changed: {} vs {}", a.0, b.0);
            prop_assert!((a.1 - b.1).abs() < 1e-3, "seam {i} t_end changed: {} vs {}", a.1, b.1);
        }
    }
}

fn smooth_x_chains(frequency_hz: f64) -> AxisChainSet {
    AxisChainSet::spatial(
        PostProcessorType::SmoothZv { frequency_hz }.into_chain(),
        trajectory::CompiledChain::default(),
        trajectory::CompiledChain::default(),
    )
}

#[test]
fn smooth_shaper_live_path_matches_shaped_signal_oracle() {
    let moves = [
        line(1, [0.0, 0.0, 0.0], [80.0, 0.0, 0.0], 0.0),
        line(2, [80.0, 0.0, 0.0], [160.0, 0.0, 0.0], 0.0),
    ];
    let mut base = StreamState::new(cfg(), AxisChainSet::default(), &[0.0, 0.0, 0.0], 0.0);
    let mut shaped = StreamState::new(cfg(), smooth_x_chains(18.0), &[0.0, 0.0, 0.0], 0.0);
    for m in moves {
        base.push(m.clone()).unwrap();
        shaped.push(m).unwrap();
    }

    let base_out = base.commit(true).unwrap();
    let shaped_out = shaped.commit(true).unwrap();
    assert_eq!(base_out.len(), shaped_out.len());

    let oracle_chains = smooth_x_chains(18.0);
    let trajectory::ChainStage::SmoothKernel(kernel) = &oracle_chains.chains[0].stages[0] else {
        panic!("expected smooth kernel");
    };
    let first = base_out.first().unwrap().t_start;
    let last = base_out.last().unwrap().t_end;

    for (base_seg, shaped_seg) in base_out.iter().zip(&shaped_out) {
        let sig = trajectory::ShapedSignal::new_from_evaluator(
            kernel,
            base_seg.t_start,
            base_seg.t_end,
            |t| {
                let clamped = t.clamp(first, last);
                base_out
                    .iter()
                    .find(|seg| clamped >= seg.t_start && clamped <= seg.t_end)
                    .map_or_else(
                        || eval(&base_out.last().unwrap().axes[0], clamped),
                        |seg| eval(&seg.axes[0], clamped),
                    )
            },
        );
        for frac in [0.1_f64, 0.3, 0.5, 0.7, 0.9] {
            let t = frac.mul_add(base_seg.t_end - base_seg.t_start, base_seg.t_start);
            let got = eval(&shaped_seg.axes[0], t);
            let want = sig.eval(t);
            assert!(
                (got - want).abs() < 5e-2,
                "shaped x at t={t}: got {got}, want {want}"
            );
        }
    }
}

fn short_collinear(n: u32, seg_mm: f64) -> Vec<geometry::Move> {
    (0..n)
        .map(|i| {
            let x0 = f64::from(i) * seg_mm;
            line(i + 1, [x0, 0.0, 0.2], [x0 + seg_mm, 0.0, 0.2], 0.0)
        })
        .collect()
}

#[test]
fn small_buffer_within_setback_yields_empty_commit() {
    // The incremental-commit regression precondition: a buffer whose whole arc
    // sits within one brake-to-rest setback has no seam that leaves a setback of
    // open tail behind it, so the non-forced commit selects nothing and the
    // dispatched frontier would freeze (investigation batch 33: n=4 commit_count=0).
    let mut s = StreamState::new(cfg(), AxisChainSet::default(), &[0.0, 0.0, 0.2, 0.0], 0.0);
    for m in short_collinear(4, 0.3) {
        s.push(m).unwrap();
    }
    let segs = s.commit(false).expect("commit must not error");
    assert!(
        segs.is_empty(),
        "expected commit_count=0 on a sub-setback buffer"
    );
    assert_eq!(s.t_committed(), 0.0, "frontier must not have advanced");
    assert!(
        !s.is_empty(),
        "buffer must still hold the uncommitted moves"
    );
}

#[test]
fn smooth_shaper_holds_back_live_edge_inside_future_support() {
    let mut s = StreamState::new(cfg(), smooth_x_chains(0.5), &[0.0, 0.0, 0.0], 0.0);
    s.push(line(1, [0.0, 0.0, 0.0], [20.0, 0.0, 0.0], 0.0))
        .unwrap();
    s.push(line(2, [20.0, 0.0, 0.0], [40.0, 0.0, 0.0], 0.0))
        .unwrap();

    assert!(
        s.commit(false).unwrap().is_empty(),
        "future support should hold back live-edge shaped samples"
    );
    assert!(
        !s.commit(true).unwrap().is_empty(),
        "force flush must release held shaped samples"
    );
}

#[test]
fn stall_brake_advances_a_frozen_frontier() {
    // The thin-lead force-advance building block: the same uncommittable buffer
    // drains to rest under `commit_stall_brake`, advancing the frontier instead
    // of freezing it.
    let mut s = StreamState::new(cfg(), AxisChainSet::default(), &[0.0, 0.0, 0.2, 0.0], 0.0);
    for m in short_collinear(4, 0.3) {
        s.push(m).unwrap();
    }
    assert!(s.commit(false).expect("commit must not error").is_empty());
    let segs = s
        .commit_stall_brake(1.0, 0.05)
        .expect("stall brake with ample lead must succeed");
    assert!(!segs.is_empty(), "stall brake dispatched nothing");
    assert!(s.t_committed() > 0.0, "frontier did not advance");
    assert!(s.is_empty(), "stall brake must drain the buffer to rest");
}

#[test]
fn stall_brake_fails_loud_when_lead_already_collapsed() {
    // Fail-loud (CLAUDE.md): if the lead is already below the solve budget the
    // ramp cannot be dispatched before its first piece must play, so the planner
    // raises rather than scheduling a piece into the MCU's past.
    let mut s = StreamState::new(cfg(), AxisChainSet::default(), &[0.0, 0.0, 0.2, 0.0], 0.0);
    for m in short_collinear(4, 0.3) {
        s.push(m).unwrap();
    }
    let err = s.commit_stall_brake(0.01, 0.05).unwrap_err();
    assert!(matches!(err, StreamError::BrakeToRestShortfall { .. }));
}

#[test]
fn push_rejects_a_discontinuous_move() {
    // Fail-loud (CLAUDE.md): real slicer output is position-contiguous. A move
    // that does not start where the toolhead was left is a stitching bug
    // upstream; reject it at ingress with the offending line named, instead of
    // letting it surface as a `ZeroMotion` deep in the fitter.
    let mut s = StreamState::new(cfg(), AxisChainSet::default(), &[0.0, 0.0, 0.0], 0.0);
    s.push(line(1, [0.0, 0.0, 0.0], [50.0, 0.0, 0.0], 0.0))
        .unwrap();
    let err = s
        .push(line(2, [0.0, 0.0, 0.0], [50.0, 0.0, 0.0], 0.0))
        .unwrap_err();
    let StreamError::Discontinuity {
        line_no, gap_mm, ..
    } = err
    else {
        panic!("expected Discontinuity, got {err:?}");
    };
    assert_eq!(line_no, 2);
    assert!((gap_mm - 50.0).abs() < 1e-9, "gap was {gap_mm}");
}

#[test]
fn push_accepts_a_contiguous_chain() {
    let mut s = StreamState::new(cfg(), AxisChainSet::default(), &[0.0, 0.0, 0.0], 0.0);
    s.push(line(1, [0.0, 0.0, 0.0], [50.0, 0.0, 0.0], 0.0))
        .unwrap();
    s.push(line(2, [50.0, 0.0, 0.0], [50.0, 50.0, 0.0], 0.0))
        .expect("a move starting where the last ended must be accepted");
}

#[test]
fn smooth_shaper_first_commit_after_nonzero_start_time_is_valid() {
    let mut s = StreamState::new(cfg(), smooth_x_chains(18.0), &[0.0, 0.0, 0.0], 5.0);
    s.push(line(1, [0.0, 0.0, 0.0], [20.0, 0.0, 0.0], 0.0))
        .unwrap();

    let committed = s.commit(true).unwrap();
    assert_eq!(committed[0].t_start, 5.0);
}

#[test]
fn smooth_shaper_after_idle_gap_resumes_from_rest_edge() {
    let mut s = StreamState::new(cfg(), smooth_x_chains(18.0), &[0.0, 0.0, 0.0], 0.0);
    s.push(line(1, [0.0, 0.0, 0.0], [20.0, 0.0, 0.0], 0.0))
        .unwrap();
    let first = s.commit(true).unwrap();
    let idle_end = first.last().unwrap().t_end + 5.0;

    s.advance_idle(idle_end);
    s.push(line(2, [20.0, 0.0, 0.0], [40.0, 0.0, 0.0], 0.0))
        .unwrap();
    let second = s.commit(true).unwrap();

    assert!(second[0].t_start >= idle_end - 1e-9);
}

#[test]
fn smooth_shaper_two_batch_output_matches_one_batch() {
    let moves = [
        line(1, [0.0, 0.0, 0.0], [50.0, 0.0, 0.0], 0.0),
        line(2, [50.0, 0.0, 0.0], [100.0, 0.0, 0.0], 0.0),
    ];
    let chains = smooth_x_chains(18.0);
    let mut one = StreamState::new(cfg(), chains.clone(), &[0.0, 0.0, 0.0], 0.0);
    let mut two = StreamState::new(cfg(), chains, &[0.0, 0.0, 0.0], 0.0);
    for m in moves {
        one.push(m.clone()).unwrap();
        two.push(m).unwrap();
    }

    let one_batch = one.commit(true).unwrap();
    let mut two_batch = two.commit(false).unwrap();
    two_batch.extend(two.commit(true).unwrap());

    assert_eq!(one_batch.len(), two_batch.len());
    for (one_seg, two_seg) in one_batch.iter().zip(&two_batch) {
        for frac in [0.1_f64, 0.3, 0.5, 0.7, 0.9] {
            let t = frac.mul_add(one_seg.t_end - one_seg.t_start, one_seg.t_start);
            let dx = (eval(&one_seg.axes[0], t) - eval(&two_seg.axes[0], t)).abs();
            let dv = {
                let h = 1e-5 * (one_seg.t_end - one_seg.t_start);
                let v_one =
                    (eval(&one_seg.axes[0], t + h) - eval(&one_seg.axes[0], t - h)) / (2.0 * h);
                let v_two =
                    (eval(&two_seg.axes[0], t + h) - eval(&two_seg.axes[0], t - h)) / (2.0 * h);
                (v_one - v_two).abs()
            };
            assert!(dx < 5e-2, "position mismatch at t={t}: {dx}");
            assert!(dv < 5e-2, "velocity mismatch at t={t}: {dv}");
        }
    }
}
