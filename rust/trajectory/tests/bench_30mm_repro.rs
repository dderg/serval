//! Reproduction + root-cause of the 4.16 s replan_overrun on the Pi:
//! a single 30mm X move at F1500 (25mm/s) as the FIRST move from idle.
//!
//! Bench evidence (verbatim structured events, two sessions an hour apart):
//!   replan_us=4160141  solve_us=4160064  beta_iters=1  beta_converged=true
//!   window_segments=2  dist_mm=30  feed_mm_s=25  nominal_s=1.2
//!
//! Plan limits (from PlannerConfig::default() in motion-bridge/src/config.rs):
//!   max_velocity=300  max_accel=3000  max_z_velocity=15  max_z_accel=100
//!   square_corner_velocity=5  shaper=smooth_mzv@50Hz
//!
//! `solve_us` is measured around `plan_velocity` in streaming::state::append_and_replan.
//! `beta_iters=1` means the outer beta derate loop converged in one pass.
//! `window_segments=2` means the planner window held 2 segments at replan time.
//!
//! DIAGNOSIS HYPOTHESIS (to be confirmed/denied by running this test):
//! The 4.16s is NOT due to N scaling with duration — N is geometry-based
//! (distance / target_grid_spacing_mm = 30/0.5 = 60, capped at min_n=20..max_n=200).
//! Hypothesis: the second segment (the advance_idle hold or a prior partial move)
//! feeds the multi-segment joining loop; each SLP iteration invokes Clarabel once
//! per segment, and the per-iteration cost at the default smooth_mzv@50Hz
//! (kernel half-support T/2 ≈ 9.5ms) inflates beta_iterate_inner's inner TOPP-RA
//! call. With beta_max_iters=10 and up to 50+30=80 SLP outer iters per segment,
//! the worst-case solve count is enormous.
//!
//! The "375mm homing" case: N = ceil(375/0.5)=750, clamped to max_n=200.
//! N=200 feeds the known Clarabel MaxIter pathology documented in
//! homing_diagnostic.rs (V9/V10 variants fail at n>=600, but n=200 hits
//! a different cost regime).

use std::time::Instant;

use geometry::segment::EMode;
use nurbs::VectorNurbs;
use temporal::multi::{GridStrategy, SegmentInput};
use trajectory::{
    AxisShaper, ELimits, RequiredShaper, ShapeBatchInput, ShapeError, ShapeSegmentInput,
    ShaperConfig,
};

/// Collinear cubic Bézier for a pure-X move of `dist_mm` mm starting from origin.
fn pure_x_collinear_cubic(dist_mm: f64) -> VectorNurbs<f64, 3> {
    VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![
            [0.0, 0.0, 0.0],
            [dist_mm / 3.0, 0.0, 0.0],
            [2.0 * dist_mm / 3.0, 0.0, 0.0],
            [dist_mm, 0.0, 0.0],
        ],
    )
    .unwrap()
}

/// Default bench limits from `PlannerConfig::default()` → `to_temporal_limits()`.
/// max_velocity=300, max_accel=3000, max_z_velocity=15, max_z_accel=100,
/// square_corner_velocity=5.
fn bench_limits() -> temporal::Limits {
    let max_velocity = 300.0_f64;
    let max_accel = 3000.0_f64;
    let max_z_velocity = 15.0_f64;
    let max_z_accel = 100.0_f64;
    let square_corner_velocity = 5.0_f64;
    temporal::Limits::new(
        [max_velocity, max_velocity, max_z_velocity],
        [max_accel, max_accel, max_z_accel],
        [max_accel * 2.0, max_accel * 2.0, max_z_accel * 2.0],
        square_corner_velocity.powi(2) / (max_accel * 0.5),
    )
}

/// Bench shaper from `PlannerConfig::default()`: smooth_mzv@50Hz on X and Y.
fn bench_shaper() -> ShaperConfig {
    ShaperConfig {
        x: RequiredShaper::SmoothMzv { frequency_hz: 50.0 },
        y: RequiredShaper::SmoothMzv { frequency_hz: 50.0 },
        z: AxisShaper::Passthrough,
    }
}

/// Bench grid strategy from `build_replan_context`:
/// Adaptive { min_n: 20, max_n: 200, target_grid_spacing_mm: 0.5 }
fn bench_grid() -> GridStrategy {
    GridStrategy::Adaptive {
        min_n: 20,
        max_n: 200,
        target_grid_spacing_mm: 0.5,
    }
}

fn shape_single(dist_mm: f64, feedrate_mm_s: f64, initial_v: f64, terminal_v: f64) -> (f64, f64) {
    let curve = pure_x_collinear_cubic(dist_mm);
    let segments = [ShapeSegmentInput {
        temporal: SegmentInput {
            curve: &curve,
            limits: bench_limits(),
            trailing_junction_chord_tolerance_mm: 0.05,
        },
        e_mode: EMode::Travel,
        extrusion_per_xy_mm: 0.0,
        e_independent: None,
        feedrate_mm_s,
    }];
    let input = ShapeBatchInput {
        segments: &segments,
        grid_strategy: bench_grid(),
        worker_threads: 1,
        shaper: bench_shaper(),
        fit_tolerance_mm: 0.005,
        beta_max_iters: 10,
        beta_convergence_ratio: 0.05,
        e_limits: ELimits {
            v_max: 50.0,
            a_max: 5000.0,
        },
        initial_v,
        terminal_v,
    };
    let t0 = Instant::now();
    let result = trajectory::shape_batch(&input).expect("shape_batch failed");
    let elapsed = t0.elapsed().as_secs_f64();
    let traj_dur = result.segments.first().map_or(f64::NAN, |s| s.t_end - s.t_start);
    (elapsed, traj_dur)
}

fn shape_two_segments(
    dist1_mm: f64,
    feed1: f64,
    dist2_mm: f64,
    feed2: f64,
) -> (f64, f64) {
    let curve1 = pure_x_collinear_cubic(dist1_mm);
    let curve2 = pure_x_collinear_cubic(dist2_mm);
    let segments = [
        ShapeSegmentInput {
            temporal: SegmentInput {
                curve: &curve1,
                limits: bench_limits(),
                trailing_junction_chord_tolerance_mm: 0.05,
            },
            e_mode: EMode::Travel,
            extrusion_per_xy_mm: 0.0,
            e_independent: None,
            feedrate_mm_s: feed1,
        },
        ShapeSegmentInput {
            temporal: SegmentInput {
                curve: &curve2,
                limits: bench_limits(),
                trailing_junction_chord_tolerance_mm: 0.05,
            },
            e_mode: EMode::Travel,
            extrusion_per_xy_mm: 0.0,
            e_independent: None,
            feedrate_mm_s: feed2,
        },
    ];
    let input = ShapeBatchInput {
        segments: &segments,
        grid_strategy: bench_grid(),
        worker_threads: 1,
        shaper: bench_shaper(),
        fit_tolerance_mm: 0.005,
        beta_max_iters: 10,
        beta_convergence_ratio: 0.05,
        e_limits: ELimits {
            v_max: 50.0,
            a_max: 5000.0,
        },
        initial_v: 0.0,
        terminal_v: 0.0,
    };
    let t0 = Instant::now();
    let result = trajectory::shape_batch(&input).expect("shape_batch (2-seg) failed");
    let elapsed = t0.elapsed().as_secs_f64();
    let traj_dur: f64 = result.segments.iter().map(|s| s.t_end - s.t_start).sum();
    (elapsed, traj_dur)
}

fn shape_topp_only(dist_mm: f64, feedrate_mm_s: f64, n: usize) -> (f64, f64) {
    let curve = pure_x_collinear_cubic(dist_mm);
    // Apply feedrate cap to per-segment limits (mirrors per_segment_limits in streaming/state.rs).
    // For a pure-X move: span[x]=dist_mm, chord_len=dist_mm, direction_fraction=1.0
    // v_max[x] = min(300, feedrate * 1.0)
    let base = bench_limits();
    let capped_lim = temporal::Limits::new(
        [feedrate_mm_s.min(base.v_max[0]), feedrate_mm_s.min(base.v_max[1]), base.v_max[2]],
        base.a_max,
        base.j_max,
        base.a_centripetal_max,
    );
    let segment = SegmentInput {
        curve: &curve,
        limits: capped_lim,
        trailing_junction_chord_tolerance_mm: 0.05,
    };
    let t0 = Instant::now();
    let result = temporal::multi::plan_batch(temporal::multi::BatchInput {
        segments: &[segment],
        grid_strategy: GridStrategy::Fixed(n),
        worker_threads: 1,
        initial_velocity: 0.0,
        terminal_velocity: 0.0,
    })
    .expect("plan_batch failed");
    let elapsed = t0.elapsed().as_secs_f64();
    let traj_dur = result.profiles[0].total_time;
    (elapsed, traj_dur)
}

/// Core reproduction: 30mm at 25mm/s with bench default limits/shaper,
/// single segment from idle (v_start=0, v_end=0). Reports solve time.
///
/// If Mac shows ~0.4-1s, Pi at 5-10x slowdown = 2-10s, confirming the repro.
#[test]
fn repro_30mm_25mms_bench_defaults_single_segment() {
    // Warm up the Clarabel allocator / JIT paths so measurements are stable.
    let _ = shape_single(30.0, 25.0, 0.0, 0.0);

    let iters = 3;
    let mut times = Vec::with_capacity(iters);
    for _ in 0..iters {
        let (elapsed, traj_dur) = shape_single(30.0, 25.0, 0.0, 0.0);
        times.push(elapsed);
        let _ = traj_dur;
    }
    times.sort_by(f64::total_cmp);
    let p50 = times[iters / 2];
    let (_, traj_dur) = shape_single(30.0, 25.0, 0.0, 0.0);

    eprintln!(
        "[repro 30mm@25mm/s single-seg] p50_solve={:.3}s traj_dur={:.3}s \
         Pi5x_est={:.2}s Pi10x_est={:.2}s",
        p50,
        traj_dur,
        p50 * 5.0,
        p50 * 10.0,
    );

    // N for 30mm with adaptive grid 0.5mm spacing: ceil(30/0.5) = 60
    // That's well under max_n=200. The bench Pi showed 4.16s.
    // Mac p50 should be 0.4-1s if this is the same code path.
    assert!(
        p50 < 5.0,
        "solve took {:.3}s — if this is > 5s something is catastrophically wrong \
         beyond the known 4s Pi issue",
        p50,
    );
    eprintln!("[N for 30mm] ceil(30/0.5)=60 grid points");
}

/// Two-segment variant reproducing window_segments=2 from the bench log.
/// The second segment in the bench window is likely a small prior segment
/// (partial tail from a preceding homing move or a dwell anchor).
/// We model it as a 1mm anchor at the same feed, which is the minimal
/// preceding uncommitted move the streaming state could carry.
#[test]
fn repro_30mm_25mms_two_segment_window() {
    let _ = shape_two_segments(1.0, 25.0, 30.0, 25.0); // warm-up

    let iters = 3;
    let mut times = Vec::with_capacity(iters);
    for _ in 0..iters {
        let (elapsed, _) = shape_two_segments(1.0, 25.0, 30.0, 25.0);
        times.push(elapsed);
    }
    times.sort_by(f64::total_cmp);
    let p50 = times[iters / 2];
    let (_, traj_dur) = shape_two_segments(1.0, 25.0, 30.0, 25.0);

    eprintln!(
        "[repro 2-seg 1mm+30mm@25mm/s] p50_solve={:.3}s traj_dur={:.3}s \
         Pi5x_est={:.2}s Pi10x_est={:.2}s",
        p50,
        traj_dur,
        p50 * 5.0,
        p50 * 10.0,
    );

    assert!(
        p50 < 5.0,
        "2-seg solve took {:.3}s",
        p50,
    );
}

/// N-sweep to find the cost inflection point.
/// bench_limits with feedrate cap: v_max_x = min(300, 25) = 25mm/s.
/// For a 30mm move at 25mm/s: optimal time ~30/25=1.2s nominal.
/// With accel: v_max=25, a_max=3000 -> t_accel = v/a = 0.0083s (tiny!), cruise ~1.19s.
/// j_max=6000: jerk-limited accel time = v/a but trapezoidal, still ~1.2s.
/// So jerk barely matters here - this is a near-constant-velocity segment.
/// The SOCP should be nearly trivial (all b_i ≈ 25^2 = 625).
#[test]
fn topp_only_n_sweep_30mm_25mms() {
    eprintln!("[N sweep, 30mm @ 25mm/s, bench limits]");
    for n in [20, 30, 40, 60, 80, 100, 150, 200] {
        let _ = shape_topp_only(30.0, 25.0, n); // warm-up
        let iters = 3;
        let mut ts = Vec::with_capacity(iters);
        for _ in 0..iters {
            let (elapsed, _) = shape_topp_only(30.0, 25.0, n);
            ts.push(elapsed);
        }
        ts.sort_by(f64::total_cmp);
        let (_, traj) = shape_topp_only(30.0, 25.0, n);
        eprintln!(
            "  N={:>3}  p50={:.4}s  traj={:.4}s  n_vars={} n_rows~{}",
            n,
            ts[iters / 2],
            traj,
            5 * n - 6,
            11 * n,
        );
    }
}

/// Control: 30mm at full speed (300mm/s). Should be fast even with same N=60.
/// If this is fast but 25mm/s is slow, the issue is specific to low-speed
/// (near-constant-velocity) segments where the SLP jerk loop behaves differently.
#[test]
fn control_30mm_300mms_vs_25mms() {
    let _ = shape_single(30.0, 300.0, 0.0, 0.0);
    let _ = shape_single(30.0, 25.0, 0.0, 0.0);

    let (t300, d300) = shape_single(30.0, 300.0, 0.0, 0.0);
    let (t25, d25) = shape_single(30.0, 25.0, 0.0, 0.0);

    eprintln!(
        "[control] 30mm@300mm/s: {:.4}s (traj {:.4}s)  30mm@25mm/s: {:.4}s (traj {:.4}s)  ratio={:.1}x",
        t300, d300, t25, d25, t25 / t300,
    );
    // Both should be sub-second on Mac. If 25mm/s is much slower, the slow
    // path is specific to low-feed segments (v_max cap -> near-constant profile).
}

/// Homing-shaped move: 375mm at 10mm/s (typical first-move homing speed).
/// N = ceil(375/0.5) = 750, clamped to max_n=200.
/// At max_n=200, uses the same cost regime as the documented V9/V10
/// Clarabel-MaxIter pathology (but feedrate cap v_max_x=10mm/s changes things).
#[test]
fn homing_shaped_375mm_10mms() {
    let _ = shape_single(375.0, 10.0, 0.0, 0.0); // warm-up

    let iters = 3;
    let mut times = Vec::with_capacity(iters);
    for _ in 0..iters {
        let (elapsed, _) = shape_single(375.0, 10.0, 0.0, 0.0);
        times.push(elapsed);
    }
    times.sort_by(f64::total_cmp);
    let p50 = times[iters / 2];
    let (_, traj_dur) = shape_single(375.0, 10.0, 0.0, 0.0);

    eprintln!(
        "[homing 375mm@10mm/s] p50_solve={:.3}s traj_dur={:.3}s \
         Pi5x_est={:.2}s Pi10x_est={:.2}s (N=min(750,200)=200)",
        p50,
        traj_dur,
        p50 * 5.0,
        p50 * 10.0,
    );
    assert!(
        p50 < 30.0,
        "homing solve took {:.3}s — catastrophically slow",
        p50,
    );
}

/// High-speed prior segment + low-speed new segment: the realistic bench scenario
/// where a fast uncommitted move (e.g., a retract or prior print move) is still in
/// the window when the slow 30mm@25mm/s move arrives.
///
/// This is the closest reproduction of `window_segments=2` with mismatched feedrates.
/// The per-axis SLP Stage 2 must reconcile the velocity mismatch at the junction,
/// which is harder than the same-feedrate case measured above.
#[test]
fn repro_mismatched_feedrate_2seg_window() {
    // First segment: a prior fast move still uncommitted in the planner window.
    // 150mm at 200mm/s models an ongoing print or retract move.
    let _ = shape_two_segments(150.0, 200.0, 30.0, 25.0); // warm-up

    let iters = 3;
    let mut times = Vec::with_capacity(iters);
    for _ in 0..iters {
        let (elapsed, _) = shape_two_segments(150.0, 200.0, 30.0, 25.0);
        times.push(elapsed);
    }
    times.sort_by(f64::total_cmp);
    let p50 = times[iters / 2];
    let (_, traj_dur) = shape_two_segments(150.0, 200.0, 30.0, 25.0);

    eprintln!(
        "[repro mismatched 150mm@200+30mm@25mm/s] p50_solve={:.3}s traj_dur={:.3}s \
         Pi5x_est={:.2}s Pi10x_est={:.2}s",
        p50,
        traj_dur,
        p50 * 5.0,
        p50 * 10.0,
    );

    // If this approaches or exceeds the 250ms real-time budget on Mac, the
    // per-axis SLP convergence is the root cause of the 4.16s Pi incident.
    assert!(
        p50 < 10.0,
        "mismatched-feedrate 2-seg solve took {:.3}s — pathological SLP convergence",
        p50,
    );
}

/// Multi-segment print window control: 4 consecutive collinear 15mm X segments
/// at 200mm/s, approximating a typical print window. All segments are pure Travel
/// so partition groups them into one XY run with no E gaps. MUST NOT regress.
#[test]
fn multi_segment_print_window_baseline() {
    let make_seg = |x0: f64, x1: f64| {
        VectorNurbs::<f64, 3>::try_new(
            3,
            vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
            vec![
                [x0, 0.0, 0.0],
                [x0 + (x1 - x0) / 3.0, 0.0, 0.0],
                [x0 + 2.0 * (x1 - x0) / 3.0, 0.0, 0.0],
                [x1, 0.0, 0.0],
            ],
        )
        .unwrap()
    };

    let c1 = make_seg(0.0, 15.0);
    let c2 = make_seg(15.0, 30.0);
    let c3 = make_seg(30.0, 45.0);
    let c4 = make_seg(45.0, 60.0);

    let segs = [
        ShapeSegmentInput {
            temporal: SegmentInput {
                curve: &c1,
                limits: bench_limits(),
                trailing_junction_chord_tolerance_mm: 0.05,
            },
            e_mode: EMode::Travel,
            extrusion_per_xy_mm: 0.0,
            e_independent: None,
            feedrate_mm_s: 200.0,
        },
        ShapeSegmentInput {
            temporal: SegmentInput {
                curve: &c2,
                limits: bench_limits(),
                trailing_junction_chord_tolerance_mm: 0.05,
            },
            e_mode: EMode::Travel,
            extrusion_per_xy_mm: 0.0,
            e_independent: None,
            feedrate_mm_s: 200.0,
        },
        ShapeSegmentInput {
            temporal: SegmentInput {
                curve: &c3,
                limits: bench_limits(),
                trailing_junction_chord_tolerance_mm: 0.05,
            },
            e_mode: EMode::Travel,
            extrusion_per_xy_mm: 0.0,
            e_independent: None,
            feedrate_mm_s: 200.0,
        },
        ShapeSegmentInput {
            temporal: SegmentInput {
                curve: &c4,
                limits: bench_limits(),
                trailing_junction_chord_tolerance_mm: 0.05,
            },
            e_mode: EMode::Travel,
            extrusion_per_xy_mm: 0.0,
            e_independent: None,
            feedrate_mm_s: 200.0,
        },
    ];

    let input = ShapeBatchInput {
        segments: &segs,
        grid_strategy: bench_grid(),
        worker_threads: 1,
        shaper: bench_shaper(),
        fit_tolerance_mm: 0.005,
        beta_max_iters: 10,
        beta_convergence_ratio: 0.05,
        e_limits: ELimits { v_max: 50.0, a_max: 5000.0 },
        initial_v: 0.0,
        terminal_v: 0.0,
    };

    let _ = trajectory::shape_batch(&input); // warm-up

    let t0 = Instant::now();
    let result = trajectory::shape_batch(&input).expect("4-seg collinear shape_batch failed");
    let elapsed = t0.elapsed().as_secs_f64();
    let traj_dur: f64 = result.segments.iter().map(|s| s.t_end - s.t_start).sum();
    eprintln!(
        "[4-seg print window] solve={:.4}s traj_dur={:.4}s segs={}",
        elapsed, traj_dur, result.segments.len(),
    );
    assert!(
        result.segments.len() >= 4,
        "expected >=4 shaped segments, got {}",
        result.segments.len()
    );
    assert!(elapsed < 5.0, "4-seg print window solve took {:.3}s", elapsed);
}
