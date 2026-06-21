use geometry::segment::{CubicSegment, SourceRange};
use nurbs::VectorNurbs;
use trajectory::streaming::{ReplanContext, ShaperState};
use trajectory::{AxisChainSet, CompiledChain, PostProcessorType};

fn trident_limits() -> temporal::Limits {
    temporal::Limits::axis_boxes(
        [500.0, 500.0, 10.0],
        [20_000.0, 20_000.0, 200.0],
        [200_000.0, 200_000.0, 1_000.0],
    )
}

fn smooth_chains() -> AxisChainSet {
    AxisChainSet::spatial(
        PostProcessorType::SmoothZv { frequency_hz: 60.0 }.into_chain(),
        PostProcessorType::SmoothZv { frequency_hz: 60.0 }.into_chain(),
        CompiledChain::default(),
    )
}

fn replan_ctx() -> ReplanContext {
    ReplanContext {
        limits: trident_limits(),
        chains: smooth_chains(),
        fit_tolerance_mm: 0.005,
        beta_max_iters: 5,
        beta_convergence_ratio: 1.02,
        worker_threads: 1,
        grid_strategy: temporal::multi::GridStrategy::Fixed(20),
        fallback_initial_v: 0.0,
        safety_mode: trajectory::SafetyMode::WorstCaseFuture,
        force_full_resolve: false,
    }
}

fn baseline_full_ctx() -> ReplanContext {
    let mut ctx = replan_ctx();
    ctx.force_full_resolve = true;
    ctx
}

fn cubic_from_cps(cps: [[f64; 2]; 4], feedrate: f64) -> CubicSegment {
    let cp3 = |p: [f64; 2]| [p[0], p[1], 0.0];
    let control = vec![cp3(cps[0]), cp3(cps[1]), cp3(cps[2]), cp3(cps[3])];
    let xyz =
        VectorNurbs::<f64, 3>::try_new(3, vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0], control)
            .unwrap();
    CubicSegment::try_new(
        xyz,
        vec![],
        feedrate,
        SourceRange {
            start_line: 0,
            end_line: 0,
        },
        None,
    )
    .unwrap()
}

fn chained_curves(
    k: usize,
    dx_mm: f64,
    amp_mm: f64,
    feedrate: f64,
) -> (Vec<CubicSegment>, Vec<f64>) {
    let wavelength = 4.0 * dx_mm;
    let two_pi = std::f64::consts::TAU;
    let y = |x: f64| amp_mm * (two_pi * x / wavelength).sin();
    let dy = |x: f64| amp_mm * (two_pi / wavelength) * (two_pi * x / wavelength).cos();

    let mut segments = Vec::with_capacity(k);
    let mut chords = Vec::with_capacity(k);
    for i in 0..k {
        let x0 = i as f64 * dx_mm;
        let x1 = (i + 1) as f64 * dx_mm;
        let p0 = [x0, y(x0)];
        let p3 = [x1, y(x1)];
        let h = (x1 - x0) / 3.0;
        let p1 = [x0 + h, y(x0) + h * dy(x0)];
        let p2 = [x1 - h, y(x1) - h * dy(x1)];
        let seg = cubic_from_cps([p0, p1, p2, p3], feedrate);
        let chord = ((p3[0] - p0[0]).powi(2) + (p3[1] - p0[1]).powi(2)).sqrt();
        segments.push(seg);
        chords.push(chord);
    }
    (segments, chords)
}

fn decel_to_corner_chain(
    run_up_segments: usize,
    leg_mm: f64,
    feedrate: f64,
) -> (Vec<CubicSegment>, Vec<f64>) {
    let mut segments = Vec::with_capacity(run_up_segments + 1);
    let mut chords = Vec::with_capacity(run_up_segments + 1);
    let h = leg_mm / 3.0;
    for i in 0..run_up_segments {
        let x0 = i as f64 * leg_mm;
        let x1 = (i + 1) as f64 * leg_mm;
        let p0 = [x0, 0.0];
        let p3 = [x1, 0.0];
        let p1 = [x0 + h, 0.0];
        let p2 = [x1 - h, 0.0];
        segments.push(cubic_from_cps([p0, p1, p2, p3], feedrate));
        chords.push(leg_mm);
    }
    let x_corner = run_up_segments as f64 * leg_mm;
    let p0 = [x_corner, 0.0];
    let p3 = [x_corner, leg_mm];
    let p1 = [x_corner, h];
    let p2 = [x_corner, leg_mm - h];
    segments.push(cubic_from_cps([p0, p1, p2, p3], feedrate));
    chords.push(leg_mm);
    (segments, chords)
}

#[derive(Debug, Clone, Copy)]
struct WorkCounts {
    window_segments: usize,
    grid_points: u64,
    chains_scheduled: u32,
    clarabel_total: u32,
    clarabel_path_jerk: u32,
    clarabel_slp9_tr: u32,
    clarabel_slp9_no_tr: u32,
    beta_iterations: u8,
    solve_us: u64,
}

fn append_counted(state: &mut ShaperState, ctx: &ReplanContext, seg: CubicSegment) -> WorkCounts {
    temporal::counters::reset();
    let report = state
        .append_and_replan(seg, ctx)
        .expect("append should plan");
    let c = temporal::counters::snapshot_global();
    WorkCounts {
        window_segments: report.window_segments,
        grid_points: c.grid_points_scheduled,
        chains_scheduled: c.chains_scheduled,
        clarabel_total: c.clarabel_calls_total,
        clarabel_path_jerk: c.clarabel_calls_path_jerk,
        clarabel_slp9_tr: c.clarabel_calls_slp9_tr,
        clarabel_slp9_no_tr: c.clarabel_calls_slp9_no_tr,
        beta_iterations: report.plan.beta_iterations,
        solve_us: report.solve_us,
    }
}

#[test]
fn continuous_throughput_window_depth_scaling() {
    const K: usize = 16;
    const DX_MM: f64 = 12.0;
    const AMP_MM: f64 = 6.0;
    const FEEDRATE: f64 = 200.0;

    let ctx = baseline_full_ctx();
    let mut state = ShaperState::new(&[0.0; 3], &ctx.chains);
    let (curves, _chords) = chained_curves(K, DX_MM, AMP_MM, FEEDRATE);

    println!();
    println!("=== Measurement 1: per-append WORK COUNTS vs LIVE WINDOW DEPTH (no drain) ===");
    println!(
        "geometry: {K} chained cubic Béziers, dx={DX_MM}mm amp={AMP_MM}mm, feedrate={FEEDRATE}mm/s"
    );
    println!("limits: trident-like CoreXY (vx=vy=500 ax=ay=20000 jx=jy=200000), Z slow");
    println!("post-proc: SmoothZV 60Hz on X/Y; grid = Fixed(20) nodes/segment");
    println!("COUNTS are deterministic / load-independent. solve_us is rough, NON-GATING.");
    println!();
    println!(
        "{:>6} | {:>11} | {:>11} | {:>9} | {:>13} | {:>9} | {:>9} | {:>9}",
        "append", "win_segs", "grid_pts", "chains", "clarabel_tot", "beta_it", "gp/seg", "solve_us",
    );
    println!("{}", "-".repeat(96));

    let mut rows: Vec<WorkCounts> = Vec::with_capacity(K);
    let mut cum_grid: u64 = 0;
    let mut cum_clarabel: u64 = 0;
    for (i, seg) in curves.into_iter().enumerate() {
        let w = append_counted(&mut state, &ctx, seg);
        cum_grid += w.grid_points;
        cum_clarabel += u64::from(w.clarabel_total);
        println!(
            "{:>6} | {:>11} | {:>11} | {:>9} | {:>13} | {:>9} | {:>9.1} | {:>9}",
            i + 1,
            w.window_segments,
            w.grid_points,
            w.chains_scheduled,
            w.clarabel_total,
            w.beta_iterations,
            w.grid_points as f64 / w.window_segments as f64,
            w.solve_us,
        );
        rows.push(w);
    }
    println!("{}", "-".repeat(96));

    let n = rows.len();
    const FIRST_REAL_WINDOW_APPEND: usize = 1;
    let first = rows[FIRST_REAL_WINDOW_APPEND];
    let last = rows[n - 1];
    let d_depth = (last.window_segments - first.window_segments) as f64;
    let grid_slope = (last.grid_points as f64 - first.grid_points as f64) / d_depth;
    let clarabel_slope =
        (f64::from(last.clarabel_total) - f64::from(first.clarabel_total)) / d_depth;

    println!(
        "first real append (depth {}): grid_pts={} clarabel_tot={}",
        first.window_segments, first.grid_points, first.clarabel_total
    );
    println!(
        "last  append      (depth {}): grid_pts={} clarabel_tot={}",
        last.window_segments, last.grid_points, last.clarabel_total
    );
    println!("marginal grid points per added window segment : {grid_slope:.1} grid_pts/seg");
    println!("marginal Clarabel solves per added window seg  : {clarabel_slope:.2} solves/seg");
    println!("cumulative Σgrid_points over {K} appends = {cum_grid} ; Σclarabel = {cum_clarabel}");
    println!(
        "BASELINE CONCLUSION (counts): per-append work grows ~LINEARLY with window depth \
         (grid_pts ≈ {:.0}/seg, clarabel ≈ {:.1}/seg) => cumulative work over a chain is \
         ~QUADRATIC in #curves.",
        grid_slope, clarabel_slope,
    );
    println!();

    assert!(
        last.grid_points > first.grid_points,
        "expected grid points to grow with window depth (last {} > first {})",
        last.grid_points,
        first.grid_points,
    );
    assert!(
        last.clarabel_total >= first.clarabel_total,
        "expected Clarabel solves to grow with window depth",
    );
    let approx_per_seg = last.grid_points as f64 / last.window_segments as f64;
    assert!(
        approx_per_seg > 1.0,
        "each window segment contributes grid nodes; got {approx_per_seg}",
    );
}

#[test]
fn continuous_throughput_steady_state_depth_with_drain() {
    const DX_MM: f64 = 12.0;
    const AMP_MM: f64 = 6.0;
    const K: usize = 18;

    let buffers_s = [0.25_f64, 0.5, 1.0];
    let feedrates = [50.0_f64, 100.0, 200.0, 350.0];

    println!();
    println!("=== Measurement 2: STEADY-STATE WINDOW DEPTH per feedrate (REALISTIC DRAIN) ===");
    println!(
        "geometry: {K} chained cubic Béziers, dx={DX_MM}mm amp={AMP_MM}mm; grid = Fixed(20)/seg"
    );
    println!(
        "drain model: playback clock at feed; host keeps buffer_s of motion queued; \
         t_dispatched advances => committed moves leave the window."
    );
    println!("All quantities below are deterministic / load-independent COUNTS.");
    println!();
    println!(
        "{:>9} | {:>9} | {:>14} | {:>14} | {:>16} | {:>16}",
        "buffer_s", "feed", "steady_depth", "ss_grid_pts", "ss_clarabel_tot", "ss_gp_per_seg",
    );
    println!("{}", "-".repeat(92));

    for &buffer_s in &buffers_s {
        for &feed in &feedrates {
            let ctx = baseline_full_ctx();
            let mut state = ShaperState::new(&[0.0; 3], &ctx.chains);
            let (curves, _chords) = chained_curves(K, DX_MM, AMP_MM, feed);

            let mut steady_depths: Vec<usize> = Vec::new();
            let mut steady_work: Vec<WorkCounts> = Vec::new();

            for (i, seg) in curves.into_iter().enumerate() {
                let playback_t = (state.t_appended - buffer_s).max(0.0);
                if playback_t > state.t_dispatched {
                    state.t_dispatched = playback_t.min(state.t_appended);
                }

                let w = append_counted(&mut state, &ctx, seg);

                let in_steady_state = i >= K / 2;
                if in_steady_state {
                    steady_depths.push(w.window_segments);
                    steady_work.push(w);
                }
            }

            let ss_depth_avg =
                steady_depths.iter().sum::<usize>() as f64 / steady_depths.len() as f64;
            let ss_grid_avg = steady_work.iter().map(|w| w.grid_points).sum::<u64>() as f64
                / steady_work.len() as f64;
            let ss_clarabel_avg = steady_work
                .iter()
                .map(|w| u64::from(w.clarabel_total))
                .sum::<u64>() as f64
                / steady_work.len() as f64;

            println!(
                "{:>9.2} | {:>9.0} | {:>14.1} | {:>14.1} | {:>16.1} | {:>16.1}",
                buffer_s,
                feed,
                ss_depth_avg,
                ss_grid_avg,
                ss_clarabel_avg,
                ss_grid_avg / ss_depth_avg.max(1.0),
            );
        }
        println!("{}", "-".repeat(92));
    }

    println!(
        "interpretation: steady_depth is bounded by the host buffer, NOT unbounded — but it \
         grows with feedrate (more short curves fit in buffer_s). Per-append work tracks \
         steady_depth, so the whole-window re-solve pays O(steady_depth) every append even \
         though only the tail is mutable."
    );
    println!();
}

#[test]
fn continuous_throughput_keep_ahead_counts() {
    const K: usize = 12;
    const DX_MM: f64 = 12.0;
    const AMP_MM: f64 = 6.0;

    let feedrates = [50.0_f64, 100.0, 200.0, 350.0, 500.0];

    println!();
    println!("=== Measurement 3: KEEP-AHEAD in COUNT terms (no-drain worst case) ===");
    println!(
        "geometry: {K} chained cubic Béziers, dx={DX_MM}mm amp={AMP_MM}mm (window grows to {K})"
    );
    println!("Σgrid_points = total deterministic solver work; Σplayback = execution time.");
    println!();
    println!(
        "{:>10} | {:>13} | {:>14} | {:>15} | {:>16}",
        "feed mm/s", "Σgrid_points", "Σclarabel_tot", "Σplayback_us", "grid_pts_per_play_ms",
    );
    println!("{}", "-".repeat(80));

    for &feed in &feedrates {
        let ctx = baseline_full_ctx();
        let mut state = ShaperState::new(&[0.0; 3], &ctx.chains);
        let (curves, chords) = chained_curves(K, DX_MM, AMP_MM, feed);

        let mut sum_grid: u64 = 0;
        let mut sum_clarabel: u64 = 0;
        for seg in curves {
            let w = append_counted(&mut state, &ctx, seg);
            sum_grid += w.grid_points;
            sum_clarabel += u64::from(w.clarabel_total);
        }

        let sum_playback_s: f64 = chords.iter().map(|c| c / feed).sum();
        let sum_playback_us = sum_playback_s * 1.0e6;
        let grid_per_play_ms = sum_grid as f64 / (sum_playback_us / 1000.0);

        println!(
            "{:>10.0} | {:>13} | {:>14} | {:>15.0} | {:>16.2}",
            feed, sum_grid, sum_clarabel, sum_playback_us, grid_per_play_ms,
        );
    }
    println!("{}", "-".repeat(80));
    println!(
        "interpretation: faster feed => shorter Σplayback => MORE solver work per millisecond \
         of real-time. The no-drain Σgrid is the same chain re-solved cumulatively (quadratic), \
         so the keep-ahead margin shrinks fastest exactly where throughput matters most."
    );
    println!();
}

#[test]
fn continuous_throughput_decel_to_corner_counts() {
    const RUN_UP: usize = 12;
    const LEG_MM: f64 = 10.0;
    const FEEDRATE: f64 = 400.0;

    let ctx = baseline_full_ctx();
    let mut state = ShaperState::new(&[0.0; 3], &ctx.chains);
    let (curves, _chords) = decel_to_corner_chain(RUN_UP, LEG_MM, FEEDRATE);
    let total = curves.len();

    println!();
    println!("=== Measurement 4: DECEL-TO-CORNER backward horizon (no drain) ===");
    println!(
        "geometry: {RUN_UP} straight {LEG_MM}mm run-up segments + 1 hard 90° corner leg, \
         feed={FEEDRATE}mm/s"
    );
    println!("The corner forces backward decel propagation — the deepest front-freeze horizon.");
    println!();
    println!(
        "{:>6} | {:>11} | {:>11} | {:>9} | {:>13} | {:>9}",
        "append", "win_segs", "grid_pts", "chains", "clarabel_tot", "beta_it",
    );
    println!("{}", "-".repeat(74));

    for (i, seg) in curves.into_iter().enumerate() {
        let w = append_counted(&mut state, &ctx, seg);
        let tag = if i + 1 == total { "  <- corner" } else { "" };
        println!(
            "{:>6} | {:>11} | {:>11} | {:>9} | {:>13} | {:>9}{}",
            i + 1,
            w.window_segments,
            w.grid_points,
            w.chains_scheduled,
            w.clarabel_total,
            w.beta_iterations,
            tag,
        );
    }
    println!("{}", "-".repeat(74));
    println!(
        "interpretation: the corner append re-solves the entire run-up because decel from the \
         corner reaches backward across it. Any front-freeze sub-window cap K must be >= this \
         backward reach to stay trajectory-neutral; this chain is the stress case for choosing K."
    );
    println!();
}

#[test]
fn continuous_throughput_isolated_walltime_estimate() {
    const DEPTH: usize = 8;
    const DX_MM: f64 = 12.0;
    const AMP_MM: f64 = 6.0;
    const FEEDRATE: f64 = 200.0;
    const REPS: usize = 7;

    println!();
    println!("=== Measurement 5: ISOLATED wall-time estimate (ROUGH, NON-GATING) ===");
    println!(
        "median of {REPS} reps of the final (depth-{DEPTH}) append solve_us; \
         single process. NOT a gate — wall-clock is load-dependent on this host."
    );

    let mut depth_solve_us: Vec<u64> = Vec::with_capacity(REPS);
    let mut depth_grid: u64 = 0;
    for _ in 0..REPS {
        let ctx = baseline_full_ctx();
        let mut state = ShaperState::new(&[0.0; 3], &ctx.chains);
        let (curves, _chords) = chained_curves(DEPTH, DX_MM, AMP_MM, FEEDRATE);
        let mut last = WorkCounts {
            window_segments: 0,
            grid_points: 0,
            chains_scheduled: 0,
            clarabel_total: 0,
            clarabel_path_jerk: 0,
            clarabel_slp9_tr: 0,
            clarabel_slp9_no_tr: 0,
            beta_iterations: 0,
            solve_us: 0,
        };
        for seg in curves {
            last = append_counted(&mut state, &ctx, seg);
        }
        depth_solve_us.push(last.solve_us);
        depth_grid = last.grid_points;
    }
    depth_solve_us.sort_unstable();
    let median = depth_solve_us[REPS / 2];
    println!(
        "depth {DEPTH}: grid_points={depth_grid} (deterministic), \
         median solve_us={median} (ROUGH estimate, non-gating), samples={depth_solve_us:?}"
    );
    println!();
}

fn sample_committed(state: &ShaperState, t_end: f64, n: usize) -> Vec<[(f64, f64); 2]> {
    let mut rows = Vec::with_capacity(n + 1);
    for i in 0..=n {
        let t = t_end * (i as f64) / (n as f64);
        let x = state.sample_axis(0, t);
        let y = state.sample_axis(1, t);
        if let (Some(x), Some(y)) = (x, y) {
            rows.push([x, y]);
        }
    }
    rows
}

fn run_chain_capture(
    curves: &[CubicSegment],
    feed: f64,
    chords: &[f64],
    buffer_s: f64,
    force_full: bool,
) -> (Vec<Vec<[(f64, f64); 2]>>, Vec<usize>, ShaperState) {
    let mut ctx = replan_ctx();
    ctx.force_full_resolve = force_full;
    let mut state = ShaperState::new(&[0.0; 3], &ctx.chains);

    let mut committed_snaps = Vec::with_capacity(curves.len());
    let mut win_segs = Vec::with_capacity(curves.len());
    let _ = chords;

    for seg in curves {
        let buffer_trailing_time = (state.t_appended - buffer_s).max(0.0);
        let snapped_to_segment_boundary = state
            .uncommitted_moves
            .iter()
            .map(|m| m.t_start)
            .filter(|&t| t <= buffer_trailing_time)
            .fold(0.0_f64, f64::max);
        if snapped_to_segment_boundary > state.t_dispatched {
            state.t_dispatched = snapped_to_segment_boundary.min(state.t_appended);
        }
        let report = state
            .append_and_replan(seg.clone(), &ctx)
            .expect("append should plan");
        win_segs.push(report.window_segments);
        let committed_region_end = state.t_dispatched.max(0.0);
        committed_snaps.push(sample_committed(&state, committed_region_end, 200));
    }
    let _ = feed;
    (committed_snaps, win_segs, state)
}

fn snap_diff(a: &[Vec<[(f64, f64); 2]>], b: &[Vec<[(f64, f64); 2]>]) -> (f64, f64) {
    let mut dp = 0.0_f64;
    let mut dv = 0.0_f64;
    let mut dv_at = (0usize, 0usize, 0usize);
    assert_eq!(a.len(), b.len(), "snapshot count mismatch");
    for (ai, (ra, rb)) in a.iter().zip(b.iter()).enumerate() {
        let m = ra.len().min(rb.len());
        for k in 0..m {
            for ax in 0..2 {
                dp = dp.max((ra[k][ax].0 - rb[k][ax].0).abs());
                let d = (ra[k][ax].1 - rb[k][ax].1).abs();
                if d > dv {
                    dv = d;
                    dv_at = (ai, k, ax);
                }
            }
        }
    }
    if dv > 0.01 {
        println!(
            "  max Δvel at append#{} sample#{}/{} axis{}: full={:.6} bnd={:.6}",
            dv_at.0 + 1,
            dv_at.1,
            a[dv_at.0].len(),
            dv_at.2,
            a[dv_at.0][dv_at.1][dv_at.2].1,
            b[dv_at.0][dv_at.1][dv_at.2].1,
        );
    }
    (dp, dv)
}

const POS_TOL_MM: f64 = 0.005;
const VEL_TOL_MM_S: f64 = 0.5;

#[test]
fn bounded_equals_full_smooth_chain() {
    const K: usize = 24;
    const DX_MM: f64 = 12.0;
    const AMP_MM: f64 = 6.0;
    const FEEDRATE: f64 = 200.0;
    const BUFFER_S: f64 = 0.25;

    let (curves, chords) = chained_curves(K, DX_MM, AMP_MM, FEEDRATE);

    let (full_snaps, full_win, full_state) =
        run_chain_capture(&curves, FEEDRATE, &chords, BUFFER_S, true);
    let (bnd_snaps, bnd_win, bnd_state) =
        run_chain_capture(&curves, FEEDRATE, &chords, BUFFER_S, false);

    let (dp, dv) = snap_diff(&full_snaps, &bnd_snaps);

    println!();
    println!("=== bounded_equals_full_smooth_chain (trajectory-neutrality) ===");
    println!("max |Δposition| over committed region = {dp:.3e} mm (tol {POS_TOL_MM})");
    println!("max |Δvelocity| over committed region = {dv:.3e} mm/s (tol {VEL_TOL_MM_S})");
    println!(
        "t_appended full={:.9} bounded={:.9}",
        full_state.t_appended, bnd_state.t_appended
    );
    println!(
        "t_decel_start full={:.9} bounded={:.9}",
        full_state.t_decel_start, bnd_state.t_decel_start
    );
    let full_max = *full_win.iter().max().unwrap();
    let bnd_max = *bnd_win.iter().max().unwrap();
    println!("window_segments per append: full(max {full_max})={full_win:?}");
    println!("window_segments per append: bounded(max {bnd_max})={bnd_win:?}");
    println!();

    assert!(
        dp < POS_TOL_MM,
        "bounded re-solve drifted committed position by {dp} mm (> {POS_TOL_MM}) — K too small or boundary pin wrong"
    );
    assert!(
        dv < VEL_TOL_MM_S,
        "bounded re-solve drifted committed velocity by {dv} mm/s (> {VEL_TOL_MM_S})"
    );
    const TAIL_FRONTIER_TOL_S: f64 = 2.0e-3;
    assert!(
        (full_state.t_appended - bnd_state.t_appended).abs() < TAIL_FRONTIER_TOL_S,
        "t_appended tail-frontier drift exceeds solver noise: full {} vs bounded {}",
        full_state.t_appended,
        bnd_state.t_appended,
    );
    assert!(
        bnd_max <= full_max,
        "bounded window_segments must never exceed full"
    );
}

#[test]
fn bounded_equals_full_decel_to_corner() {
    const RUN_UP: usize = 16;
    const LEG_MM: f64 = 10.0;
    const FEEDRATE: f64 = 400.0;
    const BUFFER_S: f64 = 0.5;

    let (curves, chords) = decel_to_corner_chain(RUN_UP, LEG_MM, FEEDRATE);

    let (full_snaps, full_win, full_state) =
        run_chain_capture(&curves, FEEDRATE, &chords, BUFFER_S, true);
    let (bnd_snaps, bnd_win, bnd_state) =
        run_chain_capture(&curves, FEEDRATE, &chords, BUFFER_S, false);

    let (dp, dv) = snap_diff(&full_snaps, &bnd_snaps);

    println!();
    println!("=== bounded_equals_full_decel_to_corner (largest backward horizon) ===");
    println!("max |Δposition| over committed region = {dp:.3e} mm (tol {POS_TOL_MM})");
    println!("max |Δvelocity| over committed region = {dv:.3e} mm/s (tol {VEL_TOL_MM_S})");
    println!(
        "t_appended full={:.9} bounded={:.9}",
        full_state.t_appended, bnd_state.t_appended
    );
    println!("window_segments per append: full   ={full_win:?}");
    println!("window_segments per append: bounded ={bnd_win:?}");
    println!(
        "corner append (last) window_segments: full={} bounded={}",
        full_win.last().unwrap(),
        bnd_win.last().unwrap()
    );
    println!();

    assert!(
        dp < POS_TOL_MM,
        "decel-to-corner bounded drifted committed position by {dp} mm (> {POS_TOL_MM})"
    );
    assert!(
        dv < VEL_TOL_MM_S,
        "decel-to-corner bounded drifted committed velocity by {dv} mm/s (> {VEL_TOL_MM_S})"
    );
    const TAIL_FRONTIER_TOL_S: f64 = 2.0e-3;
    assert!(
        (full_state.t_appended - bnd_state.t_appended).abs() < TAIL_FRONTIER_TOL_S,
        "t_appended tail-frontier drift on decel-to-corner exceeds solver noise: full {} vs bounded {}",
        full_state.t_appended,
        bnd_state.t_appended,
    );
    let bnd_corner = *bnd_win.last().unwrap();
    let bnd_steady = bnd_win[..bnd_win.len() - 1]
        .iter()
        .copied()
        .max()
        .unwrap_or(0);
    println!(
        "adaptive-K: bounded corner-append window={bnd_corner} >= pre-corner steady max={bnd_steady}"
    );
    assert!(
        bnd_corner >= bnd_steady,
        "corner append must not shrink the bounded window below the run-up reach"
    );
}

#[test]
fn bounded_work_is_capped_vs_depth() {
    const K: usize = 28;
    const DX_MM: f64 = 12.0;
    const AMP_MM: f64 = 6.0;
    const FEEDRATE: f64 = 200.0;

    println!();
    println!("=== bounded_work_is_capped_vs_depth (no-drain; window grows unbounded) ===");
    println!("Per-append WORK COUNTS, full re-solve vs bounded front-freeze, no drain.");
    println!("DETERMINISTIC counts — identical regardless of CPU load.");
    println!();
    println!(
        "{:>6} | {:>9} {:>9} | {:>10} {:>10} | {:>9} {:>9}",
        "append", "full_seg", "bnd_seg", "full_gpts", "bnd_gpts", "full_clb", "bnd_clb"
    );
    println!("{}", "-".repeat(74));

    let (curves, _chords) = chained_curves(K, DX_MM, AMP_MM, FEEDRATE);

    let mut ctx_full = replan_ctx();
    ctx_full.force_full_resolve = true;
    let mut state_full = ShaperState::new(&[0.0; 3], &ctx_full.chains);

    let ctx_bnd = replan_ctx();
    let mut state_bnd = ShaperState::new(&[0.0; 3], &ctx_bnd.chains);

    let mut full_rows: Vec<WorkCounts> = Vec::new();
    let mut bnd_rows: Vec<WorkCounts> = Vec::new();

    for seg in &curves {
        let wf = append_counted(&mut state_full, &ctx_full, seg.clone());
        let wb = append_counted(&mut state_bnd, &ctx_bnd, seg.clone());
        full_rows.push(wf);
        bnd_rows.push(wb);
    }

    for (i, (wf, wb)) in full_rows.iter().zip(bnd_rows.iter()).enumerate() {
        println!(
            "{:>6} | {:>9} {:>9} | {:>10} {:>10} | {:>9} {:>9}",
            i + 1,
            wf.window_segments,
            wb.window_segments,
            wf.grid_points,
            wb.grid_points,
            wf.clarabel_total,
            wb.clarabel_total,
        );
    }
    println!("{}", "-".repeat(74));

    let full_max_seg = full_rows.iter().map(|w| w.window_segments).max().unwrap();
    let bnd_max_seg = bnd_rows.iter().map(|w| w.window_segments).max().unwrap();
    let full_max_gp = full_rows.iter().map(|w| w.grid_points).max().unwrap();
    let bnd_max_gp = bnd_rows.iter().map(|w| w.grid_points).max().unwrap();

    let back = &bnd_rows[K / 2..];
    let back_min = back.iter().map(|w| w.window_segments).min().unwrap();
    let back_max = back.iter().map(|w| w.window_segments).max().unwrap();

    println!("full re-solve : max window_segments={full_max_seg}, max grid_points={full_max_gp}");
    println!("bounded       : max window_segments={bnd_max_seg}, max grid_points={bnd_max_gp}");
    println!(
        "bounded back-half window_segments range: [{back_min}, {back_max}] (the CAP — flat, not growing)"
    );
    println!(
        "GATE: bounded per-append work is capped at ~K_local ({bnd_max_seg} segs) while full \
         grows to depth {full_max_seg}; the bounded cap does NOT scale with window depth."
    );
    println!();

    assert_eq!(
        full_rows.last().unwrap().window_segments,
        K,
        "full re-solve window must reach the whole chain depth"
    );
    assert!(
        bnd_max_seg < full_max_seg,
        "bounded window_segments ({bnd_max_seg}) must be strictly below full depth ({full_max_seg})"
    );
    assert!(
        back_max - back_min <= 2,
        "bounded steady-state window must be flat (cap), got range [{back_min}, {back_max}]"
    );
    assert!(
        u64::from(u32::try_from(back_max).unwrap()) * 25 >= bnd_max_gp / 2,
        "grid_points should track the bounded window cap, not the full depth"
    );
}
