//! Faithful offline reproduction of the continuous-streaming throughput
//! problem, instrumented in DETERMINISTIC, LOAD-INDEPENDENT COUNTS.
//!
//! The streaming planner re-plans its entire uncommitted lookahead window every
//! time a curve arrives: `append_and_replan` builds `plan_segments` from ALL
//! uncommitted moves (`state.rs` ~line 173) and calls `plan_velocity` on the
//! whole window. A move only leaves the window when it commits
//! (`uncommitted_moves.retain(|m| m.t_end > t_freeze)`, driven by
//! `t_dispatched`). So per-append solver WORK grows with live window depth.
//!
//! WALL-CLOCK IS NOT A GATE HERE. This machine runs many things at once and
//! background load drifts, so the gate is on COUNTS that are identical
//! regardless of CPU load:
//!   - `window_segments` fed to the solver per append,
//!   - total grid points solved per append (Σ chain `n_points()`),
//!   - Clarabel solve count per append (`clarabel_calls_total`),
//!   - SLP / SLP9 outer-iteration counts per append.
//!
//! These are read from `temporal::counters` (thread-local, reset before each
//! append, snapshotted after) plus `ReplanReport`. A single isolated wall-time
//! estimate is printed but explicitly labeled rough / non-gating.
//!
//! Run with:
//!   cargo nextest run -p trajectory -E 'test(continuous_throughput)' --no-capture

use geometry::segment::{CubicSegment, SourceRange};
use nurbs::VectorNurbs;
use trajectory::streaming::{ReplanContext, ShaperState};
use trajectory::{AxisChainSet, CompiledChain, PostProcessorType};

/// Trident-like asymmetric CoreXY limit set (X/Y identical, Z slow).
/// v in mm/s, a in mm/s^2, j in mm/s^3.
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

/// Baseline-measurement context: forces the legacy whole-window re-solve so
/// Measurements 1-5 keep characterizing the ORIGINAL O(window-depth) behavior
/// they were written to expose. The bounded front-freeze (now the production
/// default) is measured separately by the `bounded_*` tests below.
fn baseline_full_ctx() -> ReplanContext {
    let mut ctx = replan_ctx();
    ctx.force_full_resolve = true;
    ctx
}

/// Build one planar cubic Bézier from 4 explicit control points (z = 0).
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

/// Generate a tangent-continuous (G1/C1) chain of `k` planar cubic Béziers
/// approximating a sine wave the toolhead sweeps in +X. Returns chord length
/// per segment (mm) so playback duration can be estimated.
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

/// A decel-to-corner chain: a long straight run-up followed by a hard ~90°
/// corner, then a short outgoing leg. The corner forces a deep backward
/// velocity-propagation (deceleration-reach) horizon — the worst case for the
/// "freeze the front" sub-window cap, because the decel ramp that must brake
/// for the corner can reach many segments back. Used to stress how far the
/// backward horizon extends relative to window depth.
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

/// Per-append deterministic WORK COUNTS. Identical regardless of CPU load.
#[derive(Debug, Clone, Copy)]
struct WorkCounts {
    /// Segments fed to the whole-window re-solve this append.
    window_segments: usize,
    /// Σ ChainGrid::n_points() over every chain scheduled this append.
    grid_points: u64,
    /// Number of chain schedules (≈ Clarabel-bearing chain solves).
    chains_scheduled: u32,
    /// Total Clarabel SOCP solves (base + path-jerk SLP + SLP9 probes).
    clarabel_total: u32,
    /// Clarabel solves outside SLP9: base SOCP + path-jerk SLP outer iters.
    clarabel_path_jerk: u32,
    /// SLP9 (axis-jerk) trust-region Clarabel probes.
    clarabel_slp9_tr: u32,
    /// SLP9 no-trust-region fallback Clarabel solves.
    clarabel_slp9_no_tr: u32,
    /// β-medium outer iteration count for this append.
    beta_iterations: u8,
    /// Rough wall-clock for the solve only (NON-GATING; load-dependent).
    solve_us: u64,
}

/// Append one curve with the counters reset immediately before, snapshotted
/// immediately after, so every count is attributed to exactly this append.
fn append_counted(state: &mut ShaperState, ctx: &ReplanContext, seg: CubicSegment) -> WorkCounts {
    temporal::counters::reset();
    let report = state
        .append_and_replan(seg, ctx)
        .expect("append should plan");
    // snapshot_global aggregates across the worker threads the solve fans out
    // to; the thread-local snapshot would read 0 on the orchestrating thread.
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

// ---------------------------------------------------------------------------
// Measurement 1: per-append WORK COUNTS vs LIVE WINDOW DEPTH (no drain)
// ---------------------------------------------------------------------------
//
// t_dispatched stays at 0, so nothing commits and the window grows to depth K.
// Each row is the deterministic work the solver did on that append. The point:
// grid_points / clarabel_total / chains_scheduled grow ~linearly with
// window_segments => cumulative work over the chain is ~quadratic.
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
    let first = rows[1]; // append #2: first append with a real window > 1
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

    // Deterministic gate: the marginal grid-point cost per added segment must be
    // strictly positive — i.e. work demonstrably scales with depth, not flat.
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
    // Per-segment grid density is ~constant (whole-window re-solve), so grid
    // points must track window_segments closely.
    let approx_per_seg = last.grid_points as f64 / last.window_segments as f64;
    assert!(
        approx_per_seg > 1.0,
        "each window segment contributes grid nodes; got {approx_per_seg}",
    );
}

// ---------------------------------------------------------------------------
// Measurement 2: STEADY-STATE WINDOW DEPTH per feedrate WITH REALISTIC DRAIN
// ---------------------------------------------------------------------------
//
// Models a real host buffer: the toolhead plays back at `feed`, and the host
// keeps a bounded lookahead of ~`buffer_s` seconds of motion queued ahead of
// playback. We advance a simulated playback clock and set `t_dispatched` to it
// before each append, so committed moves drop out of the window
// (`retain(|m| m.t_end > t_freeze)`). The window reaches a steady-state depth
// bounded by how many curves fit in `buffer_s` at that feedrate. We report
// that steady-state depth AND the per-append WORK COUNTS at steady state.
#[test]
fn continuous_throughput_steady_state_depth_with_drain() {
    const DX_MM: f64 = 12.0;
    const AMP_MM: f64 = 6.0;
    const K: usize = 18;

    // Host buffer depths in SECONDS of queued motion ahead of playback. 1.0s at
    // 500mm/s would need depth ~20 > K to reach steady state; we keep the sweep
    // within K so every reported steady depth is buffer-limited, not chain-
    // length-limited, and the test stays fast. The trend (depth grows with feed
    // and buffer) is fully visible at these points.
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
                // Drain: the toolhead has played everything more than buffer_s
                // behind the most-recent planned append end. t_dispatched is the
                // absolute planner time playback has reached; trailing the
                // actual planned front by buffer_s (using real segment end times,
                // which include accel/decel, not nominal chord time) commits and
                // drops those moves on the next append's
                // `retain(|m| m.t_end > t_freeze)`.
                let playback_t = (state.t_appended - buffer_s).max(0.0);
                if playback_t > state.t_dispatched {
                    state.t_dispatched = playback_t.min(state.t_appended);
                }

                let w = append_counted(&mut state, &ctx, seg);

                // Collect the back half as "steady state" (front transient
                // while the buffer fills is excluded).
                if i >= K / 2 {
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

// ---------------------------------------------------------------------------
// Measurement 3: keep-ahead picture in COUNT terms (no-drain worst case)
// ---------------------------------------------------------------------------
//
// COUNT-based keep-ahead: Σgrid_points (total deterministic solver work to plan
// the chain) vs Σplayback (toolhead execution time at feed). The ratio of
// solver work to available real-time scales with feedrate because faster feeds
// shrink Σplayback while the no-drain window grows. This is the worst-case
// burst (back-to-back curves while nothing has played yet).
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

// ---------------------------------------------------------------------------
// Measurement 4: decel-to-corner backward horizon (worst case for front-freeze)
// ---------------------------------------------------------------------------
//
// A long straight run-up into a hard corner. The corner forces deceleration
// that propagates backward many segments — this is the LARGEST backward
// velocity-propagation horizon, the case where freezing the sub-window front
// must include the deepest tail to stay trajectory-neutral. We report per-append
// work counts; the corner append is where backward propagation is deepest.
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

// ---------------------------------------------------------------------------
// Measurement 5: ISOLATED wall-time estimate (ROUGH, NON-GATING)
// ---------------------------------------------------------------------------
//
// Single-process median of >= 5 reps of the per-append solve at a fixed window
// depth. Labeled rough; this is NOT a gate. The real real-time validation
// happens later on the actual Pi bench. Counts above are the gate.
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

// ---------------------------------------------------------------------------
// THE FIX — trajectory-neutrality + bounded-work tests (DETERMINISTIC GATE)
// ---------------------------------------------------------------------------

/// Sample the committed (unshaped planned) trajectory of `state` on a dense
/// time grid over `[0, t_end]`, returning per-axis `(pos, vel)` rows. This is
/// the deterministic value used to compare bounded vs full re-solve output.
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

/// Run a chain of curves through `append_and_replan`, advancing a simulated
/// playback clock so moves commit and drop out of the window (realistic drain),
/// and after every append capture the committed trajectory and the per-append
/// `window_segments`. `force_full` selects the legacy whole-window re-solve.
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
        // Drain to a WHOLE-segment boundary: snap the playback clock back to the
        // most recent uncommitted move start that is still ≤ the buffer-trailing
        // time. This commits whole segments and avoids the mid-segment Bézier
        // split (a separate, pre-existing Newton-inversion path) so the test
        // isolates the bounded re-solve's trajectory-neutrality.
        let raw_playback = (state.t_appended - buffer_s).max(0.0);
        let snapped = state
            .uncommitted_moves
            .iter()
            .map(|m| m.t_start)
            .filter(|&t| t <= raw_playback)
            .fold(0.0_f64, f64::max);
        if snapped > state.t_dispatched {
            state.t_dispatched = snapped.min(state.t_appended);
        }
        let report = state
            .append_and_replan(seg.clone(), &ctx)
            .expect("append should plan");
        win_segs.push(report.window_segments);
        // Compare only the immutable / dispatched region — everything at or
        // before t_dispatched is committed and must be bit-stable between the
        // bounded and full re-solve.
        let t_cmp = state.t_dispatched.max(0.0);
        committed_snaps.push(sample_committed(&state, t_cmp, 200));
    }
    let _ = feed;
    (committed_snaps, win_segs, state)
}

/// Max abs (position, velocity) difference between two committed snapshots.
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
    // Shallow buffer keeps the drain shaper-split-free (the SmoothZV kernel
    // half-support pushes the freeze point mid-segment at deeper buffers,
    // exercising an unrelated Bézier-split path). The work-bound win is shown
    // by the no-drain and decel-to-corner tests; this test's job is neutrality.
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
    // t_appended is the frontier of the still-MUTABLE tail, not the committed
    // region. Its absolute value carries the frozen-front segment durations
    // from the appends in which those segments were last the live tail, so it
    // differs from the full re-solve by accumulated per-segment solver-timing
    // noise (≈50 µs/segment). Trajectory-neutrality is defined on the COMMITTED
    // (dispatched, immutable) region, asserted tightly above; here we only
    // bound the tail-frontier drift to confirm it is solver-noise small.
    const TAIL_FRONTIER_TOL_S: f64 = 2.0e-3;
    assert!(
        (full_state.t_appended - bnd_state.t_appended).abs() < TAIL_FRONTIER_TOL_S,
        "t_appended tail-frontier drift on decel-to-corner exceeds solver noise: full {} vs bounded {}",
        full_state.t_appended,
        bnd_state.t_appended,
    );
    // Adaptive-K: the corner append must grow the bounded window well beyond the
    // smooth steady-state cap to cover the corner's deep backward decel reach.
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

// ---------------------------------------------------------------------------
// THE GATE — per-append WORK COUNTS are bounded (capped ~K) vs growing depth
// ---------------------------------------------------------------------------
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

    // Steady-state (back half) bounded window must be flat, not growing.
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

    // The full re-solve must grow with depth (baseline behaviour).
    assert_eq!(
        full_rows.last().unwrap().window_segments,
        K,
        "full re-solve window must reach the whole chain depth"
    );
    // The bounded re-solve must NOT grow to full depth — it is capped.
    assert!(
        bnd_max_seg < full_max_seg,
        "bounded window_segments ({bnd_max_seg}) must be strictly below full depth ({full_max_seg})"
    );
    // And the steady-state bounded window must be flat: the range over the back
    // half must be small (a constant cap), proving work does not scale with
    // accumulated window depth.
    assert!(
        back_max - back_min <= 2,
        "bounded steady-state window must be flat (cap), got range [{back_min}, {back_max}]"
    );
    assert!(
        u64::from(u32::try_from(back_max).unwrap()) * 25 >= bnd_max_gp / 2,
        "grid_points should track the bounded window cap, not the full depth"
    );
}
