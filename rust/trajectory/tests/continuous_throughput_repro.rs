//! Faithful offline reproduction of the continuous-streaming throughput
//! problem.
//!
//! The streaming planner re-plans its entire uncommitted lookahead window every
//! time a curve arrives (`append_and_replan` builds `plan_segments` from ALL
//! uncommitted moves and calls `plan_velocity` on the whole window, cold). A
//! move only leaves the window when it commits (`t_end <= t_freeze`, driven by
//! `t_dispatched`). With `t_dispatched` left at 0 (no playback), appending K
//! curves grows the window to depth K and each append re-solves all K from
//! scratch.
//!
//! Measurement 1 (WINDOW-DEPTH SCALING): append K chained curves WITHOUT
//! draining and record `solve_us` / `window_segments` at each append. Shows
//! whether per-replan cost grows with window depth (=> quadratic cumulative) or
//! is flat.
//!
//! Measurement 2 (KEEP-AHEAD RATIO): for a steady-state window, compute
//! Σ(solve_us) vs Σ(playback time) over a chain at a representative feedrate,
//! swept across feedrates, to find where Σsolve/Σplayback crosses 1.0.
//!
//! These are MEASUREMENTS, not assertions. Run with:
//!   cargo nextest run -p trajectory -E 'test(continuous_throughput)' --no-capture
//! (`--no-capture` so the printed tables reach the terminal.)

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
    }
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
/// approximating a sine wave the toolhead sweeps in +X. Each segment spans
/// `dx_mm` in X; control points are placed so the outgoing tangent at the end
/// of segment i equals the incoming tangent at the start of segment i+1
/// (mirrored handles), giving a smooth chain like real slicer arc/curve output.
///
/// Returns chord length per segment (mm) so playback duration can be estimated.
fn chained_curves(
    k: usize,
    dx_mm: f64,
    amp_mm: f64,
    feedrate: f64,
) -> (Vec<CubicSegment>, Vec<f64>) {
    let wavelength = 4.0 * dx_mm;
    // y(x) = amp * sin(2*pi*x / wavelength); sample knot points at segment ends.
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
        // Cubic Hermite -> Bézier: handle length = (x1-x0)/3 along the tangent.
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

// ---------------------------------------------------------------------------
// Measurement 1: window-depth scaling (cold whole-window re-solve)
// ---------------------------------------------------------------------------
#[test]
fn continuous_throughput_window_depth_scaling() {
    const K: usize = 16;
    const DX_MM: f64 = 12.0;
    const AMP_MM: f64 = 6.0;
    const FEEDRATE: f64 = 200.0;

    let ctx = replan_ctx();
    let mut state = ShaperState::new(&[0.0; 3], &ctx.chains);
    let (curves, _chords) = chained_curves(K, DX_MM, AMP_MM, FEEDRATE);

    println!();
    println!("=== Measurement 1: WINDOW-DEPTH SCALING (no drain, t_dispatched stays 0) ===");
    println!(
        "geometry: {} chained cubic Béziers, dx={}mm amp={}mm, feedrate={}mm/s",
        K, DX_MM, AMP_MM, FEEDRATE
    );
    println!("limits: trident-like CoreXY (vx=vy=500 ax=ay=20000 jx=jy=200000), Z slow");
    println!("post-proc: SmoothZV 60Hz on X/Y");
    println!();
    println!(
        "{:>7} | {:>15} | {:>12} | {:>14} | {:>18}",
        "append", "window_segments", "solve_us", "cum_solve_us", "solve_us/depth"
    );
    println!("{}", "-".repeat(78));

    let mut cum_solve: u64 = 0;
    let mut per_depth: Vec<(usize, u64)> = Vec::new();
    for (i, seg) in curves.into_iter().enumerate() {
        let report = state
            .append_and_replan(seg, &ctx)
            .expect("append should plan");
        cum_solve += report.solve_us;
        let depth = report.window_segments;
        per_depth.push((depth, report.solve_us));
        println!(
            "{:>7} | {:>15} | {:>12} | {:>14} | {:>18.1}",
            i + 1,
            depth,
            report.solve_us,
            cum_solve,
            report.solve_us as f64 / depth as f64,
        );
    }
    println!("{}", "-".repeat(78));

    // Fit slope: solve_us ~ a + b*depth using last-half vs first-half average.
    let n = per_depth.len();
    let first_half: u64 = per_depth[..n / 2].iter().map(|(_, s)| *s).sum();
    let last_half: u64 = per_depth[n / 2..].iter().map(|(_, s)| *s).sum();
    let first_avg = first_half as f64 / (n / 2) as f64;
    let last_avg = last_half as f64 / (n - n / 2) as f64;
    let depth_first = per_depth[n / 4].0 as f64;
    let depth_last = per_depth[n - 1 - n / 4].0 as f64;
    let slope = (last_avg - first_avg) / (depth_last - depth_first);

    println!(
        "first-half avg solve_us = {:.1} (depth~{:.0}), last-half avg = {:.1} (depth~{:.0})",
        first_avg, depth_first, last_avg, depth_last
    );
    println!(
        "approx marginal cost per added window segment: {:.1} us/segment",
        slope
    );
    println!(
        "cumulative Σsolve_us over {} appends = {} us ({:.2} ms)",
        K,
        cum_solve,
        cum_solve as f64 / 1000.0
    );
    println!(
        "interpretation: per-append cost grows {} with depth => cumulative is {}",
        if slope > 0.5 { "LINEARLY" } else { "~FLAT" },
        if slope > 0.5 {
            "~QUADRATIC in #curves"
        } else {
            "~linear in #curves"
        }
    );
    println!();
}

// ---------------------------------------------------------------------------
// Measurement 2: keep-ahead ratio vs feedrate
// ---------------------------------------------------------------------------
//
// Steady-state model: the planner streams curve i, the window holds a bounded
// lookahead, each append pays solve_us. Σsolve is the total host compute to
// plan the chain; Σplayback is the wall-clock the toolhead spends executing it
// (Σ chord/feedrate, the nominal cruise duration). Ratio < 1 => host keeps
// ahead; > 1 => the planner starves the motion queue.
//
// We do NOT drain here either: this is the worst case for the current cold
// whole-window architecture (window grows to the full chain depth). That is
// faithful to "back-to-back curves arriving while nothing has played yet",
// which is exactly the cold-start / high-rate burst the hypothesis targets.
#[test]
fn continuous_throughput_keep_ahead_ratio() {
    const K: usize = 12;
    const DX_MM: f64 = 12.0;
    const AMP_MM: f64 = 6.0;
    const PI_SLOWDOWN: f64 = 10.0; // Mac wall-clock ~10x faster than Pi5.

    let feedrates = [50.0_f64, 100.0, 200.0, 350.0, 500.0];

    println!();
    println!("=== Measurement 2: KEEP-AHEAD RATIO vs FEEDRATE ===");
    println!(
        "geometry: {} chained cubic Béziers, dx={}mm amp={}mm (no drain => window grows to {})",
        K, DX_MM, AMP_MM, K
    );
    println!(
        "ratio = Σsolve_us / Σplayback_us. <1 keeps ahead, >1 starves. Pi5 ≈ {}x Mac.",
        PI_SLOWDOWN
    );
    println!();
    println!(
        "{:>10} | {:>12} | {:>14} | {:>12} | {:>14} | {:>12}",
        "feed mm/s", "Σsolve_us", "Σplayback_us", "ratio(Mac)", "ratio(Pi5~10x)", "verdict(Pi)"
    );
    println!("{}", "-".repeat(88));

    for &feed in &feedrates {
        let ctx = replan_ctx();
        let mut state = ShaperState::new(&[0.0; 3], &ctx.chains);
        let (curves, chords) = chained_curves(K, DX_MM, AMP_MM, feed);

        let mut sum_solve_us: u64 = 0;
        for seg in curves {
            let report = state
                .append_and_replan(seg, &ctx)
                .expect("append should plan");
            sum_solve_us += report.solve_us;
        }

        // Σplayback = Σ nominal cruise time per segment = Σ chord / feedrate.
        // (Lower bound on real playback; real accel/decel only makes it longer,
        //  so this is the *tightest* keep-ahead requirement.)
        let sum_playback_s: f64 = chords.iter().map(|c| c / feed).sum();
        let sum_playback_us = sum_playback_s * 1.0e6;

        let ratio_mac = sum_solve_us as f64 / sum_playback_us;
        let ratio_pi = ratio_mac * PI_SLOWDOWN;
        let verdict = if ratio_pi < 1.0 {
            "KEEPS AHEAD"
        } else {
            "STARVES"
        };

        println!(
            "{:>10.0} | {:>12} | {:>14.0} | {:>12.3} | {:>14.3} | {:>12}",
            feed, sum_solve_us, sum_playback_us, ratio_mac, ratio_pi, verdict
        );
    }
    println!("{}", "-".repeat(88));
    println!(
        "note: absolute solve_us are Mac wall-clock; Pi5 column scales them by {}x.",
        PI_SLOWDOWN
    );
    println!("faster feed => shorter Σplayback => higher ratio => crosses 1.0 sooner.");
    println!();
}
