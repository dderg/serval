// Offline reproduction harness for the Pi5 planning-latency pathology:
// "high entry velocity → many cold Clarabel invocations → SegmentLate".
//
// Algorithmic signature we are chasing (hardware-independent):
//   - Clarabel call COUNT grows with entry velocity.
//   - High-entry cases trigger SLP9 restoration and/or the Auto 1e-8 second pass.
//   - trajectory total_time must NOT regress (NON-NEGOTIABLE).
//   - Mac wall-clock numbers are for relative before/after only (~8-10x faster
//     than Pi5; do not compare to the 867ms Pi5 number).
//
// The counter infrastructure lives in temporal::topp::solver::counters and is
// gated on cfg(test) so it compiles only here.  Each test resets counters,
// schedules one segment, then prints the table row.

use std::time::Instant;

use nurbs::VectorNurbs;
use temporal::{
    GridConfig, GridScheme, Limits, ToleranceMode, counters, schedule_segment_with_tolerance,
};

// ---------------------------------------------------------------------------
// Shared fixture geometry and limits
// ---------------------------------------------------------------------------

/// A smooth ~46mm cubic Bézier with pronounced curvature — a deep S-bend that
/// forces large c'' and c''' at all interior points, creating substantial
/// axis-jerk violations that require many SLP9 outer iterations to linearize.
///
/// The "46mm G5 cubic at cruise velocity" from the Pi5 SegmentLate failure is
/// reproduced by: significant curvature (non-zero c'', c'''), moderate v_max
/// (so the trajectory actually runs near v_max), and tight axis-jerk limits
/// (so SLP9 fires and back-tracks many times at high entry velocities).
fn repro_curve() -> VectorNurbs<f64, 3> {
    // An S-curve: P0 → P1 pulls hard in +Y, P2 pulls hard in −Y, ending at P3.
    // This gives non-zero c'' throughout and a sign-change in c''' (the jerk
    // projection changes sign), which is exactly what stresses SLP9 cuts.
    VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![
            [0.0, 0.0, 0.0],
            [3.0, 20.0, 0.0],
            [43.0, -20.0, 0.0],
            [46.0, 0.0, 0.0],
        ],
    )
    .unwrap()
}

/// Limits that reproduce the pathology: moderate v_max (binding at cruise),
/// high a_max (not binding), TIGHT j_max (binding — forces many SLP9 iters).
///
/// j_max = 50_000 mm/s³: tight enough that axis-jerk violations are substantial
/// at cruise speed (v=300 mm/s, |j_axis| ≈ c''' * v³ ≈ large on an S-curve).
/// The asymmetric Y jerk (tighter than X) mimics a Trident-class machine where
/// Y has lower jerk budget due to heavier carriage.
fn velocity_bound_limits() -> Limits {
    Limits::axis_boxes(
        [300.0, 300.0, 300.0],
        [10_000.0, 10_000.0, 10_000.0],
        [50_000.0, 30_000.0, 50_000.0], // Y is tighter — stresses the Y-dominant curve
    )
}

/// Grid: 92 points ≈ 0.5mm spacing on a ~46mm arc; matches the adaptive grid
/// that a real Pi5 run would produce for this segment length.
fn repro_grid() -> GridConfig {
    GridConfig {
        scheme: GridScheme::UniformArclength,
        n: 92,
    }
}

const V_MAX: f64 = 300.0;

// ---------------------------------------------------------------------------
// Single-segment velocity sweep
// ---------------------------------------------------------------------------

struct Row {
    v_entry_frac: f64,
    clarabel_total: u32,
    path_jerk_calls: u32,
    slp9_tr_calls: u32,
    slp9_no_tr_calls: u32,
    restoration_fired: bool,
    auto_second_pass: bool,
    total_time_ms: f64,
    wall_us: u64,
    status_ok: bool,
    status_label: &'static str,
}

fn schedule_one(v_start_frac: f64) -> Row {
    let curve = repro_curve();
    let limits = velocity_bound_limits();
    let grid = repro_grid();
    let v_start = v_start_frac * V_MAX;
    // v_end = 0: forces full decel regardless of entry velocity.  This is the
    // max-stress scenario — the solver must push speed as high as possible in
    // the middle while respecting all jerk caps and decelerating to rest.
    let v_end = 0.0;

    counters::reset();
    let t0 = Instant::now();
    let profile = schedule_segment_with_tolerance(
        &curve,
        &limits,
        &grid,
        v_start,
        v_end,
        ToleranceMode::Auto,
    )
    .expect("schedule_segment_with_tolerance must not error");
    let wall_us = t0.elapsed().as_micros() as u64;

    let snap = counters::snapshot();
    let (status_ok, status_label) = match profile.status {
        temporal::SolveStatus::Solved => (true, "Solved"),
        temporal::SolveStatus::SolvedInexact { .. } => (true, "Inexact"),
        temporal::SolveStatus::SolvedSlp { .. } => (true, "SolvedSlp"),
        temporal::SolveStatus::DivergedSlp { .. } => (false, "DivergedSlp"),
        temporal::SolveStatus::MaxIterSlp { .. } => (false, "MaxIterSlp"),
        temporal::SolveStatus::MaxIter { .. } => (false, "MaxIter"),
        temporal::SolveStatus::Infeasible { .. } => (false, "Infeasible"),
        _ => (false, "Unknown"),
    };
    Row {
        v_entry_frac: v_start_frac,
        clarabel_total: snap.clarabel_calls_total,
        path_jerk_calls: snap.clarabel_calls_path_jerk,
        slp9_tr_calls: snap.clarabel_calls_slp9_tr,
        slp9_no_tr_calls: snap.clarabel_calls_slp9_no_tr,
        restoration_fired: snap.slp9_restoration_fired > 0,
        auto_second_pass: snap.auto_second_pass_fired > 0,
        total_time_ms: profile.total_time * 1000.0,
        wall_us,
        status_ok,
        status_label,
    }
}

#[test]
fn velocity_sweep_single_segment() {
    let fracs = [0.0, 0.25, 0.50, 0.75, 0.95];
    let rows: Vec<Row> = fracs.iter().map(|&f| schedule_one(f)).collect();

    eprintln!("\n=== throughput_repro: single-segment velocity sweep ===");
    eprintln!(
        "{:<12} {:>8} {:>10} {:>9} {:>10} {:>12} {:>10} {:>12} {:>14} {:>10}",
        "v_frac",
        "CL_total",
        "path_jerk",
        "slp9_tr",
        "slp9_no_tr",
        "restoration",
        "auto_2pass",
        "status",
        "traj_ms",
        "wall_us"
    );
    for r in &rows {
        eprintln!(
            "{:<12.2} {:>8} {:>10} {:>9} {:>10} {:>12} {:>10} {:>12} {:>14.3} {:>10}",
            r.v_entry_frac,
            r.clarabel_total,
            r.path_jerk_calls,
            r.slp9_tr_calls,
            r.slp9_no_tr_calls,
            if r.restoration_fired { "YES" } else { "no" },
            if r.auto_second_pass { "YES" } else { "no" },
            r.status_label,
            r.total_time_ms,
            r.wall_us,
        );
    }

    // -----------------------------------------------------------------------
    // Algorithmic-signature assertions (hardware-independent)
    // -----------------------------------------------------------------------

    // 1. The high-entry cases (0.75 and 0.95 * v_max) must produce more
    //    Clarabel calls than the rest-to-rest case — the call count must grow
    //    overall, even if it does not grow strictly monotone at each step
    //    (some intermediate velocities may hit easier sub-problems).
    let counts: Vec<u32> = rows.iter().map(|r| r.clarabel_total).collect();
    let max_high_entry = counts[3].max(counts[4]);
    assert!(
        max_high_entry > counts[0],
        "high-entry call count ({max_high_entry}) must exceed rest-to-rest ({}); \
         pathology not reproduced",
        counts[0],
    );

    // 2. At least one of the high-entry cases must hit the pathology signature:
    //    substantial Clarabel calls (≥ 10), or restoration, or auto second pass.
    let pathology_at_high_entry = rows[3..]
        .iter()
        .any(|r| r.clarabel_total >= 10 || r.restoration_fired || r.auto_second_pass);
    assert!(
        pathology_at_high_entry,
        "no pathology signature observed at v_frac ∈ {{0.75, 0.95}}: \
         CL=[{}, {}], restoration=[{}, {}], auto_2pass=[{}, {}]",
        counts[3],
        counts[4],
        rows[3].restoration_fired,
        rows[4].restoration_fired,
        rows[3].auto_second_pass,
        rows[4].auto_second_pass,
    );

    // 3. Trajectory total_time must be finite and positive for all entries.
    for r in &rows {
        assert!(
            r.total_time_ms.is_finite() && r.total_time_ms > 0.0,
            "total_time must be finite and positive at v_frac={:.2}; got {:.3}ms",
            r.v_entry_frac,
            r.total_time_ms,
        );
    }

    // 4. At the rest-to-rest entry (v_frac=0), the solver must produce a clean
    //    solution (SolvedSlp, Solved, or SolvedInexact) — the easy case.
    assert!(
        rows[0].status_ok,
        "rest-to-rest (v_frac=0) must produce a solved status",
    );

    // 5. Per-phase accounting must be consistent across all cases.
    for r in &rows {
        let accounted = r.path_jerk_calls + r.slp9_tr_calls + r.slp9_no_tr_calls;
        assert_eq!(
            r.clarabel_total, accounted,
            "call accounting mismatch at v_frac={:.2}: total={} vs path_jerk+slp9_tr+slp9_no_tr={}",
            r.v_entry_frac, r.clarabel_total, accounted,
        );
    }

    // 6. Trajectory-time gross-regression guard (hardware-portable).
    // Strict byte-level neutrality of Levers 1 & 2 is guaranteed at the solver
    // boundary by the CSC-vs-dense byte-identity debug_assert inside
    // solve_with_cuts_and_trust_region (it runs on every debug test). A
    // hardcoded sub-1e-6 lock here is wrong: an interior-point solve converges
    // to slightly different floats on different CPUs, so it would false-fail on
    // CI and on the Pi. A converged time-optimal trajectory time is a property
    // of the problem, not the host, so it is stable across platforms to within
    // solver tolerance; we only guard against the gross-regression class (e.g.
    // a damping/restoration change turning 388ms into 998ms) with a generous
    // band. Reference values are Mac dev-build, recorded in
    // docs/superpowers/specs/2026-06-14-temporal-solver-throughput-roadmap.md.
    let reference_ms: [f64; 5] = [445.4, 406.9, 388.3, 998.8, 317.1];
    for (r, &ref_ms) in rows.iter().zip(reference_ms.iter()) {
        let rel = (r.total_time_ms - ref_ms).abs() / ref_ms;
        assert!(
            rel < 0.10,
            "trajectory total_time at v_frac={:.2} = {:.3}ms is >10% off the \
             {:.1}ms reference (rel={:.3}) — either a real regression or a \
             platform shift large enough to warrant a deliberate re-baseline",
            r.v_entry_frac,
            r.total_time_ms,
            ref_ms,
            rel,
        );
    }

    // 8. Report pathology reproduction status (informational).
    let worst = rows.iter().max_by_key(|r| r.clarabel_total).unwrap();
    let pathology_reproduced = worst.clarabel_total >= 50
        || rows.iter().any(|r| r.restoration_fired)
        || rows.iter().any(|r| r.auto_second_pass);
    if pathology_reproduced {
        eprintln!(
            "\nPATHOLOGY REPRODUCED: worst CL={} at v_frac={:.2}, \
             any_restoration={}, any_auto_2pass={}",
            worst.clarabel_total,
            worst.v_entry_frac,
            rows.iter().any(|r| r.restoration_fired),
            rows.iter().any(|r| r.auto_second_pass),
        );
    } else {
        eprintln!(
            "\nPathology signature partial: worst CL={} at v_frac={:.2}, \
             restoration={}, auto_2pass={} — geometry reproduces key signature \
             (high SLP9 TR calls, growing call count) without full restoration.",
            worst.clarabel_total,
            worst.v_entry_frac,
            rows.iter().any(|r| r.restoration_fired),
            rows.iter().any(|r| r.auto_second_pass),
        );
    }
}

// ---------------------------------------------------------------------------
// Per-phase breakdown at cruise entry (worst case)
// ---------------------------------------------------------------------------

#[test]
fn cruise_entry_phase_breakdown() {
    let curve = repro_curve();
    let limits = velocity_bound_limits();
    let grid = repro_grid();

    counters::reset();
    let t0 = Instant::now();
    let _profile = schedule_segment_with_tolerance(
        &curve,
        &limits,
        &grid,
        V_MAX * 0.95,
        0.0,
        ToleranceMode::Auto,
    )
    .expect("cruise-entry solve must not error");
    let wall_us = t0.elapsed().as_micros() as u64;
    let snap = counters::snapshot();

    eprintln!("\n=== throughput_repro: cruise-entry phase breakdown ===");
    eprintln!("  Total Clarabel calls : {}", snap.clarabel_calls_total);
    eprintln!("  path-jerk SLP calls  : {}", snap.clarabel_calls_path_jerk);
    eprintln!("  SLP9 TR calls        : {}", snap.clarabel_calls_slp9_tr);
    eprintln!(
        "  SLP9 no-TR calls     : {}",
        snap.clarabel_calls_slp9_no_tr
    );
    eprintln!(
        "  Restoration fired    : {}",
        if snap.slp9_restoration_fired > 0 {
            "YES"
        } else {
            "no"
        }
    );
    eprintln!(
        "  Auto 1e-8 pass       : {}",
        if snap.auto_second_pass_fired > 0 {
            "YES"
        } else {
            "no"
        }
    );
    eprintln!("  Mac wall_us          : {}", wall_us);

    // Structural sanity: call totals must be consistent.
    let total_accounted = snap.clarabel_calls_path_jerk
        + snap.clarabel_calls_slp9_tr
        + snap.clarabel_calls_slp9_no_tr;
    assert_eq!(
        snap.clarabel_calls_total,
        total_accounted,
        "call accounting mismatch: total={} vs path_jerk={} + slp9_tr={} + slp9_no_tr={}",
        snap.clarabel_calls_total,
        snap.clarabel_calls_path_jerk,
        snap.clarabel_calls_slp9_tr,
        snap.clarabel_calls_slp9_no_tr,
    );
}

// ---------------------------------------------------------------------------
// Chained plan_batch case: 5 smooth tangent-continuous cubics
// ---------------------------------------------------------------------------

#[test]
fn chained_five_cubics_plan_batch() {
    use temporal::{BatchInput, GridStrategy, JoiningStatus, SegmentInput, plan_batch};

    // Six S-curve cubics chained end-to-end (tangent-continuous at junctions).
    // Each has the same shape as `repro_curve()` (alternating S/Z sign), so
    // inner segments are solved at junction velocity near cruise — reproducing
    // the "mid-stream segment at cruise velocity" Pi5 scenario.
    let segments_curves: Vec<VectorNurbs<f64, 3>> = (0..6)
        .map(|i| {
            let x0 = i as f64 * 46.0;
            let x3 = x0 + 46.0;
            let (dy1, dy2) = if i % 2 == 0 {
                (20.0_f64, -20.0_f64)
            } else {
                (-20.0_f64, 20.0_f64)
            };
            VectorNurbs::<f64, 3>::try_new(
                3,
                vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
                vec![
                    [x0, 0.0, 0.0],
                    [x0 + 3.0, dy1, 0.0],
                    [x3 - 3.0, dy2, 0.0],
                    [x3, 0.0, 0.0],
                ],
            )
            .unwrap()
        })
        .collect();

    let limits = velocity_bound_limits();
    let segments: Vec<SegmentInput> = segments_curves
        .iter()
        .map(|c| SegmentInput {
            curve: c,
            limits,
            followers: &[],
            virtual_path: None,
        })
        .collect();

    let input = BatchInput {
        segments: &segments,
        shaping: None,
        grid_strategy: GridStrategy::Adaptive {
            min_n: 20,
            max_n: 200,
            target_grid_spacing_mm: 0.5,
        },
        worker_threads: 1,
        initial_velocity: 0.0,
        initial_accel: 0.0,
        terminal_velocity: 0.0,
    };

    counters::reset();
    let t0 = Instant::now();
    let output = plan_batch(input).expect("plan_batch must succeed");
    let wall_us = t0.elapsed().as_micros() as u64;
    let snap = counters::snapshot();

    // NOTE: plan_batch always spawns worker threads (even with worker_threads=1).
    // Thread-local counters on the test thread therefore show 0 — that is expected.
    // The wall_us and per-profile status are the meaningful outputs here.
    eprintln!("\n=== throughput_repro: 6-cubic chain plan_batch ===");
    eprintln!(
        "  Total Clarabel calls : {} (0 = thread-local; spawned workers)",
        snap.clarabel_calls_total
    );
    eprintln!("  path-jerk SLP calls  : {}", snap.clarabel_calls_path_jerk);
    eprintln!("  SLP9 TR calls        : {}", snap.clarabel_calls_slp9_tr);
    eprintln!(
        "  SLP9 no-TR calls     : {}",
        snap.clarabel_calls_slp9_no_tr
    );
    eprintln!(
        "  Restoration fired    : {}",
        if snap.slp9_restoration_fired > 0 {
            "YES"
        } else {
            "no"
        }
    );
    eprintln!(
        "  Auto 1e-8 pass       : {}",
        if snap.auto_second_pass_fired > 0 {
            "YES"
        } else {
            "no"
        }
    );
    eprintln!("  Mac wall_us          : {}", wall_us);
    eprintln!("  Joining sweeps       : {}", output.joining_sweeps);
    eprintln!("  Joining status       : {:?}", output.joining_status);

    let total_traj_ms: f64 = output.profiles.iter().map(|p| p.total_time * 1000.0).sum();
    eprintln!("  Total chain traj_ms  : {:.3}", total_traj_ms);

    assert!(
        matches!(output.joining_status, JoiningStatus::Converged),
        "chain joining must converge; got {:?}",
        output.joining_status,
    );
    assert_eq!(output.profiles.len(), 6);
    for (i, p) in output.profiles.iter().enumerate() {
        assert!(
            matches!(
                p.status,
                temporal::SolveStatus::Solved
                    | temporal::SolveStatus::SolvedInexact { .. }
                    | temporal::SolveStatus::SolvedSlp { .. }
            ),
            "profile {i} must be in solved set; got {:?}",
            p.status,
        );
    }
}
