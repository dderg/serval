use std::time::Instant;

use nurbs::VectorNurbs;
use temporal::{
    GridConfig, GridScheme, Limits, ToleranceMode, counters, schedule_segment_with_tolerance,
};

fn repro_curve() -> VectorNurbs<f64, 3> {
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

fn velocity_bound_limits() -> Limits {
    Limits::axis_boxes(
        [300.0, 300.0, 300.0],
        [10_000.0, 10_000.0, 10_000.0],
        [50_000.0, 30_000.0, 50_000.0], // Y is tighter — stresses the Y-dominant curve
    )
}

fn repro_grid() -> GridConfig {
    GridConfig {
        scheme: GridScheme::UniformArclength,
        n: 92,
    }
}

const V_MAX: f64 = 300.0;

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
    let v_end_full_decel_to_rest = 0.0;
    let v_end = v_end_full_decel_to_rest;

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

    let counts: Vec<u32> = rows.iter().map(|r| r.clarabel_total).collect();
    let max_high_entry = counts[3].max(counts[4]);
    assert!(
        max_high_entry > counts[0],
        "high-entry call count ({max_high_entry}) must exceed rest-to-rest ({}); \
         pathology not reproduced",
        counts[0],
    );

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

    for r in &rows {
        assert!(
            r.total_time_ms.is_finite() && r.total_time_ms > 0.0,
            "total_time must be finite and positive at v_frac={:.2}; got {:.3}ms",
            r.v_entry_frac,
            r.total_time_ms,
        );
    }

    assert!(
        rows[0].status_ok,
        "rest-to-rest (v_frac=0) must produce a solved status",
    );

    for r in &rows {
        let accounted = r.path_jerk_calls + r.slp9_tr_calls + r.slp9_no_tr_calls;
        assert_eq!(
            r.clarabel_total, accounted,
            "call accounting mismatch at v_frac={:.2}: total={} vs path_jerk+slp9_tr+slp9_no_tr={}",
            r.v_entry_frac, r.clarabel_total, accounted,
        );
    }

    const PRE_LEVER3_BASELINE_MS: [f64; 5] = [445.426, 406.920, 388.272, 998.769, 317.078];
    for (r, &base_ms) in rows.iter().zip(PRE_LEVER3_BASELINE_MS.iter()) {
        assert!(
            r.total_time_ms <= base_ms * (1.0 + 1e-3),
            "trajectory total_time at v_frac={:.2} = {:.3}ms got SLOWER than the \
             pre-Lever-3 baseline {:.3}ms — non-negotiable regression",
            r.v_entry_frac,
            r.total_time_ms,
            base_ms,
        );
    }

    for i in 0..3 {
        let rel =
            (rows[i].total_time_ms - PRE_LEVER3_BASELINE_MS[i]).abs() / PRE_LEVER3_BASELINE_MS[i];
        assert!(
            rel < 1e-4,
            "v_frac={:.2} must be unchanged within tight tol; got {:.3}ms vs \
             baseline {:.3}ms (rel={:.3e})",
            rows[i].v_entry_frac,
            rows[i].total_time_ms,
            PRE_LEVER3_BASELINE_MS[i],
            rel,
        );
        assert!(
            !rows[i].restoration_fired,
            "v_frac={:.2} must not fire restoration",
            rows[i].v_entry_frac,
        );
    }

    assert!(
        rows[3].total_time_ms < PRE_LEVER3_BASELINE_MS[3],
        "v_frac=0.75 must be strictly faster than the 998.8ms crawl; got {:.3}ms",
        rows[3].total_time_ms,
    );
    assert!(
        rows[3].status_ok,
        "v_frac=0.75 must be in the solved set (feasible); got {}",
        rows[3].status_label,
    );
    assert!(
        rows[3].status_label != "SolvedSlp",
        "v_frac=0.75 is a recovered, not-provably-optimal profile and must NOT \
         wear the SolvedSlp optimal badge (fail-loud); got {}",
        rows[3].status_label,
    );

    assert!(
        !rows[4].status_ok,
        "v_frac=0.95 is geometrically infeasible and must stay a non-success; \
         got {} — seeding must not fake feasibility",
        rows[4].status_label,
    );

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

#[test]
fn chained_five_cubics_plan_batch() {
    use temporal::{BatchInput, GridStrategy, JoiningStatus, SegmentInput, plan_batch};

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
