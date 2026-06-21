use nurbs::VectorNurbs;
use temporal::{
    BatchInput, BatchOutput, GridStrategy, InfeasibleReason, Limits, SegmentInput, SolveStatus,
    plan_batch,
};

fn line(p0: [f64; 3], p1: [f64; 3]) -> VectorNurbs<f64, 3> {
    VectorNurbs::<f64, 3>::try_new(1, vec![0.0, 0.0, 1.0, 1.0], vec![p0, p1]).unwrap()
}

fn cubic(pts: [[f64; 3]; 4]) -> VectorNurbs<f64, 3> {
    VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        pts.to_vec(),
    )
    .unwrap()
}

fn s_chain() -> Vec<VectorNurbs<f64, 3>> {
    let mut curves = Vec::new();
    let mut px = 0.0_f64;
    let mut py = 0.0_f64;
    let mut up = true;
    for _ in 0..5 {
        let dy = if up { 20.0 } else { -20.0 };
        curves.push(cubic([
            [px, py, 0.0],
            [px + 10.0, py, 0.0],
            [px + 20.0, py + dy, 0.0],
            [px + 30.0, py + dy, 0.0],
        ]));
        px += 30.0;
        py += dy;
        up = !up;
    }
    curves
}

fn sharp_cubic() -> Vec<VectorNurbs<f64, 3>> {
    vec![cubic([
        [0.0, 0.0, 0.0],
        [10.0, 10.0, 0.0],
        [-10.0, 10.0, 0.0],
        [0.0, 0.0, 0.001],
    ])]
}

fn circle_chain() -> Vec<VectorNurbs<f64, 3>> {
    let k = (4.0 / 3.0) * (std::f64::consts::SQRT_2 - 1.0);
    let r = 20.0_f64;
    let quarter = |a0: [f64; 3], a1: [f64; 3], c0: [f64; 3], c1: [f64; 3]| cubic([a0, c0, c1, a1]);
    vec![
        quarter(
            [r, 0.0, 0.0],
            [0.0, r, 0.0],
            [r, r * k, 0.0],
            [r * k, r, 0.0],
        ),
        quarter(
            [0.0, r, 0.0],
            [-r, 0.0, 0.0],
            [-r * k, r, 0.0],
            [-r, r * k, 0.0],
        ),
        quarter(
            [-r, 0.0, 0.0],
            [0.0, -r, 0.0],
            [-r, -r * k, 0.0],
            [-r * k, -r, 0.0],
        ),
        quarter(
            [0.0, -r, 0.0],
            [r, 0.0, 0.0],
            [r * k, -r, 0.0],
            [r, -r * k, 0.0],
        ),
    ]
}

fn geometries() -> Vec<(&'static str, Vec<VectorNurbs<f64, 3>>)> {
    vec![
        ("straight_50mm", vec![line([0.0; 3], [50.0, 0.0, 0.0])]),
        (
            "sharp_corner",
            vec![
                line([0.0; 3], [50.0, 0.0, 0.0]),
                line([50.0, 0.0, 0.0], [50.0, 50.0, 0.0]),
            ],
        ),
        (
            "gentle_cubic",
            vec![cubic([
                [0.0, 0.0, 0.0],
                [60.0, 0.0, 0.0],
                [70.0, 30.0, 0.0],
                [100.0, 50.0, 0.0],
            ])],
        ),
        ("sharp_cubic", sharp_cubic()),
        ("s_chain_5", s_chain()),
        ("circle_chain_4", circle_chain()),
        (
            "tiny_cubic_sub_mm",
            vec![cubic([
                [0.0, 0.0, 0.0],
                [0.2, 0.0, 0.0],
                [0.4, 0.1, 0.0],
                [0.6, 0.0, 0.0],
            ])],
        ),
    ]
}

fn limit_matrix() -> Vec<(&'static str, Limits)> {
    let iso = |v, a, j| Limits::axis_boxes([v; 3], [a; 3], [j; 3]);
    vec![
        ("textbook", iso(500.0, 5_000.0, 100_000.0)),
        ("corexy_fast", iso(1_000.0, 65_000.0, 50_000_000.0)),
        (
            "trident_aniso",
            Limits::axis_boxes(
                [25.0, 1_000.0, 15.0],
                [70_000.0, 70_000.0, 100.0],
                [140_000.0, 140_000.0, 200.0],
            ),
        ),
        (
            "norm_all_textbook",
            Limits::norm_all(500.0, 5_000.0, 100_000.0),
        ),
        ("tiny_v", iso(1.0, 5_000.0, 100_000.0)),
        ("huge_v", iso(100_000.0, 5_000.0, 100_000.0)),
        ("tiny_a", iso(500.0, 10.0, 100_000.0)),
        ("huge_a", iso(500.0, 5_000_000.0, 100_000.0)),
        ("tiny_j", iso(500.0, 5_000.0, 100.0)),
        ("huge_j", iso(500.0, 5_000.0, 1e12)),
        ("tiny_everything", iso(1.0, 10.0, 100.0)),
        ("huge_everything", iso(1e6, 1e7, 1e12)),
        ("huge_a_tiny_j", iso(500.0, 5_000_000.0, 10.0)),
        ("huge_v_tiny_j", iso(1e6, 5_000.0, 1.0)),
        ("huge_v_tiny_a", iso(1e6, 1e-2, 1e6)),
        ("crawl_everything", iso(1e9, 1e-2, 1e-3)),
        (
            "aniso_x_fast",
            Limits::axis_boxes(
                [1_000.0, 1.0, 1.0],
                [50_000.0, 10.0, 10.0],
                [1e8, 100.0, 100.0],
            ),
        ),
        (
            "aniso_y_fast",
            Limits::axis_boxes(
                [1.0, 1_000.0, 1.0],
                [10.0, 50_000.0, 10.0],
                [100.0, 1e8, 100.0],
            ),
        ),
        (
            "aniso_z_fast",
            Limits::axis_boxes(
                [1.0, 1.0, 1_000.0],
                [10.0, 10.0, 50_000.0],
                [100.0, 100.0, 1e8],
            ),
        ),
        ("norm_all_unit", Limits::norm_all(1.0, 1.0, 1.0)),
    ]
}

fn adaptive() -> GridStrategy {
    GridStrategy::Adaptive {
        min_n: 20,
        max_n: 200,
        target_grid_spacing_mm: 0.5,
    }
}

fn classify_status(status: &SolveStatus) -> Result<(), String> {
    const NEGLIGIBLE_MAX_ITER_RESIDUAL: f64 = 1e-6;
    match status {
        SolveStatus::Solved | SolveStatus::SolvedInexact { .. } | SolveStatus::SolvedSlp { .. } => {
            Ok(())
        }
        SolveStatus::MaxIter { last_residual } if *last_residual < NEGLIGIBLE_MAX_ITER_RESIDUAL => {
            Ok(())
        }
        SolveStatus::Infeasible { reason, .. }
            if !matches!(reason, InfeasibleReason::SolverInfeasible) =>
        {
            Ok(())
        }
        other => Err(format!("{other:?}")),
    }
}

fn check_output(out: &BatchOutput, n_segments: usize) -> Result<(), String> {
    if out.profiles.len() != n_segments {
        return Err(format!(
            "expected {n_segments} profiles, got {}",
            out.profiles.len()
        ));
    }
    for (i, p) in out.profiles.iter().enumerate() {
        classify_status(&p.status).map_err(|s| format!("profile {i} status {s}"))?;
        let deliberate_boundary_infeasibility = matches!(p.status, SolveStatus::Infeasible { .. });
        if deliberate_boundary_infeasibility {
            continue;
        }
        if !p.total_time.is_finite() || p.total_time <= 0.0 {
            return Err(format!(
                "profile {i} total_time not finite-positive: {}",
                p.total_time
            ));
        }
        for (k, smp) in p.samples.iter().enumerate() {
            if !smp.v.is_finite() || smp.v < -1e-6 {
                return Err(format!("profile {i} sample {k} v={} invalid", smp.v));
            }
            if !smp.a.is_finite() {
                return Err(format!("profile {i} sample {k} a={} not finite", smp.a));
            }
        }
    }
    Ok(())
}

fn sweep_geometry_with_endpoints(
    geo_name: &str,
    curves: &[VectorNurbs<f64, 3>],
    initial_velocity: f64,
    terminal_velocity: f64,
) {
    let mut failures: Vec<String> = Vec::new();
    for (lim_name, limits) in limit_matrix() {
        let segments: Vec<SegmentInput> = curves
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
            grid_strategy: adaptive(),
            worker_threads: 1,
            initial_velocity,
            initial_accel: 0.0,
            terminal_velocity,
        };
        match plan_batch(input) {
            Ok(out) => {
                if let Err(why) = check_output(&out, curves.len()) {
                    failures.push(format!("[{geo_name} / {lim_name}] {why}"));
                }
            }
            Err(e) => failures.push(format!("[{geo_name} / {lim_name}] BatchError: {e}")),
        }
    }
    assert!(
        failures.is_empty(),
        "{} of {} limit configs failed for geometry '{geo_name}' \
         (v_start={initial_velocity}, v_end={terminal_velocity}):\n{}",
        failures.len(),
        limit_matrix().len(),
        failures.join("\n"),
    );
}

fn sweep_geometry(geo_name: &str, curves: &[VectorNurbs<f64, 3>]) {
    sweep_geometry_with_endpoints(geo_name, curves, 0.0, 0.0);
}

#[test]
fn stress_straight() {
    let g = &geometries()[0];
    sweep_geometry(g.0, &g.1);
}

#[test]
fn stress_sharp_corner() {
    let g = &geometries()[1];
    sweep_geometry(g.0, &g.1);
}

#[test]
fn stress_gentle_cubic() {
    let g = &geometries()[2];
    sweep_geometry(g.0, &g.1);
}

#[test]
fn stress_sharp_cubic() {
    let g = &geometries()[3];
    sweep_geometry(g.0, &g.1);
}

#[test]
fn stress_s_chain() {
    let g = &geometries()[4];
    sweep_geometry(g.0, &g.1);
}

#[test]
fn stress_circle_chain() {
    let g = &geometries()[5];
    sweep_geometry(g.0, &g.1);
}

#[test]
fn stress_tiny_cubic() {
    let g = &geometries()[6];
    sweep_geometry(g.0, &g.1);
}

#[test]
fn stress_nonzero_endpoints_straight() {
    let curves = vec![line([0.0; 3], [200.0, 0.0, 0.0])];
    sweep_geometry_with_endpoints("straight_200mm_cruise", &curves, 5.0, 5.0);
    sweep_geometry_with_endpoints("straight_200mm_fast", &curves, 200.0, 0.0);
}

#[test]
fn stress_nonzero_endpoints_gentle_cubic() {
    let g = &geometries()[2];
    sweep_geometry_with_endpoints("gentle_cubic_cruise", &g.1, 5.0, 5.0);
}

#[test]
fn stress_mixed_limits_chain() {
    let curves: Vec<VectorNurbs<f64, 3>> = (0..6)
        .map(|i| {
            line(
                [i as f64 * 25.0, 0.0, 0.0],
                [(i + 1) as f64 * 25.0, 0.0, 0.0],
            )
        })
        .collect();

    let fast = Limits::axis_boxes([1_000.0; 3], [65_000.0; 3], [50_000_000.0; 3]);
    let crawl = Limits::axis_boxes([2.0; 3], [20.0; 3], [50.0; 3]);
    let per_segment: Vec<Limits> = (0..curves.len())
        .map(|i| if i % 2 == 0 { fast } else { crawl })
        .collect();

    let segments: Vec<SegmentInput> = curves
        .iter()
        .zip(&per_segment)
        .map(|(c, l)| SegmentInput {
            curve: c,
            limits: *l,
            followers: &[],
            virtual_path: None,
        })
        .collect();
    let input = BatchInput {
        segments: &segments,
        shaping: None,
        grid_strategy: adaptive(),
        worker_threads: 2,
        initial_velocity: 0.0,
        initial_accel: 0.0,
        terminal_velocity: 0.0,
    };
    let out = plan_batch(input).expect("mixed-limits chain must not BatchError");
    check_output(&out, curves.len()).expect("mixed-limits chain outcome must be acceptable");
}
