use geometry::{FitterParams, GeometryPipeline, Item, Segment, TelemetryEvent};
use nurbs::VectorNurbs;
use temporal::{
    GridConfig, GridScheme, Limits, SolveStatus, ToleranceMode, schedule_segment_with_tolerance,
    topp::path::sample_arclength_grid,
};

fn textbook_limits() -> Limits {
    Limits::axis_boxes([500.0; 3], [5_000.0; 3], [100_000.0; 3])
}

fn build_g5_fixture_4_curve() -> VectorNurbs<f64, 3> {
    let src = "G5 X10 Y0 I3 J3 P-3 Q3 F1500\n";
    let mut pipeline = GeometryPipeline::new(FitterParams::default(), vec![]);
    let mut events: Vec<TelemetryEvent> = vec![];
    let items: Vec<_> = {
        let mut sink = |e: TelemetryEvent| events.push(e);
        pipeline.process(src, &mut sink).collect()
    };
    items
        .into_iter()
        .find_map(|it| match it {
            Item::Segment(Segment::Cubic(c)) => Some(c.xyz),
            _ => None,
        })
        .expect("G5 reduction must emit exactly one Segment::Cubic")
}

fn fifty_pct_mvc_velocities(curve: &VectorNurbs<f64, 3>, limits: &Limits) -> (f64, f64) {
    let grid = sample_arclength_grid(curve, 3).expect("arclength grid");
    let last = grid.c_prime.len() - 1;
    let b_start = limits
        .b_cent_cap(&grid.c_prime[0], &grid.c_double_prime[0], 1e-12)
        .min(1e8);
    let b_end = limits
        .b_cent_cap(&grid.c_prime[last], &grid.c_double_prime[last], 1e-12)
        .min(1e8);
    (0.5 * b_start.sqrt(), 0.5 * b_end.sqrt())
}

#[test]
fn auto_succeeds_on_straight_line() {
    let curve = VectorNurbs::<f64, 3>::try_new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![[0.0, 0.0, 0.0], [100.0, 0.0, 0.0]],
    )
    .unwrap();
    let grid = GridConfig {
        scheme: GridScheme::UniformArclength,
        n: 50,
    };
    let profile = schedule_segment_with_tolerance(
        &curve,
        &textbook_limits(),
        &grid,
        0.0,
        0.0,
        ToleranceMode::Auto,
    )
    .expect("Auto should succeed on straight line");
    assert!(matches!(
        profile.status,
        SolveStatus::Solved | SolveStatus::SolvedInexact { .. } | SolveStatus::SolvedSlp { .. }
    ));
}

#[ignore = "fixture no longer triggers Fast SLP divergence after b87cd55f endpoint fix; needs a new diverging case"]
#[test]
fn auto_falls_back_on_fixture_4_class() {
    let limits = textbook_limits();
    let curve = build_g5_fixture_4_curve();
    let grid = GridConfig {
        scheme: GridScheme::UniformArclength,
        n: 200,
    };

    let (v_start, v_end) = fifty_pct_mvc_velocities(&curve, &limits);

    let fast = schedule_segment_with_tolerance(
        &curve,
        &limits,
        &grid,
        v_start,
        v_end,
        ToleranceMode::Fast,
    )
    .expect("Fast must not return ScheduleError");
    assert!(
        !matches!(
            fast.status,
            SolveStatus::Solved | SolveStatus::SolvedInexact { .. } | SolveStatus::SolvedSlp { .. }
        ),
        "Fast must fail on this fixture (expected DivergedSlp/MaxIter/Infeasible); got {:?}",
        fast.status,
    );

    let tight = schedule_segment_with_tolerance(
        &curve,
        &limits,
        &grid,
        v_start,
        v_end,
        ToleranceMode::Tight,
    )
    .expect("Tight must not return ScheduleError");
    assert!(
        matches!(
            tight.status,
            SolveStatus::Solved | SolveStatus::SolvedInexact { .. } | SolveStatus::SolvedSlp { .. }
        ),
        "Tight must succeed on this fixture; got {:?}",
        tight.status,
    );

    let auto_profile = schedule_segment_with_tolerance(
        &curve,
        &limits,
        &grid,
        v_start,
        v_end,
        ToleranceMode::Auto,
    )
    .expect("Auto must not return ScheduleError");
    assert!(
        matches!(
            auto_profile.status,
            SolveStatus::Solved | SolveStatus::SolvedInexact { .. } | SolveStatus::SolvedSlp { .. }
        ),
        "Auto must succeed (fallback to Tight recovered the Fast failure); got {:?}",
        auto_profile.status,
    );

    assert!(
        (auto_profile.total_time - tight.total_time).abs() < 1e-6,
        "Auto time ({}) should equal Tight time ({}) on a fallback case",
        auto_profile.total_time,
        tight.total_time,
    );

    eprintln!(
        "auto_falls_back_on_fixture_4_class: v_start={v_start:.4}, v_end={v_end:.4}, \
         Fast={:?}, Tight={:?}, Auto={:?}",
        fast.status, tight.status, auto_profile.status,
    );
}
