use geometry::segment::{CubicSegment, FollowerDemand, SourceRange};
use nurbs::VectorNurbs;
use trajectory::plan_velocity::SafetyMode;
use trajectory::streaming::{ReplanContext, ShaperState};
use trajectory::{AxisChainSet, CompiledChain};

fn extruder_chains() -> AxisChainSet {
    let mut chains = AxisChainSet::passthrough_spatial();
    chains.chains.push(CompiledChain {
        kernel: None,
        gain: 0.0,
        smooth_time: 0.0,
    });
    chains.followers.push((3, vec![0, 1, 2]));
    chains
}

fn extruder_ctx() -> ReplanContext {
    const E_V_MAX: f64 = 20.0;

    let mut sets = temporal::Limits::axis_boxes([500.0; 3], [5_000.0; 3], [100_000.0; 3])
        .sets()
        .to_vec();
    sets.push(temporal::LimitSet {
        axes: temporal::AxisSet::from_indices(&[3]),
        v_max: E_V_MAX,
        a_max: 1500.0,
        j_max: 30_000.0,
    });
    let limits = temporal::Limits::try_new(&sets, 4).unwrap();

    ReplanContext {
        limits,
        chains: extruder_chains(),
        fit_tolerance_mm: 0.005,
        beta_max_iters: 5,
        beta_convergence_ratio: 1.02,
        worker_threads: 1,
        grid_strategy: temporal::multi::GridStrategy::Fixed(40),
        fallback_initial_v: 0.0,
        safety_mode: SafetyMode::WorstCaseFuture,
        force_full_resolve: false,
    }
}

fn extruding_x_segment(start_x: f64, end_x: f64, ratio: f64, feedrate: f64) -> CubicSegment {
    let p0 = [start_x, 0.0, 0.0];
    let p3 = [end_x, 0.0, 0.0];
    let lerp = |t: f64| -> [f64; 3] { [p0[0] + (p3[0] - p0[0]) * t, 0.0, 0.0] };
    let cps = vec![p0, lerp(1.0 / 3.0), lerp(2.0 / 3.0), p3];
    let xyz = VectorNurbs::<f64, 3>::try_new(3, vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0], cps)
        .unwrap();
    CubicSegment::try_new(
        xyz,
        vec![FollowerDemand {
            axis_index: 3,
            ratio,
        }],
        feedrate,
        SourceRange {
            start_line: 0,
            end_line: 0,
        },
        None,
    )
    .unwrap()
}

#[test]
fn replan_report_carries_binding_summary() {
    let ctx = extruder_ctx();
    let mut state = ShaperState::new(&[0.0; 4], &ctx.chains);

    let full_extruder_coupling = 1.0;
    let feedrate_far_above_e_v_max = 200.0;
    let mv = extruding_x_segment(
        0.0,
        50.0,
        full_extruder_coupling,
        feedrate_far_above_e_v_max,
    );
    let report = state.append_and_replan(mv, &ctx).expect("replan solves");

    let worst = report
        .binding
        .worst
        .expect("a velocity-pinned extruding move reports a worst binding");
    assert!(
        matches!(
            worst.constraint,
            temporal::BindingConstraint::Velocity { .. }
                | temporal::BindingConstraint::PaVelocity { .. }
        ),
        "expected Velocity or PaVelocity worst binding, got {:?}",
        worst.constraint
    );
    assert!(
        report.binding.histogram.iter().any(|(_, n)| *n > 0),
        "histogram must have at least one non-zero count"
    );
}
