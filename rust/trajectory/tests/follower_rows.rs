use geometry::segment::FollowerDemand;
use nurbs::algebra::PiecewisePolynomialKernel;
use nurbs::VectorNurbs;
use temporal::multi::{GridStrategy, SegmentInput};
use trajectory::{shape_batch, ShapeBatchInput, ShapeBatchOutput, ShapeSegmentInput};

const E_RATIO: f64 = 0.05;
const E_V_MAX: f64 = 75.0;
const E_FOLLOWER: &[FollowerDemand] = &[FollowerDemand {
    axis_index: 3,
    ratio: E_RATIO,
}];

fn limits() -> temporal::Limits {
    let mut sets: Vec<temporal::LimitSet> =
        temporal::Limits::axis_boxes([500.0; 3], [20_000.0; 3], [400_000.0; 3])
            .sets()
            .to_vec();
    sets.push(temporal::LimitSet {
        axes: temporal::AxisSet::from_indices(&[3]),
        v_max: E_V_MAX,
        a_max: 1500.0,
        j_max: 3000.0,
    });
    temporal::Limits::try_new(&sets, 4).unwrap()
}

fn line(start: [f64; 3], end: [f64; 3]) -> VectorNurbs<f64, 3> {
    VectorNurbs::try_new(1, vec![0.0, 0.0, 1.0, 1.0], vec![start, end]).unwrap()
}

fn solve(pa_k: f64) -> ShapeBatchOutput {
    let a = line([0.0; 3], [60.0, 0.0, 0.0]);
    let b = line([60.0, 0.0, 0.0], [60.0, 60.0, 0.0]);
    let c = line([60.0, 60.0, 0.0], [0.0, 60.0, 0.0]);
    let limits = limits();
    let seg = |curve| ShapeSegmentInput {
        temporal: SegmentInput {
            curve,
            limits,
            followers: &[],
            virtual_path: None,
        },
        followers: E_FOLLOWER,
        feedrate_mm_s: 200.0,
    };
    let segments = [seg(&a), seg(&b), seg(&c)];
    let mut chains = trajectory::AxisChainSet::spatial(
        trajectory::PostProcessorType::SmoothZv { frequency_hz: 40.0 }.into_chain(),
        trajectory::PostProcessorType::SmoothZv { frequency_hz: 40.0 }.into_chain(),
        trajectory::CompiledChain::default(),
    );
    chains.chains.push(trajectory::CompiledChain {
        kernel: None,
        gain: pa_k,
        smooth_time: 0.0,
    });
    chains.followers.push((3, vec![0, 1, 2]));
    let input = ShapeBatchInput {
        segments: &segments,
        chains: &chains,
        follower_start: &[0.0],
        follower_history: None,
        grid_strategy: GridStrategy::Fixed(201),
        worker_threads: 1,
        fit_tolerance_mm: 0.5,
        beta_max_iters: 3,
        beta_convergence_ratio: 1.02,
        initial_v: 0.0,
        initial_a: 0.0,
        terminal_v: 0.0,
        start_d2_override: None,
    };
    shape_batch(&input).expect("follower batch should solve")
}

fn bell_kernel(frequency_hz: f64) -> PiecewisePolynomialKernel<f64> {
    let t_sm = 0.8025 / frequency_hz;
    let h = t_sm / 2.0;
    let c = 15.0 / (16.0 * h.powi(5));
    PiecewisePolynomialKernel::single_poly_from_absolute(
        vec![c * h.powi(4), 0.0, -2.0 * c * h * h, 0.0, c],
        (-h, h),
    )
}

fn eval_kernel(kernel: &PiecewisePolynomialKernel<f64>, z: f64) -> f64 {
    let (lo, hi) = kernel.support();
    if z < lo || z > hi {
        return 0.0;
    }
    kernel
        .pieces
        .iter()
        .find(|p| z >= p.u_start - 1e-15 && z <= p.u_end + 1e-15)
        .map_or(0.0, |p| p.evaluate(z))
}

fn total_duration(out: &ShapeBatchOutput) -> f64 {
    out.segments.last().map_or(0.0, |s| s.t_end)
}

fn axis_velocity_at(out: &ShapeBatchOutput, axis: usize, tau: f64) -> f64 {
    for seg in &out.segments {
        if tau >= seg.t_start - 1e-12 && tau <= seg.t_end + 1e-12 {
            let curve = nurbs::eval::derivative(&seg.axes[axis]);
            let knots = curve.knots();
            let u = tau.clamp(knots[0], knots[knots.len() - 1]);
            return nurbs::eval::eval(&curve, u);
        }
    }
    0.0
}

#[test]
fn follower_batch_shaped_demand_holds_and_pa_only_tightens() {
    let with_pa = solve(0.04);
    let without_pa = solve(0.0);
    let t_pa = total_duration(&with_pa);
    let t_plain = total_duration(&without_pa);
    assert!(
        t_plain <= t_pa + 1e-9,
        "PA rows only ever tighten: plain {t_plain} vs pa {t_pa}"
    );

    let kernel = bell_kernel(40.0);
    let (k_lo, k_hi) = kernel.support();
    let dt = 1e-3;
    let total = t_plain;
    let mut tau = 0.0;
    while tau <= total {
        let mut shaped = [0.0_f64; 2];
        let mut z = k_lo;
        while z <= k_hi {
            let w = eval_kernel(&kernel, z) * dt;
            for (axis, sh) in shaped.iter_mut().enumerate() {
                let src = (tau - z).clamp(0.0, total);
                *sh += w * axis_velocity_at(&without_pa, axis, src);
            }
            z += dt;
        }
        let demand = E_RATIO * (shaped[0] * shaped[0] + shaped[1] * shaped[1]).sqrt();
        assert!(
            demand <= E_V_MAX * (1.0 + 5e-2),
            "t={tau:.4}: shaped follower demand {demand} > {E_V_MAX}"
        );
        tau += 5e-3;
    }
}
