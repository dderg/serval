//! Regression: smooth curved segments plan successfully and track the exact
//! curve; genuine cusps still fail loud. Guards the fix for the planner_fatal
//! abort on curved (G5) segments.

use nurbs::VectorNurbs;
use temporal::multi::{GridStrategy, SegmentInput};
use trajectory::{AxisChainSet, ShapeBatchInput, ShapeError, ShapeSegmentInput};

fn cubic(p: [[f64; 3]; 4]) -> VectorNurbs<f64, 3> {
    VectorNurbs::try_new(3, vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0], p.to_vec()).unwrap()
}

fn limits() -> temporal::Limits {
    let sets = temporal::Limits::axis_boxes([500.0; 3], [5_000.0; 3], [100_000.0; 3])
        .sets()
        .to_vec();
    temporal::Limits::try_new(&sets, 3).unwrap()
}

fn run(curve: &VectorNurbs<f64, 3>, feed: f64) -> Result<trajectory::ShapeBatchOutput, ShapeError> {
    let segments = [ShapeSegmentInput {
        temporal: SegmentInput {
            curve,
            limits: limits(),
            followers: &[],
            virtual_path: None,
        },
        followers: &[],
        feedrate_mm_s: feed,
    }];
    let chains = AxisChainSet::spatial(
        trajectory::CompiledChain::default(),
        trajectory::CompiledChain::default(),
        trajectory::CompiledChain::default(),
    );
    let input = ShapeBatchInput {
        chains: &chains,
        follower_start: &[],
        follower_history: None,
        segments: &segments,
        grid_strategy: GridStrategy::Adaptive {
            min_n: 20,
            max_n: 200,
            target_grid_spacing_mm: 0.5,
        },
        worker_threads: 1,
        fit_tolerance_mm: 0.005,
        beta_max_iters: 5,
        beta_convergence_ratio: 1.02,
        initial_v: 0.0,
        initial_a: 0.0,
        terminal_v: 0.0,
        start_d2_override: None,
    };
    trajectory::shape_batch(&input)
}

const ARCHES: &[(&str, [[f64; 3]; 4])] = &[
    (
        "gentle",
        [
            [150., 150., 5.],
            [150., 180., 5.],
            [200., 180., 5.],
            [200., 150., 5.],
        ],
    ),
    (
        "wide",
        [
            [100., 100., 5.],
            [100., 200., 5.],
            [250., 200., 5.],
            [250., 100., 5.],
        ],
    ),
    (
        "tight_loop",
        [
            [150., 150., 5.],
            [165., 210., 5.],
            [135., 210., 5.],
            [150., 150.5, 5.],
        ],
    ),
    (
        "s_curve",
        [
            [100., 150., 5.],
            [160., 150., 5.],
            [140., 150., 5.],
            [200., 150., 5.],
        ],
    ),
];

#[test]
fn smooth_arches_plan_successfully() {
    for (name, cps) in ARCHES {
        let curve = cubic(*cps);
        for &feed in &[25.0_f64, 50.0, 100.0] {
            let r = run(&curve, feed);
            assert!(r.is_ok(), "{name} @ {feed}mm/s should plan, got {r:?}");
        }
    }
}

#[test]
fn planned_curve_tracks_geometry_and_joints_exact() {
    use nurbs::eval::{eval as nurbs_scalar_eval, vector_eval};

    for (name, cps) in &[ARCHES[0], ARCHES[2]] {
        let curve = cubic(*cps);
        let out = run(&curve, 25.0).expect("plan ok");
        let seg = &out.segments[0];

        let poly: Vec<[f64; 3]> = (0..=5000)
            .map(|i| vector_eval(&curve, i as f64 / 5000.0))
            .collect();
        let nearest = |p: [f64; 3]| {
            poly.iter()
                .map(|q| {
                    ((p[0] - q[0]).powi(2) + (p[1] - q[1]).powi(2) + (p[2] - q[2]).powi(2)).sqrt()
                })
                .fold(f64::INFINITY, f64::min)
        };

        let (t0, t1) = (seg.t_start, seg.t_end);
        assert!(t1.is_finite() && t1 > t0);
        let mut max_dev = 0.0_f64;
        for i in 0..=200 {
            let t = t0 + (t1 - t0) * (i as f64 / 200.0);
            let p = [
                nurbs_scalar_eval(&seg.axes[0], t),
                nurbs_scalar_eval(&seg.axes[1], t),
                nurbs_scalar_eval(&seg.axes[2], t),
            ];
            max_dev = max_dev.max(nearest(p));
        }
        assert!(max_dev < 0.025, "{name} max geom deviation {max_dev} mm");

        let start = [
            nurbs_scalar_eval(&seg.axes[0], t0),
            nurbs_scalar_eval(&seg.axes[1], t0),
            nurbs_scalar_eval(&seg.axes[2], t0),
        ];
        let end = [
            nurbs_scalar_eval(&seg.axes[0], t1),
            nurbs_scalar_eval(&seg.axes[1], t1),
            nurbs_scalar_eval(&seg.axes[2], t1),
        ];
        assert!(
            nearest(start) < 1e-4 && nearest(end) < 1e-4,
            "{name} joints off curve"
        );
    }
}

#[test]
fn exact_cusp_still_fails_loud() {
    let curve = cubic([[0., 0., 0.], [0., 0., 0.], [0., 0., 0.], [5., 0., 0.]]);
    let r = run(&curve, 30.0);
    assert!(r.is_err(), "cusp must fail loud, got {r:?}");
}
