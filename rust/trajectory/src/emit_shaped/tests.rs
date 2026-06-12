use super::*;
use crate::fit::{fit_and_split, FittedSegment};
use crate::post_processor::PostProcessorType;
use crate::{
    plan_velocity, AxisChainSet, PlanInput, PlanSegment, SafetyMode, ShapeBatchInput,
    ShapeSegmentInput,
};
use geometry::segment::FollowerDemand;

const E_FOLLOWER_04: &[FollowerDemand] = &[FollowerDemand {
    axis_index: 3,
    ratio: 0.04,
}];
use nurbs::algebra::PiecewisePolynomialKernel;
use nurbs::bezier::{bezier_pieces_to_nurbs, extract_bezier_pieces, BezierPiece};
use nurbs::VectorNurbs;

fn straight_linear(start: [f64; 3], end: [f64; 3]) -> VectorNurbs<f64, 3> {
    VectorNurbs::try_new(1, vec![0.0, 0.0, 1.0, 1.0], vec![start, end]).unwrap()
}

fn default_limits() -> temporal::Limits {
    let mut sets: Vec<temporal::LimitSet> = temporal::Limits::axis_boxes(
        [500.0, 500.0, 500.0],
        [5_000.0, 5_000.0, 5_000.0],
        [100_000.0, 100_000.0, 100_000.0],
    )
    .sets()
    .to_vec();
    sets.push(temporal::LimitSet {
        axes: temporal::AxisSet::from_indices(&[3]),
        v_max: 75.0,
        a_max: 1500.0,
        j_max: 3000.0,
    });
    temporal::Limits::try_new(&sets, 4).unwrap()
}

fn default_chain_set() -> AxisChainSet {
    AxisChainSet::spatial(
        PostProcessorType::SmoothZv {
            frequency_hz: 180.0,
        }
        .into_chain(),
        PostProcessorType::SmoothZv {
            frequency_hz: 120.0,
        }
        .into_chain(),
        crate::CompiledChain::default(),
    )
}

static DEFAULT_CHAINS: std::sync::LazyLock<crate::AxisChainSet> =
    std::sync::LazyLock::new(default_chain_set);

fn assert_nurbs_near_equal(a: &ScalarNurbs<f64>, b: &ScalarNurbs<f64>, label: &str) {
    assert_eq!(a.degree(), b.degree(), "{label}: degree differs");
    assert_eq!(
        a.knots().len(),
        b.knots().len(),
        "{label}: knot count differs"
    );
    let max_knot_diff = a
        .knots()
        .iter()
        .zip(b.knots().iter())
        .map(|(ka, kb)| (ka - kb).abs())
        .fold(0.0_f64, f64::max);
    assert!(
        max_knot_diff < 1e-12,
        "{label}: knots differ by {max_knot_diff:.2e}"
    );
    assert_eq!(
        a.control_points().len(),
        b.control_points().len(),
        "{label}: control point count differs"
    );
    let max_cp_diff = a
        .control_points()
        .iter()
        .zip(b.control_points().iter())
        .map(|(ca, cb)| (ca - cb).abs())
        .fold(0.0_f64, f64::max);
    assert!(
        max_cp_diff < 1e-12,
        "{label}: control points differ by {max_cp_diff:.2e} mm"
    );
}

#[test]
fn empty_history_matches_shape_batch_byte_identical() {
    let curve = straight_linear([0.0, 0.0, 0.0], [50.0, 0.0, 0.0]);
    let plan_segs = [PlanSegment {
        temporal: temporal::multi::SegmentInput {
            curve: &curve,
            limits: default_limits(),
            followers: &[],
            virtual_path: None,
        },
        followers: E_FOLLOWER_04,
        feedrate_mm_s: 100.0,
    }];

    let plan_input = PlanInput {
        follower_history: None,
        segments: &plan_segs,
        grid_strategy: temporal::multi::GridStrategy::Fixed(10),
        worker_threads: 1,
        chains: &DEFAULT_CHAINS,
        fit_tolerance_mm: 0.5,
        beta_max_iters: 5,
        beta_convergence_ratio: 1.02,
        initial_v: 0.0,
        initial_a: 0.0,
        terminal_v: 0.0,
        safety_mode: SafetyMode::TerminalKnown,
        start_d2_override: None,
    };
    let planned = plan_velocity(&plan_input)
        .expect("plan_velocity should succeed")
        .fitted;
    assert_eq!(planned.len(), 1);

    let kernels: [Option<PiecewisePolynomialKernel<f64>>; 4] = [
        crate::PostProcessorType::SmoothZv {
            frequency_hz: 180.0,
        }
        .into_chain()
        .kernel,
        crate::PostProcessorType::SmoothZv {
            frequency_hz: 120.0,
        }
        .into_chain()
        .kernel,
        None,
        None,
    ];
    let meta = [EmitSegmentMeta {
        followers: E_FOLLOWER_04.to_vec(),
    }];

    let batch_t_start = 0.0;
    let batch_t_end = planned[0].t_end;

    let emitted = emit_shaped(
        &planned,
        &meta,
        &AxisChainSet::spatial_from_kernels(&kernels),
        &PerAxisHistory::empty(),
        &crate::emit_shaped::FollowerAnchor {
            t: batch_t_start,
            values: &[],
        },
        batch_t_start,
        batch_t_end,
    )
    .map(|e| e.segments)
    .expect("emit_shaped should succeed");

    let segs = [ShapeSegmentInput {
        temporal: plan_segs[0].temporal,
        followers: plan_segs[0].followers,
        feedrate_mm_s: plan_segs[0].feedrate_mm_s,
    }];
    let chains = default_chain_set();
    let shape_input = ShapeBatchInput {
        chains: &chains,
        follower_start: &[],
        follower_history: None,
        segments: &segs,
        grid_strategy: temporal::multi::GridStrategy::Fixed(10),
        worker_threads: 1,
        fit_tolerance_mm: 0.5,
        beta_max_iters: 5,
        beta_convergence_ratio: 1.02,
        initial_v: 0.0,
        initial_a: 0.0,
        terminal_v: 0.0,
        start_d2_override: None,
    };
    let reference = crate::shape_batch(&shape_input).expect("shape_batch should succeed");

    assert_eq!(emitted.len(), reference.segments.len());
    for (i, (a, b)) in emitted.iter().zip(reference.segments.iter()).enumerate() {
        assert_nurbs_near_equal(&a.axes[0], &b.axes[0], &format!("seg{i} X"));
        assert_nurbs_near_equal(&a.axes[1], &b.axes[1], &format!("seg{i} Y"));
        assert_nurbs_near_equal(&a.axes[2], &b.axes[2], &format!("seg{i} Z"));
        assert_eq!(a.followers, b.followers, "seg{i}: followers differ");
        #[allow(clippy::float_cmp)]
        {
            assert_eq!(a.t_start, b.t_start, "seg{i}: t_start differs");
            assert_eq!(a.t_end, b.t_end, "seg{i}: t_end differs");
        }
    }
}

#[test]
fn pad_segment_axis_with_history_seam_reads_history_tail() {
    let x_nurbs = bezier_pieces_to_nurbs(&[BezierPiece {
        u_start: 1.0,
        u_end: 2.0,
        coeffs: vec![10.0, 20.0],
    }]);
    let y_nurbs = bezier_pieces_to_nurbs(&[BezierPiece {
        u_start: 1.0,
        u_end: 2.0,
        coeffs: vec![0.0],
    }]);
    let z_nurbs = bezier_pieces_to_nurbs(&[BezierPiece {
        u_start: 1.0,
        u_end: 2.0,
        coeffs: vec![0.0],
    }]);
    let fitted = vec![FittedSegment {
        axes: [x_nurbs, y_nurbs, z_nurbs],
        t_start: 1.0,
        t_end: 2.0,
        virtual_s_of_t: None,
    }];

    let history_x = vec![BezierPiece {
        u_start: 0.0,
        u_end: 1.0,
        coeffs: vec![0.0, 10.0],
    }];

    let t_sm_half = 0.3;
    let padded =
        crate::pad::pad_segment_axis_with_history(0, 0, &fitted, &history_x, t_sm_half, 1.0, 2.0);

    let pieces = extract_bezier_pieces(&padded);
    assert!(
        pieces[0].u_start <= 0.7 + 1e-12,
        "padded must cover at least back to 0.7, got {}",
        pieces[0].u_start,
    );

    let val_08 = pieces
        .iter()
        .find(|p| 0.8 >= p.u_start - 1e-12 && 0.8 <= p.u_end + 1e-12)
        .expect("padded curve should cover t = 0.8")
        .evaluate(0.8);
    assert!(
        (val_08 - 8.0).abs() < 1e-9,
        "expected 8.0 from history at t=0.8, got {val_08}",
    );

    let val_10 = pieces
        .iter()
        .find(|p| 1.0 >= p.u_start - 1e-12 && 1.0 <= p.u_end + 1e-12)
        .expect("padded curve should cover t = 1.0")
        .evaluate(1.0);
    assert!(
        (val_10 - 10.0).abs() < 1e-9,
        "expected 10.0 at seam, got {val_10}",
    );

    let padded_no_history = crate::pad::pad_segment_axis(0, 0, &fitted, t_sm_half, 1.0, 2.0);
    let pieces_no_history = extract_bezier_pieces(&padded_no_history);
    let val_08_no_history = pieces_no_history
        .iter()
        .find(|p| 0.8 >= p.u_start - 1e-12 && 0.8 <= p.u_end + 1e-12)
        .expect("padded curve should cover t = 0.8")
        .evaluate(0.8);
    // With no history the left pad continues at the segment's entry velocity (slope 20 through
    // position 10 at the t=1.0 seam) rather than holding the start position: at t=0.8 that is
    // 10 + 20*(0.8 - 1.0) = 6.0.
    assert!(
        (val_08_no_history - 6.0).abs() < 1e-9,
        "no-history path should extrapolate at entry velocity to 6.0 at t=0.8, got {val_08_no_history}",
    );
    assert!(
        (val_08 - val_08_no_history).abs() > 1.0,
        "history vs no-history must disagree at t=0.8 (history 8.0 vs vel-extrapolated 6.0)",
    );
}

#[test]
fn empty_history_pad_matches_legacy() {
    let x_nurbs = bezier_pieces_to_nurbs(&[BezierPiece {
        u_start: 0.0,
        u_end: 1.0,
        coeffs: vec![0.0, 10.0],
    }]);
    let y_nurbs = bezier_pieces_to_nurbs(&[BezierPiece {
        u_start: 0.0,
        u_end: 1.0,
        coeffs: vec![0.0],
    }]);
    let z_nurbs = bezier_pieces_to_nurbs(&[BezierPiece {
        u_start: 0.0,
        u_end: 1.0,
        coeffs: vec![0.0],
    }]);
    let fitted = vec![FittedSegment {
        axes: [x_nurbs, y_nurbs, z_nurbs],
        t_start: 0.0,
        t_end: 1.0,
        virtual_s_of_t: None,
    }];

    let t_sm_half = 0.1;
    for axis in 0..3 {
        let with_history =
            crate::pad::pad_segment_axis_with_history(0, axis, &fitted, &[], t_sm_half, 0.0, 1.0);
        let legacy = crate::pad::pad_segment_axis(0, axis, &fitted, t_sm_half, 0.0, 1.0);
        assert_nurbs_near_equal(&with_history, &legacy, &format!("axis {axis}"));
    }
}

#[test]
fn constant_y_axis_emits_cubic_matching_moving_x_corexy_degree_invariant() {
    // Regression: when the fitter produces a degree-5 FittedSegment and Y is
    // bitwise-constant, emit_shaped returned the fitter's native degree-5 curve
    // for Y while fitting X to degree-3 via fit_c2_cubic. The degree mismatch
    // caused add_with_knot_union to return KnotMismatch and panicked at
    // motion-bridge/src/enqueue.rs:30 on any CoreXY dispatch.
    //
    // Trigger condition: pure-X jogs queued back-to-back while the first is
    // in flight; the terminal-decel splice rebuilds Y as bitwise-constant.

    let x_composed: Vec<[BezierPiece<f64>; 3]> = (0..4)
        .map(|i| {
            let s = f64::from(i);
            [
                BezierPiece {
                    u_start: s,
                    u_end: s + 1.0,
                    coeffs: vec![s * 10.0, 10.0, 0.5, 0.1],
                },
                BezierPiece {
                    u_start: s,
                    u_end: s + 1.0,
                    coeffs: vec![0.0],
                },
                BezierPiece {
                    u_start: s,
                    u_end: s + 1.0,
                    coeffs: vec![0.0],
                },
            ]
        })
        .collect();

    let fitted_from_fitter =
        fit_and_split(&x_composed, 0.005, None).expect("fit_and_split must succeed");

    let degree5_x = &fitted_from_fitter.axes[0];
    assert!(
        degree5_x.degree() >= 4,
        "expected fitter to produce degree >= 4 for X, got {}",
        degree5_x.degree(),
    );

    let y_constant_val = 25.0_f64;
    let degree5_constant_y = bezier_pieces_to_nurbs(&[BezierPiece {
        u_start: fitted_from_fitter.t_start,
        u_end: fitted_from_fitter.t_end,
        coeffs: vec![y_constant_val, 0.0, 0.0, 0.0, 0.0, 0.0],
    }]);
    assert_eq!(
        degree5_constant_y.degree(),
        degree5_x.degree(),
        "test setup: Y must be same degree as X to match the live crash precondition",
    );

    let constant_cps = degree5_constant_y.control_points();
    let all_equal = constant_cps
        .iter()
        .all(|c| (c - constant_cps[0]).abs() < 1e-12);
    assert!(
        all_equal,
        "test setup: Y control points must be bitwise-constant to trigger the bug branch",
    );

    let constant_z = bezier_pieces_to_nurbs(&[BezierPiece {
        u_start: fitted_from_fitter.t_start,
        u_end: fitted_from_fitter.t_end,
        coeffs: vec![0.0; degree5_x.degree() as usize + 1],
    }]);

    let fitted = FittedSegment {
        axes: [degree5_x.clone(), degree5_constant_y, constant_z],
        t_start: fitted_from_fitter.t_start,
        t_end: fitted_from_fitter.t_end,
        virtual_s_of_t: None,
    };

    let kernels: [Option<PiecewisePolynomialKernel<f64>>; 4] = [
        crate::PostProcessorType::SmoothZv {
            frequency_hz: 186.0,
        }
        .into_chain()
        .kernel,
        None,
        None,
        None,
    ];
    let meta = [EmitSegmentMeta {
        followers: E_FOLLOWER_04.to_vec(),
    }];

    let emitted = emit_shaped(
        &[fitted],
        &meta,
        &AxisChainSet::spatial_from_kernels(&kernels),
        &PerAxisHistory::empty(),
        &crate::emit_shaped::FollowerAnchor {
            t: fitted_from_fitter.t_start,
            values: &[],
        },
        fitted_from_fitter.t_start,
        fitted_from_fitter.t_end,
    )
    .map(|e| e.segments)
    .expect("emit_shaped must not return an error");

    for (i, seg) in emitted.iter().enumerate() {
        assert_eq!(
            seg.axes[0].degree(),
            seg.axes[1].degree(),
            "segment {i}: X degree {} != Y degree {} — CoreXY motor-union \
             add_with_knot_union will panic with KnotMismatch (constant-Y \
             axis must be refit to cubic, not returned as-is from the fitter)",
            seg.axes[0].degree(),
            seg.axes[1].degree(),
        );
    }
}

fn golden_fixture_segments() -> Vec<FittedSegment> {
    let lin = |p0: f64, p1: f64, t0: f64, t1: f64| {
        nurbs::ScalarNurbs::try_new(
            3,
            vec![t0, t0, t0, t0, t1, t1, t1, t1],
            vec![p0, p0 + (p1 - p0) / 3.0, p0 + 2.0 * (p1 - p0) / 3.0, p1],
        )
        .unwrap()
    };
    let curved = |p0: f64, p1: f64, p2: f64, p3: f64, t0: f64, t1: f64| {
        nurbs::ScalarNurbs::try_new(
            3,
            vec![t0, t0, t0, t0, t1, t1, t1, t1],
            vec![p0, p1, p2, p3],
        )
        .unwrap()
    };
    vec![
        FittedSegment {
            axes: [
                lin(0.0, 30.0, 0.0, 1.0),
                curved(0.0, 2.0, 8.0, 10.0, 0.0, 1.0),
                lin(5.0, 5.0, 0.0, 1.0),
            ],
            t_start: 0.0,
            t_end: 1.0,
            virtual_s_of_t: None,
        },
        FittedSegment {
            axes: [
                curved(30.0, 40.0, 42.0, 45.0, 1.0, 2.0),
                lin(10.0, 25.0, 1.0, 2.0),
                lin(5.0, 5.0, 1.0, 2.0),
            ],
            t_start: 1.0,
            t_end: 2.0,
            virtual_s_of_t: None,
        },
        FittedSegment {
            axes: [
                lin(45.0, 50.0, 2.0, 2.5),
                lin(25.0, 25.0, 2.0, 2.5),
                lin(5.0, 6.0, 2.0, 2.5),
            ],
            t_start: 2.0,
            t_end: 2.5,
            virtual_s_of_t: None,
        },
    ]
}

#[test]
fn passthrough_chains_reproduce_legacy_output_bitwise() {
    let planned = golden_fixture_segments();
    let meta: Vec<EmitSegmentMeta> = (0..3)
        .map(|_| EmitSegmentMeta { followers: vec![] })
        .collect();
    let kernels: [Option<PiecewisePolynomialKernel<f64>>; 4] = [
        crate::PostProcessorType::SmoothZv { frequency_hz: 50.0 }
            .into_chain()
            .kernel,
        crate::PostProcessorType::SmoothMzv { frequency_hz: 40.0 }
            .into_chain()
            .kernel,
        None,
        None,
    ];
    let out = emit_shaped(
        &planned,
        &meta,
        &AxisChainSet::spatial_from_kernels(&kernels),
        &PerAxisHistory::empty(),
        &crate::emit_shaped::FollowerAnchor {
            t: 0.0,
            values: &[],
        },
        0.0,
        2.5,
    )
    .unwrap()
    .segments;

    let golden = include_str!("golden_passthrough_capture.txt");
    for (i, seg) in out.iter().enumerate() {
        assert_eq!(seg.axes.len(), 3);
        for (ax, curve) in seg.axes.iter().enumerate() {
            let cps: Vec<u64> = curve.control_points().iter().map(|c| c.to_bits()).collect();
            let knots: Vec<u64> = curve.knots().iter().map(|k| k.to_bits()).collect();
            let want_cps = golden_line(golden, &format!("SEG{i} AX{ax} CPS "));
            let want_knots = golden_line(golden, &format!("SEG{i} AX{ax} KNOTS "));
            assert_eq!(
                format!("{cps:?}"),
                want_cps,
                "seg {i} axis {ax}: control points diverged from pre-refactor capture"
            );
            assert_eq!(
                format!("{knots:?}"),
                want_knots,
                "seg {i} axis {ax}: knots diverged from pre-refactor capture"
            );
        }
    }
}

fn golden_line(golden: &str, prefix: &str) -> String {
    golden
        .lines()
        .find_map(|l| l.strip_prefix(prefix))
        .unwrap_or_else(|| panic!("golden capture missing line with prefix '{prefix}'"))
        .to_string()
}

fn e_follower_chains(
    gain: f64,
    kernels: &[Option<PiecewisePolynomialKernel<f64>>; 4],
) -> AxisChainSet {
    let mut chains = AxisChainSet::spatial_from_kernels(kernels);
    chains
        .chains
        .push(crate::CompiledChain { kernel: None, gain });
    chains.followers.push((3, vec![0, 1, 2]));
    chains
}

#[test]
fn follower_track_integral_matches_ratio_times_arclength() {
    let planned = golden_fixture_segments();
    let ratio = 0.05;
    let meta: Vec<EmitSegmentMeta> = (0..3)
        .map(|_| EmitSegmentMeta {
            followers: vec![FollowerDemand {
                axis_index: 3,
                ratio,
            }],
        })
        .collect();
    let chains = e_follower_chains(0.0, &[None, None, None, None]);
    let start = 7.0;
    let out = emit_shaped(
        &planned,
        &meta,
        &chains,
        &PerAxisHistory::empty(),
        &crate::emit_shaped::FollowerAnchor {
            t: 0.0,
            values: &[start],
        },
        0.0,
        2.5,
    )
    .unwrap()
    .segments;

    let spatial: Vec<ScalarNurbs<f64>> = (0..3)
        .map(|ax| {
            let pieces: Vec<BezierPiece<f64>> = out
                .iter()
                .flat_map(|seg| extract_bezier_pieces(&seg.axes[ax]))
                .collect();
            bezier_pieces_to_nurbs(&pieces)
        })
        .collect();
    let odo = crate::odometer::Odometer::build(&spatial, 0.0, 2.5, 64).unwrap();
    let expected_end = start + ratio * odo.distance_at(2.5);

    let last = out.last().unwrap();
    let got_end = nurbs::eval::eval(&last.axes[3], 2.5);
    assert!(
        (got_end - expected_end).abs() < 1e-6,
        "follower end {got_end} != start + ratio·arclength {expected_end}"
    );

    for w in out.windows(2) {
        let t = w[0].t_end;
        let left = nurbs::eval::eval(&w[0].axes[3], t);
        let right = nurbs::eval::eval(&w[1].axes[3], t);
        assert!(
            (left - right).abs() < 1e-6,
            "follower position jump at seam t={t}: {left} vs {right}"
        );
        let dl = nurbs::eval::derivative(&w[0].axes[3]);
        let dr = nurbs::eval::derivative(&w[1].axes[3]);
        let vl = nurbs::eval::eval(&dl, t);
        let vr = nurbs::eval::eval(&dr, t);
        assert!(
            (vl - vr).abs() < 1e-6,
            "follower velocity jump at seam t={t}: {vl} vs {vr}"
        );
    }
}

fn corner_segments() -> Vec<FittedSegment> {
    let lin = |p0: f64, p1: f64, t0: f64, t1: f64| {
        nurbs::ScalarNurbs::try_new(
            3,
            vec![t0, t0, t0, t0, t1, t1, t1, t1],
            vec![p0, p0 + (p1 - p0) / 3.0, p0 + 2.0 * (p1 - p0) / 3.0, p1],
        )
        .unwrap()
    };
    vec![
        FittedSegment {
            axes: [
                lin(0.0, 40.0, 0.0, 1.0),
                lin(0.0, 0.0, 0.0, 1.0),
                lin(0.0, 0.0, 0.0, 1.0),
            ],
            t_start: 0.0,
            t_end: 1.0,
            virtual_s_of_t: None,
        },
        FittedSegment {
            axes: [
                lin(40.0, 40.0, 1.0, 2.0),
                lin(0.0, 40.0, 1.0, 2.0),
                lin(0.0, 0.0, 1.0, 2.0),
            ],
            t_start: 1.0,
            t_end: 2.0,
            virtual_s_of_t: None,
        },
    ]
}

#[test]
fn follower_samples_post_kernel_path() {
    let ratio = 0.05;
    let meta: Vec<EmitSegmentMeta> = (0..2)
        .map(|_| EmitSegmentMeta {
            followers: vec![FollowerDemand {
                axis_index: 3,
                ratio,
            }],
        })
        .collect();
    let kernels: [Option<PiecewisePolynomialKernel<f64>>; 4] = [
        crate::PostProcessorType::SmoothMzv { frequency_hz: 10.0 }
            .into_chain()
            .kernel,
        crate::PostProcessorType::SmoothMzv { frequency_hz: 10.0 }
            .into_chain()
            .kernel,
        None,
        None,
    ];

    let corner = corner_segments();
    let shaped_out = emit_shaped(
        &corner,
        &meta,
        &e_follower_chains(0.0, &kernels),
        &PerAxisHistory::empty(),
        &crate::emit_shaped::FollowerAnchor {
            t: 0.0,
            values: &[0.0],
        },
        0.0,
        2.0,
    )
    .unwrap()
    .segments;
    let nominal_length = 80.0;
    let shaped_end = nurbs::eval::eval(&shaped_out.last().unwrap().axes[3], 2.0);
    assert!(
        shaped_end < ratio * nominal_length - 1e-3,
        "kernel shortcuts the corner: follower end {shaped_end} must fall short \
         of ratio·nominal {}",
        ratio * nominal_length
    );

    let straight = golden_fixture_segments();
    let straight_meta: Vec<EmitSegmentMeta> = (0..3)
        .map(|_| EmitSegmentMeta {
            followers: vec![FollowerDemand {
                axis_index: 3,
                ratio,
            }],
        })
        .collect();
    let passthrough_out = emit_shaped(
        &straight,
        &straight_meta,
        &e_follower_chains(0.0, &[None, None, None, None]),
        &PerAxisHistory::empty(),
        &crate::emit_shaped::FollowerAnchor {
            t: 0.0,
            values: &[0.0],
        },
        0.0,
        2.5,
    )
    .unwrap()
    .segments;
    let spatial: Vec<ScalarNurbs<f64>> = (0..3)
        .map(|ax| {
            let pieces: Vec<BezierPiece<f64>> = passthrough_out
                .iter()
                .flat_map(|seg| extract_bezier_pieces(&seg.axes[ax]))
                .collect();
            bezier_pieces_to_nurbs(&pieces)
        })
        .collect();
    let odo = crate::odometer::Odometer::build(&spatial, 0.0, 2.5, 64).unwrap();
    let got = nurbs::eval::eval(&passthrough_out.last().unwrap().axes[3], 2.5);
    assert!(
        (got - ratio * odo.distance_at(2.5)).abs() < 1e-6,
        "passthrough follower must pay out exactly ratio·realized length"
    );
}

#[test]
fn pa_gain_boosts_follower_during_accel() {
    let accel = 20.0;
    let quad_x = bezier_pieces_to_nurbs(&[BezierPiece {
        u_start: 0.0,
        u_end: 1.0,
        coeffs: vec![0.0, 0.0, accel / 2.0, 0.0],
    }]);
    let lin0 = crate::beta::constant_cubic_nurbs(0.0, 0.0, 1.0);
    let planned = vec![FittedSegment {
        axes: [quad_x, lin0.clone(), lin0],
        t_start: 0.0,
        t_end: 1.0,
        virtual_s_of_t: None,
    }];
    let ratio = 0.05;
    let k = 0.04;
    let meta = vec![EmitSegmentMeta {
        followers: vec![FollowerDemand {
            axis_index: 3,
            ratio,
        }],
    }];
    let out = emit_shaped(
        &planned,
        &meta,
        &e_follower_chains(k, &[None, None, None, None]),
        &PerAxisHistory::empty(),
        &crate::emit_shaped::FollowerAnchor {
            t: 0.0,
            values: &[0.0],
        },
        0.0,
        1.0,
    )
    .unwrap()
    .segments;

    let track = &out[0].axes[3];
    let t = 0.5;
    let s_dot = accel * t;
    let s_ddot = accel;
    let expected_v = ratio * s_dot + k * ratio * s_ddot;
    let d1 = nurbs::eval::derivative(track);
    let got_v = nurbs::eval::eval(&d1, t);
    assert!(
        ((got_v - expected_v) / expected_v).abs() < 1e-3,
        "PA-boosted follower velocity at mid-accel: got {got_v}, want {expected_v}"
    );
}

#[test]
fn follower_only_move_emits_planned_track() {
    let length = 4.0;
    let s_of_t = bezier_pieces_to_nurbs(&[BezierPiece {
        u_start: 0.0,
        u_end: 1.0,
        coeffs: vec![0.0, 0.0, length, 0.0],
    }]);
    let const_axis = crate::beta::constant_cubic_nurbs(12.0, 0.0, 1.0);
    let planned = vec![FittedSegment {
        axes: [const_axis.clone(), const_axis.clone(), const_axis],
        t_start: 0.0,
        t_end: 1.0,
        virtual_s_of_t: Some(s_of_t.clone()),
    }];
    let ratio = -1.0;
    let start = 9.0;
    let meta = vec![EmitSegmentMeta {
        followers: vec![FollowerDemand {
            axis_index: 3,
            ratio,
        }],
    }];
    let out = emit_shaped(
        &planned,
        &meta,
        &e_follower_chains(0.0, &[None, None, None, None]),
        &PerAxisHistory::empty(),
        &crate::emit_shaped::FollowerAnchor {
            t: 0.0,
            values: &[start],
        },
        0.0,
        1.0,
    )
    .unwrap()
    .segments;

    for ax in 0..3 {
        assert!(
            (nurbs::eval::eval(&out[0].axes[ax], 0.7) - 12.0).abs() < 1e-9,
            "spatial axis {ax} must stay parked during a follower-only move"
        );
    }
    let track = &out[0].axes[3];
    for t in [0.0, 0.3, 0.6, 1.0] {
        let want = start + ratio * nurbs::eval::eval(&s_of_t, t);
        let got = nurbs::eval::eval(track, t);
        assert!(
            (got - want).abs() < 1e-4,
            "follower-only track at t={t}: got {got}, want {want}"
        );
    }
}
