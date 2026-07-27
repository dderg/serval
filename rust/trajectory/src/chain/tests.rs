use super::*;
use crate::algos::{
    LinearPressureAdvance, ModeInverse, ReciprPressureAdvance, SmoothBell, SmoothTriangle,
    TanhPressureAdvance,
};
use crate::kernel::build_smooth_bell_kernel;

fn pa(k: f64) -> PostProcessorInstance {
    PostProcessorInstance::new("pa", &LinearPressureAdvance, vec![k])
}
fn mi(frequency_hz: f64, damping_ratio: f64) -> PostProcessorInstance {
    PostProcessorInstance::new("belt", &ModeInverse, vec![frequency_hz, damping_ratio])
}
fn bell(smooth_time: f64) -> PostProcessorInstance {
    PostProcessorInstance::new("is", &SmoothBell, vec![smooth_time])
}
fn st(smooth_time: f64) -> PostProcessorInstance {
    PostProcessorInstance::new("st", &SmoothTriangle, vec![smooth_time])
}
fn nlpa(
    algo: &'static dyn crate::algos::PostProcessorAlgo,
    linear_advance: f64,
    nonlinear_offset: f64,
    linearization_velocity: f64,
) -> PostProcessorInstance {
    PostProcessorInstance::new(
        "nlpa",
        algo,
        vec![linear_advance, nonlinear_offset, linearization_velocity],
    )
}

#[test]
fn nonlinear_pa_types_compile_to_their_own_model() {
    for (algo, model) in [
        (
            &TanhPressureAdvance as &'static dyn crate::algos::PostProcessorAlgo,
            AdvanceModel::Tanh,
        ),
        (&ReciprPressureAdvance, AdvanceModel::Reciprocal),
    ] {
        let c = CompiledChain::compile(&[nlpa(algo, 0.02, 0.05, 20.0)]).unwrap();
        let ChainStage::NonlinearAdvance(adv) = c.stages[0] else {
            panic!("{} must compile to an advance stage", algo.type_name());
        };
        assert_eq!(
            adv,
            NonlinearAdvance {
                model,
                linear_advance: 0.02,
                nonlinear_offset: 0.05,
                linearization_velocity: 20.0,
            }
        );
        assert_eq!(c.max_input_window(), (0.0, 0.0));
    }
}

#[test]
fn nonlinear_pa_without_an_offset_is_the_linear_operator() {
    let c = CompiledChain::compile(&[nlpa(&TanhPressureAdvance, 0.02, 0.0, 20.0)]).unwrap();
    assert!(matches!(
        c.stages[0],
        ChainStage::DerivativeGains { k1, k2: 0.0 } if k1 == 0.02
    ));
}

#[test]
fn nonlinear_pa_with_no_advance_at_all_is_a_no_op() {
    let c = CompiledChain::compile(&[nlpa(&ReciprPressureAdvance, 0.0, 0.0, 20.0)]).unwrap();
    assert!(c.stages.is_empty());
}

#[test]
fn nonlinear_pa_rejects_a_zero_linearization_velocity() {
    assert!(nlpa(&TanhPressureAdvance, 0.02, 0.05, 0.0)
        .validate()
        .is_err());
}

#[test]
fn nonlinear_pa_occupies_the_single_gain_slot() {
    let err = CompiledChain::compile(&[nlpa(&TanhPressureAdvance, 0.02, 0.05, 20.0), pa(0.04)])
        .unwrap_err();
    assert!(matches!(
        err,
        PostProcessorError::UnsupportedComposition { .. }
    ));
}

/// Both shapes are odd, saturate at `nonlinear_offset`, and start with the
/// same small-signal slope; the reciprocal one rises toward the bound far
/// more slowly, so it commands less advance everywhere past rest.
#[test]
fn the_two_shapes_share_a_slope_at_rest_and_a_bound_at_speed() {
    let advance = |model| NonlinearAdvance {
        model,
        linear_advance: 0.0,
        nonlinear_offset: 0.05,
        linearization_velocity: 2.0,
    };
    let (tanh, recipr) = (
        advance(AdvanceModel::Tanh),
        advance(AdvanceModel::Reciprocal),
    );
    assert!((tanh.slope(0.0) - recipr.slope(0.0)).abs() < 1e-12);
    assert!((tanh.slope(0.0) - 0.025).abs() < 1e-12);
    for adv in [tanh, recipr] {
        let far = adv.advance(1e6);
        assert!(
            (0.0499..=0.05).contains(&far),
            "must saturate at 0.05: {far}"
        );
        assert!((adv.advance(-3.0) + adv.advance(3.0)).abs() < 1e-12);
    }
    assert!(
        recipr.advance(4.0) < 0.8 * tanh.advance(4.0),
        "the reciprocal shape must lag tanh well before saturation: {} vs {}",
        recipr.advance(4.0),
        tanh.advance(4.0)
    );
    for v in [-9.0, -0.4, 0.4, 9.0] {
        let h = 1e-6;
        for adv in [tanh, recipr] {
            let slope = (adv.advance(v + h) - adv.advance(v - h)) / (2.0 * h);
            let curvature = (adv.slope(v + h) - adv.slope(v - h)) / (2.0 * h);
            assert!((adv.slope(v) - slope).abs() < 1e-6, "slope at {v}");
            assert!(
                (adv.curvature(v) - curvature).abs() < 1e-5,
                "curvature at {v}"
            );
        }
    }
}

#[test]
fn compile_empty_chain_is_identity() {
    let c = CompiledChain::compile(&[]).unwrap();
    assert!(c.stages.is_empty());
}

#[test]
fn compile_kernel_plus_gain() {
    let c = CompiledChain::compile(&[bell(0.01605), pa(0.04)]).unwrap();
    assert!(matches!(c.stages[0], ChainStage::SmoothKernel(_)));
    assert!(matches!(
        c.stages[1],
        ChainStage::DerivativeGains { k1, k2: 0.0 } if k1 == 0.04
    ));
}

#[test]
fn compile_preserves_declaration_order() {
    let a = CompiledChain::compile(&[bell(0.01605), pa(0.04)]).unwrap();
    let b = CompiledChain::compile(&[pa(0.04), bell(0.01605)]).unwrap();
    assert!(matches!(a.stages[0], ChainStage::SmoothKernel(_)));
    assert!(matches!(a.stages[1], ChainStage::DerivativeGains { .. }));
    assert!(matches!(b.stages[0], ChainStage::DerivativeGains { .. }));
    assert!(matches!(b.stages[1], ChainStage::SmoothKernel(_)));
}

#[test]
fn compile_smooth_triangle_plus_gain() {
    let c = CompiledChain::compile(&[st(0.04), pa(0.04)]).unwrap();
    assert!(matches!(c.stages[0], ChainStage::SmoothKernel(_)));
    assert!(matches!(
        c.stages[1],
        ChainStage::DerivativeGains { k1, k2: 0.0 } if k1 == 0.04
    ));
    let (lo, hi) = c.max_input_window();
    assert!((hi - 0.02).abs() < 1e-12 && (lo + 0.02).abs() < 1e-12);
}

#[test]
fn compile_gain_before_smooth_triangle_preserves_order() {
    let c = CompiledChain::compile(&[pa(0.04), st(0.04)]).unwrap();
    assert!(matches!(c.stages[0], ChainStage::DerivativeGains { .. }));
    assert!(matches!(c.stages[1], ChainStage::SmoothKernel(_)));
}

#[test]
fn compile_two_kernels_rejected() {
    let err = CompiledChain::compile(&[bell(0.01605), bell(0.0200625)]).unwrap_err();
    assert!(matches!(
        err,
        PostProcessorError::UnsupportedComposition { .. }
    ));
}

#[test]
fn compile_smooth_triangle_and_input_shaper_rejected_as_two_kernels() {
    let err = CompiledChain::compile(&[bell(0.01605), st(0.04)]).unwrap_err();
    assert!(matches!(
        err,
        PostProcessorError::UnsupportedComposition { .. }
    ));
}

#[test]
fn compile_zero_smooth_time_is_passthrough() {
    let c = CompiledChain::compile(&[st(0.0)]).unwrap();
    assert!(
        c.stages.is_empty(),
        "smooth_time=0 must contribute no stage"
    );
    assert_eq!(c.max_input_window(), (0.0, 0.0));
}

#[test]
fn compile_zero_smooth_time_leaves_only_the_gain() {
    let c = CompiledChain::compile(&[st(0.0), pa(0.04)]).unwrap();
    assert_eq!(c.stages.len(), 1);
    assert!(matches!(
        c.stages[0],
        ChainStage::DerivativeGains { k1, k2: 0.0 } if k1 == 0.04
    ));
}

#[test]
fn compile_disabled_smooth_triangle_does_not_conflict_with_input_shaper() {
    let c = CompiledChain::compile(&[bell(0.01605), st(0.0)]).unwrap();
    assert_eq!(c.stages.len(), 1);
    assert!(matches!(c.stages[0], ChainStage::SmoothKernel(_)));
}

#[test]
fn compile_rejects_negative_or_non_finite_smooth_time() {
    for bad in [-0.01, f64::NAN, f64::INFINITY] {
        let err = CompiledChain::compile(&[st(bad)]).unwrap_err();
        assert!(
            matches!(err, PostProcessorError::BadParam { .. }),
            "smooth_time={bad} should be rejected"
        );
    }
}

#[test]
fn compile_two_gains_rejected() {
    let err = CompiledChain::compile(&[pa(0.04), pa(0.01)]).unwrap_err();
    assert!(matches!(
        err,
        PostProcessorError::UnsupportedComposition { .. }
    ));
}

#[test]
fn set_param_updates_gain() {
    let mut inst = pa(0.04);
    inst.set_param("k", 0.06).unwrap();
    let c = CompiledChain::compile(std::slice::from_ref(&inst)).unwrap();
    assert!(matches!(c.stages[0], ChainStage::DerivativeGains { k1, k2: 0.0 } if k1 == 0.06));
}

#[test]
fn set_param_unknown_key_fails() {
    let mut inst = bell(0.01605);
    assert!(inst.set_param("k", 1.0).is_err());
}

#[test]
fn set_param_rejects_negative_and_non_finite_gain() {
    let mut inst = pa(0.04);
    for bad in [-0.01, f64::NAN, f64::INFINITY] {
        assert!(
            matches!(
                inst.set_param("k", bad),
                Err(PostProcessorError::BadParam { .. })
            ),
            "k={bad} should be rejected"
        );
    }
    let c = CompiledChain::compile(std::slice::from_ref(&inst)).unwrap();
    assert!(
        matches!(c.stages[0], ChainStage::DerivativeGains { k1, k2: 0.0 } if k1 == 0.04),
        "rejected updates must not mutate the gain"
    );
    inst.set_param("k", 0.0).expect("k=0 is a valid no-op gain");
}

#[test]
fn set_param_rejects_negative_and_non_finite_smooth_time() {
    let mut inst = bell(0.01605);
    for bad in [-1.0, f64::NAN, f64::INFINITY] {
        assert!(
            matches!(
                inst.set_param("smooth_time", bad),
                Err(PostProcessorError::BadParam { .. })
            ),
            "smooth_time={bad} should be rejected"
        );
    }
    let c = CompiledChain::compile(std::slice::from_ref(&inst)).unwrap();
    let ChainStage::SmoothKernel(kernel) = &c.stages[0] else {
        panic!("expected smooth kernel stage");
    };
    assert_eq!(
        kernel.support(),
        build_smooth_bell_kernel(0.01605).support()
    );
}

#[test]
fn compile_rejects_directly_constructed_bad_params() {
    for bad in [-1.0, f64::NAN, f64::INFINITY] {
        let err = CompiledChain::compile(&[bell(bad)]).unwrap_err();
        assert!(matches!(err, PostProcessorError::BadParam { .. }));
    }
    for bad in [-0.01, f64::NAN, f64::INFINITY] {
        let err = CompiledChain::compile(&[pa(bad)]).unwrap_err();
        assert!(matches!(err, PostProcessorError::BadParam { .. }));
    }
}

#[test]
fn compile_mode_inverse_after_kernel_produces_the_inversion_gains() {
    let c = CompiledChain::compile(&[bell(0.0015), mi(131.0, 0.05)]).unwrap();
    let omega = 2.0 * std::f64::consts::PI * 131.0;
    assert!(matches!(c.stages[0], ChainStage::SmoothKernel(_)));
    assert!(matches!(
        c.stages[1],
        ChainStage::DerivativeGains { k1, k2 }
            if k1 == 2.0 * 0.05 / omega && k2 == 1.0 / (omega * omega)
    ));
}

#[test]
fn compile_mode_inverse_without_kernel_rejected() {
    let err = CompiledChain::compile(&[mi(131.0, 0.05)]).unwrap_err();
    assert!(matches!(
        err,
        PostProcessorError::AccelGainNeedsPrecedingKernel { .. }
    ));
    assert!(err.to_string().contains("smoothing kernel"), "got: {err}");
}

#[test]
fn compile_mode_inverse_before_kernel_rejected() {
    let err = CompiledChain::compile(&[mi(131.0, 0.05), bell(0.0015)]).unwrap_err();
    assert!(matches!(
        err,
        PostProcessorError::AccelGainNeedsPrecedingKernel { .. }
    ));
}

#[test]
fn compile_mode_inverse_after_a_disabled_kernel_rejected() {
    let err = CompiledChain::compile(&[st(0.0), mi(131.0, 0.05)]).unwrap_err();
    assert!(matches!(
        err,
        PostProcessorError::AccelGainNeedsPrecedingKernel { .. }
    ));
}

#[test]
fn compile_mode_inverse_with_zero_damping_keeps_only_the_accel_gain() {
    let c = CompiledChain::compile(&[bell(0.0015), mi(40.0, 0.0)]).unwrap();
    let omega = 2.0 * std::f64::consts::PI * 40.0;
    assert!(matches!(
        c.stages[1],
        ChainStage::DerivativeGains { k1: 0.0, k2 } if k2 == 1.0 / (omega * omega)
    ));
}

#[test]
fn mode_inverse_set_param_updates_both_keys() {
    let mut inst = mi(131.0, 0.05);
    inst.set_param("frequency_hz", 128.5).unwrap();
    inst.set_param("damping_ratio", 0.1).unwrap();
    let c = CompiledChain::compile(&[bell(0.0015), inst]).unwrap();
    let omega = 2.0 * std::f64::consts::PI * 128.5;
    assert!(matches!(
        c.stages[1],
        ChainStage::DerivativeGains { k1, k2 }
            if k1 == 2.0 * 0.1 / omega && k2 == 1.0 / (omega * omega)
    ));
}

#[test]
fn mode_inverse_set_param_rejects_out_of_bound_values() {
    let mut inst = mi(131.0, 0.05);
    for bad in [0.0, -40.0, f64::NAN, f64::INFINITY] {
        assert!(
            matches!(
                inst.set_param("frequency_hz", bad),
                Err(PostProcessorError::BadParam { .. })
            ),
            "frequency_hz={bad} should be rejected"
        );
    }
    for bad in [-0.01, 1.0, 1.5, f64::NAN, f64::INFINITY] {
        assert!(
            matches!(
                inst.set_param("damping_ratio", bad),
                Err(PostProcessorError::BadParam { .. })
            ),
            "damping_ratio={bad} should be rejected"
        );
    }
    let c = CompiledChain::compile(&[bell(0.0015), inst]).unwrap();
    let omega = 2.0 * std::f64::consts::PI * 131.0;
    assert!(
        matches!(
            c.stages[1],
            ChainStage::DerivativeGains { k1, k2 }
                if k1 == 2.0 * 0.05 / omega && k2 == 1.0 / (omega * omega)
        ),
        "rejected updates must not mutate the gains"
    );
}

#[test]
fn compile_rejects_directly_constructed_bad_mode_inverse_params() {
    for bad in [0.0, -40.0, f64::NAN, f64::INFINITY] {
        let err = CompiledChain::compile(&[bell(0.0015), mi(bad, 0.05)]).unwrap_err();
        assert!(matches!(err, PostProcessorError::BadParam { .. }));
    }
    for bad in [-0.01, 1.0, f64::NAN, f64::INFINITY] {
        let err = CompiledChain::compile(&[bell(0.0015), mi(40.0, bad)]).unwrap_err();
        assert!(matches!(err, PostProcessorError::BadParam { .. }));
    }
}

#[test]
fn follower_supports_cascade_on_top_of_the_leaders() {
    let leader = CompiledChain::compile(&[bell(0.01605)]).unwrap();
    let follower = CompiledChain::compile(&[pa(0.04), bell(0.0321)]).unwrap();
    let (lead_lo, lead_hi) = leader.max_input_window();
    let (own_lo, own_hi) = follower.max_input_window();
    let set = AxisChainSet {
        chains: vec![
            leader.clone(),
            leader.clone(),
            CompiledChain::default(),
            follower,
        ],
        followers: vec![(3, vec![0, 1, 2])],
    };
    assert_eq!(set.axis_support(0), (lead_lo, lead_hi));
    assert_eq!(set.axis_support(3), (own_lo + lead_lo, own_hi + lead_hi));
    assert_eq!(set.forward_support(), own_hi + lead_hi);
    assert_eq!(set.back_support(), (own_lo + lead_lo).abs());
    assert_eq!(set.direct_forward_support(), lead_hi);
    assert_eq!(set.max_follower_own_forward_support(), own_hi);
    assert!(set.has_own_kernel(3));
    assert!(!set.has_own_kernel(2));
}

#[test]
fn kernel_free_followers_do_not_gate_the_shaper() {
    let leader = CompiledChain::compile(&[bell(0.01605)]).unwrap();
    let follower = CompiledChain::compile(&[pa(0.04)]).unwrap();
    let (lead_lo, lead_hi) = leader.max_input_window();
    let set = AxisChainSet {
        chains: vec![
            leader.clone(),
            leader.clone(),
            CompiledChain::default(),
            follower,
        ],
        followers: vec![(3, vec![0, 1, 2])],
    };
    assert_eq!(set.axis_support(3), (lead_lo, lead_hi));
    assert_eq!(set.forward_support(), lead_hi);
    assert_eq!(set.max_follower_own_forward_support(), 0.0);
}
