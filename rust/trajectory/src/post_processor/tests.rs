use super::*;

fn pa(k: f64) -> PostProcessorInstance {
    PostProcessorInstance::new(
        "pa",
        PostProcessorType::LinearPressureAdvance {
            k,
            smooth_time: 0.0,
        },
    )
}
fn zv(hz: f64) -> PostProcessorInstance {
    PostProcessorInstance::new("is", PostProcessorType::SmoothZv { frequency_hz: hz })
}

#[test]
fn compile_empty_chain_is_identity() {
    let c = CompiledChain::compile(&[]).unwrap();
    assert!(c.kernel.is_none());
    assert_eq!(c.gain, 0.0);
}

#[test]
fn compile_kernel_plus_gain() {
    let c = CompiledChain::compile(&[zv(50.0), pa(0.04)]).unwrap();
    assert!(c.kernel.is_some());
    assert_eq!(c.gain, 0.04);
}

#[test]
fn compile_order_irrelevant_for_linear_ops() {
    let a = CompiledChain::compile(&[zv(50.0), pa(0.04)]).unwrap();
    let b = CompiledChain::compile(&[pa(0.04), zv(50.0)]).unwrap();
    assert_eq!(a.gain, b.gain);
    assert_eq!(a.kernel.is_some(), b.kernel.is_some());
}

#[test]
fn compile_two_kernels_rejected() {
    let err = CompiledChain::compile(&[zv(50.0), zv(40.0)]).unwrap_err();
    assert!(matches!(
        err,
        PostProcessorError::UnsupportedComposition { .. }
    ));
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
    assert_eq!(c.gain, 0.06);
}

#[test]
fn set_param_unknown_key_fails() {
    let mut inst = zv(50.0);
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
    assert_eq!(c.gain, 0.04, "rejected updates must not mutate the gain");
    inst.set_param("k", 0.0).expect("k=0 is a valid no-op gain");
}

#[test]
fn set_param_updates_smooth_time_and_rejects_bad_values() {
    let mut inst = pa(0.04);
    inst.set_param("smooth_time", 0.03).unwrap();
    let c = CompiledChain::compile(std::slice::from_ref(&inst)).unwrap();
    assert_eq!(c.smooth_time, 0.03);
    for bad in [-0.01, f64::NAN, f64::INFINITY] {
        assert!(matches!(
            inst.set_param("smooth_time", bad),
            Err(PostProcessorError::BadParam { .. })
        ));
    }
    let c = CompiledChain::compile(std::slice::from_ref(&inst)).unwrap();
    assert_eq!(
        c.smooth_time, 0.03,
        "rejected updates must not mutate smooth_time"
    );
}

fn t_squared_cubic() -> nurbs::ScalarNurbs<f64> {
    nurbs::ScalarNurbs::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![0.0, 0.0, 1.0 / 3.0, 1.0],
    )
    .unwrap()
}

#[test]
fn derivative_gain_applied_exactly_on_nurbs() {
    let out = apply_derivative_gain(&t_squared_cubic(), 0.5);
    for t in [0.0, 0.25, 0.5, 0.75, 1.0] {
        assert!((nurbs::eval::eval(&out, t) - (t * t + t)).abs() < 1e-12);
    }
}

#[test]
fn derivative_gain_preserves_degree_and_pieces() {
    let out = apply_derivative_gain(&t_squared_cubic(), 0.5);
    assert_eq!(out.degree(), 3);
}
