use super::*;
use crate::kernel::build_smooth_zv_kernel;
use crate::algos::{
    LinearPressureAdvance, SmoothTriangle, SmoothZv, SMOOTH_ZV_T_SM_PER_HZ,
};

fn pa(k: f64) -> PostProcessorInstance {
    PostProcessorInstance::new("pa", &LinearPressureAdvance, vec![k])
}
fn zv(hz: f64) -> PostProcessorInstance {
    PostProcessorInstance::new("is", &SmoothZv, vec![hz])
}
fn st(smooth_time: f64) -> PostProcessorInstance {
    PostProcessorInstance::new("st", &SmoothTriangle, vec![smooth_time])
}

#[test]
fn compile_empty_chain_is_identity() {
    let c = CompiledChain::compile(&[]).unwrap();
    assert!(c.stages.is_empty());
}

#[test]
fn compile_kernel_plus_gain() {
    let c = CompiledChain::compile(&[zv(50.0), pa(0.04)]).unwrap();
    assert!(matches!(c.stages[0], ChainStage::SmoothKernel(_)));
    assert!(matches!(
        c.stages[1],
        ChainStage::LinearPressureAdvance { k } if k == 0.04
    ));
}

#[test]
fn compile_preserves_declaration_order() {
    let a = CompiledChain::compile(&[zv(50.0), pa(0.04)]).unwrap();
    let b = CompiledChain::compile(&[pa(0.04), zv(50.0)]).unwrap();
    assert!(matches!(a.stages[0], ChainStage::SmoothKernel(_)));
    assert!(matches!(
        a.stages[1],
        ChainStage::LinearPressureAdvance { .. }
    ));
    assert!(matches!(
        b.stages[0],
        ChainStage::LinearPressureAdvance { .. }
    ));
    assert!(matches!(b.stages[1], ChainStage::SmoothKernel(_)));
}

#[test]
fn compile_smooth_triangle_plus_gain() {
    let c = CompiledChain::compile(&[st(0.04), pa(0.04)]).unwrap();
    assert!(matches!(c.stages[0], ChainStage::SmoothKernel(_)));
    assert!(matches!(
        c.stages[1],
        ChainStage::LinearPressureAdvance { k } if k == 0.04
    ));
    let (lo, hi) = c.max_half_support();
    assert!((hi - 0.02).abs() < 1e-12 && (lo + 0.02).abs() < 1e-12);
}

#[test]
fn compile_gain_before_smooth_triangle_preserves_order() {
    let c = CompiledChain::compile(&[pa(0.04), st(0.04)]).unwrap();
    assert!(matches!(
        c.stages[0],
        ChainStage::LinearPressureAdvance { .. }
    ));
    assert!(matches!(c.stages[1], ChainStage::SmoothKernel(_)));
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
fn compile_smooth_triangle_and_input_shaper_rejected_as_two_kernels() {
    let err = CompiledChain::compile(&[zv(50.0), st(0.04)]).unwrap_err();
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
    assert_eq!(c.max_half_support(), (0.0, 0.0));
}

#[test]
fn compile_zero_smooth_time_leaves_only_the_gain() {
    let c = CompiledChain::compile(&[st(0.0), pa(0.04)]).unwrap();
    assert_eq!(c.stages.len(), 1);
    assert!(matches!(
        c.stages[0],
        ChainStage::LinearPressureAdvance { k } if k == 0.04
    ));
}

#[test]
fn compile_disabled_smooth_triangle_does_not_conflict_with_input_shaper() {
    let c = CompiledChain::compile(&[zv(50.0), st(0.0)]).unwrap();
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
    assert!(matches!(c.stages[0], ChainStage::LinearPressureAdvance { k } if k == 0.06));
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
    assert!(
        matches!(c.stages[0], ChainStage::LinearPressureAdvance { k } if k == 0.04),
        "rejected updates must not mutate the gain"
    );
    inst.set_param("k", 0.0).expect("k=0 is a valid no-op gain");
}

#[test]
fn set_param_rejects_non_positive_and_non_finite_shaper_frequency() {
    let mut inst = zv(50.0);
    for bad in [0.0, -1.0, f64::NAN, f64::INFINITY] {
        assert!(
            matches!(
                inst.set_param("frequency_hz", bad),
                Err(PostProcessorError::BadParam { .. })
            ),
            "frequency_hz={bad} should be rejected"
        );
    }
    let c = CompiledChain::compile(std::slice::from_ref(&inst)).unwrap();
    let ChainStage::SmoothKernel(kernel) = &c.stages[0] else {
        panic!("expected smooth kernel stage");
    };
    assert_eq!(
        kernel.support(),
        build_smooth_zv_kernel(SMOOTH_ZV_T_SM_PER_HZ / 50.0).support()
    );
}

#[test]
fn compile_rejects_directly_constructed_bad_params() {
    for bad in [0.0, -1.0, f64::NAN, f64::INFINITY] {
        let err = CompiledChain::compile(&[zv(bad)]).unwrap_err();
        assert!(matches!(err, PostProcessorError::BadParam { .. }));
    }
    for bad in [-0.01, f64::NAN, f64::INFINITY] {
        let err = CompiledChain::compile(&[pa(bad)]).unwrap_err();
        assert!(matches!(err, PostProcessorError::BadParam { .. }));
    }
}
