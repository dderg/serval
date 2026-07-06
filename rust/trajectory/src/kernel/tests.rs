#![allow(deprecated)]

use super::*;

#[test]
fn smooth_zv_kernel_is_normalized() {
    let kernel = build_smooth_zv_kernel(0.8025 / 150.0);
    let (lo, hi) = kernel.support();
    let n = 1000;
    let step = (hi - lo) / f64::from(n);
    let mut integral = 0.0;
    for i in 0..=n {
        let t = lo + f64::from(i) * step;
        let w = if i == 0 || i == n {
            1.0
        } else if i % 2 == 0 {
            2.0
        } else {
            4.0
        };
        integral += w * kernel.pieces[0].evaluate(t);
    }
    integral *= step / 3.0;
    assert!((integral - 1.0).abs() < 1e-6, "integral = {integral}");
}

#[test]
fn smooth_mzv_kernel_is_normalized() {
    let kernel = build_smooth_mzv_kernel(0.95625 / 120.0);
    let (lo, hi) = kernel.support();
    let n = 1000;
    let step = (hi - lo) / f64::from(n);
    let mut integral = 0.0;
    for i in 0..=n {
        let t = lo + f64::from(i) * step;
        let w = if i == 0 || i == n {
            1.0
        } else if i % 2 == 0 {
            2.0
        } else {
            4.0
        };
        integral += w * kernel.pieces[0].evaluate(t);
    }
    integral *= step / 3.0;
    assert!((integral - 1.0).abs() < 1e-6, "integral = {integral}");
}

#[test]
fn kernel_vanishes_at_boundaries() {
    let kernel = build_smooth_zv_kernel(0.8025 / 150.0);
    let (lo, hi) = kernel.support();
    assert!(kernel.pieces[0].evaluate(lo).abs() < 1e-12);
    assert!(kernel.pieces[0].evaluate(hi).abs() < 1e-12);
}

#[test]
fn kernel_derivative_vanishes_at_boundaries() {
    let kernel = build_smooth_zv_kernel(0.8025 / 150.0);
    let (lo, hi) = kernel.support();
    let dk = kernel.pieces[0].differentiate();
    let lo_tolerance = 1e-10;
    let hi_tolerance_after_cancellation_at_2h = 1e-8;
    assert!(
        dk.evaluate(lo).abs() < lo_tolerance,
        "lo = {}",
        dk.evaluate(lo)
    );
    assert!(
        dk.evaluate(hi).abs() < hi_tolerance_after_cancellation_at_2h,
        "hi = {}",
        dk.evaluate(hi)
    );
}

#[test]
fn kernel_is_positive_inside() {
    let kernel = build_smooth_zv_kernel(0.8025 / 150.0);
    let (lo, hi) = kernel.support();
    let n = 100;
    for i in 1..n {
        let t = lo + (hi - lo) * f64::from(i) / f64::from(n);
        assert!(kernel.pieces[0].evaluate(t) > 0.0, "negative at t={t}");
    }
}

#[test]
fn kernel_peak_at_center() {
    let kernel = build_smooth_zv_kernel(0.8025 / 150.0);
    let center_val = kernel.pieces[0].evaluate(0.0);
    let off_center = kernel.pieces[0].evaluate(0.001);
    assert!(center_val > off_center);
}

#[test]
fn smooth_zv_support_width() {
    let f = 150.0;
    let chain = crate::CompiledChain::compile(&[crate::PostProcessorInstance::new(
        "is",
        &crate::post_processors::SmoothZv,
        vec![f],
    )])
    .expect("single post-processor always compiles");
    let crate::ChainStage::SmoothKernel(kernel) = &chain.stages[0] else {
        panic!("expected smooth kernel stage");
    };
    let (lo, hi) = kernel.support();
    let expected_t_sm = 0.8025 / f;
    assert!((hi - lo - expected_t_sm).abs() < 1e-12);
}

#[test]
fn smooth_mzv_support_width() {
    let f = 120.0;
    let chain = crate::CompiledChain::compile(&[crate::PostProcessorInstance::new(
        "is",
        &crate::post_processors::SmoothMzv,
        vec![f],
    )])
    .expect("single post-processor always compiles");
    let crate::ChainStage::SmoothKernel(kernel) = &chain.stages[0] else {
        panic!("expected smooth kernel stage");
    };
    let (lo, hi) = kernel.support();
    let expected_t_sm = 0.95625 / f;
    assert!((hi - lo - expected_t_sm).abs() < 1e-12);
}

fn eval_kernel(kernel: &PiecewisePolynomialKernel<f64>, t: f64) -> f64 {
    for piece in &kernel.pieces {
        if t >= piece.u_start && t <= piece.u_end {
            return piece.evaluate(t);
        }
    }
    0.0
}

#[test]
fn smooth_triangle_kernel_is_normalized() {
    let kernel = build_smooth_triangle_kernel(0.04);
    let mut total = 0.0;
    for piece in &kernel.pieces {
        let (lo, hi) = (piece.u_start, piece.u_end);
        let n = 1000;
        let step = (hi - lo) / f64::from(n);
        for i in 0..=n {
            let t = lo + f64::from(i) * step;
            let w = if i == 0 || i == n {
                1.0
            } else if i % 2 == 0 {
                2.0
            } else {
                4.0
            };
            total += w * piece.evaluate(t) * step / 3.0;
        }
    }
    assert!((total - 1.0).abs() < 1e-9, "integral = {total}");
}

#[test]
fn smooth_triangle_kernel_vanishes_at_boundaries() {
    let kernel = build_smooth_triangle_kernel(0.04);
    let (lo, hi) = kernel.support();
    assert!(eval_kernel(&kernel, lo).abs() < 1e-12);
    assert!(eval_kernel(&kernel, hi).abs() < 1e-12);
}

#[test]
fn smooth_triangle_kernel_is_symmetric() {
    let kernel = build_smooth_triangle_kernel(0.04);
    let hst = 0.02;
    for i in 0..=10 {
        let t = hst * f64::from(i) / 10.0;
        let l = eval_kernel(&kernel, -t);
        let r = eval_kernel(&kernel, t);
        assert!((l - r).abs() < 1e-12, "asymmetry at t={t}: {l} vs {r}");
    }
}

#[test]
fn smooth_triangle_kernel_peaks_at_center() {
    let smooth_time = 0.04;
    let kernel = build_smooth_triangle_kernel(smooth_time);
    let hst = smooth_time / 2.0;
    assert!((eval_kernel(&kernel, 0.0) - 1.0 / hst).abs() < 1e-12);
    assert!(eval_kernel(&kernel, 0.0) > eval_kernel(&kernel, 0.5 * hst));
}

#[test]
fn smooth_triangle_support_width() {
    let smooth_time = 0.04;
    let chain = crate::CompiledChain::compile(&[crate::PostProcessorInstance::new(
        "st",
        &crate::post_processors::SmoothTriangle,
        vec![smooth_time],
    )])
    .expect("single post-processor always compiles");
    let crate::ChainStage::SmoothKernel(kernel) = &chain.stages[0] else {
        panic!("expected smooth kernel stage");
    };
    let (lo, hi) = kernel.support();
    assert!((hi - lo - smooth_time).abs() < 1e-12);
    assert!((lo + smooth_time / 2.0).abs() < 1e-12);
}
