use nurbs::algebra::PiecewisePolynomialKernel;

use super::ShapedSignal;

fn degree_six_kernel() -> PiecewisePolynomialKernel {
    PiecewisePolynomialKernel::single_poly_from_absolute(
        vec![1.0, 0.0, -3.0, 0.0, 3.0, 0.0, -1.0],
        (-1.0, 1.0),
    )
}

fn binomial(n: usize, k: usize) -> f64 {
    (0..k).fold(1.0, |value, i| value * (n - i) as f64 / (i + 1) as f64)
}

#[test]
fn degree_thirteen_convolution_is_exact() {
    let kernel = degree_six_kernel();
    let signal = ShapedSignal::new_from_evaluator(&kernel, |t| t.powi(7), Vec::new(), 7);
    let t = 0.37_f64;
    let mut expected = 0.0;
    for input_power in 0..=7 {
        let input_coefficient = binomial(7, input_power)
            * t.powi((7 - input_power) as i32)
            * if input_power % 2 == 0 { 1.0 } else { -1.0 };
        for (kernel_power, kernel_coefficient) in [(0, 1.0), (2, -3.0), (4, 3.0), (6, -1.0)] {
            let power = input_power + kernel_power;
            if power % 2 == 0 {
                expected += input_coefficient * kernel_coefficient * 2.0 / (power + 1) as f64;
            }
        }
    }
    assert!((signal.eval(t) - expected).abs() < 1e-12);
}

#[test]
#[should_panic(expected = "exceeds exact quadrature degree 19")]
fn convolution_rejects_unrepresentable_product_degree() {
    let kernel = degree_six_kernel();
    let _ = ShapedSignal::new_from_evaluator(&kernel, |t| t.powi(14), Vec::new(), 14);
}

fn previous_f64(value: f64) -> f64 {
    if value > 0.0 {
        f64::from_bits(value.to_bits() - 1)
    } else if value == 0.0 {
        -f64::from_bits(1)
    } else {
        f64::from_bits(value.to_bits() + 1)
    }
}

#[test]
fn cut_transitions_land_on_the_exact_evaluator_comparison_flip() {
    let kernel = crate::kernel::build_smooth_mzv_kernel(22.428_571_428_571_43);
    let (k_lo, k_hi) = kernel.support();
    let kernel_breaks: Vec<f64> = ShapedSignal::kernel_cut_boundaries(&kernel).collect();
    assert_eq!(kernel_breaks.first().copied(), Some(k_lo));
    assert_eq!(kernel_breaks.last().copied(), Some(k_hi));
    let mut shifted_alignments = 0;
    let mut window_edges_past_ownership = 0;
    for input_break in [
        0.0,
        0.271_374_837_662_063_9,
        0.705_162_418_830_348_1,
        3.012_830_318_217_121_7,
        18.451_759_619_643_7,
    ] {
        for &kernel_break in &kernel_breaks {
            let mut transitions = Vec::new();
            ShapedSignal::output_cut_transitions(
                &kernel,
                input_break,
                kernel_break,
                &mut transitions,
            );
            assert_eq!(transitions.len(), 1);
            let cut = transitions[0];
            let owned = |t: f64| t - input_break >= kernel_break;
            let inside_window = |t: f64| {
                if kernel_break == k_lo {
                    input_break < t - k_lo
                } else if kernel_break == k_hi {
                    input_break <= t - k_hi
                } else {
                    true
                }
            };
            assert!(owned(cut) && inside_window(cut));
            let before = previous_f64(cut);
            assert!(!owned(before) || !inside_window(before));
            if cut != input_break + kernel_break {
                shifted_alignments += 1;
            }
            if owned(before) {
                assert!(kernel_break == k_lo || kernel_break == k_hi);
                window_edges_past_ownership += 1;
            }
        }
    }
    assert!(
        shifted_alignments > 0,
        "kernel/input pair set never exercises a cancellation-shifted cut alignment"
    );
    assert!(
        window_edges_past_ownership > 0,
        "kernel/input pair set never exercises a window edge later than the ownership flip"
    );
}

fn line_moments(
    intercept: f64,
    slope: f64,
) -> impl Fn(f64, f64, usize, f64, [&mut [f64]; super::MOMENT_ORDERS]) -> bool {
    move |lo, hi, degree, origin, [position, velocity, acceleration]| {
        let at_origin = intercept + slope * origin;
        let span =
            |power: i32| ((hi - origin).powi(power) - (lo - origin).powi(power)) / f64::from(power);
        for power in 0..=degree {
            let exponent = power as i32;
            position[power] = at_origin * span(exponent + 1) + slope * span(exponent + 2);
            velocity[power] = slope * span(exponent + 1);
        }
        acceleration.fill(0.0);
        true
    }
}

#[test]
fn a_line_far_from_the_origin_shapes_to_a_line_within_a_few_ulps() {
    let kernel = crate::build_smooth_mzv_kernel(50.0);
    let (intercept, slope) = (134.7, 40.0);
    let signal = ShapedSignal::new_from_polynomial_evaluator(
        &kernel,
        move |t| intercept + slope * t,
        Vec::new(),
        1,
        line_moments(intercept, slope),
        Vec::new(),
    );
    let step = 1e-6;
    let worst_bend = (1..100)
        .map(|k| {
            let t = 0.095 + step * f64::from(k);
            (signal.eval(t + step) - 2.0 * signal.eval(t) + signal.eval(t - step)).abs()
        })
        .fold(0.0, f64::max);
    assert!(
        worst_bend <= 64.0 * f64::EPSILON * intercept,
        "the shaped line bends by {worst_bend:e} mm between 1 µs samples"
    );
}

#[test]
fn kernel_mass_integrates_every_piece() {
    let mass = super::kernel_mass(&degree_six_kernel());
    assert!(
        (mass - 32.0 / 35.0).abs() <= 4.0 * f64::EPSILON,
        "mass {mass}"
    );
}

#[test]
fn the_shaper_kernels_have_unit_mass() {
    for kernel in [
        crate::build_smooth_mzv_kernel(50.0),
        crate::build_smooth_zv_kernel(44.0),
    ] {
        let mass = super::kernel_mass(&kernel);
        assert!((mass - 1.0).abs() <= 1e-12, "mass {mass}");
    }
}
