use super::*;
use nurbs::algebra::PiecewisePolynomialKernel;

fn test_bell_kernel(frequency_hz: f64) -> PiecewisePolynomialKernel<f64> {
    let t_sm = 0.8025 / frequency_hz;
    let h = t_sm / 2.0;
    let c = 15.0 / (16.0 * h.powi(5));
    let coeffs = vec![c * h.powi(4), 0.0, -2.0 * c * h * h, 0.0, c];
    PiecewisePolynomialKernel::single_poly_from_absolute(coeffs, (-h, h))
}

fn kernel_half_support(kernel: &PiecewisePolynomialKernel<f64>) -> f64 {
    kernel.support().1
}

#[test]
fn time_map_of_constant_speed_is_uniform() {
    let b = vec![4.0; 5];
    let h = vec![1.0; 4];
    let t = frozen_time_map(&b, &h);
    for (i, ti) in t.iter().enumerate() {
        assert!((ti - 0.5 * i as f64).abs() < 1e-12);
    }
}

#[test]
fn identity_window_is_identity() {
    let w = WindowOperator::identity(5);
    let signal = [1.0, 2.0, 3.0, 4.0, 5.0];
    for i in 0..5 {
        let row = w.row(i);
        let applied: f64 = row.weights.iter().map(|&(j, wj)| wj * signal[j]).sum();
        assert!((applied + row.history - signal[i]).abs() < 1e-12);
    }
}

#[test]
fn kernel_window_weights_sum_to_one_in_the_interior() {
    let kernel = test_bell_kernel(40.0);
    let b = vec![100.0_f64.powi(2); 200];
    let h = vec![0.1; 199];
    let t = frozen_time_map(&b, &h);
    let w = WindowOperator::from_kernel(&kernel, &t, &WindowHistory::empty());
    let mid = w.row(100);
    let total: f64 = mid.weights.iter().map(|&(_, wj)| wj).sum();
    assert!((total - 1.0).abs() < 2e-3, "got {total}");
}

#[test]
fn history_supplies_the_left_edge() {
    let kernel = test_bell_kernel(40.0);
    let b = vec![100.0_f64.powi(2); 200];
    let h = vec![0.1; 199];
    let t = frozen_time_map(&b, &h);
    let hist = WindowHistory::constant_signal(100.0, kernel_half_support(&kernel), 64);
    let w = WindowOperator::from_kernel(&kernel, &t, &hist);
    let row0 = w.row(0);
    let interior: f64 = row0.weights.iter().map(|&(_, wj)| wj).sum::<f64>() * 100.0;
    assert!((interior + row0.history - 100.0).abs() < 1.0);
}

#[test]
fn right_edge_extends_with_terminal_hold() {
    let kernel = test_bell_kernel(40.0);
    let b = vec![100.0_f64.powi(2); 200];
    let h = vec![0.1; 199];
    let t = frozen_time_map(&b, &h);
    let hist = WindowHistory::constant_signal(100.0, kernel_half_support(&kernel), 64);
    let w = WindowOperator::from_kernel(&kernel, &t, &hist);
    let signal = vec![100.0; 200];
    let last = w.row(199);
    let applied: f64 = last.weights.iter().map(|&(j, wj)| wj * signal[j]).sum();
    assert!(
        (applied + last.history - 100.0).abs() < 1.0,
        "got {applied}"
    );
}

#[test]
#[should_panic(expected = "zero speed")]
fn zero_speed_interval_panics() {
    let b = vec![0.0, 0.0];
    let h = vec![1.0];
    let _ = frozen_time_map(&b, &h);
}
