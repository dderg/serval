use super::*;
use trajectory::{build_smooth_mzv_kernel, build_smooth_zv_kernel};

const H: f64 = 2.5e-4;

fn kernels() -> Vec<PiecewisePolynomialKernel> {
    vec![
        build_smooth_mzv_kernel(50.0),
        build_smooth_zv_kernel(44.0),
        build_smooth_mzv_kernel(191.0),
    ]
}

fn value(kernel: &PiecewisePolynomialKernel, u: f64) -> f64 {
    piece_at(&kernel.pieces, u).map_or(0.0, |piece| piece.evaluate(u))
}

fn slope(kernel: &PiecewisePolynomialKernel, u: f64) -> f64 {
    piece_at(&kernel.pieces, u).map_or(0.0, |piece| piece.differentiate().evaluate(u))
}

#[test]
fn each_cell_bounds_the_kernel_and_its_slope_about_their_linearisation() {
    for kernel in kernels() {
        let cells = KernelCells::new(&kernel, H);
        for (offset, (value_cell, slope_cell)) in cells.value.iter().zip(&cells.slope).enumerate() {
            let a = (cells.first + offset as i64) as f64 * H;
            let mid = a + 0.5 * H;
            let mut worst = (0.0_f64, 0.0_f64);
            for step in 0..=400 {
                let u = a + H * f64::from(step) / 400.0;
                let line = |cell: &Linear| cell.at + cell.slope * (u - mid);
                worst.0 = worst.0.max((value(&kernel, u) - line(value_cell)).abs());
                worst.1 = worst.1.max((slope(&kernel, u) - line(slope_cell)).abs());
            }
            assert!(
                worst.0 <= value_cell.miss * (1.0 + 1e-9) + 1e-12,
                "{worst:?} {value_cell:?}"
            );
            assert!(
                worst.1 <= slope_cell.miss * (1.0 + 1e-9) + 1e-9,
                "{worst:?} {slope_cell:?}"
            );
            assert!(
                value_cell.miss <= 4.0 * worst.0 + 1e-9 * value_cell.at.abs().max(1.0),
                "the bound is loose: {} against {}",
                value_cell.miss,
                worst.0
            );
        }
    }
}

#[test]
fn the_masses_bracket_the_kernel_parts() {
    for kernel in kernels() {
        let cells = KernelCells::new(&kernel, H);
        let (lo, hi) = kernel.support();
        let steps = 20_000;
        let width = (hi - lo) / f64::from(steps);
        let (positive, negative) = (0..steps).fold((0.0, 0.0), |(p, n), step| {
            let w = value(&kernel, lo + (f64::from(step) + 0.5) * width) * width;
            if w >= 0.0 { (p + w, n) } else { (p, n - w) }
        });
        for (name, (least, most), truth) in [
            ("positive", cells.positive_mass, positive),
            ("negative", cells.negative_mass, negative),
        ] {
            let slack = 1e-6 * (positive + negative);
            assert!(
                least <= truth + slack && truth <= most + slack,
                "{name} mass {truth} outside [{least}, {most}]"
            );
        }
    }
}

#[test]
fn the_jerk_rate_bounds_the_slope_variation() {
    for kernel in kernels() {
        let cells = KernelCells::new(&kernel, H);
        let (lo, hi) = kernel.support();
        let steps = 20_000;
        let width = (hi - lo) / f64::from(steps);
        let variation: f64 = (0..steps)
            .map(|step| {
                let u = lo + f64::from(step) * width;
                (slope(&kernel, u + width) - slope(&kernel, u)).abs()
            })
            .sum();
        assert!(cells.jerk_rate_per_accel >= variation * (1.0 - 1e-6));
        assert!(cells.jerk_rate_per_accel <= 2.0 * variation + 1e-6);
    }
}

/// `|w(b⁺) − w(b⁻)|` summed over the piece ends `b` in `(from, to]`.
fn steps_within(kernel: &PiecewisePolynomialKernel, from: f64, to: f64) -> f64 {
    let left = |b: f64| {
        kernel
            .pieces
            .iter()
            .find(|piece| piece.u_start < b && b <= piece.u_end)
            .map_or(0.0, |piece| piece.evaluate(b))
    };
    kernel
        .pieces
        .iter()
        .flat_map(|piece| [piece.u_start, piece.u_end])
        .filter(|&b| b > from && b <= to)
        .map(|b| (value(kernel, b) - left(b)).abs())
        .sum()
}

#[test]
fn the_drift_weights_bound_the_kernel_moved_by_a_shift_within_a_cell() {
    for kernel in kernels() {
        let cells = KernelCells::new(&kernel, H);
        assert_eq!(cells.drift_weights.len(), cells.value.len() + 1);
        for (index, weight) in cells.drift_weights.iter().enumerate() {
            let a = (cells.first + index as i64 - 1) as f64 * H;
            for shift in [H / 7.0, H / 2.0, H] {
                for step in 0..=200 {
                    let u = a + H * f64::from(step) / 200.0;
                    let moved = (value(&kernel, u + shift) - value(&kernel, u)).abs();
                    let allowed = shift * weight + steps_within(&kernel, u, u + shift);
                    assert!(
                        moved <= allowed * (1.0 + 1e-9) + 1e-9,
                        "cell {index}: w moves {moved} over {shift} from {u}, weight {weight}"
                    );
                }
            }
        }
        let peak = cells.drift_weights.iter().fold(0.0_f64, |a, &b| a.max(b));
        let mean = cells.drift_weights.iter().sum::<f64>() / cells.drift_weights.len() as f64;
        assert!(
            mean < 0.8 * peak,
            "the weights follow the kernel's slope: mean {mean} against peak {peak}"
        );
    }
}
