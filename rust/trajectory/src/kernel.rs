use nurbs::algebra::PiecewisePolynomialKernel;
use nurbs::bezier::BezierPiece;

#[inline(never)]
pub fn build_smooth_bell_kernel(smooth_time: f64) -> PiecewisePolynomialKernel {
    assert!(
        smooth_time > 0.0,
        "build_smooth_bell_kernel requires smooth_time > 0, got {smooth_time}"
    );
    let h = smooth_time / 2.0;
    let c = 15.0 / (16.0 * h.powi(5));
    let coeffs = vec![
        c * h.powi(4),    // t^0
        0.0,              // t^1
        -2.0 * c * h * h, // t^2
        0.0,              // t^3
        c,                // t^4
    ];
    PiecewisePolynomialKernel::single_poly_from_absolute(coeffs, (-h, h))
}

/// Coefficients from Kalico bleeding_edge_v2 `shaper_defs.py`
/// (`get_zv_smoother` / `get_mzv_smoother`): polynomials optimized in Maxima
/// so a damped oscillator's residual vibration stays low in a band around
/// the target frequency when the kernel spans `SMOOTH_*_DURATION_PER_HZ / f`.
/// Ascending powers of `t / smooth_time`, support `[-1/2, 1/2]`.
const SMOOTH_ZV_UNIT_COEFFS: &[f64] = &[
    0.01966833207740377,
    -1.465471373781904,
    29.52796003014231,
    5.861885495127615,
    -118.4265334338076,
];

const SMOOTH_MZV_UNIT_COEFFS: &[f64] = &[
    1.713117990217123,
    1.57172781617736,
    -62.18762409216703,
    -37.75923018121473,
    698.0200035767849,
    125.8892756660212,
    -1906.717580206364,
];

pub const SMOOTH_ZV_DURATION_PER_HZ: f64 = 0.8025;
pub const SMOOTH_MZV_DURATION_PER_HZ: f64 = 0.95625;

pub fn build_smooth_zv_kernel(frequency_hz: f64) -> PiecewisePolynomialKernel {
    assert!(
        frequency_hz.is_finite() && frequency_hz > 0.0,
        "build_smooth_zv_kernel requires frequency_hz > 0, got {frequency_hz}"
    );
    build_unit_polynomial_kernel(
        "build_smooth_zv_kernel",
        SMOOTH_ZV_UNIT_COEFFS,
        SMOOTH_ZV_DURATION_PER_HZ / frequency_hz,
    )
}

pub fn build_smooth_mzv_kernel(frequency_hz: f64) -> PiecewisePolynomialKernel {
    assert!(
        frequency_hz.is_finite() && frequency_hz > 0.0,
        "build_smooth_mzv_kernel requires frequency_hz > 0, got {frequency_hz}"
    );
    build_unit_polynomial_kernel(
        "build_smooth_mzv_kernel",
        SMOOTH_MZV_UNIT_COEFFS,
        SMOOTH_MZV_DURATION_PER_HZ / frequency_hz,
    )
}

fn build_unit_polynomial_kernel(
    name: &str,
    unit_coeffs: &[f64],
    smooth_time: f64,
) -> PiecewisePolynomialKernel {
    assert!(
        smooth_time > 0.0,
        "{name} requires smooth_time > 0, got {smooth_time}"
    );
    let h = smooth_time / 2.0;
    let scaled: Vec<f64> = unit_coeffs
        .iter()
        .enumerate()
        .map(|(k, &c)| c / smooth_time.powi(k as i32 + 1))
        .collect();
    let mut integral = 0.0;
    let mut first_moment = 0.0;
    for (k, &c) in scaled.iter().enumerate() {
        if k % 2 == 0 {
            integral += 2.0 * c * h.powi(k as i32 + 1) / (k as f64 + 1.0);
        } else {
            first_moment += 2.0 * c * h.powi(k as i32 + 2) / (k as f64 + 2.0);
        }
    }
    let mean = first_moment / integral;
    let coeffs = shift_polynomial(&scaled, mean)
        .iter()
        .map(|c| c / integral)
        .collect();
    PiecewisePolynomialKernel::single_poly_from_absolute(coeffs, (-h - mean, h - mean))
}

fn shift_polynomial(coeffs: &[f64], shift: f64) -> Vec<f64> {
    let mut shifted = vec![0.0; coeffs.len()];
    for (k, &c) in coeffs.iter().enumerate() {
        let mut term = c;
        let mut j = k;
        loop {
            shifted[j] += term;
            if j == 0 {
                break;
            }
            term *= shift * (j as f64) / ((k - j + 1) as f64);
            j -= 1;
        }
    }
    shifted
}

pub fn build_smooth_triangle_kernel(smooth_time: f64) -> PiecewisePolynomialKernel {
    assert!(
        smooth_time > 0.0,
        "build_smooth_triangle_kernel requires smooth_time > 0, got {smooth_time}"
    );
    let hst = smooth_time / 2.0;
    let inv_hst2 = 1.0 / (hst * hst);
    let rising = BezierPiece {
        u_start: -hst,
        u_end: 0.0,
        coeffs: vec![0.0, inv_hst2],
    };
    let falling = BezierPiece {
        u_start: 0.0,
        u_end: hst,
        coeffs: vec![1.0 / hst, -inv_hst2],
    };
    PiecewisePolynomialKernel::from_pieces(vec![rising, falling])
        .expect("triangle kernel pieces are contiguous by construction")
}

#[cfg(test)]
mod tests;
