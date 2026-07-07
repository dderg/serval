use nurbs::algebra::PiecewisePolynomialKernel;
use nurbs::bezier::BezierPiece;

#[inline(never)]
fn build_bell_kernel(t_sm: f64) -> PiecewisePolynomialKernel {
    let h = t_sm / 2.0;
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

pub fn build_smooth_zv_kernel(t_sm: f64) -> PiecewisePolynomialKernel {
    build_bell_kernel(t_sm)
}

pub fn build_smooth_mzv_kernel(t_sm: f64) -> PiecewisePolynomialKernel {
    build_bell_kernel(t_sm)
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
