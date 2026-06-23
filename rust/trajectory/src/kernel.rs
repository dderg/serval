use nurbs::algebra::PiecewisePolynomialKernel;

#[inline(never)]
fn build_bell_kernel(t_sm: f64) -> PiecewisePolynomialKernel<f64> {
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

pub fn build_smooth_zv_kernel(t_sm: f64) -> PiecewisePolynomialKernel<f64> {
    build_bell_kernel(t_sm)
}

pub fn build_smooth_mzv_kernel(t_sm: f64) -> PiecewisePolynomialKernel<f64> {
    build_bell_kernel(t_sm)
}

#[cfg(test)]
mod tests;
