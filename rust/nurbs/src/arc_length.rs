const GAUSS_LEGENDRE_5_NODES: [f64; 5] = [
    -0.906_179_845_938_664,
    -0.538_469_310_105_683_1,
    0.0,
    0.538_469_310_105_683_1,
    0.906_179_845_938_664,
];
const GAUSS_LEGENDRE_5_WEIGHTS: [f64; 5] = [
    0.236_926_885_056_189_1,
    0.478_628_670_499_366_5,
    0.568_888_888_888_888_9,
    0.478_628_670_499_366_5,
    0.236_926_885_056_189_1,
];

pub(crate) fn integrate_arc_length<F: Fn(f64) -> f64>(
    integrand: F,
    u_start: f64,
    u_end: f64,
    quadrature_points: usize,
) -> f64 {
    debug_assert_eq!(
        quadrature_points, 5,
        "v1 supports only 5-point Gauss-Legendre"
    );

    let half_range = (u_end - u_start) * (0.5);
    let midpoint = (u_start + u_end) * (0.5);

    let mut sum = 0.0;
    for i in 0..5 {
        let node = GAUSS_LEGENDRE_5_NODES[i];
        let weight = GAUSS_LEGENDRE_5_WEIGHTS[i];
        let u = midpoint + half_range * node;
        sum = crate::fmadd(integrand(u), weight, sum);
    }

    sum * half_range
}

use crate::eval::{vector_derivative, vector_eval};

#[must_use]
pub fn path_arc_length(xyz: &crate::VectorNurbs<3>) -> f64 {
    let knots = xyz.knots();
    let u_start = knots[0];
    let u_end = knots[knots.len() - 1];

    let deriv = vector_derivative(xyz);

    let speed = |u: f64| -> f64 {
        let d = vector_eval(&deriv, u);
        (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt()
    };

    let span = u_end - u_start;
    let mut prev_estimate: Option<f64> = None;
    let mut subintervals: usize = 1;

    loop {
        let mut sum = 0.0_f64;
        for i in 0..subintervals {
            let a = u_start + span * (i as f64) / (subintervals as f64);
            let b = u_start + span * ((i + 1) as f64) / (subintervals as f64);
            sum += integrate_arc_length(speed, a, b, 5);
        }

        if let Some(prev) = prev_estimate {
            let tol = 1e-9 * sum.abs().max(1e-300);
            if (sum - prev).abs() < tol {
                return sum;
            }
        }

        if subintervals >= 64 {
            return sum;
        }

        prev_estimate = Some(sum);
        subintervals *= 2;
    }
}

#[cfg(test)]
mod tests;
