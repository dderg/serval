use nurbs::VectorNurbs;
use nurbs::eval::{vector_derivative, vector_eval};

const THETA_COLLINEAR_RAD: f64 = 1e-3;

#[must_use]
pub fn are_collinear(left: &VectorNurbs<f64, 3>, right: &VectorNurbs<f64, 3>) -> bool {
    let t_left = forward_unit_tangent_at_end(left);
    let t_right = forward_unit_tangent_at_start(right);
    tangents_collinear(&t_left, &t_right)
}

fn tangents_collinear(t_left: &[f64; 3], t_right: &[f64; 3]) -> bool {
    let left_degenerate = t_left[0].abs() + t_left[1].abs() + t_left[2].abs() < 1e-12;
    let right_degenerate = t_right[0].abs() + t_right[1].abs() + t_right[2].abs() < 1e-12;
    if left_degenerate || right_degenerate {
        return false;
    }
    turn_angle(t_left, t_right) <= THETA_COLLINEAR_RAD
}

fn turn_angle(t_left: &[f64; 3], t_right: &[f64; 3]) -> f64 {
    let dot =
        (t_left[0] * t_right[0] + t_left[1] * t_right[1] + t_left[2] * t_right[2]).clamp(-1.0, 1.0);
    let sin_half = ((1.0 - dot) * 0.5).max(0.0).sqrt();
    2.0 * sin_half.asin()
}

fn forward_unit_tangent_at_end(curve: &VectorNurbs<f64, 3>) -> [f64; 3] {
    let u_end = *curve.knots().last().expect("knots non-empty");
    let d1 = vector_derivative(curve);
    let t = vector_eval(&d1.as_view(), u_end);
    normalize_3(t)
}

fn forward_unit_tangent_at_start(curve: &VectorNurbs<f64, 3>) -> [f64; 3] {
    let u_start = curve.knots()[0];
    let d1 = vector_derivative(curve);
    let t = vector_eval(&d1.as_view(), u_start);
    normalize_3(t)
}

#[inline]
fn normalize_3(v: [f64; 3]) -> [f64; 3] {
    let m = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    if m < 1e-12 {
        [0.0; 3]
    } else {
        [v[0] / m, v[1] / m, v[2] / m]
    }
}

#[cfg(test)]
mod tests;
