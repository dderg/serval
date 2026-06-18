const ORTHONORMALITY_TOL: f64 = 1e-9;

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn norm_sq(a: [f64; 3]) -> f64 {
    dot(a, a)
}

pub(super) fn is_orthonormal(u: [f64; 3], v: [f64; 3]) -> bool {
    (norm_sq(u) - 1.0).abs() < ORTHONORMALITY_TOL
        && (norm_sq(v) - 1.0).abs() < ORTHONORMALITY_TOL
        && dot(u, v).abs() < ORTHONORMALITY_TOL
}
