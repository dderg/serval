use crate::vec3::{dot, norm_sq};

const ORTHONORMALITY_TOL: f64 = 1e-9;

pub(super) fn is_orthonormal(u: [f64; 3], v: [f64; 3]) -> bool {
    (norm_sq(u) - 1.0).abs() < ORTHONORMALITY_TOL
        && (norm_sq(v) - 1.0).abs() < ORTHONORMALITY_TOL
        && dot(u, v).abs() < ORTHONORMALITY_TOL
}
