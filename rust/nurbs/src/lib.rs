pub mod error;
pub use error::{AlgebraError, ConstructError, KnotError, NurbsError};

pub mod view;

pub mod scalar;
pub use scalar::ScalarNurbs;

pub mod vector;
pub use vector::VectorNurbs;

pub mod eval;

pub mod arc_length;

pub mod algebra;

pub mod knot;

pub mod bezier;
pub mod chebyshev;
pub use bezier::BezierPiece;

pub const MAX_DEGREE: usize = 20;

pub const WORKSPACE_SIZE: usize = MAX_DEGREE + 1;

pub const MIN_PARAMETRIC_SPEED: f64 = 1e-9;

const _: () = assert!(WORKSPACE_SIZE == MAX_DEGREE + 1);
const _: () = assert!(MIN_PARAMETRIC_SPEED > 0.0);

/// `a * b + c` contracted the fastest way the target allows. Hot numeric
/// loops must use this instead of `f64::mul_add`: wasm32 has no fma
/// instruction, so `mul_add` lowers there to a software double-double
/// emulation costing ~100x the two ops it replaces (measured 5x on the whole
/// playground pipeline).
#[inline]
pub fn fmadd(a: f64, b: f64, c: f64) -> f64 {
    #[cfg(target_arch = "wasm32")]
    {
        a * b + c
    }
    #[cfg(not(target_arch = "wasm32"))]
    {
        a.mul_add(b, c)
    }
}
