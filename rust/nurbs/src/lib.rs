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
