pub mod error;
pub use error::{AlgebraError, ArcLengthError, ConstructError, KnotError, NurbsError};

mod view;
pub use view::{NurbsView, VectorNurbsView};

mod scalar;
pub use scalar::{ScalarNurbs, ScalarNurbsRef};

mod vector;
pub use vector::{VectorNurbs, VectorNurbsRef};

pub mod eval;

pub mod arc_length;
pub use arc_length::{ArcLengthTable, ArcLengthTableRef};

pub mod algebra;

pub mod knot;
pub use knot::KnotVector;

pub mod bezier;
pub mod chebyshev;
pub use bezier::BezierPiece;

pub const MAX_DEGREE: usize = 20;

pub const WORKSPACE_SIZE: usize = MAX_DEGREE + 1;

pub const MIN_PARAMETRIC_SPEED: f64 = 1e-9;

const _: () = assert!(WORKSPACE_SIZE == MAX_DEGREE + 1);
const _: () = assert!(MIN_PARAMETRIC_SPEED > 0.0);
