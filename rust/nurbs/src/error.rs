use core::fmt;

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ConstructError {
    DegreeExceeded { actual: u8, max: u8 },
    KnotCountMismatch { expected: usize, got: usize },
    KnotsNotClamped,
    KnotsNotMonotone,
    DegenerateKnotRange,
}

impl fmt::Display for ConstructError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::DegreeExceeded { actual, max } => {
                write!(f, "degree {actual} exceeds maximum {max}")
            }
            Self::KnotCountMismatch { expected, got } => {
                write!(f, "knot count: expected {expected}, got {got}")
            }
            Self::KnotsNotClamped => write!(f, "knot vector is not clamped open"),
            Self::KnotsNotMonotone => write!(f, "knot vector is not non-decreasing"),
            Self::DegenerateKnotRange => {
                write!(f, "knot range is degenerate (knots[last] <= knots[0])")
            }
        }
    }
}

impl core::error::Error for ConstructError {}

#[derive(Debug, Clone, PartialEq)]
pub enum ArcLengthError {
    ToleranceNotMet {
        achieved_residual: f64,
        samples_used: usize,
    },
    DegenerateCurve,
}

impl fmt::Display for ArcLengthError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::ToleranceNotMet {
                achieved_residual,
                samples_used,
            } => write!(
                f,
                "arc-length builder hit cap of {samples_used} samples; achieved residual {achieved_residual:?}"
            ),
            Self::DegenerateCurve => write!(
                f,
                "arc-length integration produced a curve with total length below MIN_PARAMETRIC_SPEED"
            ),
        }
    }
}

impl core::error::Error for ArcLengthError {}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum AlgebraError {
    DegreeExceeded { result_degree: u8, max: u8 },
    KnotMismatch,
    SupportMismatch,
}

impl fmt::Display for AlgebraError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::DegreeExceeded { result_degree, max } => {
                write!(f, "result degree {result_degree} exceeds maximum {max}")
            }
            Self::KnotMismatch => write!(f, "operands have incompatible knot vectors"),
            Self::SupportMismatch => write!(f, "Bezier pieces have mismatched support"),
        }
    }
}

impl core::error::Error for AlgebraError {}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum KnotError {
    BoundaryInsertion,
    MultiplicityExceeded {
        existing: u8,
        requested: u8,
        max: u8,
    },
    OutOfRange,
    Invalid,
}

impl fmt::Display for KnotError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::BoundaryInsertion => {
                write!(f, "cannot insert knot at clamped boundary")
            }
            Self::MultiplicityExceeded {
                existing,
                requested,
                max,
            } => {
                write!(
                    f,
                    "knot multiplicity {existing} + {requested} exceeds max {max}"
                )
            }
            Self::OutOfRange => write!(f, "knot value out of knot vector range"),
            Self::Invalid => write!(f, "knot vector violates monotone or length invariants"),
        }
    }
}

impl core::error::Error for KnotError {}

#[derive(Debug, Clone, PartialEq)]
pub enum NurbsError {
    Construct(ConstructError),
    ArcLength(ArcLengthError),
    Algebra(AlgebraError),
    Knot(KnotError),
}

impl fmt::Display for NurbsError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Construct(e) => write!(f, "{e}"),
            Self::ArcLength(e) => write!(f, "{e}"),
            Self::Algebra(e) => write!(f, "{e}"),
            Self::Knot(e) => write!(f, "{e}"),
        }
    }
}

impl core::error::Error for NurbsError {}

impl From<ConstructError> for NurbsError {
    fn from(e: ConstructError) -> Self {
        Self::Construct(e)
    }
}
impl From<ArcLengthError> for NurbsError {
    fn from(e: ArcLengthError) -> Self {
        Self::ArcLength(e)
    }
}
impl From<AlgebraError> for NurbsError {
    fn from(e: AlgebraError) -> Self {
        Self::Algebra(e)
    }
}
impl From<KnotError> for NurbsError {
    fn from(e: KnotError) -> Self {
        Self::Knot(e)
    }
}

#[cfg(test)]
mod tests;
