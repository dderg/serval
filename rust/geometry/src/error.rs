#[derive(Debug, Clone, Copy, PartialEq)]
pub enum GeometryError {
    NotSinglePieceCubic { reason: &'static str },
    FollowerInvariantViolation { reason: &'static str },
    ZeroMotion,
    NonPlanarBasis { reason: &'static str },
    DegenerateArc { reason: &'static str },
    DegenerateClothoid { reason: &'static str },
    InvalidLowering { reason: &'static str },
}
