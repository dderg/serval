use nurbs::VectorNurbs;

/// A follower axis (extruder) slaved to a segment's arc length. The ratio is
/// `de/ds` and may ramp linearly along the segment: it runs from `ratio` at the
/// start to `ratio_end` at the end, so the extruder position is
/// `e(s) = e0 + ratio·s + (ratio_end − ratio)·s²/(2·len)`. The common slicer
/// case is constant (`ratio == ratio_end`); ramps only arise inside corner
/// blends, where they keep the extruder velocity continuous across the corner.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FollowerDemand {
    pub axis_index: usize,
    pub ratio: f64,
    pub ratio_end: f64,
}

impl FollowerDemand {
    #[must_use]
    pub fn constant(axis_index: usize, ratio: f64) -> Self {
        Self {
            axis_index,
            ratio,
            ratio_end: ratio,
        }
    }

    #[must_use]
    pub fn ramp(axis_index: usize, ratio_start: f64, ratio_end: f64) -> Self {
        Self {
            axis_index,
            ratio: ratio_start,
            ratio_end,
        }
    }

    #[must_use]
    pub fn is_ramped(&self) -> bool {
        self.ratio_end != self.ratio
    }

    #[must_use]
    pub fn max_abs_ratio(&self) -> f64 {
        self.ratio.abs().max(self.ratio_end.abs())
    }

    /// `de/ds` at arc-length `s` over a segment of length `len`.
    #[must_use]
    pub fn ratio_at(&self, s: f64, len: f64) -> f64 {
        if self.ratio_end == self.ratio {
            self.ratio
        } else {
            self.ratio + (self.ratio_end - self.ratio) * (s / len)
        }
    }

    /// The ramp slope `dr/ds = (ratio_end − ratio)/len`; zero for a constant.
    #[must_use]
    pub fn ratio_slope(&self, len: f64) -> f64 {
        if self.ratio_end == self.ratio {
            0.0
        } else {
            (self.ratio_end - self.ratio) / len
        }
    }

    /// Extruder offset from the segment start at arc-length `s`:
    /// `ratio·s + (ratio_end − ratio)·s²/(2·len)`.
    #[must_use]
    pub fn offset_at(&self, s: f64, len: f64) -> f64 {
        if self.ratio_end == self.ratio {
            self.ratio * s
        } else {
            self.ratio * s + (self.ratio_end - self.ratio) * s * s / (2.0 * len)
        }
    }

    /// Total extruder delta over the whole segment: `(ratio + ratio_end)/2·len`.
    #[must_use]
    pub fn delta_over(&self, len: f64) -> f64 {
        0.5 * (self.ratio + self.ratio_end) * len
    }

    /// The sub-span `[s_lo, s_hi]` of a length-`len` segment as its own demand,
    /// interpolating the ramp at each end. A constant stays constant.
    #[must_use]
    pub fn span(&self, s_lo: f64, s_hi: f64, len: f64) -> Self {
        Self {
            axis_index: self.axis_index,
            ratio: self.ratio_at(s_lo, len),
            ratio_end: self.ratio_at(s_hi, len),
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct CubicSegment {
    pub xyz: VectorNurbs<3>,
    pub followers: Vec<FollowerDemand>,
    pub feedrate_mm_s: f64,
    pub source: SourceRange,
    pub virtual_path_mm: Option<f64>,
}

impl CubicSegment {
    pub fn try_new(
        xyz: VectorNurbs<3>,
        followers: Vec<FollowerDemand>,
        feedrate_mm_s: f64,
        source: SourceRange,
    ) -> Result<Self, crate::GeometryError> {
        if xyz.degree() != 3 {
            return Err(crate::GeometryError::NotSinglePieceCubic {
                reason: "degree != 3",
            });
        }
        if xyz.control_points().len() != 4 {
            return Err(crate::GeometryError::NotSinglePieceCubic {
                reason: "control_points.len() != 4",
            });
        }
        let expected_knots: [f64; 8] = [0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];
        if xyz.knots() != expected_knots.as_slice() {
            return Err(crate::GeometryError::NotSinglePieceCubic {
                reason: "knot vector not clamped [0,0,0,0,1,1,1,1]",
            });
        }

        for (i, f) in followers.iter().enumerate() {
            if !f.ratio.is_finite() || !f.ratio_end.is_finite() {
                return Err(crate::GeometryError::FollowerInvariantViolation {
                    reason: "follower ratio must be finite",
                });
            }
            if f.max_abs_ratio() == 0.0 {
                return Err(crate::GeometryError::FollowerInvariantViolation {
                    reason: "follower ratio must be nonzero",
                });
            }
            if followers[..i].iter().any(|p| p.axis_index == f.axis_index) {
                return Err(crate::GeometryError::FollowerInvariantViolation {
                    reason: "duplicate follower axis",
                });
            }
        }

        for cp in xyz.control_points() {
            for &v in cp {
                if !v.is_finite() {
                    return Err(crate::GeometryError::NotSinglePieceCubic {
                        reason: "control point contains non-finite value",
                    });
                }
            }
        }
        if !feedrate_mm_s.is_finite() {
            return Err(crate::GeometryError::FollowerInvariantViolation {
                reason: "feedrate_mm_s must be finite",
            });
        }

        Ok(Self {
            xyz,
            followers,
            feedrate_mm_s,
            virtual_path_mm: None,
            source,
        })
    }

    pub fn try_new_virtual(
        xyz: VectorNurbs<3>,
        followers: Vec<FollowerDemand>,
        feedrate_mm_s: f64,
        source: SourceRange,
        virtual_path_mm: f64,
    ) -> Result<Self, crate::GeometryError> {
        if !(virtual_path_mm.is_finite() && virtual_path_mm > 0.0) {
            return Err(crate::GeometryError::FollowerInvariantViolation {
                reason: "virtual path length must be finite and positive",
            });
        }
        if followers.is_empty() {
            return Err(crate::GeometryError::FollowerInvariantViolation {
                reason: "virtual path requires at least one follower",
            });
        }
        let first = xyz.control_points()[0];
        let displaced = xyz
            .control_points()
            .iter()
            .any(|p| p.iter().zip(&first).any(|(a, b)| (a - b).abs() > 1e-9));
        if displaced {
            return Err(crate::GeometryError::FollowerInvariantViolation {
                reason: "virtual path xyz curve must have zero displacement",
            });
        }
        let mut segment = Self::try_new(xyz, followers, feedrate_mm_s, source)?;
        segment.virtual_path_mm = Some(virtual_path_mm);
        Ok(segment)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SourceRange {
    pub start_line: u32,
    pub end_line: u32,
}

#[cfg(test)]
mod tests;
