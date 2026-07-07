use nurbs::VectorNurbs;

#[non_exhaustive]
#[derive(Debug, Clone, PartialEq)]
pub enum Segment {
    Cubic(CubicSegment),
    CornerBlend(CornerBlendSlot),
    Junction(JunctionDeviation),
}

#[derive(Debug, Clone, PartialEq)]
pub struct CornerBlendSlot {
    pub position: [f64; 3],
    pub t_in: [f64; 3],
    pub t_out: [f64; 3],
    pub seg_len_in: f64,
    pub seg_len_out: f64,
    pub tolerance_budget_mm: f64,
    pub default_family: BlendFamily,
    pub feedrate_mm_s: f64,
    pub source: SourceRange,
}

#[derive(Debug, Clone, PartialEq)]
pub struct JunctionDeviation {
    pub position: [f64; 3],
    pub angle_deg: f64,
    pub feedrate_mm_s: f64,
    pub source: SourceRange,
}

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
    pub split_info: Option<SplitInfo>,
    pub virtual_path_mm: Option<f64>,
}

impl CubicSegment {
    pub fn try_new(
        xyz: VectorNurbs<3>,
        followers: Vec<FollowerDemand>,
        feedrate_mm_s: f64,
        source: SourceRange,
        split_info: Option<SplitInfo>,
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
        if let Some(info) = &split_info {
            if !info.s_lo_mm.is_finite() || !info.s_hi_mm.is_finite() {
                return Err(crate::GeometryError::FollowerInvariantViolation {
                    reason: "split_info arc-length range contains non-finite value",
                });
            }
        }

        Ok(Self {
            xyz,
            followers,
            feedrate_mm_s,
            virtual_path_mm: None,
            source,
            split_info,
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
        let mut segment = Self::try_new(xyz, followers, feedrate_mm_s, source, None)?;
        segment.virtual_path_mm = Some(virtual_path_mm);
        Ok(segment)
    }
}

#[must_use]
pub fn split_cubic_bezier(xyz: &VectorNurbs<3>, s: f64) -> (VectorNurbs<3>, VectorNurbs<3>) {
    assert_eq!(xyz.degree(), 3, "split_cubic_bezier: degree must be 3");
    let cps = xyz.control_points();
    assert_eq!(
        cps.len(),
        4,
        "split_cubic_bezier: must have 4 control points"
    );
    let expected_knots: [f64; 8] = [0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];
    assert_eq!(
        xyz.knots(),
        expected_knots.as_slice(),
        "split_cubic_bezier: knot vector must be clamped [0,0,0,0,1,1,1,1]",
    );
    assert!(
        s > 0.0 && s < 1.0,
        "split_cubic_bezier: s = {s} must be strictly interior to (0, 1)",
    );

    let p0 = cps[0];
    let p1 = cps[1];
    let p2 = cps[2];
    let p3 = cps[3];

    let lerp = |a: [f64; 3], b: [f64; 3], t: f64| -> [f64; 3] {
        [
            a[0] + (b[0] - a[0]) * t,
            a[1] + (b[1] - a[1]) * t,
            a[2] + (b[2] - a[2]) * t,
        ]
    };
    let q0 = lerp(p0, p1, s);
    let q1 = lerp(p1, p2, s);
    let q2 = lerp(p2, p3, s);
    let r0 = lerp(q0, q1, s);
    let r1 = lerp(q1, q2, s);
    let s0 = lerp(r0, r1, s);

    let knots = vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];
    let left = VectorNurbs::<3>::try_new(3, knots.clone(), vec![p0, q0, r0, s0])
        .expect("split_cubic_bezier: left half is a valid single-piece cubic Bézier");
    let right = VectorNurbs::<3>::try_new(3, knots, vec![s0, r1, q2, p3])
        .expect("split_cubic_bezier: right half is a valid single-piece cubic Bézier");
    (left, right)
}

#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BlendFamily {
    CubicBezier,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SplitInfo {
    pub sub_index: u32,
    pub sub_count: u32,
    pub s_lo_mm: f64,
    pub s_hi_mm: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SourceRange {
    pub start_line: u32,
    pub end_line: u32,
}

#[cfg(test)]
mod tests;
