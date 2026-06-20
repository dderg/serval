pub mod arc;
mod basis;
pub mod clothoid;
pub mod line;
pub mod lowering;
pub mod profile;

pub use arc::Arc;
pub use clothoid::Clothoid;
pub use line::Line;
pub use profile::CurvatureProfile;

use crate::{FollowerDemand, GeometryError};

#[derive(Debug, Clone, PartialEq)]
pub enum Segment {
    Line(Line),
    Arc(Arc),
    Clothoid(Clothoid),
}

impl CurvatureProfile for Segment {
    fn s_len(&self) -> f64 {
        match self {
            Segment::Line(l) => l.s_len(),
            Segment::Arc(a) => a.s_len(),
            Segment::Clothoid(c) => c.s_len(),
        }
    }

    fn kappa(&self, s: f64) -> f64 {
        match self {
            Segment::Line(l) => l.kappa(s),
            Segment::Arc(a) => a.kappa(s),
            Segment::Clothoid(c) => c.kappa(s),
        }
    }

    fn dkappa_ds(&self, s: f64) -> f64 {
        match self {
            Segment::Line(l) => l.dkappa_ds(s),
            Segment::Arc(a) => a.dkappa_ds(s),
            Segment::Clothoid(c) => c.dkappa_ds(s),
        }
    }

    fn kappa_peak(&self) -> (f64, f64) {
        match self {
            Segment::Line(l) => l.kappa_peak(),
            Segment::Arc(a) => a.kappa_peak(),
            Segment::Clothoid(c) => c.kappa_peak(),
        }
    }

    fn kappa_endpoints(&self) -> (f64, f64) {
        match self {
            Segment::Line(l) => l.kappa_endpoints(),
            Segment::Arc(a) => a.kappa_endpoints(),
            Segment::Clothoid(c) => c.kappa_endpoints(),
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct PathSegment {
    pub spatial: Option<Segment>,
    pub followers: Vec<FollowerDemand>,
    pub virtual_path_mm: Option<f64>,
}

impl PathSegment {
    pub fn try_new(
        spatial: Segment,
        followers: Vec<FollowerDemand>,
    ) -> Result<Self, GeometryError> {
        validate_followers(&followers)?;
        Ok(Self {
            spatial: Some(spatial),
            followers,
            virtual_path_mm: None,
        })
    }

    pub fn try_new_virtual(
        followers: Vec<FollowerDemand>,
        virtual_path_mm: f64,
    ) -> Result<Self, GeometryError> {
        if !(virtual_path_mm.is_finite() && virtual_path_mm > 0.0) {
            return Err(GeometryError::FollowerInvariantViolation {
                reason: "virtual path length must be finite and positive",
            });
        }
        if followers.is_empty() {
            return Err(GeometryError::ZeroMotion);
        }
        validate_followers(&followers)?;
        Ok(Self {
            spatial: None,
            followers,
            virtual_path_mm: Some(virtual_path_mm),
        })
    }

    pub fn s_len(&self) -> f64 {
        if let Some(vp) = self.virtual_path_mm {
            vp
        } else {
            self.spatial
                .as_ref()
                .expect("non-virtual PathSegment must have spatial segment")
                .s_len()
        }
    }
}

fn validate_followers(followers: &[FollowerDemand]) -> Result<(), GeometryError> {
    for (i, f) in followers.iter().enumerate() {
        if !f.ratio.is_finite() || f.ratio == 0.0 {
            return Err(GeometryError::FollowerInvariantViolation {
                reason: "follower ratio must be finite and nonzero",
            });
        }
        if followers[..i].iter().any(|p| p.axis_index == f.axis_index) {
            return Err(GeometryError::FollowerInvariantViolation {
                reason: "duplicate follower axis",
            });
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests;
