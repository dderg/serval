use crate::GeometryError;

use super::basis::is_orthonormal;
use super::profile::CurvatureProfile;

#[derive(Debug, Clone, PartialEq)]
pub struct Arc {
    pub origin: [f64; 3],
    pub u: [f64; 3],
    pub v: [f64; 3],
    pub radius: f64,
    pub start_angle: f64,
    pub sweep: f64,
}

impl Arc {
    pub fn try_new(
        origin: [f64; 3],
        u: [f64; 3],
        v: [f64; 3],
        radius: f64,
        start_angle: f64,
        sweep: f64,
    ) -> Result<Self, GeometryError> {
        if radius <= 0.0 || !radius.is_finite() {
            return Err(GeometryError::DegenerateArc {
                reason: "radius must be positive and finite",
            });
        }
        if sweep == 0.0 || !sweep.is_finite() {
            return Err(GeometryError::DegenerateArc {
                reason: "sweep must be nonzero and finite",
            });
        }
        if !(radius * sweep.abs() > 0.0) {
            return Err(GeometryError::DegenerateArc {
                reason: "radius * |sweep| underflows to a zero-length arc",
            });
        }
        if !is_orthonormal(u, v) {
            return Err(GeometryError::NonPlanarBasis {
                reason: "u and v must be orthonormal unit vectors",
            });
        }
        Ok(Self {
            origin,
            u,
            v,
            radius,
            start_angle,
            sweep,
        })
    }
}

impl CurvatureProfile for Arc {
    fn s_len(&self) -> f64 {
        self.radius * self.sweep.abs()
    }

    fn kappa(&self, _s: f64) -> f64 {
        1.0 / self.radius
    }

    fn dkappa_ds(&self, _s: f64) -> f64 {
        0.0
    }

    fn kappa_peak(&self) -> (f64, f64) {
        let kappa = 1.0 / self.radius;
        (0.0, kappa)
    }

    fn kappa_endpoints(&self) -> (f64, f64) {
        let kappa = 1.0 / self.radius;
        (kappa, kappa)
    }
}
