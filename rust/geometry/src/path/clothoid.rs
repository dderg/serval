use crate::GeometryError;

use super::basis::is_orthonormal;
use super::profile::CurvatureProfile;

#[derive(Debug, Clone, PartialEq)]
pub struct Clothoid {
    pub start_pose: [f64; 3],
    pub u: [f64; 3],
    pub v: [f64; 3],
    pub kappa_0: f64,
    pub sigma: f64,
    pub length: f64,
}

impl Clothoid {
    pub fn try_new(
        start_pose: [f64; 3],
        u: [f64; 3],
        v: [f64; 3],
        kappa_0: f64,
        sigma: f64,
        length: f64,
    ) -> Result<Self, GeometryError> {
        if !length.is_finite() || length <= 0.0 {
            return Err(GeometryError::DegenerateClothoid {
                reason: "length must be finite and positive",
            });
        }
        if !kappa_0.is_finite() {
            return Err(GeometryError::DegenerateClothoid {
                reason: "kappa_0 must be finite",
            });
        }
        if !sigma.is_finite() {
            return Err(GeometryError::DegenerateClothoid {
                reason: "sigma must be finite",
            });
        }
        if !is_orthonormal(u, v) {
            return Err(GeometryError::NonPlanarBasis {
                reason: "u and v must be orthonormal unit vectors",
            });
        }
        Ok(Self {
            start_pose,
            u,
            v,
            kappa_0,
            sigma,
            length,
        })
    }
}

impl CurvatureProfile for Clothoid {
    fn s_len(&self) -> f64 {
        self.length
    }

    fn kappa(&self, s: f64) -> f64 {
        self.kappa_0 + self.sigma * s
    }

    fn dkappa_ds(&self, _s: f64) -> f64 {
        self.sigma
    }

    fn kappa_peak(&self) -> (f64, f64) {
        let kappa_start = self.kappa_0.abs();
        let kappa_end = (self.kappa_0 + self.sigma * self.length).abs();
        if kappa_start >= kappa_end {
            (0.0, kappa_start)
        } else {
            (self.length, kappa_end)
        }
    }

    fn kappa_endpoints(&self) -> (f64, f64) {
        (self.kappa_0, self.kappa_0 + self.sigma * self.length)
    }
}
