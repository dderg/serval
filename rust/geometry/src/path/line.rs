use crate::GeometryError;

use super::profile::CurvatureProfile;

#[derive(Debug, Clone, PartialEq)]
pub struct Line {
    pub start: [f64; 3],
    pub end: [f64; 3],
}

impl Line {
    pub fn try_new(start: [f64; 3], end: [f64; 3]) -> Result<Self, GeometryError> {
        let len = crate::vec3::dist(start, end);
        if len == 0.0 {
            return Err(GeometryError::ZeroMotion);
        }
        Ok(Self { start, end })
    }

    pub fn length(&self) -> f64 {
        crate::vec3::dist(self.start, self.end)
    }
}

impl CurvatureProfile for Line {
    fn s_len(&self) -> f64 {
        self.length()
    }

    fn kappa(&self, _s: f64) -> f64 {
        0.0
    }

    fn dkappa_ds(&self, _s: f64) -> f64 {
        0.0
    }

    fn kappa_peak(&self) -> (f64, f64) {
        (0.0, 0.0)
    }

    fn kappa_endpoints(&self) -> (f64, f64) {
        (0.0, 0.0)
    }
}
