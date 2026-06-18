use crate::GeometryError;

use super::profile::CurvatureProfile;

#[derive(Debug, Clone, PartialEq)]
pub struct Line {
    pub start: [f64; 3],
    pub end: [f64; 3],
}

impl Line {
    pub fn try_new(start: [f64; 3], end: [f64; 3]) -> Result<Self, GeometryError> {
        let len = euclidean_distance(start, end);
        if len == 0.0 {
            return Err(GeometryError::ZeroMotion);
        }
        Ok(Self { start, end })
    }

    pub fn length(&self) -> f64 {
        euclidean_distance(self.start, self.end)
    }
}

fn euclidean_distance(a: [f64; 3], b: [f64; 3]) -> f64 {
    let dx = b[0] - a[0];
    let dy = b[1] - a[1];
    let dz = b[2] - a[2];
    (dx * dx + dy * dy + dz * dz).sqrt()
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
