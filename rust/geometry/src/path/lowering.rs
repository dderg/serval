use crate::GeometryError;

use super::{Arc, Clothoid, Line, PathSegment, Segment};

mod fresnel;

pub trait PositionProfile {
    fn point_at(&self, s: f64) -> [f64; 3];
    fn heading_at(&self, s: f64) -> [f64; 3];
    /// Derivative of the unit heading along arc length, `dĥ/ds = κ·n̂`. With the
    /// tangential acceleration `a_t` and speed `v`, the exact per-axis acceleration
    /// is `a_t·ĥ + v²·(dĥ/ds)` — the centripetal term the lowering needs without a
    /// finite difference.
    fn dheading_ds(&self, s: f64) -> [f64; 3];
}

#[derive(Debug, Clone, PartialEq)]
pub struct LoweredSample {
    pub t_s: f64,
    pub position: Option<[f64; 3]>,
    pub followers: Vec<f64>,
}

pub fn lower_constant_speed(
    seg: &PathSegment,
    speed_mm_s: f64,
    rate_hz: f64,
) -> Result<Vec<LoweredSample>, GeometryError> {
    if !(speed_mm_s.is_finite() && speed_mm_s > 0.0) {
        return Err(GeometryError::InvalidLowering {
            reason: "speed must be finite and positive",
        });
    }
    if !(rate_hz.is_finite() && rate_hz > 0.0) {
        return Err(GeometryError::InvalidLowering {
            reason: "rate must be finite and positive",
        });
    }
    if let Some(spatial) = &seg.spatial {
        if !spatial_anchors_finite(spatial) {
            return Err(GeometryError::InvalidLowering {
                reason: "spatial anchor is not finite",
            });
        }
    }

    let s_len = seg.s_len();
    let dt = 1.0 / rate_hz;
    let total_t = s_len / speed_mm_s;
    let count = total_t / dt;
    if count >= usize::MAX as f64 {
        return Err(GeometryError::InvalidLowering {
            reason: "sample count exceeds addressable range",
        });
    }
    let n = count.ceil() as usize;

    let mut samples = Vec::with_capacity(n + 1);
    for k in 0..=n {
        let t = (k as f64 * dt).min(total_t);
        let s = (speed_mm_s * t).min(s_len);
        let position = seg.spatial.as_ref().map(|spatial| spatial.point_at(s));
        let followers = seg.followers.iter().map(|f| f.ratio * s).collect();
        samples.push(LoweredSample {
            t_s: t,
            position,
            followers,
        });
    }
    Ok(samples)
}

fn spatial_anchors_finite(seg: &Segment) -> bool {
    match seg {
        Segment::Line(l) => l.start.iter().chain(l.end.iter()).all(|c| c.is_finite()),
        Segment::Arc(a) => a.start_angle.is_finite() && a.origin.iter().all(|c| c.is_finite()),
        Segment::Clothoid(c) => c.start_pose.iter().all(|p| p.is_finite()),
    }
}

fn axpby(a: f64, u: [f64; 3], b: f64, v: [f64; 3]) -> [f64; 3] {
    [
        a * u[0] + b * v[0],
        a * u[1] + b * v[1],
        a * u[2] + b * v[2],
    ]
}

fn translate(origin: [f64; 3], offset: [f64; 3]) -> [f64; 3] {
    [
        origin[0] + offset[0],
        origin[1] + offset[1],
        origin[2] + offset[2],
    ]
}

impl PositionProfile for Line {
    fn point_at(&self, s: f64) -> [f64; 3] {
        let t = s / self.length();
        [
            self.start[0] + t * (self.end[0] - self.start[0]),
            self.start[1] + t * (self.end[1] - self.start[1]),
            self.start[2] + t * (self.end[2] - self.start[2]),
        ]
    }

    fn heading_at(&self, _s: f64) -> [f64; 3] {
        let len = self.length();
        [
            (self.end[0] - self.start[0]) / len,
            (self.end[1] - self.start[1]) / len,
            (self.end[2] - self.start[2]) / len,
        ]
    }

    fn dheading_ds(&self, _s: f64) -> [f64; 3] {
        [0.0, 0.0, 0.0]
    }
}

impl Arc {
    fn angle_at(&self, s: f64) -> f64 {
        self.start_angle + self.sweep.signum() * s / self.radius
    }
}

impl PositionProfile for Arc {
    fn point_at(&self, s: f64) -> [f64; 3] {
        let theta = self.angle_at(s);
        let offset = axpby(
            self.radius * theta.cos(),
            self.u,
            self.radius * theta.sin(),
            self.v,
        );
        translate(self.origin, offset)
    }

    fn heading_at(&self, s: f64) -> [f64; 3] {
        let theta = self.angle_at(s);
        let sign = self.sweep.signum();
        axpby(-sign * theta.sin(), self.u, sign * theta.cos(), self.v)
    }

    fn dheading_ds(&self, s: f64) -> [f64; 3] {
        // d/ds of the heading turns toward the centre at rate 1/radius, independent
        // of sweep direction: `-(1/r)·(cosθ·u + sinθ·v)` (the inward radial).
        let theta = self.angle_at(s);
        let inv_r = 1.0 / self.radius;
        axpby(-inv_r * theta.cos(), self.u, -inv_r * theta.sin(), self.v)
    }
}

impl Clothoid {
    fn heading_angle_at(&self, s: f64) -> f64 {
        self.kappa_0 * s + 0.5 * self.sigma * s * s
    }
}

impl PositionProfile for Clothoid {
    fn point_at(&self, s: f64) -> [f64; 3] {
        let (cx, cy) = fresnel::clothoid_offset(self.kappa_0, self.sigma, s);
        translate(self.start_pose, axpby(cx, self.u, cy, self.v))
    }

    fn heading_at(&self, s: f64) -> [f64; 3] {
        let phi = self.heading_angle_at(s);
        axpby(phi.cos(), self.u, phi.sin(), self.v)
    }

    fn dheading_ds(&self, s: f64) -> [f64; 3] {
        // `κ(s)·n̂`: the local curvature `κ_0 + σ·s` times the normal (heading
        // rotated +90°), `-sinφ·u + cosφ·v`.
        let phi = self.heading_angle_at(s);
        let kappa = self.kappa_0 + self.sigma * s;
        axpby(-kappa * phi.sin(), self.u, kappa * phi.cos(), self.v)
    }
}

impl PositionProfile for Segment {
    fn point_at(&self, s: f64) -> [f64; 3] {
        match self {
            Segment::Line(l) => l.point_at(s),
            Segment::Arc(a) => a.point_at(s),
            Segment::Clothoid(c) => c.point_at(s),
        }
    }

    fn heading_at(&self, s: f64) -> [f64; 3] {
        match self {
            Segment::Line(l) => l.heading_at(s),
            Segment::Arc(a) => a.heading_at(s),
            Segment::Clothoid(c) => c.heading_at(s),
        }
    }

    fn dheading_ds(&self, s: f64) -> [f64; 3] {
        match self {
            Segment::Line(l) => l.dheading_ds(s),
            Segment::Arc(a) => a.dheading_ds(s),
            Segment::Clothoid(c) => c.dheading_ds(s),
        }
    }
}

#[cfg(test)]
mod tests;
