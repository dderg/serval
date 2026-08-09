use super::{Arc, Clothoid, Line, Segment};

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
            self.radius * libm::cos(theta),
            self.u,
            self.radius * libm::sin(theta),
            self.v,
        );
        translate(self.origin, offset)
    }

    fn heading_at(&self, s: f64) -> [f64; 3] {
        let theta = self.angle_at(s);
        let sign = self.sweep.signum();
        axpby(
            -sign * libm::sin(theta),
            self.u,
            sign * libm::cos(theta),
            self.v,
        )
    }

    fn dheading_ds(&self, s: f64) -> [f64; 3] {
        // d/ds of the heading turns toward the centre at rate 1/radius, independent
        // of sweep direction: `-(1/r)·(cosθ·u + sinθ·v)` (the inward radial).
        let theta = self.angle_at(s);
        let inv_r = 1.0 / self.radius;
        axpby(
            -inv_r * libm::cos(theta),
            self.u,
            -inv_r * libm::sin(theta),
            self.v,
        )
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
        axpby(libm::cos(phi), self.u, libm::sin(phi), self.v)
    }

    fn dheading_ds(&self, s: f64) -> [f64; 3] {
        // `κ(s)·n̂`: the local curvature `κ_0 + σ·s` times the normal (heading
        // rotated +90°), `-sinφ·u + cosφ·v`.
        let phi = self.heading_angle_at(s);
        let kappa = self.kappa_0 + self.sigma * s;
        axpby(
            -kappa * libm::sin(phi),
            self.u,
            kappa * libm::cos(phi),
            self.v,
        )
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
