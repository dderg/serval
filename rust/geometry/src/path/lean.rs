use std::f64::consts::{FRAC_PI_2, PI};

use super::{Arc, Clothoid, Line, Segment};

/// How much of a segment's motion one axis carries: the largest component
/// on that axis of the unit heading and of the unit normal anywhere along
/// the segment, and of any unit vector in its plane. A path acceleration
/// with tangential part `a_t` and normal part `a_n` puts `a_t·ĥ_k + a_n·n̂_k`
/// on axis `k`, so `|a_k| ≤ |a_t|·heading + |a_n|·normal` and, `ĥ` and `n̂`
/// spanning the plane, `|a_k| ≤ |a|·plane`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AxisLean {
    pub heading: f64,
    pub normal: f64,
    pub plane: f64,
}

impl Segment {
    #[must_use]
    pub fn lean_on(&self, axis: usize) -> AxisLean {
        match self {
            Segment::Line(line) => line.lean_on(axis),
            Segment::Arc(arc) => arc.lean_on(axis),
            Segment::Clothoid(clothoid) => clothoid.lean_on(axis),
        }
    }
}

impl Line {
    fn lean_on(&self, axis: usize) -> AxisLean {
        let heading = (self.end[axis] - self.start[axis]).abs() / self.length();
        AxisLean {
            heading,
            normal: 0.0,
            plane: heading,
        }
    }
}

impl Arc {
    fn lean_on(&self, axis: usize) -> AxisLean {
        let quarter_turn = self.sweep.signum() * FRAC_PI_2;
        let first = self.start_angle + quarter_turn;
        let last = self.start_angle + self.sweep + quarter_turn;
        planar_lean(self.u[axis], self.v[axis], first.min(last), first.max(last))
    }
}

impl Clothoid {
    fn lean_on(&self, axis: usize) -> AxisLean {
        let heading_angle = |s: f64| self.kappa_0 * s + 0.5 * self.sigma * s * s;
        let mut lo = heading_angle(0.0).min(heading_angle(self.length));
        let mut hi = heading_angle(0.0).max(heading_angle(self.length));
        let turning_point = -self.kappa_0 / self.sigma;
        if turning_point > 0.0 && turning_point < self.length {
            lo = lo.min(heading_angle(turning_point));
            hi = hi.max(heading_angle(turning_point));
        }
        planar_lean(self.u[axis], self.v[axis], lo, hi)
    }
}

/// A planar curve heads along `cos φ·u + sin φ·v` with normal
/// `−sin φ·u + cos φ·v`; on axis `k` these are `plane·cos(φ − α)` and
/// `−plane·sin(φ − α)` with `plane = |(u_k, v_k)|`, `α = atan2(v_k, u_k)`,
/// maximised over the heading angles `[lo, hi]` the curve sweeps.
fn planar_lean(u_k: f64, v_k: f64, lo: f64, hi: f64) -> AxisLean {
    let plane = u_k.hypot(v_k);
    let alpha = v_k.atan2(u_k);
    AxisLean {
        heading: plane * max_abs_cos(lo - alpha, hi - alpha),
        normal: plane * max_abs_cos(lo - alpha - FRAC_PI_2, hi - alpha - FRAC_PI_2),
        plane,
    }
}

fn max_abs_cos(lo: f64, hi: f64) -> f64 {
    let first_extremum = (lo / PI).ceil() * PI;
    if first_extremum <= hi {
        return 1.0;
    }
    libm::cos(lo).abs().max(libm::cos(hi).abs())
}

#[cfg(test)]
mod tests;
