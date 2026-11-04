use std::f64::consts::{FRAC_PI_2, PI};

use super::{Arc, Clothoid, Line, Segment};
use crate::vec3;

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

    /// The largest `|d·t̂|` anywhere along the segment: how fast a motor
    /// whose position is `d·p` moves per unit of path speed, at worst.
    #[must_use]
    pub fn heading_lean_along(&self, direction: [f64; 3]) -> f64 {
        match self {
            Segment::Line(line) => {
                vec3::dot(direction, vec3::sub(line.end, line.start)).abs() / line.length()
            }
            Segment::Arc(arc) => {
                let (lo, hi) = arc.heading_range();
                planar_heading_lean(
                    vec3::dot(direction, arc.u),
                    vec3::dot(direction, arc.v),
                    lo,
                    hi,
                )
            }
            Segment::Clothoid(clothoid) => {
                let (lo, hi) = clothoid.heading_range();
                planar_heading_lean(
                    vec3::dot(direction, clothoid.u),
                    vec3::dot(direction, clothoid.v),
                    lo,
                    hi,
                )
            }
        }
    }

    /// The least and largest component on `axis` of the unit heading
    /// anywhere along the segment.
    #[must_use]
    pub fn heading_span_on(&self, axis: usize) -> (f64, f64) {
        match self {
            Segment::Line(line) => {
                let heading = (line.end[axis] - line.start[axis]) / line.length();
                (heading, heading)
            }
            Segment::Arc(arc) => {
                let (lo, hi) = arc.heading_range();
                planar_heading_span(arc.u[axis], arc.v[axis], lo, hi)
            }
            Segment::Clothoid(clothoid) => {
                let (lo, hi) = clothoid.heading_range();
                planar_heading_span(clothoid.u[axis], clothoid.v[axis], lo, hi)
            }
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
        let (lo, hi) = self.heading_range();
        planar_lean(self.u[axis], self.v[axis], lo, hi)
    }

    fn heading_range(&self) -> (f64, f64) {
        let quarter_turn = self.sweep.signum() * FRAC_PI_2;
        let first = self.start_angle + quarter_turn;
        let last = self.start_angle + self.sweep + quarter_turn;
        (first.min(last), first.max(last))
    }
}

impl Clothoid {
    fn lean_on(&self, axis: usize) -> AxisLean {
        let (lo, hi) = self.heading_range();
        planar_lean(self.u[axis], self.v[axis], lo, hi)
    }

    fn heading_range(&self) -> (f64, f64) {
        let heading_angle = |s: f64| self.kappa_0 * s + 0.5 * self.sigma * s * s;
        let mut lo = heading_angle(0.0).min(heading_angle(self.length));
        let mut hi = heading_angle(0.0).max(heading_angle(self.length));
        let turning_point = -self.kappa_0 / self.sigma;
        if turning_point > 0.0 && turning_point < self.length {
            lo = lo.min(heading_angle(turning_point));
            hi = hi.max(heading_angle(turning_point));
        }
        (lo, hi)
    }
}

/// A planar curve heads along `cos φ·u + sin φ·v` with normal
/// `−sin φ·u + cos φ·v`; on axis `k` these are `plane·cos(φ − α)` and
/// `−plane·sin(φ − α)` with `plane = |(u_k, v_k)|`, `α = atan2(v_k, u_k)`,
/// maximised over the heading angles `[lo, hi]` the curve sweeps.
fn planar_lean(u_k: f64, v_k: f64, lo: f64, hi: f64) -> AxisLean {
    let plane = libm::hypot(u_k, v_k);
    let alpha = libm::atan2(v_k, u_k);
    AxisLean {
        heading: planar_heading_lean(u_k, v_k, lo, hi),
        normal: plane * max_abs_cos(lo - alpha - FRAC_PI_2, hi - alpha - FRAC_PI_2),
        plane,
    }
}

fn planar_heading_lean(u_k: f64, v_k: f64, lo: f64, hi: f64) -> f64 {
    let plane = libm::hypot(u_k, v_k);
    let alpha = libm::atan2(v_k, u_k);
    plane * max_abs_cos(lo - alpha, hi - alpha)
}

/// The range of `plane·cos(φ − α)` over the heading angles `[lo, hi]`.
fn planar_heading_span(u_k: f64, v_k: f64, lo: f64, hi: f64) -> (f64, f64) {
    let plane = libm::hypot(u_k, v_k);
    let alpha = libm::atan2(v_k, u_k);
    let (from, to) = (lo - alpha, hi - alpha);
    let ends = [libm::cos(from), libm::cos(to)];
    let crest = (from / (2.0 * PI)).ceil() * 2.0 * PI;
    let trough = ((from - PI) / (2.0 * PI)).ceil() * 2.0 * PI + PI;
    let top = if crest <= to {
        1.0
    } else {
        ends[0].max(ends[1])
    };
    let bottom = if trough <= to {
        -1.0
    } else {
        ends[0].min(ends[1])
    };
    (plane * bottom, plane * top)
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
