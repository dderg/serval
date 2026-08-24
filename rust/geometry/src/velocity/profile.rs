//! Analytic triple-limited (velocity / acceleration / jerk) motion profile.
//!
//! A profile is a sequence of constant-jerk phases — the 7-segment S-curve and
//! its degeneracies — held as closed-form breakpoints and evaluated exactly.
//! Nothing is integrated on a grid, so rest (`v = 0`) is an ordinary phase
//! boundary rather than a singularity, the acceleration is stored per phase (it
//! is never a finite difference of `v`), and sampling density is decoupled from
//! correctness: every sample is exact wherever it is taken.
//!
//! [`plan`] connects an entry speed to an exit speed across a fixed distance
//! under a flat speed ceiling: ramp up to a peak, optionally cruise, ramp down —
//! each ramp itself jerk-up / hold-accel / jerk-down as the limits allow.

const PHASE_SOLVE_EPS: f64 = 1e-12;

#[cfg(test)]
const EPS: f64 = PHASE_SOLVE_EPS;

#[cfg(test)]
#[derive(Clone, Copy, Debug)]
struct Break {
    s: f64,
    v: f64,
    a: f64,
    /// Jerk of the phase that starts at this breakpoint (zero on the terminal).
    j: f64,
    /// Duration of that phase.
    dt: f64,
}

#[cfg(test)]
pub(super) struct Profile {
    breaks: Vec<Break>,
    length: f64,
}

#[cfg(test)]
impl Profile {
    pub(super) fn at(&self, s: f64) -> (f64, f64) {
        let s = s.clamp(0.0, self.length);
        let last = &self.breaks[self.breaks.len() - 1];
        if s >= last.s - EPS {
            return (last.v, last.a);
        }
        let b = self.breaks[self.locate(s)];
        let phase = StraightPhase {
            t0: 0.0,
            dt: b.dt,
            s0: b.s,
            v0: b.v,
            a0: b.a,
            j: b.j,
        };
        let t = phase
            .time_at_distance(s)
            .expect("profile phase distance must be solvable");
        let (_, v, a) = phase.state_at(t);
        (v, a)
    }

    fn locate(&self, s: f64) -> usize {
        let mut i = 0;
        while i + 1 < self.breaks.len() && self.breaks[i + 1].s <= s {
            i += 1;
        }
        i.min(self.breaks.len().saturating_sub(2)).max(0)
    }
}

#[cfg(test)]
/// Duration of a velocity ramp of magnitude `dv >= 0` that starts and ends at
/// zero acceleration, under `a_max` / `j_max`. Triangular (jerk never reaches
/// `a_max`) below `a_max^2 / j_max`, trapezoidal above; `a/0` accel-limited when
/// jerk is infinite.
fn ramp_time(dv: f64, a_max: f64, j_max: f64) -> f64 {
    if dv <= EPS {
        return 0.0;
    }
    if !j_max.is_finite() {
        return dv / a_max;
    }
    if dv <= a_max * a_max / j_max {
        2.0 * (dv / j_max).sqrt()
    } else {
        a_max / j_max + dv / a_max
    }
}

#[cfg(test)]
/// Distance covered by a ramp from `v0` to `v1` (either direction). The speed
/// profile of a zero-accel-to-zero-accel ramp is point-symmetric about its
/// midpoint, so its mean speed is exactly `(v0 + v1) / 2`.
fn ramp_dist(v0: f64, v1: f64, a_max: f64, j_max: f64) -> f64 {
    0.5 * (v0 + v1) * ramp_time((v1 - v0).abs(), a_max, j_max)
}

#[cfg(test)]
/// Highest peak speed `<= v_max` whose ramp-up-from-`v0` plus ramp-down-to-`v1`
/// fits in `length`. Distance is monotone in the peak, so a bisection pins it;
/// if even `v_max` fits, the surplus becomes cruise (handled by the caller).
fn peak_velocity(v0: f64, v1: f64, length: f64, v_max: f64, a_max: f64, j_max: f64) -> f64 {
    let span = |vp: f64| ramp_dist(v0, vp, a_max, j_max) + ramp_dist(vp, v1, a_max, j_max);
    if span(v_max) <= length {
        return v_max;
    }
    let mut lo = v0.max(v1);
    let mut hi = v_max;
    for _ in 0..80 {
        let mid = 0.5 * (lo + hi);
        if span(mid) <= length {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    0.5 * (lo + hi)
}

#[cfg(test)]
struct Builder {
    breaks: Vec<Break>,
    s: f64,
    v: f64,
    a: f64,
}

#[cfg(test)]
impl Builder {
    fn phase(&mut self, j: f64, dt: f64) {
        if dt <= 0.0 {
            return;
        }
        self.breaks.push(Break {
            s: self.s,
            v: self.v,
            a: self.a,
            j,
            dt,
        });
        self.s += self.v * dt + 0.5 * self.a * dt * dt + j * dt * dt * dt / 6.0;
        self.v += self.a * dt + 0.5 * j * dt * dt;
        self.a += j * dt;
    }

    /// Append a zero-accel-to-zero-accel velocity ramp from the current speed to
    /// `v_to`: jerk on, optional accel hold, jerk off (or a single accel hold for
    /// infinite jerk, where the acceleration steps).
    fn ramp(&mut self, v_to: f64, a_max: f64, j_max: f64) {
        let dv = v_to - self.v;
        if dv.abs() <= EPS {
            return;
        }
        let dir = dv.signum();
        let adv = dv.abs();
        if !j_max.is_finite() {
            self.a = dir * a_max;
            self.phase(0.0, adv / a_max);
            self.a = 0.0;
            return;
        }
        if adv <= a_max * a_max / j_max {
            let t_j = (adv / j_max).sqrt();
            self.phase(dir * j_max, t_j);
            self.phase(-dir * j_max, t_j);
        } else {
            let t_j = a_max / j_max;
            let t_h = adv / a_max - a_max / j_max;
            self.phase(dir * j_max, t_j);
            self.phase(0.0, t_h);
            self.phase(-dir * j_max, t_j);
        }
        self.a = 0.0;
    }
}

/// One constant-jerk phase of a straight-line motion, rebased to its move's local
/// time and arc-length. On `[t0, t0 + dt]` the arc-length advanced from the move
/// start is exactly `s0 + v0*tau + a0*tau^2/2 + j*tau^3/6` (`tau = t - t0`), so a
/// straight move lowers to one exact cubic per phase — no fitting, no overshoot.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StraightPhase {
    pub t0: f64,
    pub dt: f64,
    pub s0: f64,
    pub v0: f64,
    pub a0: f64,
    pub j: f64,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PhaseSolveError {
    OutsidePhase,
    NonFinite,
    NonMonotone,
    DidNotConverge,
}

impl StraightPhase {
    pub fn end_time(&self) -> f64 {
        self.t0 + self.dt
    }

    pub fn end_distance(&self) -> f64 {
        let tau = self.dt;
        self.s0 + self.v0 * tau + 0.5 * self.a0 * tau * tau + self.j * tau * tau * tau / 6.0
    }

    pub fn state_at(&self, t: f64) -> (f64, f64, f64) {
        let tau = (t - self.t0).clamp(0.0, self.dt);
        let s =
            self.s0 + self.v0 * tau + 0.5 * self.a0 * tau * tau + self.j * tau * tau * tau / 6.0;
        let v = self.v0 + self.a0 * tau + 0.5 * self.j * tau * tau;
        let a = self.a0 + self.j * tau;
        (s, v, a)
    }

    pub fn time_at_distance(&self, distance: f64) -> Result<f64, PhaseSolveError> {
        let end_time = self.end_time();
        let end_distance = self.end_distance();
        if ![
            self.t0,
            self.dt,
            self.s0,
            self.v0,
            self.a0,
            self.j,
            distance,
            end_time,
            end_distance,
        ]
        .into_iter()
        .all(f64::is_finite)
        {
            return Err(PhaseSolveError::NonFinite);
        }
        if self.dt <= 0.0 || end_distance <= self.s0 {
            return Err(PhaseSolveError::NonMonotone);
        }

        let end_velocity = self.v0 + self.a0 * self.dt + 0.5 * self.j * self.dt * self.dt;
        let mut minimum_velocity = self.v0.min(end_velocity);
        if self.j != 0.0 {
            let turning_time = -self.a0 / self.j;
            if turning_time > 0.0 && turning_time < self.dt {
                minimum_velocity = minimum_velocity.min(
                    self.v0 + self.a0 * turning_time + 0.5 * self.j * turning_time * turning_time,
                );
            }
        }
        if minimum_velocity < 0.0 {
            return Err(PhaseSolveError::NonMonotone);
        }
        if distance < self.s0 || distance > end_distance {
            return Err(PhaseSolveError::OutsidePhase);
        }
        if distance == self.s0 {
            return Ok(self.t0);
        }
        if distance == end_distance {
            return Ok(end_time);
        }

        let phase_distance = self.v0 * self.dt
            + 0.5 * self.a0 * self.dt * self.dt
            + self.j * self.dt * self.dt * self.dt / 6.0;
        let target_distance = distance - self.s0;
        let position =
            |tau: f64| self.v0 * tau + 0.5 * self.a0 * tau * tau + self.j * tau * tau * tau / 6.0;
        let velocity = |tau: f64| self.v0 + self.a0 * tau + 0.5 * self.j * tau * tau;
        let tolerance = PHASE_SOLVE_EPS * (1.0 + target_distance.abs());
        let mut lo = 0.0;
        let mut hi = self.dt;
        let mut tau = self.dt * target_distance / phase_distance;

        for _ in 0..64 {
            let residual = position(tau) - target_distance;
            if !residual.is_finite() {
                return Err(PhaseSolveError::NonFinite);
            }
            if residual.abs() <= tolerance {
                return Ok(self.t0 + tau);
            }
            if residual < 0.0 {
                lo = tau;
            } else {
                hi = tau;
            }
            let slope = velocity(tau);
            let newton = tau - residual / slope;
            tau = if slope > 0.0 && newton.is_finite() && newton > lo && newton < hi {
                newton
            } else {
                0.5 * (lo + hi)
            };
        }

        Err(PhaseSolveError::DidNotConverge)
    }
}

#[cfg(test)]
/// Plan a triple-limited profile from `v0` to `v1` across `length` under a flat
/// ceiling `v_max`. Entry and exit sit at zero acceleration (run anchors).
pub(super) fn plan(v0: f64, v1: f64, length: f64, v_max: f64, a_max: f64, j_max: f64) -> Profile {
    let v_peak = peak_velocity(v0, v1, length, v_max, a_max, j_max);
    let mut b = Builder {
        breaks: Vec::new(),
        s: 0.0,
        v: v0,
        a: 0.0,
    };
    b.ramp(v_peak, a_max, j_max);
    let cruise = length - b.s - ramp_dist(v_peak, v1, a_max, j_max);
    if cruise > EPS && v_peak > EPS {
        b.phase(0.0, cruise / v_peak);
    }
    b.ramp(v1, a_max, j_max);
    b.breaks.push(Break {
        s: b.s,
        v: v1,
        a: 0.0,
        j: 0.0,
        dt: 0.0,
    });
    Profile {
        breaks: b.breaks,
        length: b.s,
    }
}

#[cfg(test)]
mod tests;
