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

const EPS: f64 = 1e-12;

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

pub(super) struct Profile {
    breaks: Vec<Break>,
    length: f64,
}

impl Profile {
    /// `(v, a)` at arc-length `s`, clamped to the profile's span. The terminal
    /// breakpoint carries the exact exit state `(v1, 0)`, returned directly at the
    /// end so a sub-ulp `solve_tau` shortfall can't leave a residual acceleration
    /// (and, under infinite jerk, so the stop is the exit anchor, not the last
    /// phase's `-a_max` step).
    pub(super) fn at(&self, s: f64) -> (f64, f64) {
        let s = s.clamp(0.0, self.length);
        let last = &self.breaks[self.breaks.len() - 1];
        if s >= last.s - EPS {
            return (last.v, last.a);
        }
        let b = self.breaks[self.locate(s)];
        let tau = b.solve_tau(s - b.s);
        (b.v + b.a * tau + 0.5 * b.j * tau * tau, b.a + b.j * tau)
    }

    fn locate(&self, s: f64) -> usize {
        let mut i = 0;
        while i + 1 < self.breaks.len() && self.breaks[i + 1].s <= s {
            i += 1;
        }
        i.min(self.breaks.len().saturating_sub(2)).max(0)
    }
}

impl Break {
    /// Time `tau` into this phase at which it has advanced `ds` in arc-length,
    /// i.e. the root of `v*tau + a*tau^2/2 + j*tau^3/6 = ds` on `[0, dt]`. The
    /// integrand is non-decreasing (speed stays non-negative within a phase), so
    /// the root is unique and bracketed; bisection with a Newton step is robust.
    fn solve_tau(&self, ds: f64) -> f64 {
        if ds <= 0.0 || self.dt <= 0.0 {
            return 0.0;
        }
        let pos =
            |tau: f64| self.v * tau + 0.5 * self.a * tau * tau + self.j * tau * tau * tau / 6.0;
        let (mut lo, mut hi) = (0.0, self.dt);
        if pos(hi) <= ds {
            return hi;
        }
        for _ in 0..64 {
            let mid = 0.5 * (lo + hi);
            let f = pos(mid) - ds;
            if f.abs() <= EPS * (1.0 + ds) {
                return mid;
            }
            if f < 0.0 {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        0.5 * (lo + hi)
    }
}

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

/// Distance covered by a ramp from `v0` to `v1` (either direction). The speed
/// profile of a zero-accel-to-zero-accel ramp is point-symmetric about its
/// midpoint, so its mean speed is exactly `(v0 + v1) / 2`.
fn ramp_dist(v0: f64, v1: f64, a_max: f64, j_max: f64) -> f64 {
    0.5 * (v0 + v1) * ramp_time((v1 - v0).abs(), a_max, j_max)
}

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

struct Builder {
    breaks: Vec<Break>,
    s: f64,
    v: f64,
    a: f64,
}

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

impl Profile {
    /// Per-span closed-form phases, each rebased to span-local time and
    /// arc-length (`t0 = 0`, `s0 = 0` at the span start). `spans` are
    /// `(s_start, len)` in the profile's own arc-length — one per move of a
    /// straight run, so every move lowers from the run's single analytic profile
    /// and collinear seams stay C1-continuous by construction.
    pub(super) fn phases_for_spans(&self, spans: &[(f64, f64)]) -> Vec<Vec<StraightPhase>> {
        let mut timed: Vec<(f64, &Break)> = Vec::with_capacity(self.breaks.len());
        let mut t = 0.0;
        for b in &self.breaks {
            if b.dt <= 0.0 {
                continue;
            }
            timed.push((t, b));
            t += b.dt;
        }
        spans
            .iter()
            .map(|&(s0, len)| self.clip(s0, s0 + len, &timed))
            .collect()
    }

    fn phase_end_s(&self, i: usize, timed: &[(f64, &Break)]) -> f64 {
        if i + 1 < timed.len() {
            timed[i + 1].1.s
        } else {
            self.length
        }
    }

    fn time_at(&self, s: f64, timed: &[(f64, &Break)]) -> f64 {
        for (i, &(t0, b)) in timed.iter().enumerate() {
            if s <= self.phase_end_s(i, timed) + EPS {
                return t0 + b.solve_tau((s - b.s).max(0.0));
            }
        }
        timed.last().map_or(0.0, |&(t0, b)| t0 + b.dt)
    }

    fn clip(&self, s_lo: f64, s_hi: f64, timed: &[(f64, &Break)]) -> Vec<StraightPhase> {
        let t_base = self.time_at(s_lo, timed);
        let mut out = Vec::new();
        for (i, &(t0, b)) in timed.iter().enumerate() {
            let p_end = self.phase_end_s(i, timed);
            if p_end <= s_lo + EPS || b.s >= s_hi - EPS {
                continue;
            }
            // Use the phase's own exact `0` / `dt` at natural boundaries, so an
            // interior phase chains bit-exactly (the arc-length `s_hi - b.s` loses
            // precision against a large `b.s`); only solve for a genuine clip,
            // which happens once per phase at a move seam.
            let entry = if s_lo <= b.s + EPS {
                0.0
            } else {
                b.solve_tau(s_lo - b.s)
            };
            let exit = if s_hi >= p_end - EPS {
                b.dt
            } else {
                b.solve_tau(s_hi - b.s)
            };
            if exit <= entry + EPS {
                continue;
            }
            out.push(StraightPhase {
                t0: (t0 + entry) - t_base,
                dt: exit - entry,
                s0: b.s
                    + b.v * entry
                    + 0.5 * b.a * entry * entry
                    + b.j * entry * entry * entry / 6.0
                    - s_lo,
                v0: b.v + b.a * entry + 0.5 * b.j * entry * entry,
                a0: b.a + b.j * entry,
                j: b.j,
            });
        }
        out
    }
}

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
