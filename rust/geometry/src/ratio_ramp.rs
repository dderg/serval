//! How long a follower's ratio ramp spreads an extrusion-ratio step.
//!
//! The projection smooths the extrusion ratio along the path with the
//! kernel `K(x) = ¾(1 − x²)` of half-width `h = v·τ/2` at leader speed `v`.
//! `∫₋₁ˣ K = 3u² − 2u³` with `u = (x + 1)/2`, so a ratio step `Δr` reaches
//! the follower as the smoothstep `r̃ = r_b + Δr·S(t/τ)` over about `τ` in
//! time, `S′ ≤ 3/2`, `|S″| ≤ 6`. An advance of slope `k` reads the flow
//! `f = r̃·v` as `f + k·ḟ`; with the leaders at speed `v` and acceleration
//! `a` across the step, what the ramp adds to the follower is
//!
//! - velocity: `Δr·((v + k·a)·S + k·v·S′/τ)`, at most
//!   `Δr·(v + k|a|)·m(p)` with `p = k·v/((v + k|a|)·τ)` and
//!   `m(p) = max_u (S + 6p·u(1 − u))`, reached at
//!   `u* = ((1 − 2p) + √(1 + 4p²))/2`: an overshoot of `m(p) − 1` times the
//!   step over the advanced flow after it;
//! - acceleration: `Δr·((v + 2k·a)·S′/τ + k·v·S″/τ²)`, at most
//!   `Δr·(v + 2k|a|)/τ·g(p)` with `p = k·v/((v + 2k|a|)·τ)` and
//!   `g(p) = max_u |S′ + p·S″|`: `6p` for `p ≥ ½` (the ends of the ramp,
//!   `|S″| = 6`), `3/2 + 6p²` below (inside it, at `u = ½ − p`).
//!
//! The rest of the advanced flow, `r̃·a` and `k·r̃·ȧ`, is the path's own
//! demand, which the ramp's adds to: the larger ratio about the step times
//! `|a| + k·|ȧ|`. At a steady speed the bounds are exact:
//! `τ ≥ √(6kΔ/A)` wherever `τ ≤ 2k`, with `Δ = |Δr|·v`, and `τ ≥ 3Δ/(2A)`
//! without an advance. The ramp time is the shortest in
//! [`RATIO_RAMP_MIN_TIME_S`, `RATIO_RAMP_MAX_TIME_S`] whose addition fits
//! within what the path's own demand leaves of the follower's limits, and
//! never within less than [`RAMP_HEADROOM_FLOOR`] of them: the ramp's
//! addition vanishes with its step, so a small step takes a short ramp even
//! where the path alone runs at the limits. Holding the path within the
//! limits is the planner's: where even the longest ramp does not fit, or a
//! rest of the leaders leaves the ramp less path than it takes
//! ([`rest_room_reach_mm`]), it lowers the leader speed at the step
//! ([`RatioRampRule::step_speed_cap`]).

/// The shortest ratio ramp.
pub const RATIO_RAMP_MIN_TIME_S: f64 = 0.005;
/// The longest ratio ramp: past it the planner slows the step instead of
/// smearing the flow over more path.
pub const RATIO_RAMP_MAX_TIME_S: f64 = 0.03;
/// The shaped path speed under which a ratio-ramped advance releases its
/// lead around the leaders' slowest point instead of reading the path
/// speed through it. Near a reversal the path speed's second derivative
/// carries the tangent's turning `a⊥²/v`, which grows without bound as the
/// speed minimum `v` falls, whatever the planner bounds of the path's own
/// acceleration and jerk; under this speed the lead the advance holds,
/// `k·r·v`, is too small to matter to the flow.
pub const ADVANCE_VALLEY_SPEED_MM_S: f64 = 2.0;
/// The steepest a ratio ramp's half-width climbs per unit of path away
/// from a node of its width field, a rest of the leaders among them: the
/// window `d` along the path from a rest is at most `ENVELOPE_SLOPE·d`
/// wide.
pub const RATIO_RAMP_ENVELOPE_SLOPE: f64 = 0.25;
/// The share of the path between a ratio step and a rest of the leaders
/// that the step's ramp takes on that side. A window `u` past the step,
/// under the rest's cone `c·(d − u)`, reaches back to the step only while
/// `u ≤ c·(d − u)`: out to `c/(1 + c)` of the way, where a window of that
/// half-width laid freely would end too.
const REST_ROOM_RAMP_SHARE: f64 = RATIO_RAMP_ENVELOPE_SLOPE / (1.0 + RATIO_RAMP_ENVELOPE_SLOPE);

const BISECTIONS: usize = 64;
/// The share of each follower limit a ratio ramp may always add to the
/// path's own demand, however near the limit that already runs. With no
/// floor the headroom closes as the path's demand reaches the limit, and
/// the ramp time leaps from the shortest to the longest for any step there,
/// however small.
const RAMP_HEADROOM_FLOOR: f64 = 0.25;

/// How far along the path a rest of the leaders narrows the ramp of a
/// ratio step they cross at up to `speed`: past it, the step's share of
/// the room holds the longest ramp.
#[must_use]
pub fn rest_room_reach_mm(speed: f64) -> f64 {
    0.5 * speed * RATIO_RAMP_MAX_TIME_S / REST_ROOM_RAMP_SHARE
}

/// The longest ramp a ratio step crossed at `speed`, `room_mm` along the
/// path from a rest of the leaders, gets: the half of it on the rest's side
/// laid over the step's share of the room.
fn rest_room_ramp_time(speed: f64, room_mm: f64) -> f64 {
    if speed > 0.0 {
        2.0 * REST_ROOM_RAMP_SHARE * room_mm / speed
    } else {
        f64::INFINITY
    }
}

/// The fastest leaders crossing a ratio step at `speed` under an
/// acceleration of up to `accel` move within half the longest ramp of it,
/// the window the projection reads a step's crossing over.
fn crossing_speed(speed: f64, accel: f64) -> f64 {
    speed + 0.5 * accel * RATIO_RAMP_MAX_TIME_S
}

/// What a follower's ratio ramp is held to: the advance's slope, the
/// follower motor's velocity and acceleration limits, and the jerk gain of
/// the kernel smoothing the advanced follower, infinite without one.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RatioRampRule {
    pub advance_slope_s: f64,
    pub max_velocity_mm_s: f64,
    pub max_accel_mm_s2: f64,
    pub kernel_jerk_per_accel_hz: f64,
}

/// A ratio step as the leaders cross it: the step, the larger ratio on
/// either side, and the magnitudes of the leaders' speed, acceleration and
/// jerk there.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RatioStep {
    pub ratio_step: f64,
    pub peak_ratio: f64,
    pub speed_mm_s: f64,
    pub accel_mm_s2: f64,
    pub jerk_mm_s3: f64,
}

/// `max_u (S(u) + 6p·u(1 − u))` with `S(u) = 3u² − 2u³`.
fn velocity_overshoot(p: f64) -> f64 {
    let u = 0.5 * ((1.0 - 2.0 * p) + (1.0 + 4.0 * p * p).sqrt());
    u * u * (3.0 - 2.0 * u) + 6.0 * p * u * (1.0 - u)
}

/// `max_u |S′(u) + p·S″(u)|`.
fn acceleration_gain(p: f64) -> f64 {
    if p >= 0.5 { 6.0 * p } else { 1.5 + 6.0 * p * p }
}

/// `k·v/(w·τ)`, zero where the leaders neither move nor accelerate.
fn ramp_ratio(k: f64, speed: f64, weight: f64, tau: f64) -> f64 {
    if weight > 0.0 {
        k * speed / (weight * tau)
    } else {
        0.0
    }
}

impl RatioRampRule {
    /// The follower's peak velocity and acceleration over a ramp of `tau`
    /// seconds across `step`: the leaders' own advanced flow plus what the
    /// ramp adds to it.
    #[must_use]
    pub fn demand(&self, tau: f64, step: &RatioStep) -> (f64, f64) {
        let (own_velocity, own_accel) = self.own_demand(step);
        let (ramp_velocity, ramp_accel) = self.ramp_demand(tau, step);
        (own_velocity + ramp_velocity, own_accel + ramp_accel)
    }

    /// The advanced flow at the larger ratio about `step`, ramp or not.
    fn own_demand(&self, step: &RatioStep) -> (f64, f64) {
        let k = self.advance_slope_s;
        let (v, a, j) = (
            step.speed_mm_s,
            step.accel_mm_s2.abs(),
            step.jerk_mm_s3.abs(),
        );
        (step.peak_ratio * (v + k * a), step.peak_ratio * (a + k * j))
    }

    /// What a ramp of `tau` seconds adds to the follower's peak velocity
    /// and acceleration across `step`: proportional to the step, and
    /// shrinking as the ramp lengthens.
    fn ramp_demand(&self, tau: f64, step: &RatioStep) -> (f64, f64) {
        let k = self.advance_slope_s;
        let (v, a) = (step.speed_mm_s, step.accel_mm_s2.abs());
        let advanced = v + k * a;
        let rate = v + 2.0 * k * a;
        (
            step.ratio_step
                * (velocity_overshoot(ramp_ratio(k, v, advanced, tau)) - 1.0)
                * advanced,
            step.ratio_step * rate / tau * acceleration_gain(ramp_ratio(k, v, rate, tau)),
        )
    }

    /// How much a ramp across `step` may add to the follower's velocity and
    /// acceleration: what the leaders' own flow leaves of each limit, and
    /// never under [`RAMP_HEADROOM_FLOOR`] of it.
    fn ramp_headroom(&self, step: &RatioStep) -> (f64, f64) {
        let (own_velocity, own_accel) = self.own_demand(step);
        let headroom = |limit: f64, own: f64| (limit - own).max(RAMP_HEADROOM_FLOOR * limit);
        (
            headroom(self.max_velocity_mm_s, own_velocity),
            headroom(self.max_accel_mm_s2, own_accel),
        )
    }

    /// The ramp time for `step`: the shortest whose demand fits within the
    /// ramp's headroom ([`Self::ramp_headroom`]), or the longest ramp where
    /// none does. Its demand is continuous in the ramp time and every input
    /// and falls as the ramp lengthens, and the headroom is continuous and
    /// positive, so the ramp time is continuous in its inputs and the
    /// shortest as the step vanishes.
    #[must_use]
    pub fn ramp_time(&self, step: &RatioStep) -> f64 {
        assert!(
            [
                step.ratio_step,
                step.peak_ratio,
                step.speed_mm_s,
                step.accel_mm_s2
            ]
            .iter()
            .all(|value| value.is_finite())
                && step.ratio_step >= 0.0
                && step.speed_mm_s >= 0.0,
            "a ratio ramp is asked for an ill-formed step {step:?}"
        );
        let (velocity_headroom, accel_headroom) = self.ramp_headroom(step);
        let fits = |tau: f64| {
            let (velocity, accel) = self.ramp_demand(tau, step);
            velocity <= velocity_headroom && accel <= accel_headroom
        };
        if fits(RATIO_RAMP_MIN_TIME_S) {
            return RATIO_RAMP_MIN_TIME_S;
        }
        if !fits(RATIO_RAMP_MAX_TIME_S) {
            return RATIO_RAMP_MAX_TIME_S;
        }
        let (mut short, mut long) = (RATIO_RAMP_MIN_TIME_S, RATIO_RAMP_MAX_TIME_S);
        for _ in 0..BISECTIONS {
            let mid = 0.5 * (short + long);
            if fits(mid) {
                long = mid;
            } else {
                short = mid;
            }
        }
        long
    }

    /// The largest leader speed at which a ratio step of `ratio_step`
    /// whose larger side extrudes `peak_ratio` per path millimetre, crossed
    /// under an acceleration of up to `accel` and a jerk of up to `jerk`
    /// `rest_room_mm` along the path from the leaders' nearest rest, fits
    /// the limits within the longest ramp the room leaves it; zero where
    /// even a crossing at rest does not. The leaders' own flow is read at
    /// the step and the ramp's addition where the projection derives it: at
    /// the fastest the leaders move within half the longest ramp of the
    /// step, up to `accel` times that faster than at it and no faster than
    /// `fastest_mm_s`, the fastest they move anywhere near it.
    #[must_use]
    pub fn step_speed_cap(
        &self,
        ratio_step: f64,
        peak_ratio: f64,
        accel: f64,
        jerk: f64,
        rest_room_mm: f64,
        fastest_mm_s: f64,
    ) -> f64 {
        assert!(
            rest_room_mm >= 0.0,
            "a ratio step lies {rest_room_mm} mm from the leaders' nearest rest"
        );
        assert!(
            fastest_mm_s >= 0.0,
            "the leaders move at up to {fastest_mm_s} mm/s near a ratio step"
        );
        if ratio_step <= 0.0 {
            return f64::INFINITY;
        }
        let step_at = |speed_mm_s: f64| RatioStep {
            ratio_step,
            peak_ratio,
            speed_mm_s,
            accel_mm_s2: accel,
            jerk_mm_s3: jerk,
        };
        let fits_at = |speed_mm_s: f64| {
            let crossing = crossing_speed(speed_mm_s, accel).min(speed_mm_s.max(fastest_mm_s));
            let tau = RATIO_RAMP_MAX_TIME_S.min(rest_room_ramp_time(crossing, rest_room_mm));
            let (own_velocity, own_accel) = self.own_demand(&step_at(speed_mm_s));
            let (ramp_velocity, ramp_accel) = self.ramp_demand(tau, &step_at(crossing));
            own_velocity + ramp_velocity <= self.max_velocity_mm_s
                && own_accel + ramp_accel <= self.max_accel_mm_s2
        };
        if !fits_at(0.0) {
            return 0.0;
        }
        let (velocity, accel_demand) = self.demand(
            RATIO_RAMP_MAX_TIME_S,
            &RatioStep {
                ratio_step,
                peak_ratio,
                speed_mm_s: 1.0,
                accel_mm_s2: 0.0,
                jerk_mm_s3: 0.0,
            },
        );
        let (mut slow, mut fast) = (
            0.0,
            (self.max_velocity_mm_s / velocity).min(self.max_accel_mm_s2 / accel_demand),
        );
        if fits_at(fast) {
            return fast;
        }
        for _ in 0..BISECTIONS {
            let mid = 0.5 * (slow + fast);
            if fits_at(mid) {
                slow = mid;
            } else {
                fast = mid;
            }
        }
        slow
    }
}

#[cfg(test)]
mod tests;
