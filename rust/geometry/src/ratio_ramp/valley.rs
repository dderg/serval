//! Where a ratio-ramped advance releases its lead and rebuilds it: the
//! advance valley, laid by the follower projection and bounded by the
//! turning guard from the same rule.

use super::{ADVANCE_VALLEY_SPEED_MM_S, RATIO_RAMP_MAX_TIME_S};

/// The ramp time over which a turnaround or crawl valley releases and
/// rebuilds the advance's lead, the longest, whatever the step there; and
/// the furthest a rest valley reaches past its spill.
pub const VALLEY_RAMP_TIME_S: f64 = RATIO_RAMP_MAX_TIME_S;

/// The time a turnaround's rate takes to build the speed the leaders cross
/// its seam with: the shaped leaders' velocities there are zero to their
/// fit's rounding, far under the rate times this.
pub const TURNAROUND_SPEED_TIME_S: f64 = 1e-6;

/// How far leaders crossing a point at path speed `speed` and path
/// acceleration `rate` reach from it in half of `ramp_time`.
#[must_use]
pub fn leader_reach(speed: f64, rate: f64, ramp_time: f64) -> f64 {
    (speed + 0.5 * rate * ramp_time) * 0.5 * ramp_time
}

/// How far each side of a turnaround or crawl valley reaches from it.
#[must_use]
pub fn valley_reach(speed: f64, rate: f64) -> f64 {
    leader_reach(speed, rate, VALLEY_RAMP_TIME_S)
}

/// The widest a crawl valley's side is where the leaders' path
/// acceleration at their slowest point is at most `rate`: a crawl is laid
/// under [`ADVANCE_VALLEY_SPEED_MM_S`].
#[must_use]
pub fn widest_crawl_reach(rate: f64) -> f64 {
    valley_reach(ADVANCE_VALLEY_SPEED_MM_S, rate)
}

/// The leaders turn around where they cross at a speed their rate builds
/// within [`TURNAROUND_SPEED_TIME_S`], the rounding of an exact zero, while
/// that rate itself is no rounding.
#[must_use]
pub fn turns_around(speed: f64, rate: f64) -> bool {
    rate > 0.0 && speed <= rate * TURNAROUND_SPEED_TIME_S
}

/// The floor a valley at a crawl of `speed` and `rate` keeps flat (see
/// [`AdvanceValley`]): `v²/a`, about where the acceleration takes over from
/// the crawl, and none where the leaders turn around or rest.
#[must_use]
pub fn crawl_floor(speed: f64, rate: f64) -> f64 {
    if turns_around(speed, rate) || speed == 0.0 {
        0.0
    } else {
        speed * speed / rate
    }
}

/// A stretch of path where a ratio-ramped advance reads the laid ratio
/// weighted down to zero: `W = 0` on `[start, end]`, where the leaders'
/// path speed is zero or a crawl, and `W = ψ(x) = 1 − (1 − x)³` within
/// `before`/`after` of it, meeting 1 flat to second order. `x` is the
/// distance `d` out over that width, both flattened over the valley's
/// `floor` `f` as `√(d² + f²) − f`: with no floor, where the leaders rest
/// or turn around, the weight rises off zero with a slope their zero speed
/// makes harmless; across a crawl at speed `v` and acceleration `a`, the
/// floor is `v²/a` ([`crawl_floor`]), about where the acceleration takes
/// over from the crawl, and the weight leaves zero flat, since `W'·ṡ²` of
/// opposite signs either side would step the advanced velocity. The laid
/// ratio, and so the material, is untouched; only the lead the advance
/// builds is released over the path before and rebuilt after.
///
/// Rising from a rest the leaders leave at acceleration `a`, the lead
/// `k·r·W·ṡ` catches up with the advance's `k·r·ṡ` at a rate of at most
/// `1.653·k·r·a` (the peak of `d/du (u·ψ(u²))`): the advance's natural
/// rate, overshot by what catching up from zero takes, and over a shaped
/// start whose acceleration builds from zero, much less. A crawl's floor
/// keeps that, since the leaders cover it within about `v/a`.
///
/// - A turnaround: the leaders' path speed passes through zero at a seam
///   while its rate does not ([`turns_around`]), so the rate steps from
///   `−a` to `+a`, and an advance reading `r·ṡ` would step the follower's
///   velocity by `2·k·r·a` whatever the ratio's own shape. The valley is
///   that seam, each side reaching as far as the leaders turn around in
///   half the ramp time ([`valley_reach`]).
/// - A crawl: the leaders' slowest point, under
///   [`ADVANCE_VALLEY_SPEED_MM_S`], at a seam or inside a leg. The path
///   speed's second derivative carries the tangent's turning there,
///   `a⊥²/v`, which an advance would turn into follower acceleration
///   however the planner bounds the path's own. Only where that leaves the
///   follower's limit: releasing the lead asks for some too.
/// - A rest: an extrude-only move or a dwell, where the shaped leaders come
///   to rest between the kernel spill of the move before and of the move
///   after, and the fit's back-and-forth near the rest point has its own
///   turnarounds and short idle spans an advance would turn into steps. The
///   valley is the stretch where the leaders rest, each side spanning the
///   spill, and past the raw seam only where the lead or a ratio step near
///   the rest needs it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AdvanceValley {
    pub start: f64,
    pub end: f64,
    pub before: f64,
    pub after: f64,
    pub floor: f64,
}

impl AdvanceValley {
    #[must_use]
    pub fn reach_start(&self) -> f64 {
        self.start - self.before
    }

    #[must_use]
    pub fn reach_end(&self) -> f64 {
        self.end + self.after
    }

    /// Whether the valley holds the weight flat at zero at `s`: on
    /// `[start, end]`, and within its floor `f` either side, over which the
    /// weight leaves zero flat (`ψ ≈ 3d²/(2f·w)` for `d ≪ f`); where it has
    /// no floor, to `rounding`.
    #[must_use]
    pub fn holds_zero_at(&self, s: f64, rounding: f64) -> bool {
        let flat = self.floor.max(rounding);
        s >= self.start - flat && s <= self.end + flat
    }

    /// `(W, W', W'')` at `s`, the derivatives along the path.
    #[must_use]
    pub fn weight(&self, s: f64) -> (f64, f64, f64) {
        let (distance, width, outward) = if s < self.start {
            (self.start - s, self.before, -1.0)
        } else if s > self.end {
            (s - self.end, self.after, 1.0)
        } else {
            return (0.0, 0.0, 0.0);
        };
        if distance >= width {
            return (1.0, 0.0, 0.0);
        }
        let floor = self.floor.min(width);
        let spread = libm::hypot(distance, floor);
        let slope = distance / spread;
        let bend = floor / spread * (floor / spread) / spread;
        let span = flattened(width, floor);
        let inside = 1.0 - flattened(distance, floor) / span;
        (
            1.0 - inside * inside * inside,
            outward * 3.0 * inside * inside * slope / span,
            -6.0 * inside * slope * slope / (span * span) + 3.0 * inside * inside * bend / span,
        )
    }
}

/// `d²/(√(d² + f²) + f) = √(d² + f²) − f`, zero at zero distance.
fn flattened(distance: f64, floor: f64) -> f64 {
    if distance == 0.0 {
        return 0.0;
    }
    distance * distance / (libm::hypot(distance, floor) + floor)
}

/// Bounds on one side of a valley's weight where the distance `d` from its
/// zero stretch, the side's width `w` and the valley's floor `f` are each
/// known within a range. With the share of the side still ahead
/// `ι = 1 − (√(d² + f̃²) − f̃)/w̃`, `w̃ = √(w² + f̃²) − f̃`, `f̃ = min(f, w)`,
/// the slope `σ = d/√(d² + f̃²)` and the bend `β = f̃²/(d² + f̃²)^{3/2}`
/// ([`AdvanceValley::weight`]):
/// `W = 1 − ι³`, `|dW/dd| = 3ι²σ/w̃`, `d²W/dd² = (3ι²/w̃)·β − 6ισ²/w̃²`.
///
/// Each factor is monotone in each of `d`, `w`, `f` over the side: `w̃`
/// rises with `w` and falls with `f`; `ι` falls with `d`, rises with `w`
/// and with `f`; `σ` rises with `d` and falls with `f`; `β` falls with `d`
/// and peaks over `f` at `f = √2·d`. So every factor's range is read at
/// the corners of the ranges, and the products of the non-negative factors
/// bound the weight and its derivatives wherever in the ranges the valley
/// lies. Past the side, `W = 1` and its derivatives vanish, which the
/// bounds include where the distance may reach the width.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SideWeight {
    pub weight: (f64, f64),
    /// `|dW/dd|`.
    pub slope: (f64, f64),
    /// `6ισ²/w̃²`, the part of `−d²W/dd²` the weight's shape carries.
    pub fold: (f64, f64),
    /// `3ι²/w̃`, which the floor's bend `β` scales into `d²W/dd²`.
    pub bend_scale: (f64, f64),
    /// `β`; unbounded where the distance and the floor may both vanish.
    pub bend: (f64, f64),
}

impl SideWeight {
    /// Beyond the side's reach: `W = 1`, flat.
    pub const OUTSIDE: Self = Self {
        weight: (1.0, 1.0),
        slope: (0.0, 0.0),
        fold: (0.0, 0.0),
        bend_scale: (0.0, 0.0),
        bend: (0.0, 0.0),
    };

    /// The side's weight over `distance`, `width` and `floor`, each a
    /// `(least, largest)` range.
    #[must_use]
    pub fn within(distance: (f64, f64), width: (f64, f64), floor: (f64, f64)) -> Self {
        let ((d0, d1), (w0, w1), (f0, f1)) = (distance, width, floor);
        assert!(
            0.0 <= d0 && d0 <= d1 && 0.0 < w0 && w0 <= w1 && 0.0 <= f0 && f0 <= f1,
            "a valley side's ranges are out of order: distance {distance:?}, width {width:?}, \
             floor {floor:?}"
        );
        if d0 >= w1 {
            return Self::OUTSIDE;
        }
        let (fe0, fe1) = (f0.min(w0), f1.min(w1));
        let inside = |d: f64, w: f64, f: f64| 1.0 - flattened(d, f) / flattened(w, f);
        let inside = (
            inside(d1, w0, fe0).clamp(0.0, 1.0),
            inside(d0, w1, fe1).clamp(0.0, 1.0),
        );
        let span = (flattened(w0, fe1), flattened(w1, fe0));
        let slope_at = |d: f64, f: f64| if f == 0.0 { 1.0 } else { d / libm::hypot(d, f) };
        let slope = (
            if d0 == 0.0 { 0.0 } else { slope_at(d0, fe1) },
            slope_at(d1, fe0),
        );
        let bend_at = |d: f64, f: f64| {
            if f == 0.0 {
                0.0
            } else {
                let spread = libm::hypot(d, f);
                f / spread * (f / spread) / spread
            }
        };
        let steepest_floor = (std::f64::consts::SQRT_2 * d0).clamp(fe0, fe1);
        let bend = (
            bend_at(d1, fe0).min(bend_at(d1, fe1)),
            if d0 == 0.0 && fe0 == 0.0 && fe1 > 0.0 {
                f64::INFINITY
            } else {
                bend_at(d0, steepest_floor)
            },
        );
        let (i0, i1) = inside;
        Self {
            weight: (1.0 - i1 * i1 * i1, 1.0 - i0 * i0 * i0),
            slope: (
                3.0 * i0 * i0 * slope.0 / span.1,
                3.0 * i1 * i1 * slope.1 / span.0,
            ),
            fold: (
                6.0 * i0 * slope.0 * slope.0 / (span.1 * span.1),
                6.0 * i1 * slope.1 * slope.1 / (span.0 * span.0),
            ),
            bend_scale: (3.0 * i0 * i0 / span.1, 3.0 * i1 * i1 / span.0),
            bend,
        }
    }
}

#[cfg(test)]
mod tests;
