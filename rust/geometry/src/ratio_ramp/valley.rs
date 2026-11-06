//! Where a ratio-ramped advance releases its lead and rebuilds it: the
//! advance valley, laid by the follower projection and bounded by the
//! turning guard from the same rule.

use super::{ADVANCE_VALLEY_SPEED_MM_S, RATIO_RAMP_MAX_TIME_S, RATIO_RAMP_MIN_TIME_S};

/// The ramp time over which a turnaround or crawl valley releases and
/// rebuilds the advance's lead, the longest, whatever the step there; and
/// the furthest a rest valley reaches past its spill.
pub const VALLEY_RAMP_TIME_S: f64 = RATIO_RAMP_MAX_TIME_S;

/// The time a turnaround's rate takes to build the speed the leaders cross
/// its seam with: the shaped leaders' velocities there are zero to their
/// fit's rounding, far under the rate times this.
pub const TURNAROUND_SPEED_TIME_S: f64 = 1e-6;

/// The longest a crawl valley's side may take the leaders before its
/// crawl: the share of the pump's lead (`MAX_LEAD_SECS`, 2 s, in
/// motion-core's pump loop) the turning guard and the follower projection
/// spend holding motion back for the valleys still to come within reach
/// of it. The guard leaves room for a side that wide; the projection keeps
/// that much path untaken behind its table's end.
pub const VALLEY_LEAD_BUDGET_S: f64 = 0.1;

/// The widest a side's floor is kept, as a share of the side: a crawl
/// floor `v²/a` past it would flatten the weight's start and steepen its
/// end, and leave whoever bounds the side without the crawl's
/// acceleration a floor as wide as the side to bound the rebuild over.
const CRAWL_FLOOR_SIDE_SHARE: f64 = 0.125;

/// The floor a side `width` wide keeps of `floor`.
#[must_use]
pub fn kept_floor(floor: f64, width: f64) -> f64 {
    floor.min(CRAWL_FLOOR_SIDE_SHARE * width)
}

/// How many geometric pieces [`SideWeight::within`] bounds a width range
/// over: each pairs a narrower span of widths with the floors and shares
/// ahead they allow.
const SIDE_WIDTH_PIECES: i32 = 8;

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

/// The share of a crawl's leaders' reach ([`valley_reach`]) a ratio step
/// at the crawl, or inside its valley's sides, ramps within either side of
/// it. The reach at a crawl of speed `v` is at least `v·τ/2`, `τ` the
/// [`VALLEY_RAMP_TIME_S`], so a window `w` of this share is at least
/// `v·τ_min/2`, the half-width the shortest ramp
/// ([`RATIO_RAMP_MIN_TIME_S`]) lays at that speed. Over it the step's flow
/// asks `¾·|Δ|·ṡ²/w` with `ṡ² ≤ v² + 2A·d` a distance `d` from the crawl:
/// at most the `1.5·|Δ|·v/τ_min` the shortest ramp asks at the crawl's
/// speed, plus `1.5·|Δ|·A·d/w`, which no share changes for a step within
/// the window of the crawl (`d ≤ 2w`). It is the narrowest share whose
/// flow is no steeper than a ramp the rule lays: any wider releases the
/// advance over more path.
const CRAWL_STEP_WINDOW_REACH_SHARE: f64 = RATIO_RAMP_MIN_TIME_S / VALLEY_RAMP_TIME_S;

/// The half-width a crawl valley caps a ratio step's ramp to, and holds
/// its weight at zero over on either side of the step, where the leaders'
/// reach at the crawl is `reach`: the step's flow `r̃·ṡ` alone reaches the
/// follower there, `r̃′·ṡ²` its acceleration, which needs no advance's
/// room, and the valley's sides start past it.
#[must_use]
pub fn crawl_step_window(reach: f64) -> f64 {
    CRAWL_STEP_WINDOW_REACH_SHARE * reach
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
/// distance `d` out over that width, both flattened over the side's floor
/// `f`, kept within [`CRAWL_FLOOR_SIDE_SHARE`] of the side, as
/// `√(d² + f²) − f`: with no floor, where the leaders rest or turn
/// around, the weight rises off zero with a slope their zero speed makes
/// harmless; across a crawl at speed `v` and acceleration `a`, the floor
/// is `v²/a` ([`crawl_floor`]), about where the acceleration takes over
/// from the crawl, and the weight leaves zero flat, since `W'·ṡ²` of
/// opposite signs either side would step the advanced velocity. Each side
/// keeps the floor of the crawl it leaves from, so a valley widened over a
/// later crawl rebuilds the lead from that crawl alone. The laid ratio,
/// and so the material, is untouched; only the lead the advance builds is
/// released over the path before and rebuilt after.
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
///   that seam, each side reaching at least as far as the leaders turn
///   around in half the ramp time ([`valley_reach`]), and as far as the
///   rebuild needs to fit the follower's limit ([`crawl_side_width`]).
/// - A crawl: the leaders' slowest point, under
///   [`ADVANCE_VALLEY_SPEED_MM_S`], at a seam or inside a leg. The path
///   speed's second derivative carries the tangent's turning there,
///   `a⊥²/v`, which an advance would turn into follower acceleration
///   however the planner bounds the path's own. Only where that leaves the
///   follower's limit: releasing the lead asks for some too. Its sides are
///   sized as a turnaround's.
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
    pub floor_before: f64,
    pub floor_after: f64,
    pub kind: ValleyKind,
}

/// Whether a valley releases the lead around a turnaround or crawl, or a
/// rest (see [`AdvanceValley`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ValleyKind {
    Crawl,
    Rest,
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
    /// `[start, end]`, and within each side's floor `f` as the side keeps it
    /// ([`kept_floor`]), over which the weight leaves zero flat
    /// (`ψ ≈ 3d²/(2f·w)` for `d ≪ f`); where a side has no floor, to
    /// `rounding`.
    #[must_use]
    pub fn holds_zero_at(&self, s: f64, rounding: f64) -> bool {
        s >= self.start - kept_floor(self.floor_before, self.before).max(rounding)
            && s <= self.end + kept_floor(self.floor_after, self.after).max(rounding)
    }

    /// `(W, W', W'')` at `s`, the derivatives along the path.
    #[must_use]
    pub fn weight(&self, s: f64) -> (f64, f64, f64) {
        let (distance, width, floor, outward) = if s < self.start {
            (self.start - s, self.before, self.floor_before, -1.0)
        } else if s > self.end {
            (s - self.end, self.after, self.floor_after, 1.0)
        } else {
            return (0.0, 0.0, 0.0);
        };
        let (weight, slope, bend) = side_weight(distance, width, floor);
        (weight, outward * slope, bend)
    }
}

/// `(W, dW/dd, d²W/dd²)` of a valley's side `width` wide whose floor is
/// `floor`, at the distance `d` out from its zero stretch
/// ([`AdvanceValley`]).
#[must_use]
pub fn side_weight(distance: f64, width: f64, floor: f64) -> (f64, f64, f64) {
    if distance <= 0.0 {
        return (0.0, 0.0, 0.0);
    }
    if distance >= width {
        return (1.0, 0.0, 0.0);
    }
    let floor = kept_floor(floor, width);
    let spread = libm::hypot(distance, floor);
    let slope = distance / spread;
    let bend = floor / spread * (floor / spread) / spread;
    let span = flattened(width, floor);
    let inside = 1.0 - flattened(distance, floor) / span;
    (
        1.0 - inside * inside * inside,
        3.0 * inside * inside * slope / span,
        -6.0 * inside * slope * slope / (span * span) + 3.0 * inside * inside * bend / span,
    )
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
/// `ι = 1 − (√(d² + f̃²) − f̃)/w̃`, `w̃ = √(w² + f̃²) − f̃`, `f̃` the floor kept
/// ([`kept_floor`]),
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
/// bounds include where the distance may reach the width. The largest
/// slope, fold and bend scale read `ι` and `w̃` of the same side: with the
/// flattened distance `x = w̃·(1 − ι)`, `ι²/w̃ = (w̃ − x)²/w̃³` peaks over
/// the width at `w̃ = 3x` and `ι/w̃² = (w̃ − x)/w̃³` at `w̃ = 1.5x`, so each is
/// read at the nearest distance and the width in range closest to its
/// peak, where the corners alone would pair the widest share ahead with
/// the narrowest side. Since `f̃ ≤ w/8`, `w̃ ≥ (√(65/64) − 1/8)·w` and no
/// narrower than the narrowest width flattened over the most floor it keeps.
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
    /// `(least, largest)` range: the union of the bounds over
    /// [`SIDE_WIDTH_PIECES`] geometric pieces of the width range, each of
    /// which pairs its widths with floors no wider than they are.
    #[must_use]
    pub fn within(distance: (f64, f64), width: (f64, f64), floor: (f64, f64)) -> Self {
        let (w0, w1) = width;
        assert!(
            0.0 < w0 && w0 <= w1,
            "a valley side's width range is out of order: {width:?}"
        );
        let ratio = libm::pow(w1 / w0, 1.0 / f64::from(SIDE_WIDTH_PIECES));
        let edge = |piece: i32| match piece {
            0 => w0,
            SIDE_WIDTH_PIECES => w1,
            _ => (w0 * libm::pow(ratio, f64::from(piece))).min(w1),
        };
        (0..SIDE_WIDTH_PIECES)
            .map(|piece| {
                let (from, to) = (edge(piece), edge(piece + 1));
                Self::within_widths(distance, (from.min(to), to), floor)
            })
            .reduce(Self::union)
            .expect("a width range has pieces")
    }

    fn union(self, other: Self) -> Self {
        let hull = |a: (f64, f64), b: (f64, f64)| (a.0.min(b.0), a.1.max(b.1));
        Self {
            weight: hull(self.weight, other.weight),
            slope: hull(self.slope, other.slope),
            fold: hull(self.fold, other.fold),
            bend_scale: hull(self.bend_scale, other.bend_scale),
            bend: hull(self.bend, other.bend),
        }
    }

    fn within_widths(distance: (f64, f64), width: (f64, f64), floor: (f64, f64)) -> Self {
        SideSpan::over(width, floor).weight_within(distance)
    }
}

/// A side's width and floor ranges with what [`SideWeight`]'s bounds read
/// of them alone: the floors kept, the widths flattened over them, and the
/// span the slope, fold and bend scale are read within.
#[derive(Debug, Clone, Copy, PartialEq)]
struct SideSpan {
    width: (f64, f64),
    kept: (f64, f64),
    flattened: (f64, f64),
    span: (f64, f64),
}

impl SideSpan {
    fn over(width: (f64, f64), floor: (f64, f64)) -> Self {
        let ((w0, w1), (f0, f1)) = (width, floor);
        assert!(
            0.0 < w0 && w0 <= w1 && 0.0 <= f0 && f0 <= f1,
            "a valley side's ranges are out of order: width {width:?}, floor {floor:?}"
        );
        let (fe0, fe1) = (kept_floor(f0, w0), kept_floor(f1, w1));
        Self {
            width,
            kept: (fe0, fe1),
            flattened: (flattened(w0, fe0), flattened(w1, fe1)),
            span: (flattened(w0, kept_floor(fe1, w0)), flattened(w1, fe0)),
        }
    }

    fn weight_within(&self, distance: (f64, f64)) -> SideWeight {
        let (d0, d1) = distance;
        assert!(
            0.0 <= d0 && d0 <= d1,
            "a valley side's distance range is out of order: {distance:?}"
        );
        if d0 >= self.width.1 {
            return SideWeight::OUTSIDE;
        }
        let (fe0, fe1) = self.kept;
        let span = self.span;
        let inside = (
            (1.0 - flattened(d1, fe0) / self.flattened.0).clamp(0.0, 1.0),
            (1.0 - flattened(d0, fe1) / self.flattened.1).clamp(0.0, 1.0),
        );
        let nearest = flattened(d0, fe1);
        let share_peak = |peak_at: f64, power: i32| {
            let span = (peak_at * nearest).clamp(span.0.max(nearest), span.1);
            if span <= nearest {
                0.0
            } else {
                libm::pow(span - nearest, f64::from(power)) / (span * span * span)
            }
        };
        let (folding, rising) = (share_peak(1.5, 1), share_peak(3.0, 2));
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
        SideWeight {
            weight: (1.0 - i1 * i1 * i1, 1.0 - i0 * i0 * i0),
            slope: (3.0 * i0 * i0 * slope.0 / span.1, 3.0 * rising * slope.1),
            fold: (
                6.0 * i0 * slope.0 * slope.0 / (span.1 * span.1),
                6.0 * folding * slope.1 * slope.1,
            ),
            bend_scale: (3.0 * i0 * i0 / span.1, 3.0 * rising),
            bend,
        }
    }
}

/// How many pieces of its width [`CrawlSide::rebuild`] bounds a side over.
const CRAWL_SIDE_DISTANCE_PIECES: u32 = 32;

/// How many geometric pieces [`CrawlSide::rebuild`] splits a floor range
/// into, each paired with the widths and distances on its own.
const CRAWL_SIDE_FLOOR_PIECES: i32 = 8;

/// The narrowest floor, as a share of the side, a range of floors is split
/// down to geometrically before its last piece runs to its least floor.
const CRAWL_SIDE_FLOOR_SHARE_MIN: f64 = 1.0 / 65_536.0;

/// The widths a crawl side is laid at past its leaders' reach: a geometric
/// grid of [`CRAWL_SIDE_GRID_STEPS_PER_DOUBLING`] steps per doubling from
/// this, so that whoever bounds the leaders looser picks from the same
/// widths and never a narrower one ([`crawl_side_width`]).
const CRAWL_SIDE_GRID_UNIT_MM: f64 = 1e-3;
const CRAWL_SIDE_GRID_STEPS_PER_DOUBLING: f64 = 8.0;

/// The widest a crawl side is laid: past it no room fits a rebuild.
pub const CRAWL_SIDE_WIDEST_MM: f64 = 1e3;

/// How the leaders cross one side of a crawl or turnaround valley, as the
/// advance rebuilding (or releasing) its lead over it reads them.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CrawlSide {
    /// The path speed at the crawl, `v`.
    pub speed: f64,
    /// The most path acceleration `|s̈|` across the side, `A`: at least the
    /// crawl's whole acceleration `a`, so a crawl floor `v²/a` is at least
    /// `v²/A`.
    pub accel: f64,
    /// The fastest the leaders cross the side.
    pub fastest: f64,
    /// The range the side's floor lies in: none, or a crawl's
    /// ([`crawl_floor`]) of at most `v` and at most `A`.
    pub floor: (f64, f64),
    /// The range of the path acceleration away from the crawl, `o`: `s̈`
    /// over the after side, `−s̈` over the before side, within `±A`, where
    /// the leaders lie further than [`Self::signed_from`] from the crawl.
    pub outward: (f64, f64),
    /// The distance from the crawl within which `o` is known only within
    /// `±A`.
    pub signed_from: f64,
}

impl CrawlSide {
    /// The leaders within `accel` either way across the side.
    #[must_use]
    pub fn either_way(speed: f64, accel: f64, fastest: f64, floor: (f64, f64)) -> Self {
        Self {
            speed,
            accel,
            fastest,
            floor,
            outward: (-accel, accel),
            signed_from: 0.0,
        }
    }

    /// The most `|W″·ṡ³ + 3W′·ṡ·s̈|` asks over a side `width` wide, which an
    /// advance of slope `k` at ratio `r` turns into `k·r` times as much
    /// follower acceleration. With `Ŵ` the weight over the distance `d`
    /// from the crawl, that is `Ŵ″·ṡ³ + 3Ŵ′·ṡ·o`, `Ŵ′ ≥ 0`. Leaving the
    /// crawl at `v`, `ṡ² = v² + 2∫o·dd`: with `o` within `±A` up to the
    /// distance `s` ([`Self::signed_from`]) and within `[o₀, o₁]` past it,
    /// the leaders are at most `√(v² + 2A·min(d, s) + 2·max(o₁, 0)·(d − s)⁺)`
    /// fast at a distance `d`, never past [`Self::fastest`], and at least
    /// `√(2·max(o₀, 0)·(d − s)⁺)`; the side is split into
    /// [`CRAWL_SIDE_DISTANCE_PIECES`] pieces of distance, each bounded by
    /// [`SideWeight::at_width`] with the speed and `o` ranges over it, and
    /// its bend by [`coupled_bend`] too, which neither a vanishing floor
    /// nor a vanishing distance makes diverge. `Ŵ″ = bend_scale·β − fold`:
    /// the rebuild's largest and least values pair each term's own extreme,
    /// so a fold and a slope of opposite signs, where the leaders speed up
    /// away from the crawl, offset.
    #[must_use]
    pub fn rebuild(&self, width: f64) -> f64 {
        let (out_lo, out_hi) = self.outward;
        assert!(
            width > 0.0
                && self.speed >= 0.0
                && self.accel >= 0.0
                && self.fastest >= self.speed
                && -self.accel <= out_lo
                && out_lo <= out_hi
                && out_hi <= self.accel
                && self.signed_from >= 0.0,
            "a crawl side is ill-formed: {self:?} over {width} mm"
        );
        floor_pieces(self.floor, width)
            .into_iter()
            .map(|floor| self.rebuild_over(width, floor))
            .fold(0.0, f64::max)
    }

    fn rebuild_over(&self, width: f64, floor: (f64, f64)) -> f64 {
        let step = width / f64::from(CRAWL_SIDE_DISTANCE_PIECES);
        let coupled = coupled_bend(
            self.speed,
            self.accel,
            self.crawl_floor(width, floor),
            width,
        );
        let side = SideSpan::over((width, width), floor);
        let speed_sq = self.speed * self.speed;
        let (accel, signed_from) = (self.accel, self.signed_from);
        (0..CRAWL_SIDE_DISTANCE_PIECES)
            .map(|piece| {
                let near = f64::from(piece) * step;
                let far = if piece + 1 == CRAWL_SIDE_DISTANCE_PIECES {
                    width
                } else {
                    f64::from(piece + 1) * step
                };
                let (out_lo, out_hi) = if near >= signed_from {
                    self.outward
                } else {
                    (-accel, accel)
                };
                let weight = side.weight_within((near, far));
                let rising = 2.0 * accel * far.min(signed_from)
                    + 2.0 * self.outward.1.max(0.0) * (far - signed_from).max(0.0);
                let fastest = libm::sqrt(speed_sq + rising).min(self.fastest);
                let slowest =
                    libm::sqrt(2.0 * self.outward.0.max(0.0) * (near - signed_from).max(0.0))
                        .min(fastest);
                signed_rebuild(&weight, coupled, (slowest, fastest), (out_lo, out_hi))
            })
            .fold(0.0, f64::max)
    }

    /// The floor the bend of a side `width` wide reads over the piece
    /// `floor` of its floor range: the crawl's own where the piece reaches
    /// the floor the side keeps of it.
    fn crawl_floor(&self, width: f64, floor: (f64, f64)) -> f64 {
        if floor.1 >= kept_floor(self.floor.1, width) {
            self.floor.1
        } else {
            floor.1
        }
    }

    /// [`RebuildsWithin`] a side `width` wide.
    #[must_use]
    pub fn rebuilds_within(&self, width: f64) -> RebuildsWithin {
        assert!(
            width > 0.0,
            "a crawl side spans a positive width, not {width} mm"
        );
        RebuildsWithin {
            width,
            fastest: self.fastest,
            pieces: floor_pieces(self.floor, width)
                .into_iter()
                .map(|floor| {
                    (
                        SideSpan::over((width, width), floor),
                        coupled_bend(
                            self.speed,
                            self.accel,
                            self.crawl_floor(width, floor),
                            width,
                        ),
                    )
                })
                .collect(),
        }
    }
}

/// The most `|W″·ṡ³ + 3W′·ṡ·s̈|` asks of leaders within a distance range of
/// a crawl, within a speed range and with their path acceleration away
/// from the crawl within a range there, over a side of one width
/// ([`CrawlSide::rebuilds_within`]): `Ŵ″·ṡ³ + 3Ŵ′·ṡ·o` read as
/// [`CrawlSide::rebuild`] reads each piece of its side, with `Ŵ`, `Ŵ′` and
/// `Ŵ″` read by [`SideWeight`] over that distance and `β·ṡ³` within
/// [`coupled_bend`] too, the side's floor range split as
/// [`CrawlSide::rebuild`] splits it. It reads the leaders where they are,
/// which bounds a side across a long crawl far closer than
/// [`CrawlSide::rebuild`], whose speed grows from the crawl's with the
/// strongest acceleration anywhere along the side; where they are known to
/// speed up away from the crawl, the fold and the slope offset. What reads
/// neither the distance nor the leaders there is read once per floor
/// piece.
#[derive(Debug, Clone, PartialEq)]
pub struct RebuildsWithin {
    width: f64,
    fastest: f64,
    pieces: Vec<(SideSpan, f64)>,
}

impl RebuildsWithin {
    /// The most the rebuild asks of leaders within `distance` of the crawl,
    /// within `speed` there and with their path acceleration away from the
    /// crawl, `o`, within `outward`, each a `(least, largest)` range. It
    /// grows as any of the ranges widens.
    #[must_use]
    pub fn at(&self, distance: (f64, f64), speed: (f64, f64), outward: (f64, f64)) -> f64 {
        assert!(
            0.0 <= distance.0
                && distance.0 <= distance.1
                && 0.0 <= speed.0
                && speed.0 <= speed.1
                && outward.0 <= outward.1,
            "a crawl side's leaders are ill-formed: {speed:?} mm/s, {outward:?} mm/s² away from \
             the crawl within {distance:?} mm"
        );
        let fastest = speed.1.min(self.fastest);
        let speed = (speed.0.min(fastest), fastest);
        self.pieces
            .iter()
            .map(|(side, coupled)| {
                signed_rebuild(&side.weight_within(distance), *coupled, speed, outward)
            })
            .fold(0.0, f64::max)
    }

    /// Whether the rebuild asks no more than `room` of leaders between two
    /// readings of them, `near` and `far`, each their distance from the
    /// crawl, signed (negative inside the zero stretch), and their speed
    /// there, `near.0 ≤ far.0`. Between the readings `|s̈|` is within
    /// `accel`, and `ṡ²` changes by at most `2·growth` per mm: from either
    /// reading, `ṡ² ≤ v² + 2·growth·|Δd|`. Readings far apart say nothing of
    /// the leaders between them but that, so the part of the side between
    /// them is read in pieces no wider than the side over
    /// [`CRAWL_SIDE_DISTANCE_PIECES`], each at the speed both bounds leave
    /// over it ([`Self::at`]), once the whole of it read at once does not
    /// fit.
    #[must_use]
    pub fn fits_between(
        &self,
        near: (f64, f64),
        far: (f64, f64),
        (growth, accel): (f64, f64),
        room: f64,
    ) -> bool {
        assert!(
            near.0 <= far.0 && near.1 >= 0.0 && far.1 >= 0.0 && growth >= 0.0,
            "a crawl side's readings are ill-formed: {near:?} to {far:?}, ṡ² growing at most \
             {growth} mm/s² per mm"
        );
        let (lo, hi) = (near.0.max(0.0), far.0.min(self.width));
        if lo > hi {
            return true;
        }
        let speed_over = |(from, to): (f64, f64)| {
            let from_near = near.1 * near.1 + 2.0 * growth * (to - near.0);
            let from_far = far.1 * far.1 + 2.0 * growth * (far.0 - from);
            libm::sqrt(from_near.min(from_far))
        };
        let fits =
            |range: (f64, f64)| self.at(range, (0.0, speed_over(range)), (-accel, accel)) <= room;
        if fits((lo, hi)) {
            return true;
        }
        let longest = self.width / f64::from(CRAWL_SIDE_DISTANCE_PIECES);
        let pieces = libm::ceil((hi - lo) / longest).max(1.0) as u32;
        let step = (hi - lo) / f64::from(pieces);
        (0..pieces).all(|piece| {
            let from = lo + f64::from(piece) * step;
            let to = if piece + 1 == pieces { hi } else { from + step };
            fits((from, to))
        })
    }
}

/// The most `|Ŵ″·ṡ³ + 3Ŵ′·ṡ·o|` asks of leaders where the side's weight
/// lies within `weight`, its bend's `β·ṡ³` within `coupled`, their speed
/// within `[slowest, fastest]` and `o` within `[out_lo, out_hi]`. With
/// `Ŵ″ = bend_scale·β − fold` and `Ŵ′ ≥ 0`, its largest and least values
/// pair each term's own extreme, so a fold and a slope of opposite signs,
/// where the leaders speed up away from the crawl, offset.
fn signed_rebuild(
    weight: &SideWeight,
    coupled: f64,
    (slowest, fastest): (f64, f64),
    (out_lo, out_hi): (f64, f64),
) -> f64 {
    let (fastest_cubed, slowest_cubed) = (fastest * fastest * fastest, slowest * slowest * slowest);
    let bend = if weight.bend.1 == 0.0 || fastest == 0.0 {
        0.0
    } else {
        weight.bend_scale.1 * (weight.bend.1 * fastest_cubed).min(coupled)
    };
    let least_bend = weight.bend_scale.0 * weight.bend.0 * slowest_cubed;
    let lead = |out: f64| {
        3.0 * if out >= 0.0 {
            weight.slope.1 * fastest * out
        } else {
            weight.slope.0 * slowest * out
        }
    };
    let least_lead = |out: f64| {
        3.0 * if out >= 0.0 {
            weight.slope.0 * slowest * out
        } else {
            weight.slope.1 * fastest * out
        }
    };
    let largest = bend - weight.fold.0 * slowest_cubed + lead(out_hi);
    let least = least_bend - weight.fold.1 * fastest_cubed + least_lead(out_lo);
    largest.max(-least).max(0.0)
}

/// The floor range `floor` within a side `width` wide, in geometric pieces
/// ([`CRAWL_SIDE_FLOOR_PIECES`]) down to [`CRAWL_SIDE_FLOOR_SHARE_MIN`] of
/// the side and one more to its least floor; a single floor is one piece.
fn floor_pieces(floor: (f64, f64), width: f64) -> Vec<(f64, f64)> {
    let most = kept_floor(floor.1, width);
    let least = floor.0.min(most);
    if least == most {
        return vec![(least, most)];
    }
    let split_from = (CRAWL_SIDE_FLOOR_SHARE_MIN * width).clamp(least, most);
    let ratio = libm::pow(most / split_from, 1.0 / f64::from(CRAWL_SIDE_FLOOR_PIECES));
    let edge = |piece: i32| match piece {
        0 => split_from,
        CRAWL_SIDE_FLOOR_PIECES => most,
        _ => (split_from * libm::pow(ratio, f64::from(piece))).min(most),
    };
    let mut pieces = Vec::with_capacity(CRAWL_SIDE_FLOOR_PIECES as usize + 1);
    if least < split_from {
        pieces.push((least, split_from));
    }
    pieces.extend((0..CRAWL_SIDE_FLOOR_PIECES).map(|piece| {
        let (lo, hi) = (edge(piece), edge(piece + 1));
        (lo.min(hi), hi)
    }));
    pieces
}

/// `σ̇³·β` at most over a side `width` wide whose floor is at most
/// `floor`: the floor's bend `β = f̃²/(d² + f̃²)^{3/2}`, `f̃ = min(f, w)`,
/// times the cube of the path speed at a distance `d` from a crawl at
/// `v_c` with the path acceleration within `A` between, `σ̇² ≤ v_c² + 2A·d`.
/// With `x = d/f̃`, `σ̇³·β ≤ ((q + 2A·f̃·x)/(1 + x²))^{3/2}/f̃` for any
/// `q ≥ v_c²`, and `(q + 2A·f̃·x)/(1 + x²)` peaks over `x` at
/// `(q + √(q² + 4A²f̃²))/2`, which rises with `q` and, `q` held, with `f̃`.
/// `speed` bounds `v_c`, and a crawl floor `f = v_c²/a` the side keeps
/// whole, with `a ≤ A`, keeps `v_c² ≤ A·f̃` too, where the bound is at most
/// `(φ·A·f̃)^{3/2}/f̃ = φ^{3/2}·A^{3/2}·√f̃`, `φ` the golden ratio. With no
/// floor at all `β` vanishes.
#[must_use]
pub fn coupled_bend(speed: f64, accel: f64, floor: f64, width: f64) -> f64 {
    let kept = kept_floor(floor, width);
    if kept == 0.0 {
        return 0.0;
    }
    let level = accel * kept;
    let crawl_speed_sq = speed * speed;
    let q = if kept < floor {
        crawl_speed_sq
    } else {
        level.min(crawl_speed_sq)
    };
    let peak = 0.5 * (q + libm::hypot(q, 2.0 * level));
    peak * libm::sqrt(peak) / kept
}

fn crawl_side_grid(step: i32) -> f64 {
    CRAWL_SIDE_GRID_UNIT_MM * libm::exp2(f64::from(step) / CRAWL_SIDE_GRID_STEPS_PER_DOUBLING)
}

/// The first step of the crawl side grid past `width`.
fn crawl_side_grid_step_past(width: f64) -> i32 {
    let mut step = (libm::log2(width / CRAWL_SIDE_GRID_UNIT_MM)
        * CRAWL_SIDE_GRID_STEPS_PER_DOUBLING)
        .floor() as i32;
    while crawl_side_grid(step) <= width {
        step += 1;
    }
    while crawl_side_grid(step - 1) > width {
        step -= 1;
    }
    step
}

/// The width a crawl side is laid at: its leaders' `reach` where the
/// rebuild `fits` it, else the first width of the side grid past the reach
/// it fits; `None` where none up to [`CRAWL_SIDE_WIDEST_MM`] does. Whoever
/// reads the leaders within looser bounds, and so finds the rebuild
/// fitting at fewer widths, never picks a narrower side: every grid width
/// it fits at, a tighter reading fits at too.
#[must_use]
pub fn crawl_side_width(reach: f64, mut fits: impl FnMut(f64) -> bool) -> Option<f64> {
    assert!(
        reach > 0.0 && reach.is_finite(),
        "a crawl side is asked for past a reach of {reach} mm"
    );
    if fits(reach) {
        return Some(reach);
    }
    (crawl_side_grid_step_past(reach)..crawl_side_grid_step_past(CRAWL_SIDE_WIDEST_MM))
        .map(crawl_side_grid)
        .find(|&width| fits(width))
}

/// The widest width of the crawl side grid ([`crawl_side_width`]) past
/// `reach` and within `limit`, `None` where the grid has none between.
#[must_use]
pub fn widest_crawl_side_within(reach: f64, limit: f64) -> Option<f64> {
    if limit <= reach {
        return None;
    }
    let past_limit = crawl_side_grid_step_past(limit.min(CRAWL_SIDE_WIDEST_MM));
    let widest = crawl_side_grid(past_limit - 1);
    (widest > reach).then_some(widest)
}

#[cfg(test)]
mod tests;
