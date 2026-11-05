use std::sync::{Arc, LazyLock, Mutex};

use nurbs::algebra::PiecewisePolynomialKernel;

/// How many cells [`SettleSide`] tabulates a side of the leaders' kernels
/// over.
pub const SETTLE_CELLS: usize = 16;

/// The share of `max_path_deviation` a shaped toolhead still settling may
/// stand from the point it rests at while an extrude-only move runs; the
/// output fit keeps the rest.
pub const SETTLE_SHARE_OF_PATH_DEVIATION: f64 = 0.5;

static SETTLES: LazyLock<Mutex<Vec<Arc<RestSettle>>>> =
    LazyLock::new(|| Mutex::new(vec![Arc::new(RestSettle::UNSHAPED)]));

fn settles() -> std::sync::MutexGuard<'static, Vec<Arc<RestSettle>>> {
    SETTLES
        .lock()
        .expect("a thread registering a rest settle panicked")
}

/// A [`RestSettle`] the moves' caps share.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RestSettleId(u32);

impl RestSettleId {
    pub const UNSHAPED: Self = Self(0);

    #[must_use]
    pub fn settle(self) -> Arc<RestSettle> {
        Arc::clone(&settles()[self.0 as usize])
    }

    /// The settle bounding both `self` and `other` ([`RestSettle::wider`]).
    #[must_use]
    pub(super) fn wider(self, other: Self) -> Self {
        if self == other {
            return self;
        }
        self.settle().wider(&other.settle()).registered()
    }
}

/// How the shaped toolhead settles onto a point the planned path rests at.
///
/// A kernel `w` shapes a planned path `x` into `∫ w(u)·x(t − u) du`. Past a
/// planned stop at `P` at `t₀`, `x` stays at `P`, and before it the planned
/// path travels at most `A·s²/2` in the last `s` seconds under a tangential
/// acceleration within `A`, whatever its turns. So `h` seconds after the
/// stop the shaped toolhead stands within `A·∫_{u>h} |w(u)|·(u − h)²/2 du`
/// of `P`, moves at most `A·∫_{u>h} |w|·(u − h)` and accelerates at most
/// `A·∫_{u>h} |w|`; its jerk is at most `A` times `|w(h⁺)|` plus the
/// variation of `w` beyond `h`. Before a planned start the same holds of
/// the kernel's other side. The leaders' kernels act on separate axes, so
/// each bound is the root sum of their squares.
///
/// [`RestSettle::sides`] tabulate those per unit of acceleration at
/// `hᵢ = i·reach/SETTLE_CELLS`: `[0]` after a stop, `[1]` before a start.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RestSettle {
    pub radius_mm: f64,
    pub sides: [SettleSide; 2],
}

/// One side of a [`RestSettle`]: the kernels' reach there and, at the
/// start of each cell, the shaped toolhead's distance from the rest point,
/// speed, acceleration and jerk per unit of planned acceleration. Each
/// falls to 0 at the reach.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SettleSide {
    pub reach_s: f64,
    pub distance_s2: [f64; SETTLE_CELLS],
    pub speed_s: [f64; SETTLE_CELLS],
    pub accel: [f64; SETTLE_CELLS],
    pub jerk_per_s: [f64; SETTLE_CELLS],
}

/// How the leaders' motion settling under an extrude-only move reaches
/// the extruder, the ratio it lays the shaped path at through the advance,
/// and what the extruder may do: its caps and the extrude-only move's own,
/// whose motion starts from rest as the hold ends and comes to rest as the
/// next hold starts.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SettleDemand {
    pub ratio: f64,
    pub advance_slope_s: f64,
    pub advance_curvature_s2_per_mm: f64,
    pub own_jerk_per_accel_hz: f64,
    pub max_velocity_mm_s: f64,
    pub max_accel_mm_s2: f64,
    pub extrude_only_velocity_mm_s: f64,
    pub extrude_only_accel_mm_s2: f64,
}

impl SettleSide {
    const UNSHAPED: Self = Self {
        reach_s: 0.0,
        distance_s2: [0.0; SETTLE_CELLS],
        speed_s: [0.0; SETTLE_CELLS],
        accel: [0.0; SETTLE_CELLS],
        jerk_per_s: [0.0; SETTLE_CELLS],
    };

    fn of_kernels(kernels: &[&PiecewisePolynomialKernel], ahead: bool) -> Self {
        let reach_s = kernels
            .iter()
            .map(|kernel| {
                let (back, forward) = kernel.support();
                if ahead { forward } else { -back }
            })
            .fold(0.0, f64::max);
        let cell_s = reach_s / SETTLE_CELLS as f64;
        let mut side = Self {
            reach_s,
            ..Self::UNSHAPED
        };
        for cell in 0..SETTLE_CELLS {
            let tails: Vec<nurbs::algebra::KernelTail> = kernels
                .iter()
                .map(|kernel| kernel.tail(cell as f64 * cell_s, ahead))
                .collect();
            let root_sum = |of: fn(&nurbs::algebra::KernelTail) -> f64| {
                tails
                    .iter()
                    .map(|tail| of(tail).powi(2))
                    .sum::<f64>()
                    .sqrt()
            };
            side.distance_s2[cell] = root_sum(|tail| tail.half_second_moment_s2);
            side.speed_s[cell] = root_sum(|tail| tail.first_moment_s);
            side.accel[cell] = root_sum(|tail| tail.mass);
            side.jerk_per_s[cell] = root_sum(|tail| tail.variation_per_s);
        }
        side
    }

    fn cell_s(&self) -> f64 {
        self.reach_s / SETTLE_CELLS as f64
    }

    /// The cell whose start is at or below `h`, `None` at or past the
    /// reach, where nothing is left to settle.
    fn cell_at(&self, h: f64) -> Option<usize> {
        (h < self.reach_s).then(|| ((h / self.cell_s()) as usize).min(SETTLE_CELLS - 1))
    }

    /// The least hold, a cell start or the reach, after which the shaped
    /// toolhead settling under planned accelerations within `accel` stays
    /// within `radius_mm` of the rest point and asks the extruder for no
    /// more than `demand` leaves it.
    #[must_use]
    pub fn hold_s(&self, radius_mm: f64, accel: f64, demand: &SettleDemand) -> f64 {
        (0..SETTLE_CELLS)
            .find(|&cell| {
                accel * self.distance_s2[cell] <= radius_mm && self.fits(cell, accel, demand)
            })
            .map_or(self.reach_s, |cell| cell as f64 * self.cell_s())
    }

    /// Whether, held to the start of `cell`, the extruder carries the
    /// settling motion and the extrude-only move together within its caps:
    /// in every later cell, the motion's demand there on top of the move's,
    /// which has sped up at most to the cell's end since the hold.
    fn fits(&self, cell: usize, accel: f64, demand: &SettleDemand) -> bool {
        if demand.ratio == 0.0 {
            return true;
        }
        (cell..SETTLE_CELLS).all(|later| {
            let speed = accel * self.speed_s[later];
            let shaped_accel = accel * self.accel[later];
            let jerk = if demand.own_jerk_per_accel_hz.is_finite() {
                demand.own_jerk_per_accel_hz * shaped_accel
            } else {
                accel * self.jerk_per_s[later]
            };
            let base = demand.ratio * shaped_accel;
            let since_hold_s = (later + 1 - cell) as f64 * self.cell_s();
            let extrude_only_velocity = demand
                .extrude_only_velocity_mm_s
                .min(demand.extrude_only_accel_mm_s2 * since_hold_s);
            demand.ratio * demand.advance_slope_s.mul_add(shaped_accel, speed)
                + extrude_only_velocity
                <= demand.max_velocity_mm_s
                && demand.advance_curvature_s2_per_mm * base * base
                    + demand.advance_slope_s * demand.ratio * jerk
                    + base
                    + demand.extrude_only_accel_mm_s2
                    <= demand.max_accel_mm_s2
        })
    }

    /// At least each of `self`'s and `other`'s values at every cell start
    /// of the longer reach: a value holds from its cell start to the next.
    fn wider(&self, other: &Self) -> Self {
        let reach_s = self.reach_s.max(other.reach_s);
        let cell_s = reach_s / SETTLE_CELLS as f64;
        let larger = |values: fn(&Self) -> &[f64; SETTLE_CELLS]| -> [f64; SETTLE_CELLS] {
            std::array::from_fn(|cell| {
                let h = cell as f64 * cell_s;
                [self, other]
                    .iter()
                    .filter_map(|side| side.cell_at(h).map(|at| values(side)[at]))
                    .fold(0.0, f64::max)
            })
        };
        Self {
            reach_s,
            distance_s2: larger(|side| &side.distance_s2),
            speed_s: larger(|side| &side.speed_s),
            accel: larger(|side| &side.accel),
            jerk_per_s: larger(|side| &side.jerk_per_s),
        }
    }

    fn check(&self) -> Result<(), &'static str> {
        let finite = |value: &f64| value.is_finite() && *value >= 0.0;
        let settling = |values: &[f64; SETTLE_CELLS]| {
            values.iter().all(finite) && values.windows(2).all(|pair| pair[0] >= pair[1])
        };
        let tabulated = finite(&self.reach_s)
            && settling(&self.distance_s2)
            && settling(&self.speed_s)
            && settling(&self.accel)
            && self.jerk_per_s.iter().all(finite);
        if tabulated {
            Ok(())
        } else {
            Err(
                "a rest settle must tabulate a finite, non-negative reach and finite, \
                 non-negative distances, speeds and accelerations that fall cell by cell",
            )
        }
    }
}

impl RestSettle {
    pub const UNSHAPED: Self = Self {
        radius_mm: 0.0,
        sides: [SettleSide::UNSHAPED; 2],
    };

    /// The settle of the shaped leaders' `kernels` onto a rest point within
    /// [`SETTLE_SHARE_OF_PATH_DEVIATION`] of `max_path_deviation_mm`, the
    /// unshaped one without any kernel, shared by every move whose caps
    /// carry it.
    #[must_use]
    pub fn of_kernels(
        kernels: &[&PiecewisePolynomialKernel],
        max_path_deviation_mm: f64,
    ) -> RestSettleId {
        if kernels.is_empty() {
            return RestSettleId::UNSHAPED;
        }
        Self {
            radius_mm: SETTLE_SHARE_OF_PATH_DEVIATION * max_path_deviation_mm,
            sides: [true, false].map(|ahead| SettleSide::of_kernels(kernels, ahead)),
        }
        .registered()
    }

    /// The settle bounding both `self` and `other`: the smaller radius and
    /// the larger value of every side.
    fn wider(&self, other: &Self) -> Self {
        let radius_mm = if self.sides[0].reach_s == 0.0 && self.sides[1].reach_s == 0.0 {
            other.radius_mm
        } else if other.sides[0].reach_s == 0.0 && other.sides[1].reach_s == 0.0 {
            self.radius_mm
        } else {
            self.radius_mm.min(other.radius_mm)
        };
        Self {
            radius_mm,
            sides: [0, 1].map(|side| self.sides[side].wider(&other.sides[side])),
        }
    }

    fn registered(self) -> RestSettleId {
        let mut settles = settles();
        let index = match settles.iter().position(|settle| **settle == self) {
            Some(index) => index,
            None => {
                settles.push(Arc::new(self));
                settles.len() - 1
            }
        };
        RestSettleId(u32::try_from(index).expect("fewer than 2³² distinct rest settles"))
    }

    pub(super) fn check(&self) -> Result<(), &'static str> {
        if !(self.radius_mm.is_finite() && self.radius_mm >= 0.0) {
            return Err("a rest settle's radius must be finite and non-negative");
        }
        self.sides.iter().try_for_each(SettleSide::check)
    }
}

#[cfg(test)]
mod tests;
