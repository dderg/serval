use std::sync::{Arc, LazyLock, Mutex};

use nurbs::algebra::PiecewisePolynomialKernel;

/// How many cells [`LeaderSpill`] tabulates the leaders' kernels over.
pub const SPILL_CELLS: usize = 16;

/// The spills the configurations built so far hold, each once: every
/// move's caps name theirs by [`LeaderSpillId`] instead of carrying the
/// table.
static SPILLS: LazyLock<Mutex<Vec<Arc<LeaderSpill>>>> =
    LazyLock::new(|| Mutex::new(vec![Arc::new(LeaderSpill::UNSHAPED)]));

fn spills() -> std::sync::MutexGuard<'static, Vec<Arc<LeaderSpill>>> {
    SPILLS
        .lock()
        .expect("a thread registering a leader spill panicked")
}

/// A [`LeaderSpill`] the moves' caps share.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LeaderSpillId(u32);

impl LeaderSpillId {
    pub const UNSHAPED: Self = Self(0);

    #[must_use]
    pub fn spill(self) -> Arc<LeaderSpill> {
        Arc::clone(&spills()[self.0 as usize])
    }

    /// The spill bounding both `self` and `other` ([`LeaderSpill::wider`]).
    #[must_use]
    pub(super) fn wider(self, other: Self) -> Self {
        if self == other {
            return self;
        }
        self.spill().wider(&other.spill()).registered()
    }
}

/// What the leaders' kernels make of the planned speed about a moment.
///
/// A kernel `w` turns a planned speed that stays within a ceiling `C` and
/// changes by at most `A` per second about `t₀`,
/// `v(t) ≤ min(C, v₀ + A·|t − t₀|)`, into a shaped speed at `t₀` of at most
/// `∫|w(u)|·min(C, v₀ + A·|u|) du = mass·C − A·Φ((C − v₀)/A)`, where
/// `Φ(x) = ∫|w(u)|·(x − |u|)⁺ du` is convex with slope `W(x)`, the mass of
/// `|w|` within `x` of the moment. The bound reaches `mass·C` as `v₀`
/// reaches `C` and slopes off from it as gently as the kernel's core is
/// light, so a planned speed lowered a little lowers the shaped one a
/// little.
///
/// The spill keeps `mass = ∫|w|`, the largest among the followed axes'
/// kernels (1 where none shapes them), the largest first moment
/// `∫|u|·|w(u)| du` among them, and at the ends `xᵢ = (i + 1)·cell_s` of
/// [`SPILL_CELLS`] cells spanning the kernels' reach the least `W` and `Φ`
/// among them. Within a cell `Φ` is at least its value at the cell's start
/// plus `W` there times the way in, within the first at least
/// `core_density·x²`, `core_density` the least `|w|` there, and, read
/// against `mass·C`, at least `mass·x − moment`, the line each kernel's
/// `Φ` settles on past its reach, so the bound never exceeds the linear
/// `mass·v₀ + moment·A`. An unshaped path keeps all of it at the moment: no
/// cells, `W = 1`, `Φ(x) = x`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LeaderSpill {
    pub mass: f64,
    pub moment_s: f64,
    pub cell_s: f64,
    pub core_density_per_s: f64,
    pub mass_within: [f64; SPILL_CELLS],
    pub spill_within_s: [f64; SPILL_CELLS],
}

impl LeaderSpill {
    pub const UNSHAPED: Self = Self {
        mass: 1.0,
        moment_s: 0.0,
        cell_s: 0.0,
        core_density_per_s: 0.0,
        mass_within: [1.0; SPILL_CELLS],
        spill_within_s: [0.0; SPILL_CELLS],
    };

    /// The spill of the shaped leaders' `kernels`, the unshaped one without
    /// any, shared by every move whose caps carry it.
    #[must_use]
    pub fn of_kernels(kernels: &[&PiecewisePolynomialKernel]) -> LeaderSpillId {
        if kernels.is_empty() {
            return LeaderSpillId::UNSHAPED;
        }
        let reach = kernels
            .iter()
            .map(|kernel| {
                let (back, forward) = kernel.support();
                forward.max(-back)
            })
            .fold(0.0, f64::max);
        let cell_s = reach / SPILL_CELLS as f64;
        let ends: Vec<f64> = (1..=SPILL_CELLS).map(|end| end as f64 * cell_s).collect();
        let mut spill = Self {
            mass: Self::UNSHAPED.mass,
            moment_s: Self::UNSHAPED.moment_s,
            cell_s,
            core_density_per_s: f64::INFINITY,
            mass_within: [f64::INFINITY; SPILL_CELLS],
            spill_within_s: [f64::INFINITY; SPILL_CELLS],
        };
        for kernel in kernels {
            let (mass, moment_s) = kernel.absolute_moments();
            spill.mass = spill.mass.max(mass);
            spill.moment_s = spill.moment_s.max(moment_s);
            spill.core_density_per_s = spill
                .core_density_per_s
                .min(kernel.least_absolute_within(cell_s));
            for (cell, (mass, integral)) in
                kernel.absolute_mass_within(&ends).into_iter().enumerate()
            {
                spill.mass_within[cell] = spill.mass_within[cell].min(mass);
                spill.spill_within_s[cell] = spill.spill_within_s[cell].min(integral);
            }
        }
        spill.registered()
    }

    /// The fastest the shaped path moves at a moment the planned path
    /// crosses at `planned`, its planned speed staying within `ceiling` and
    /// changing by at most `accel` per second about it.
    #[must_use]
    pub fn shaped_speed(&self, planned: f64, ceiling: f64, accel: f64) -> f64 {
        assert!(
            ceiling.is_finite() && accel.is_finite() && accel >= 0.0,
            "the shaped speed is bounded under a ceiling of {ceiling} mm/s and an acceleration \
             of {accel} mm/s²"
        );
        let room = (ceiling - planned).max(0.0);
        if accel == 0.0 {
            return self.mass * (ceiling - room);
        }
        self.mass * ceiling - accel * self.least_spill_within(room / accel)
    }

    /// The spill bounding both `self` and `other`: the larger mass and
    /// moment, and the least `W` and `Φ` either tells at the ends of the
    /// wider cells.
    fn wider(&self, other: &Self) -> Self {
        let cell_s = self.cell_s.max(other.cell_s);
        let end = |cell: usize| (cell + 1) as f64 * cell_s;
        Self {
            mass: self.mass.max(other.mass),
            moment_s: self.moment_s.max(other.moment_s),
            cell_s,
            core_density_per_s: self
                .least_density_within(cell_s)
                .min(other.least_density_within(cell_s)),
            mass_within: std::array::from_fn(|cell| {
                self.least_mass_within(end(cell))
                    .min(other.least_mass_within(end(cell)))
            }),
            spill_within_s: std::array::from_fn(|cell| {
                self.least_spill_within(end(cell))
                    .min(other.least_spill_within(end(cell)))
            }),
        }
    }

    /// The id of the one registered copy of this spill.
    fn registered(self) -> LeaderSpillId {
        let mut spills = spills();
        let index = match spills.iter().position(|spill| **spill == self) {
            Some(index) => index,
            None => {
                spills.push(Arc::new(self));
                spills.len() - 1
            }
        };
        LeaderSpillId(u32::try_from(index).expect("fewer than 2³² distinct leader spills"))
    }

    pub(super) fn check(&self) -> Result<(), &'static str> {
        let finite_non_negative = |values: &[f64]| {
            values
                .iter()
                .all(|value| value.is_finite() && *value >= 0.0)
        };
        let rising = |values: &[f64]| values.windows(2).all(|pair| pair[0] <= pair[1]);
        if !(self.mass.is_finite() && self.mass >= 1.0) {
            return Err("the leaders' spill must be finite, its mass at least 1");
        }
        if !(finite_non_negative(&[self.moment_s, self.cell_s, self.core_density_per_s])
            && finite_non_negative(&self.mass_within)
            && finite_non_negative(&self.spill_within_s)
            && rising(&self.mass_within)
            && rising(&self.spill_within_s)
            && self.mass_within[SPILL_CELLS - 1] <= self.mass)
        {
            return Err(
                "the leaders' spill must tabulate a finite, non-negative, rising mass within its \
                 mass and a rising integral of it",
            );
        }
        Ok(())
    }

    fn reach_s(&self) -> f64 {
        SPILL_CELLS as f64 * self.cell_s
    }

    /// The cell end at or below `x`, `None` within the first cell.
    fn end_below(&self, x: f64) -> Option<usize> {
        if x >= self.reach_s() {
            return Some(SPILL_CELLS - 1);
        }
        ((x / self.cell_s) as usize).checked_sub(1)
    }

    /// At most `W(x)`, and the slope `Φ` keeps from `x` on.
    fn least_mass_within(&self, x: f64) -> f64 {
        match self.end_below(x) {
            None => 2.0 * self.core_density_per_s * x,
            Some(cell) => self.mass_within[cell],
        }
    }

    /// At most `Φ(x)`.
    fn least_spill_within(&self, x: f64) -> f64 {
        let within_cells = match self.end_below(x) {
            None => self.core_density_per_s * x * x,
            Some(cell) => {
                let end = (cell + 1) as f64 * self.cell_s;
                self.spill_within_s[cell] + self.mass_within[cell] * (x - end)
            }
        };
        within_cells.max(self.mass * x - self.moment_s)
    }

    /// The largest `m` with `W(x) ≥ 2m·x` on `(0, radius]` the spill tells.
    fn least_density_within(&self, radius: f64) -> f64 {
        let core = if self.cell_s > 0.0 {
            self.core_density_per_s
        } else {
            f64::INFINITY
        };
        (0..SPILL_CELLS)
            .take_while(|&cell| (cell + 1) as f64 * self.cell_s < radius)
            .map(|cell| {
                let next_end = if cell + 1 < SPILL_CELLS {
                    (cell + 2) as f64 * self.cell_s
                } else {
                    f64::INFINITY
                };
                self.mass_within[cell] / (2.0 * next_end.min(radius))
            })
            .fold(core, f64::min)
    }
}

#[cfg(test)]
mod tests;
