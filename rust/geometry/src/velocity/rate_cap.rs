use super::seam_envelope::ReachableSpeed;

const GOLDEN_SECTION_STEPS: u32 = 48;
const BISECTION_STEPS: u32 = 64;
const BUDGET_GRID: usize = 16;
const SETTLE_ROUNDS_MAX: usize = 64;
/// The relative rounding of a settled budget.
const SETTLED_BUDGET: f64 = 1e-9;
/// The relative rounding of a run time.
const TIME_ROUNDING: f64 = 1e-12;

/// A motor rate under a speed ceiling `v` and an acceleration budget `A`
/// that an advance makes grow with both: `per_velocity·v + per_accel·A`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct LinearRate {
    pub(super) per_velocity: f64,
    pub(super) per_accel: f64,
}

impl LinearRate {
    pub(super) fn at(&self, v: f64, accel: f64) -> f64 {
        self.per_velocity * v + self.per_accel * accel
    }

    /// The ceiling and budget under which `rate ≤ max` and the budget
    /// fits every other cap, `fitting_ceiling(A)` the fastest speed at
    /// which a budget `A` fits them, on a move the plan crosses at no more
    /// than `reachable`. Where `reached`, the ceiling held to the reachable
    /// peak, fits the whole budget, both stay. Otherwise every budget `A`
    /// takes the fastest speed it fits at, and the budget is the one whose
    /// pair takes the run soonest ([`ReachableSpeed::run_time`]) with the
    /// moves around it at the budget it settles on, as the same caps on the
    /// same extrusion trade theirs ([`settled_budget`]).
    pub(super) fn capped(
        &self,
        max: f64,
        reachable: &ReachableSpeed,
        (ceiling, accel): (f64, f64),
        fitting_ceiling: impl Fn(f64) -> f64,
    ) -> (f64, f64) {
        let reached = ceiling.min(reachable.peak_mm_s);
        let speed_at = |budget: f64| {
            reached
                .min(fitting_ceiling(budget))
                .min((max - self.per_accel * budget) / self.per_velocity)
        };
        if speed_at(accel) >= reached {
            return (reached, accel);
        }
        let budget_top = largest_where(accel, |budget| speed_at(budget) > 0.0);
        let budget = settled_budget(budget_top, |budget, around_accel| {
            let speed = speed_at(budget);
            if speed > 0.0 {
                reachable.run_time((speed, budget), around_accel)
            } else {
                f64::INFINITY
            }
        });
        let speed = speed_at(budget);
        assert!(
            speed > 0.0 && budget > 0.0,
            "no budget up to {accel} fits rate {self:?} ≤ {max} at any speed under {reached}: \
             it picked {budget} at {speed}"
        );
        (speed, budget)
    }
}

/// The budget on `(0, top]` whose run is soonest with the moves around it
/// at that same budget, `time(A, around)` the run time. The soonest budget
/// with the moves around at `around`, `F(around)`, never rises as `around`
/// does: the faster the moves around, the less the move's own budget buys.
/// So `F(A) − A` falls, its one root is where the budget settles, at or
/// above `F(top)`: false position finds it, with a bisection after any
/// step that fails to halve the bracket.
fn settled_budget(top: f64, time: impl Fn(f64, f64) -> f64) -> f64 {
    let soonest = |around: f64| least_on(0.0, top, |budget| time(budget, around));
    let excess_at = |around: f64| soonest(around) - around;
    let top_excess = excess_at(top);
    if top_excess >= 0.0 {
        return top;
    }
    let floor = top + top_excess;
    let floor_excess = excess_at(floor);
    if floor_excess <= 0.0 {
        return floor;
    }
    let (mut under, mut under_excess) = (floor, floor_excess);
    let (mut over, mut over_excess) = (top, top_excess);
    let mut bisect = false;
    for _ in 0..SETTLE_ROUNDS_MAX {
        let width = over - under;
        if width <= SETTLED_BUDGET * top {
            return under;
        }
        let guess = if bisect {
            under + 0.5 * width
        } else {
            under + width * under_excess / (under_excess - over_excess)
        };
        let excess = excess_at(guess);
        assert!(
            excess.is_finite(),
            "the budget a run settles on is not finite at {guess}"
        );
        if excess >= 0.0 {
            (under, under_excess) = (guess, excess);
        } else {
            (over, over_excess) = (guess, excess);
        }
        bisect = !bisect && over - under > 0.5 * width;
    }
    panic!(
        "the budget the moves around settle on is not found in {SETTLE_ROUNDS_MAX} rounds: \
         it lies in [{under}, {over}] of [0, {top}]"
    );
}

/// Where `time` is least on `(low, high]`: the least of `BUDGET_GRID`
/// even steps, refined by golden-section search between its neighbours;
/// a budget gives way to a smaller one only for time it saves beyond
/// rounding.
fn least_on(low: f64, high: f64, time: impl Fn(f64) -> f64) -> f64 {
    let faster = |time: f64, than: f64| time < than * (1.0 - TIME_ROUNDING);
    let step = (high - low) / BUDGET_GRID as f64;
    let (mut grid_best, mut grid_time) = (high, time(high));
    for index in (1..BUDGET_GRID).rev() {
        let at = low + step * index as f64;
        let at_time = time(at);
        if faster(at_time, grid_time) {
            (grid_best, grid_time) = (at, at_time);
        }
    }
    let shrink = 0.5 * (5.0_f64.sqrt() - 1.0);
    let (mut low_end, mut high_end) = ((grid_best - step).max(low), (grid_best + step).min(high));
    let mut lower = high_end - shrink * (high_end - low_end);
    let mut upper = low_end + shrink * (high_end - low_end);
    let (mut lower_time, mut upper_time) = (time(lower), time(upper));
    for _ in 0..GOLDEN_SECTION_STEPS {
        if lower_time <= upper_time {
            high_end = upper;
            upper = lower;
            upper_time = lower_time;
            lower = high_end - shrink * (high_end - low_end);
            lower_time = time(lower);
        } else {
            low_end = lower;
            lower = upper;
            lower_time = upper_time;
            upper = low_end + shrink * (high_end - low_end);
            upper_time = time(upper);
        }
    }
    let (best, best_time) = if lower_time <= upper_time {
        (lower, lower_time)
    } else {
        (upper, upper_time)
    };
    if faster(best_time, grid_time) {
        best
    } else {
        grid_best
    }
}

/// The largest value in `(0, high]` at which `holds`, which holds below
/// wherever it holds.
fn largest_where(high: f64, holds: impl Fn(f64) -> bool) -> f64 {
    if holds(high) {
        return high;
    }
    let (mut holding, mut failing) = (0.0, high);
    for _ in 0..BISECTION_STEPS {
        let mid = 0.5 * (holding + failing);
        if holds(mid) {
            holding = mid;
        } else {
            failing = mid;
        }
    }
    holding
}
