use super::extrusion_reach::ReachableSpeed;

const GOLDEN_SECTION_STEPS: u32 = 64;
const BISECTION_STEPS: u32 = 64;

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

    /// `rate ≤ max` on a move the plan crosses at no more than `reachable`,
    /// under a ceiling held to the reachable peak, which loses the plan
    /// nothing. Where the budget does not fit at that ceiling, the trade
    /// ([`Self::traded`]) reads the move's own ceiling: a reachable peak
    /// that rises with a looser limit leaves the pair it picks standing.
    pub(super) fn capped(
        &self,
        max: f64,
        reachable: &ReachableSpeed,
        (ceiling, accel): (f64, f64),
    ) -> (f64, f64) {
        let reached = ceiling.min(reachable.peak_mm_s);
        if self.at(reached, accel) <= max {
            return (reached, accel);
        }
        let (speed, budget) = self.traded(max, reachable, (ceiling, accel));
        (speed.min(reachable.peak_mm_s), budget)
    }

    /// Of the ceilings `v` and budgets `A` on `rate = max`, the pair that
    /// takes the run `reachable` bounds, from one seam's limit to the
    /// other's, soonest. The run at `A` peaks at
    /// [`ReachableSpeed::run_peak`], and a ceiling above that only gives up
    /// budget, so the ceiling stays at or under where it meets that peak;
    /// the budget stays whole where the plan never reaches the ceiling it
    /// leaves. Where the run peaks faster than `rate = max` lets the move go
    /// at any budget, the budget stays and the ceiling gives way, or each
    /// gets half of `max` when neither alone fits.
    fn traded(
        &self,
        max: f64,
        reachable: &ReachableSpeed,
        (ceiling, accel): (f64, f64),
    ) -> (f64, f64) {
        let Self {
            per_velocity,
            per_accel,
        } = *self;
        let cut_ceiling = (max - per_accel * accel) / per_velocity;
        if per_accel == 0.0 {
            return (cut_ceiling, accel);
        }
        let still_speed = max / per_velocity;
        let budget_at = |v: f64| (max - per_velocity * v) / per_accel;
        let peak_at = |v: f64| reachable.run_peak(budget_at(v));
        if peak_at(still_speed) >= still_speed {
            return if cut_ceiling > 0.0 {
                (cut_ceiling, accel)
            } else {
                (0.5 * still_speed, accel.min(0.5 * max / per_accel))
            };
        }
        let (mut under, mut over) = (0.0, still_speed);
        for _ in 0..BISECTION_STEPS {
            let mid = 0.5 * (under + over);
            if peak_at(mid) > mid {
                under = mid;
            } else {
                over = mid;
            }
        }
        let top = under.min(ceiling);
        if top <= cut_ceiling {
            return (cut_ceiling, accel);
        }
        let speed = least_on(cut_ceiling.max(0.0), top, |v| {
            run_time(reachable, v, budget_at(v))
        });
        (speed, budget_at(speed).min(accel))
    }
}

/// The least time the run `reachable` bounds takes from its entry seam's
/// limit to its exit seam's at speeds up to `ceiling` and accelerations up
/// to `accel`.
fn run_time(reachable: &ReachableSpeed, ceiling: f64, accel: f64) -> f64 {
    let run = reachable.run_mm;
    let climb = 2.0 * accel * run;
    let entry = reachable.entry.at(accel).min(ceiling);
    let exit = reachable
        .exit
        .at(accel)
        .min(ceiling)
        .min((entry * entry + climb).sqrt());
    let entry = entry.min((exit * exit + climb).sqrt());
    let peak = ceiling.min((0.5 * (entry * entry + exit * exit) + accel * run).sqrt());
    let ramps_mm = (2.0 * peak * peak - entry * entry - exit * exit) / (2.0 * accel);
    (2.0 * peak - entry - exit) / accel + (run - ramps_mm).max(0.0) / peak
}

/// Where `time` is least on `[low, high]` as golden-section search finds
/// it, `high` itself where that is no slower.
fn least_on(low: f64, high: f64, time: impl Fn(f64) -> f64) -> f64 {
    let shrink = 0.5 * (5.0_f64.sqrt() - 1.0);
    let (mut low_end, mut high_end) = (low, high);
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
    if time(high) <= best_time { high } else { best }
}
