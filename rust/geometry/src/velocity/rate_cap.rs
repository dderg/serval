use super::seam_envelope::ReachableSpeed;

const GOLDEN_SECTION_STEPS: u32 = 64;

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
    /// pair takes the run soonest ([`ReachableSpeed::run_time`]): a looser
    /// limit only raises the speed each budget fits at, so the least time
    /// it finds never rises.
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
        let budget_top = if self.per_accel > 0.0 {
            accel.min(max / self.per_accel)
        } else {
            accel
        };
        let budget = least_on(0.0, budget_top, |budget| {
            let speed = speed_at(budget);
            if speed > 0.0 {
                reachable.run_time(speed, budget)
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
        if std::env::var("ZZTRADE").is_ok() {
            let time = |b: f64| reachable.run_time(speed_at(b), b);
            let grid: Vec<String> = (1..=10)
                .map(|i| {
                    let b = budget_top * i as f64 / 10.0;
                    format!("{b:.0}:{:.0}:{:.6}", speed_at(b), time(b))
                })
                .collect();
            eprintln!(
                "ZZTRADE reached {reached:.4} accel {accel:.1} top {budget_top:.1} -> ({speed:.4}, {budget:.1}) t {:.6} | {} | behind {:?} ahead {:?} len {}",
                time(budget),
                grid.join(" "),
                reachable.behind.terms(),
                reachable.ahead.terms(),
                reachable.length_mm
            );
        }
        (speed, budget)
    }
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
