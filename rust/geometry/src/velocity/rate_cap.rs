use super::z_caps::crossing_time;

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

    /// `rate ≤ max`: keep the ceiling and cut the budget, or keep the
    /// budget and cut the ceiling, whichever crosses the segment sooner;
    /// when neither alone fits, each gets half of `max`.
    pub(super) fn capped(
        &self,
        max: f64,
        length: f64,
        kappa_peak: f64,
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
        let cut_accel = (max - per_velocity * ceiling) / per_accel;
        match (cut_ceiling > 0.0, cut_accel > 0.0) {
            (true, true) => {
                let keep_ceiling = (ceiling, cut_accel);
                let keep_accel = (cut_ceiling, accel);
                if crossing_time(length, kappa_peak, keep_ceiling)
                    <= crossing_time(length, kappa_peak, keep_accel)
                {
                    keep_ceiling
                } else {
                    keep_accel
                }
            }
            (true, false) => (cut_ceiling, accel),
            (false, true) => (ceiling, cut_accel),
            (false, false) => (0.5 * max / per_velocity, accel.min(0.5 * max / per_accel)),
        }
    }
}
