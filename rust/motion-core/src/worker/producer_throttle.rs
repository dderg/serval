//! A producer slowed on purpose: the simulator's stand-in for a host too
//! weak to plan a print in realtime. Set [`PRODUCER_THROTTLE_ENV`] to
//! `<realtime_factor>@<from_stream_s>` and, for everything it plans past
//! stream time `from_stream_s`, the planning side spends
//! `1 / realtime_factor` of the planned motion's duration, brakes to rest
//! included, as a starved host would.
//!
//! The time is owed when the motion leaves planning and paid at most
//! [`MAX_HOLD_PER_STEP`] per planning step. A real host's cost goes with the
//! moves it plans, one step at a time; planning stages that hold motion back
//! (the turning guard) release seconds of it in one step, and charging that
//! whole release to the step would blind the pacer for seconds no real
//! planner spends in one step.

use std::time::Duration;

pub const PRODUCER_THROTTLE_ENV: &str = "SERVAL_PRODUCER_THROTTLE";
pub(crate) const MAX_HOLD_PER_STEP: Duration = Duration::from_millis(20);

#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct ProducerThrottle {
    realtime_factor: f64,
    from_stream_s: f64,
    owed_s: f64,
}

impl ProducerThrottle {
    pub(crate) fn from_env() -> Option<Self> {
        let spec = std::env::var(PRODUCER_THROTTLE_ENV).ok()?;
        let throttle =
            Self::parse(&spec).unwrap_or_else(|error| panic!("{PRODUCER_THROTTLE_ENV}: {error}"));
        tracing::warn!(
            subsystem = "motion",
            event = "producer_throttle_active",
            realtime_factor = throttle.realtime_factor,
            from_stream_s = throttle.from_stream_s,
            "[ingress] the producer is throttled below realtime on purpose ({PRODUCER_THROTTLE_ENV})"
        );
        Some(throttle)
    }

    pub(crate) fn parse(spec: &str) -> Result<Self, String> {
        let (factor, from) = spec
            .split_once('@')
            .ok_or_else(|| format!("expected <realtime_factor>@<from_stream_s>, got {spec:?}"))?;
        let realtime_factor: f64 = factor
            .trim()
            .parse()
            .map_err(|e| format!("realtime factor {factor:?}: {e}"))?;
        let from_stream_s: f64 = from
            .trim()
            .parse()
            .map_err(|e| format!("stream time {from:?}: {e}"))?;
        if !(realtime_factor.is_finite() && realtime_factor > 0.0) {
            return Err(format!(
                "realtime factor must be finite and positive, got {realtime_factor}"
            ));
        }
        if !(from_stream_s.is_finite() && from_stream_s >= 0.0) {
            return Err(format!(
                "stream time must be finite and non-negative, got {from_stream_s}"
            ));
        }
        Ok(Self {
            realtime_factor,
            from_stream_s,
            owed_s: 0.0,
        })
    }

    /// How long this planning step holds the producer, having planned the
    /// stream from `planned_from_s` through `planned_to_s`.
    pub(crate) fn hold(&mut self, planned_from_s: f64, planned_to_s: f64) -> Duration {
        let throttled_s = planned_to_s - planned_from_s.max(self.from_stream_s);
        if throttled_s > 0.0 {
            self.owed_s += throttled_s / self.realtime_factor;
        }
        let paid_s = self.owed_s.min(MAX_HOLD_PER_STEP.as_secs_f64());
        self.owed_s -= paid_s;
        Duration::from_secs_f64(paid_s)
    }
}

#[cfg(test)]
mod tests;
