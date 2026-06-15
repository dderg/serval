use std::collections::HashMap;
use std::time::{Duration, Instant};
use temporal::BindingConstraint;
use trajectory::ReplanBindingSummary;

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BindingLabel {
    pub limit: String,
    pub derivative: &'static str,
    pub via_pa: bool,
}

pub fn label_binding(c: BindingConstraint, names: &[String]) -> Option<BindingLabel> {
    let (set, derivative, via_pa) = match c {
        BindingConstraint::Velocity { set } => (set, "velocity", false),
        BindingConstraint::AccelNorm { set } => (set, "accel", false),
        BindingConstraint::JerkNorm { set } => (set, "jerk", false),
        BindingConstraint::PaVelocity { set } => (set, "velocity", true),
        BindingConstraint::PaAccel { set } => (set, "accel", true),
        BindingConstraint::PaJerk { set } => (set, "jerk", true),
        BindingConstraint::None | BindingConstraint::Boundary => return None,
        _ => {
            debug_assert!(false, "unhandled BindingConstraint in label_binding");
            return None;
        }
    };
    let limit = names
        .get(set)
        .cloned()
        .unwrap_or_else(|| "runtime_caps".to_string());
    Some(BindingLabel {
        limit,
        derivative,
        via_pa,
    })
}

/// Fields carried by the per-move `replan_anytime` structured event. The gap is
/// the deterministic closeness-to-limit measure derived from the verified worst
/// binding ratio: `gap = (1 - ratio).max(0)` — `0` means the trajectory sits on
/// the kinematic limit, `0.06` means it runs ~6% below. `ratio` and `constraint`
/// both come from `check_chain`'s `BindingSummary`, so neither is fabricated.
#[derive(Debug, Clone, PartialEq)]
pub struct AnytimeEventFields {
    pub gap: f64,
    pub limiter_limit: String,
    pub limiter_derivative: &'static str,
    pub limiter_via_pa: bool,
    pub binding_ratio: f64,
}

#[must_use]
pub fn anytime_event_fields(
    binding: &ReplanBindingSummary,
    names: &[String],
) -> AnytimeEventFields {
    match binding
        .worst
        .and_then(|w| label_binding(w.constraint, names).map(|l| (l, w.ratio)))
    {
        Some((l, ratio)) => AnytimeEventFields {
            gap: (1.0 - ratio).max(0.0),
            limiter_limit: l.limit,
            limiter_derivative: l.derivative,
            limiter_via_pa: l.via_pa,
            binding_ratio: ratio,
        },
        // No binding reported ⇒ no measurable shortfall. A gap of 0 honestly
        // says "no conservativeness measured", never inventing 100% from a
        // missing ratio.
        None => AnytimeEventFields {
            gap: 0.0,
            limiter_limit: String::from("none"),
            limiter_derivative: "none",
            limiter_via_pa: false,
            binding_ratio: 0.0,
        },
    }
}

pub const ROLLUP_INTERVAL: Duration = Duration::from_secs(1);

pub struct BindingAccumulator {
    window: HashMap<BindingConstraint, u64>,
    window_samples: u64,
    worst: Option<(BindingConstraint, f64, f64)>,
    last_rollup: Instant,
}

impl BindingAccumulator {
    pub fn new(now: Instant) -> Self {
        Self {
            window: HashMap::new(),
            window_samples: 0,
            worst: None,
            last_rollup: now,
        }
    }

    pub fn record(&mut self, summary: &ReplanBindingSummary, t: f64) {
        for (c, n) in &summary.histogram {
            *self.window.entry(*c).or_insert(0) += u64::from(*n);
            self.window_samples += u64::from(*n);
        }
        if let Some(w) = &summary.worst {
            if self.worst.is_none_or(|(_, r, _)| w.ratio > r) {
                self.worst = Some((w.constraint, w.ratio, t));
            }
        }
    }

    pub fn maybe_rollup(&mut self, now: Instant, names: &[String]) {
        if now.duration_since(self.last_rollup) >= ROLLUP_INTERVAL && !self.window.is_empty() {
            self.emit(names);
            self.reset(now);
        }
    }

    pub fn flush(&mut self, now: Instant, names: &[String]) {
        if !self.window.is_empty() {
            self.emit(names);
            self.reset(now);
        }
    }

    fn reset(&mut self, now: Instant) {
        self.window.clear();
        self.window_samples = 0;
        self.worst = None;
        self.last_rollup = now;
    }

    fn emit(&self, names: &[String]) {
        if let Some((c, ratio, t)) = self.worst {
            if let Some(l) = label_binding(c, names) {
                tracing::info!(
                    subsystem = "motion",
                    event = "binding_rollup",
                    limit = %l.limit,
                    derivative = l.derivative,
                    via_pa = l.via_pa,
                    ratio,
                    t,
                    window_samples = self.window_samples,
                    "binding rollup"
                );
            }
        }
        for (c, count) in &self.window {
            if let Some(l) = label_binding(*c, names) {
                tracing::info!(
                    subsystem = "motion",
                    event = "binding_hist",
                    limit = %l.limit,
                    derivative = l.derivative,
                    via_pa = l.via_pa,
                    count = *count,
                    "binding histogram"
                );
            }
        }
    }

    #[cfg(test)]
    pub fn window_count(&self, c: BindingConstraint) -> u64 {
        self.window.get(&c).copied().unwrap_or(0)
    }

    #[cfg(test)]
    pub fn worst(&self) -> Option<(BindingConstraint, f64, f64)> {
        self.worst
    }
}

#[cfg(test)]
mod tests;
