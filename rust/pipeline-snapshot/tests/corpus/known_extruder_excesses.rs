//! The extruder excesses known engine bugs leave on a case, pinned per
//! class by count and worst value under the [`super::ratchet`] rules.
//!
//! TODO(shaped-turning, #685): the planner holds a kernel-free advance's
//! demand along the path, `r·(a_t + k·j_t)`, within `[axis e] max_accel`,
//! but not the shaped path's turning term `k·r·a⊥²/v`, which slow sharp
//! corners raise far past it. Every [`ExcessClass::Turning`] pin is one the
//! shaped-speed predictor #685 designs removes.

use super::extruder::{AccelCause, ExtruderAudit, ExtruderExcess, ToolheadPhase};
use super::ratchet::{Tally, departures, rounded, tally};

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum ExcessClass {
    /// An acceleration excess while the toolhead moves that the path's
    /// turning does not explain.
    Accel,
    /// An acceleration excess while the toolhead moves that the shaped
    /// path's turning asks of a kernel-free advance, see
    /// [`AccelCause::PathTurning`].
    Turning,
    /// A velocity excess while the toolhead moves.
    Velocity,
    /// A velocity excess in a rest transition, where the extruder chain
    /// moves e for the motion around a rest.
    RestTransitionVelocity,
    /// A velocity step beyond its allowance.
    VelocityStep,
}

impl ExcessClass {
    fn of(excess: &ExtruderExcess) -> Option<(Self, f64)> {
        match *excess {
            ExtruderExcess::Acceleration {
                value,
                phase: ToolheadPhase::Moving,
                cause,
                ..
            } => Some((
                match cause {
                    AccelCause::PathTurning => Self::Turning,
                    AccelCause::Unattributed => Self::Accel,
                },
                value,
            )),
            ExtruderExcess::Velocity {
                value,
                phase: ToolheadPhase::Moving,
                ..
            } => Some((Self::Velocity, value)),
            ExtruderExcess::Velocity {
                value,
                phase: ToolheadPhase::RestTransition,
                ..
            } => Some((Self::RestTransitionVelocity, value)),
            ExtruderExcess::VelocityStep { step, .. } => Some((Self::VelocityStep, step)),
            ExtruderExcess::Acceleration { .. } | ExtruderExcess::Velocity { .. } => None,
        }
    }

    /// The worst value as pinned: mm/s² to one decimal, mm/s to four.
    fn printed(self, worst: f64) -> f64 {
        match self {
            Self::Accel | Self::Turning => rounded(worst, 1),
            Self::Velocity | Self::RestTransitionVelocity | Self::VelocityStep => rounded(worst, 4),
        }
    }
}

/// How many excesses of `class` the case named `case` leaves, and the
/// worst of them as [`ExcessClass::printed`] rounds it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct KnownExcesses {
    pub case: &'static str,
    pub class: ExcessClass,
    pub count: usize,
    pub worst: f64,
}

pub const KNOWN: &[KnownExcesses] = &[
];

/// Where the audit of the case named `case` departs from what [`KNOWN`]
/// pins for it, as one failure line.
pub fn extruder_excess(case: &str, audit: &ExtruderAudit) -> Option<String> {
    const SHOWN: usize = 3;
    let mut classified: Vec<(ExcessClass, f64)> = Vec::new();
    let mut unpinned: Vec<String> = Vec::new();
    for excess in &audit.excesses {
        match ExcessClass::of(excess) {
            Some(observation) => classified.push(observation),
            None => unpinned.push(excess.to_string()),
        }
    }
    let pinned: Vec<Tally<ExcessClass>> = KNOWN
        .iter()
        .filter(|known| known.case == case)
        .map(|known| Tally {
            class: known.class,
            count: known.count,
            worst: known.worst,
        })
        .collect();
    let departures = departures(&tally(classified), &pinned, ExcessClass::printed);
    (!departures.is_empty() || !unpinned.is_empty()).then(|| {
        let shown: Vec<&str> = unpinned.iter().take(SHOWN).map(String::as_str).collect();
        format!(
            "extruder excesses depart from the known ones ({}); {} outside any class: {}",
            departures.join("; "),
            unpinned.len(),
            shown.join("; ")
        )
    })
}
