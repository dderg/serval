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
    // TODO(bowden-switch): slow corner nodes outside a seam's time reach narrow its window.
    KnownExcesses {
        case: "trident_follower_cache_regression/tools/bench/configs/profiles/p1_bowden_bedslinger_ramp.cfg",
        class: ExcessClass::Accel,
        count: 413,
        worst: 26088.6,
    },
    KnownExcesses {
        case: "trident_follower_cache_regression/tools/bench/configs/profiles/p1_bowden_bedslinger_ramp.cfg",
        class: ExcessClass::Velocity,
        count: 72,
        worst: 55.2373,
    },
    KnownExcesses {
        case: "voron_first_layers/tools/bench/configs/profiles/p1_bowden_bedslinger_ramp.cfg",
        class: ExcessClass::Accel,
        count: 29,
        worst: 22676.6,
    },
    KnownExcesses {
        case: "voron_first_layers/tools/bench/configs/profiles/p1_bowden_bedslinger_ramp.cfg",
        class: ExcessClass::Velocity,
        count: 5,
        worst: 48.2076,
    },
    KnownExcesses {
        case: "voron_first_layers_zhop/tools/bench/configs/profiles/p1_bowden_bedslinger_ramp.cfg",
        class: ExcessClass::Accel,
        count: 44,
        worst: 26251.3,
    },
    KnownExcesses {
        case: "voron_first_layers_zhop/tools/bench/configs/profiles/p1_bowden_bedslinger_ramp.cfg",
        class: ExcessClass::Velocity,
        count: 16,
        worst: 56.9817,
    },
    KnownExcesses {
        case: "trident_follower_cache_regression:200-300/tools/bench/configs/profiles/p1_bowden_bedslinger_ramp.cfg",
        class: ExcessClass::Accel,
        count: 134,
        worst: 25146.0,
    },
    KnownExcesses {
        case: "trident_follower_cache_regression:200-300/tools/bench/configs/profiles/p1_bowden_bedslinger_ramp.cfg",
        class: ExcessClass::Velocity,
        count: 40,
        worst: 55.2064,
    },
    KnownExcesses {
        case: "trident_follower_cache_regression:820-900/tools/bench/configs/profiles/p1_bowden_bedslinger_ramp.cfg",
        class: ExcessClass::Accel,
        count: 156,
        worst: 26393.8,
    },
    KnownExcesses {
        case: "trident_follower_cache_regression:820-900/tools/bench/configs/profiles/p1_bowden_bedslinger_ramp.cfg",
        class: ExcessClass::Velocity,
        count: 24,
        worst: 54.9216,
    },
    KnownExcesses {
        case: "voron_first_layers:3550-3700/tools/bench/configs/profiles/p1_bowden_bedslinger_ramp.cfg",
        class: ExcessClass::Accel,
        count: 44,
        worst: 26914.4,
    },
    KnownExcesses {
        case: "voron_first_layers:3550-3700/tools/bench/configs/profiles/p1_bowden_bedslinger_ramp.cfg",
        class: ExcessClass::Velocity,
        count: 17,
        worst: 56.4596,
    },
    KnownExcesses {
        case: "corner_flow travel 50/tools/bench/configs/profiles/p1_bowden_bedslinger_ramp.cfg",
        class: ExcessClass::Accel,
        count: 9,
        worst: 5583.8,
    },
    KnownExcesses {
        case: "corner_flow travel 150/tools/bench/configs/profiles/p1_bowden_bedslinger_ramp.cfg",
        class: ExcessClass::Accel,
        count: 9,
        worst: 5495.1,
    },
    KnownExcesses {
        case: "corner_flow travel 300/tools/bench/configs/profiles/p1_bowden_bedslinger_ramp.cfg",
        class: ExcessClass::Accel,
        count: 9,
        worst: 5495.1,
    },
    // TODO(floor-guard): the print-start prime steps e velocity.
    KnownExcesses {
        case: "voron_dense_top_layers/rust/motion-core/tests/configs/neptune_sim_print.cfg",
        class: ExcessClass::VelocityStep,
        count: 1,
        worst: 0.9949,
    },
    KnownExcesses {
        case: "voron_dense_top_layers_zhop/rust/motion-core/tests/configs/neptune_sim_print.cfg",
        class: ExcessClass::VelocityStep,
        count: 1,
        worst: 0.9949,
    },
    // TODO(bound-ext, floor-slivers): the ladder chases an unshaped onset's singular advance.
    KnownExcesses {
        case: "voron_dense_top_layers/rust/motion-core/tests/configs/neptune_sim_print.cfg",
        class: ExcessClass::Accel,
        count: 6474,
        worst: 2350675.7,
    },
    KnownExcesses {
        case: "voron_first_layers/rust/motion-core/tests/configs/neptune_sim_print.cfg",
        class: ExcessClass::Accel,
        count: 3593,
        worst: 310242.5,
    },
    KnownExcesses {
        case: "voron_first_layers/rust/motion-core/tests/configs/neptune_sim_print.cfg",
        class: ExcessClass::Turning,
        count: 23,
        worst: 118874.7,
    },
    KnownExcesses {
        case: "trident_follower_cache_regression/rust/motion-core/tests/configs/neptune_sim_print.cfg",
        class: ExcessClass::Accel,
        count: 1195,
        worst: 1245046.3,
    },
    KnownExcesses {
        case: "trident_follower_cache_regression/rust/motion-core/tests/configs/neptune_sim_print.cfg",
        class: ExcessClass::Turning,
        count: 8,
        worst: 21399.0,
    },
    KnownExcesses {
        case: "corner_flow zigzag 50/rust/motion-core/tests/configs/neptune_sim_print.cfg",
        class: ExcessClass::Accel,
        count: 448,
        worst: 250608.1,
    },
    KnownExcesses {
        case: "corner_flow zigzag 150/rust/motion-core/tests/configs/neptune_sim_print.cfg",
        class: ExcessClass::Accel,
        count: 448,
        worst: 250615.0,
    },
    KnownExcesses {
        case: "corner_flow zigzag 300/rust/motion-core/tests/configs/neptune_sim_print.cfg",
        class: ExcessClass::Accel,
        count: 448,
        worst: 250587.4,
    },
    KnownExcesses {
        case: "voron_dense_top_layers_zhop/rust/motion-core/tests/configs/neptune_sim_print.cfg",
        class: ExcessClass::Accel,
        count: 6373,
        worst: 2350706.7,
    },
    KnownExcesses {
        case: "voron_first_layers_zhop/rust/motion-core/tests/configs/neptune_sim_print.cfg",
        class: ExcessClass::Accel,
        count: 3578,
        worst: 310242.5,
    },
    KnownExcesses {
        case: "voron_first_layers_zhop/rust/motion-core/tests/configs/neptune_sim_print.cfg",
        class: ExcessClass::Turning,
        count: 23,
        worst: 118874.7,
    },
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
