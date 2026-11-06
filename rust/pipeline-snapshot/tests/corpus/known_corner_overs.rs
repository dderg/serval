//! The extruding vertices the path-accuracy audit finds past their corner
//! budget on a case, pinned by count and worst share of the budget under
//! the [`super::ratchet`] rules.
//!
//! TODO(fit-jerk, #796): the turning guard under-bounds the advance's jerk
//! demand where the toolhead accelerates out of a rest the extrusion ratio
//! steps across at a lowered acceleration (repro: `origin/fit-jerk-repro`),
//! so the planner's approach stage leaves the move departing such a rest
//! at its own acceleration. An explicit `corner_deviation` then misses the
//! corners within the shaper's reach of that departure. These pins go to
//! 0 when the guard fix lands and the departure slows again.

use super::path_accuracy::PathAccuracy;
use super::ratchet::{Tally, departures, rounded, tally};

/// The one class the pins count: a vertex past its budget.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct OverBudget;

/// How many extruding vertices of the case named `case` miss past their
/// budget, and the worst share of it, to three decimals.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct KnownCornerOvers {
    pub case: &'static str,
    pub count: usize,
    pub worst_share: f64,
}

pub const KNOWN: &[KnownCornerOvers] = &[
    KnownCornerOvers {
        case: "corner_flow flow 1.5x 150/p2_kernel_cd005",
        count: 1,
        worst_share: 1.245,
    },
    KnownCornerOvers {
        case: "corner_flow flow 1.5x 150/p2_ramp_cd005",
        count: 1,
        worst_share: 1.245,
    },
    KnownCornerOvers {
        case: "corner_flow flow 1.5x 150/p3_kernel_cd005",
        count: 1,
        worst_share: 1.981,
    },
    KnownCornerOvers {
        case: "corner_flow flow 1.5x 150/p3_ramp_cd005",
        count: 1,
        worst_share: 1.677,
    },
    KnownCornerOvers {
        case: "corner_flow flow 1.5x 300/p2_kernel_cd005",
        count: 1,
        worst_share: 1.245,
    },
    KnownCornerOvers {
        case: "corner_flow flow 1.5x 300/p2_ramp_cd005",
        count: 1,
        worst_share: 1.245,
    },
    KnownCornerOvers {
        case: "corner_flow flow 1.5x 300/p3_kernel_cd005",
        count: 1,
        worst_share: 1.981,
    },
    KnownCornerOvers {
        case: "corner_flow flow 1.5x 300/p3_ramp_cd005",
        count: 1,
        worst_share: 1.677,
    },
    KnownCornerOvers {
        case: "corner_flow flow 1.5x 50/p2_kernel_cd005",
        count: 1,
        worst_share: 1.244,
    },
    KnownCornerOvers {
        case: "corner_flow flow 1.5x 50/p2_ramp_cd005",
        count: 1,
        worst_share: 1.244,
    },
    KnownCornerOvers {
        case: "corner_flow flow 1.5x 50/p3_kernel_cd005",
        count: 1,
        worst_share: 1.606,
    },
    KnownCornerOvers {
        case: "corner_flow flow 1.5x 50/p3_ramp_cd005",
        count: 1,
        worst_share: 1.473,
    },
    KnownCornerOvers {
        case: "voron_dense_top_layers/p2_kernel_cd005",
        count: 1,
        worst_share: 1.189,
    },
    KnownCornerOvers {
        case: "voron_dense_top_layers/p2_ramp_cd005",
        count: 1,
        worst_share: 1.189,
    },
    KnownCornerOvers {
        case: "voron_dense_top_layers/p3_kernel_cd005",
        count: 1,
        worst_share: 1.797,
    },
    KnownCornerOvers {
        case: "voron_dense_top_layers/p3_ramp_cd005",
        count: 1,
        worst_share: 1.797,
    },
    KnownCornerOvers {
        case: "voron_first_layers/p2_kernel_cd005",
        count: 2,
        worst_share: 1.125,
    },
    KnownCornerOvers {
        case: "voron_first_layers/p2_ramp_cd005",
        count: 2,
        worst_share: 1.128,
    },
    KnownCornerOvers {
        case: "voron_first_layers/p3_kernel_cd005",
        count: 5,
        worst_share: 1.235,
    },
    KnownCornerOvers {
        case: "voron_first_layers/p3_ramp_cd005",
        count: 5,
        worst_share: 1.228,
    },
];

fn printed(_: OverBudget, share: f64) -> f64 {
    rounded(share, 3)
}

/// Where the audit of the case named `case` departs from what [`KNOWN`]
/// pins for it, as one failure line.
pub fn corner_overs(case: &str, report: &PathAccuracy) -> Option<String> {
    let observed = tally(
        report
            .over_budget
            .iter()
            .map(|(_, miss, budget, _)| (OverBudget, miss / budget)),
    );
    let pinned: Vec<Tally<OverBudget>> = KNOWN
        .iter()
        .filter(|known| known.case == case)
        .map(|known| Tally {
            class: OverBudget,
            count: known.count,
            worst: known.worst_share,
        })
        .collect();
    let departures = departures(&observed, &pinned, printed);
    (!departures.is_empty()).then(|| {
        let first = report
            .over_budget
            .first()
            .map_or_else(String::new, |(at, miss, budget, _)| {
                format!(", first {at:?} by {miss:.4} mm against {budget:.4} mm")
            });
        format!(
            "corner overs depart from the known ones ({}){first}",
            departures.join("; ")
        )
    })
}
