//! The open fit violations — accepted pieces outside the fit tolerance
//! that no [`motion_pipeline::fit_audit::Exemption`] covers — known engine
//! bugs leave on a case, pinned per fit and class by count and worst
//! acceleration over the tolerance under the [`super::ratchet`] rules.

use motion_pipeline::fit_audit::FitViolation;

use super::ratchet::{Tally, departures, rounded, tally};

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum FitViolationClass {
    /// A piece accepted at the fit floor whose position holds the
    /// tolerance and whose acceleration leaves it across a break in its
    /// edge sliver.
    FloorEdgeBreak,
}

impl FitViolationClass {
    fn of(violation: &FitViolation) -> Option<Self> {
        (violation.accepted_at_floor
            && violation.break_in_edge_sliver
            && violation.position_over_tolerance <= 1.0)
            .then_some(Self::FloorEdgeBreak)
    }
}

/// How many open violations of `class` the fit named `fit` leaves on the
/// case named `case`, and the worst acceleration over the tolerance among
/// them, to two decimals.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct KnownFitViolations {
    pub case: &'static str,
    pub fit: &'static str,
    pub class: FitViolationClass,
    pub count: usize,
    pub worst: f64,
}

pub const KNOWN: &[KnownFitViolations] = &[
    // TODO(bound-ext, floor-slivers): floor pieces on cap-contact onsets 14.5 ns apart fail OnsetFloor.
    KnownFitViolations {
        case: "voron_first_layers:1400-4499@74s/tools/bench/configs/trident_kernel.cfg",
        fit: "materialize_source",
        class: FitViolationClass::FloorEdgeBreak,
        count: 2123,
        worst: 11.09,
    },
];

/// Where the open violations of the case named `case` depart from what
/// [`KNOWN`] pins for it, as one failure line.
pub fn fit_violations(case: &str, open: &[FitViolation]) -> Option<String> {
    let mut classified: Vec<((&'static str, FitViolationClass), f64)> = Vec::new();
    let mut unpinned: Vec<&FitViolation> = Vec::new();
    for violation in open {
        match FitViolationClass::of(violation) {
            Some(class) => classified.push((
                (violation.fit_context, class),
                violation.acceleration_over_tolerance,
            )),
            None => unpinned.push(violation),
        }
    }
    let pinned: Vec<Tally<(&'static str, FitViolationClass)>> = KNOWN
        .iter()
        .filter(|known| known.case == case)
        .map(|known| Tally {
            class: (known.fit, known.class),
            count: known.count,
            worst: known.worst,
        })
        .collect();
    let departures = departures(&tally(classified), &pinned, |_, worst| rounded(worst, 2));
    (!departures.is_empty() || !unpinned.is_empty()).then(|| {
        format!(
            "open fit violations depart from the known ones ({}); {} outside any class",
            departures.join("; "),
            unpinned.len()
        )
    })
}
