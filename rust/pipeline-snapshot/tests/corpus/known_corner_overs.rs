//! The extruding vertices the path-accuracy audit finds past their corner
//! budget on a case, pinned by count and worst share of the budget under
//! the [`super::ratchet`] rules.

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

pub const KNOWN: &[KnownCornerOvers] = &[];

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
