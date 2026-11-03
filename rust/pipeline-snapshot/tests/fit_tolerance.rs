//! `max_path_deviation` and `max_accel_deviation` are promises: every piece
//! a fit accepts must stay within them against the exact signal it replaces,
//! everywhere inside the piece — not just at the ladder's probe points. This
//! replays every snapshot case and a whole dense real print through the
//! pipeline with the fit audit on, and fails on any accepted piece that
//! leaves the tolerance.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use motion_pipeline::fit_audit::{FitViolation, take_violations};
mod corpus;

fn dense_cases() -> Vec<(String, String, PathBuf)> {
    corpus::sim_cases(None)
        .into_iter()
        .filter(|(name, _, _)| name.starts_with("voron_dense_top_layers/"))
        .collect()
}

fn run(name: &str, gcode: &str, config: &Path) -> Vec<FitViolation> {
    take_violations();
    corpus::plan(name, gcode, config);
    take_violations()
}

fn audited_violations() -> Vec<(String, FitViolation)> {
    corpus::snapshot_cases()
        .into_iter()
        .chain(dense_cases())
        .flat_map(|(name, gcode, config)| {
            run(&name, &gcode, &config)
                .into_iter()
                .map(move |violation| (name.clone(), violation))
        })
        .collect()
}

fn table(violations: &[(String, FitViolation)]) -> String {
    let mut rows: BTreeMap<(&str, &str, usize), (usize, f64, f64)> = BTreeMap::new();
    for (name, violation) in violations {
        let row = rows
            .entry((name.as_str(), violation.fit_context, violation.axis))
            .or_insert((0, 0.0, 0.0));
        row.0 += 1;
        row.1 = row.1.max(violation.acceleration_over_tolerance);
        row.2 = row.2.max(violation.position_over_tolerance);
    }
    rows.into_iter()
        .map(|((name, context, axis), (count, acceleration, position))| {
            format!(
                "{name}: {context} axis {axis}: {count} pieces, worst accel {acceleration:.2}x, \
                 worst position {position:.2}x"
            )
        })
        .collect::<Vec<_>>()
        .join("\n")
}

/// Fits whose exact signal is assembled from other tracks' pieces — the
/// follower projections and their kernel outputs — where coalesced seeds
/// and exact convolution transitions leave real breaks within a floor of a
/// span edge. Elsewhere an edge-sliver change does not prove a break: a
/// materialized disk rail's square-root approach to its curvature cap is
/// steep enough to trip it while smooth.
fn assembled_from_pieces(fit_context: &str) -> bool {
    fit_context.starts_with("follower") || fit_context.starts_with("advanced_follower")
}

/// Where a span's edge acceleration changes by more than the tolerance
/// across its edge sliver, the ladder reads it past the sliver. Read at the
/// seam instead and a break that seed coalescing hid there pins the piece
/// to the acceleration on the break's far side. The shaper's unit tests pin
/// the read itself; this holds the whole pipeline to it.
#[test]
fn no_follower_fit_piece_is_pinned_to_a_break_hidden_in_its_edge_sliver() {
    let pinned: Vec<(String, FitViolation)> = audited_violations()
        .into_iter()
        .filter(|(_, violation)| {
            violation.break_in_edge_sliver && assembled_from_pieces(violation.fit_context)
        })
        .collect();
    assert!(
        pinned.is_empty(),
        "{} accepted fit pieces leave the tolerance across a break in their edge sliver:\n{}",
        pinned.len(),
        table(&pinned)
    );
}

#[test]
fn every_accepted_fit_piece_holds_the_fit_tolerance_against_the_exact_signal() {
    let violations: Vec<(String, FitViolation)> = audited_violations()
        .into_iter()
        .filter(|(_, violation)| violation.exemption.is_none())
        .collect();
    assert!(
        violations.is_empty(),
        "{} accepted fit pieces leave the fit tolerance:\n{}",
        violations.len(),
        table(&violations)
    );
}
