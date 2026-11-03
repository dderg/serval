//! `max_path_deviation` and `max_accel_deviation` are promises: every piece
//! a fit accepts must stay within them against the exact signal it replaces,
//! everywhere inside the piece — not just at the ladder's probe points. A
//! follower's acceleration promise is per piece: the looser of
//! `max_accel_deviation` and the acceleration that spends its position
//! tolerance over the piece. This replays every snapshot case and a whole
//! dense real print through the pipeline with the fit audit on, and fails
//! on any accepted piece that leaves the tolerance.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use motion_pipeline::fit_audit::{FitViolation, take_violations};
mod corpus;

const DENSE_PRINT: &str = "voron_dense_top_layers";

/// A chain the bench configs leave out, held to the audit on the dense print
/// too: a 70 000 mm/s² Trident whose kernel-free pressure advance under a
/// ratio ramp fits the follower's advanced source to the position-equivalent
/// budget.
/// TODO: the same Trident with a smoothing kernel after the advance leaves
/// 61 `follower_source` pieces up to 1.94x over the absolute budget, which
/// the error's interpolant between probes does not see.
const TRIDENT_CONFIGS: [&str; 1] = ["tools/bench/configs/trident_ramp.cfg"];

fn dense_case(config: &str) -> Vec<(String, String, PathBuf)> {
    let gcode = std::fs::read_to_string(
        corpus::repo().join(format!("tools/sim/gcode/{DENSE_PRINT}.gcode")),
    )
    .expect("read the dense print");
    vec![(
        format!("{DENSE_PRINT}/{config}"),
        gcode,
        corpus::repo().join(config),
    )]
}

fn run(name: &str, gcode: &str, config: &Path) -> Vec<FitViolation> {
    take_violations();
    corpus::plan(name, gcode, config);
    take_violations()
}

fn audited_violations(cases: Vec<(String, String, PathBuf)>) -> Vec<(String, FitViolation)> {
    cases
        .into_iter()
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

/// Every accepted piece of `cases` holds the fit tolerance against its
/// exact signal, and none of a follower's is pinned to a break hidden in
/// its edge sliver: where a span's edge acceleration changes by more than
/// the tolerance across the sliver, the ladder reads it past the sliver,
/// and read at the seam instead a break that seed coalescing hid there pins
/// the piece to the acceleration on the break's far side. The shaper's unit
/// tests pin that read; this holds the whole pipeline to both.
fn assert_every_accepted_piece_holds_the_fit_tolerance(cases: Vec<(String, String, PathBuf)>) {
    let audited = audited_violations(cases);
    let pinned: Vec<(String, FitViolation)> = audited
        .iter()
        .filter(|(_, violation)| {
            violation.break_in_edge_sliver && assembled_from_pieces(violation.fit_context)
        })
        .cloned()
        .collect();
    assert!(
        pinned.is_empty(),
        "{} accepted fit pieces leave the tolerance across a break in their edge sliver:\n{}",
        pinned.len(),
        table(&pinned)
    );
    let violations: Vec<(String, FitViolation)> = audited
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

#[test]
fn every_snapshot_case_holds_the_fit_tolerance_against_the_exact_signal() {
    assert_every_accepted_piece_holds_the_fit_tolerance(corpus::snapshot_cases());
}

#[test]
fn the_dense_print_holds_the_fit_tolerance_on_the_fast_bench_config() {
    assert_every_accepted_piece_holds_the_fit_tolerance(dense_case(corpus::SIM_CONFIGS[0]));
}

#[test]
fn the_dense_print_holds_the_fit_tolerance_on_the_printer_config() {
    assert_every_accepted_piece_holds_the_fit_tolerance(dense_case(corpus::SIM_CONFIGS[1]));
}

#[test]
fn the_dense_print_holds_the_fit_tolerance_on_the_trident_configs() {
    for config in TRIDENT_CONFIGS {
        assert_every_accepted_piece_holds_the_fit_tolerance(dense_case(config));
    }
}
