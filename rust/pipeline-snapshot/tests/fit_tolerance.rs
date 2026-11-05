//! `max_path_deviation` and `max_accel_deviation` are promises: every piece
//! a fit accepts must stay within them against the exact signal it replaces,
//! everywhere inside the piece — not just at the ladder's probe points. A
//! follower's acceleration promise is per piece: the looser of
//! `max_accel_deviation` and the acceleration that spends its position
//! tolerance over the piece. This replays every snapshot case and whole
//! real prints through the pipeline with the fit audit on, and fails
//! on any accepted piece that leaves the tolerance, and on any planned
//! extruder that leaves the limits it was planned under beyond what
//! `corpus/known_extruder_excesses.rs` pins. Each set of cases is its own
//! test, so the audited replays run side by side.

use std::collections::BTreeMap;
use std::path::PathBuf;

use motion_pipeline::fit_audit::{FitViolation, take_violations};
mod corpus;

const DENSE_PRINT: &str = "voron_dense_top_layers";

fn dense_cases(configs: &[&str]) -> Vec<(String, String, PathBuf)> {
    print_cases(DENSE_PRINT, configs)
}

fn print_cases(print: &str, configs: &[&str]) -> Vec<(String, String, PathBuf)> {
    let gcode =
        std::fs::read_to_string(corpus::repo().join(format!("tools/sim/gcode/{print}.gcode")))
            .unwrap_or_else(|e| panic!("read the sim print {print}: {e}"));
    configs
        .iter()
        .map(|config| {
            (
                format!("{print}/{config}"),
                gcode.clone(),
                corpus::repo().join(config),
            )
        })
        .collect()
}

/// What the audited replays of a set of cases leave behind: the fit
/// pieces that left the tolerance, and per case the extruder's excesses
/// over the limits it was planned under.
#[derive(Default)]
struct Audited {
    fit: Vec<(String, FitViolation)>,
    extruder: Vec<String>,
}

fn audited(cases: Vec<(String, String, PathBuf)>) -> Audited {
    let mut audited = Audited::default();
    for (name, gcode, config) in cases {
        take_violations();
        let snapshot = corpus::plan(&name, &gcode, &config);
        audited.fit.extend(
            take_violations()
                .into_iter()
                .map(|violation| (name.clone(), violation)),
        );
        let Some(audit) = snapshot
            .as_ref()
            .and_then(|snapshot| corpus::extruder::extruder_audit(snapshot, &gcode, &config))
        else {
            continue;
        };
        println!("{name}: {}", audit.summary());
        if let Some(excess) = corpus::known_extruder_excesses::extruder_excess(&name, &audit) {
            audited.extruder.push(format!("{name}: {excess}"));
        }
    }
    audited
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

/// Every accepted piece of `cases` holds the fit tolerance, bar the
/// exempt, and the extruder holds the limits it was planned under. Where
/// a span's edge acceleration changes by more than the tolerance across its
/// edge sliver, the ladder reads it past the sliver; read at the seam
/// instead, a break that seed coalescing hid there pins the piece to the
/// acceleration on the break's far side, which the count of pinned follower
/// pieces names. The shaper's unit tests pin the read itself; this holds the
/// whole pipeline to it.
fn assert_every_accepted_fit_piece_holds(cases: Vec<(String, String, PathBuf)>) {
    let audited = audited(cases);
    let violations: Vec<(String, FitViolation)> = audited
        .fit
        .into_iter()
        .filter(|(_, violation)| violation.exemption.is_none())
        .collect();
    let pinned = violations
        .iter()
        .filter(|(_, violation)| {
            violation.break_in_edge_sliver && assembled_from_pieces(violation.fit_context)
        })
        .count();
    assert!(
        violations.is_empty() && audited.extruder.is_empty(),
        "{} accepted fit pieces leave the fit tolerance, {pinned} of them follower pieces \
         across a break in their edge sliver:\n{}\n{} cases leave their extruder limits:\n{}",
        violations.len(),
        table(&violations),
        audited.extruder.len(),
        audited.extruder.join("\n")
    );
}

/// The snapshot cases; `scripts/review_baselines.py` reads this test's
/// table by name.
#[test]
fn every_accepted_fit_piece_holds_the_fit_tolerance_against_the_exact_signal() {
    assert_every_accepted_fit_piece_holds(corpus::snapshot_cases());
}

#[test]
fn every_fit_piece_of_the_dense_print_holds_the_fit_tolerance_on_the_bench_configs() {
    assert_every_accepted_fit_piece_holds(dense_cases(&corpus::SIM_CONFIGS));
}

/// The first layers' slow perimeters, primes and retracts on the bench
/// printer's kernel-free extruder chain.
///
/// TODO(first-layers-floor-sliver): on `fast.cfg` the x/y source fit accepts
/// two sub-microsecond floor pieces (82.389 s on y, 128.561 s on x) whose
/// acceleration misses the tolerance 338× and 263×; add `fast.cfg` here once
/// they hold.
#[test]
fn every_fit_piece_of_the_first_layers_holds_the_fit_tolerance_on_the_bench_printer() {
    assert_every_accepted_fit_piece_holds(print_cases(
        "voron_first_layers",
        &["tools/bench/printer.cfg"],
    ));
}

/// The Trident print that first broke the follower cache, on the bench
/// configs' kernel-free extruder chain.
#[test]
fn every_fit_piece_of_the_follower_cache_print_holds_the_fit_tolerance_on_the_bench_configs() {
    assert_every_accepted_fit_piece_holds(print_cases(
        "trident_follower_cache_regression",
        &corpus::SIM_CONFIGS,
    ));
}

/// A 70 000 mm/s² Trident whose kernel-free pressure advance under a ratio
/// ramp fits the follower's advanced source to the position-equivalent
/// budget.
#[test]
fn every_fit_piece_of_the_dense_print_holds_the_fit_tolerance_on_the_trident_ramp_chain() {
    assert_every_accepted_fit_piece_holds(dense_cases(&["tools/bench/configs/trident_ramp.cfg"]));
}

/// The same Trident with a smoothing kernel after the advance, whose plain
/// follower source fits to the absolute budget.
#[test]
fn every_fit_piece_of_the_dense_print_holds_the_fit_tolerance_on_the_trident_kernel_chain() {
    assert_every_accepted_fit_piece_holds(dense_cases(&["tools/bench/configs/trident_kernel.cfg"]));
}

/// A 3 000 mm/s² bowden bedslinger planning its corners by deviation, whose
/// source fit lays degree-13 pieces a tenth of a second long next to its
/// cap contacts.
#[test]
fn every_fit_piece_of_the_dense_print_holds_the_fit_tolerance_on_the_bowden_corner_deviation_profile()
 {
    assert_every_accepted_fit_piece_holds(dense_cases(&[
        "tools/bench/configs/p1_bowden_bedslinger_kernel_cd0.04.cfg",
    ]));
}
