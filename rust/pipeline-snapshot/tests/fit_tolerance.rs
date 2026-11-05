//! `max_path_deviation` and `max_accel_deviation` are promises: every piece
//! a fit accepts must stay within them against the exact signal it replaces,
//! everywhere inside the piece — not just at the ladder's probe points. A
//! follower's acceleration promise is per piece: the looser of
//! `max_accel_deviation` and the acceleration that spends its position
//! tolerance over the piece. This replays every snapshot case, real prints
//! on the bench configs and stretches of them on every machine profile
//! through the pipeline with the fit audit on, and fails on any accepted
//! piece that leaves the tolerance beyond what
//! `corpus/known_fit_violations.rs` pins, on any planned extruder that
//! leaves the limits it was planned under beyond what
//! `corpus/known_extruder_excesses.rs` pins, and on any retract or prime
//! planned outside the toolhead's stop (`corpus/retract_placement.rs`).
//! Each set of cases is its own test, so the audited replays run side by
//! side.

use std::collections::{BTreeMap, BTreeSet};
use std::path::PathBuf;

use motion_pipeline::fit_audit::{FitViolation, take_violations};
mod corpus;

const DENSE_PRINT: &str = "voron_dense_top_layers";

fn dense_cases(configs: &[&str]) -> Vec<(String, String, PathBuf)> {
    corpus::print_cases(DENSE_PRINT, configs)
}

/// Held across an audited replay: the fit audit gathers violations from
/// every thread into one list, so two replays at once would take each
/// other's — or drop them — when the tests share a process.
static AUDITING: std::sync::Mutex<()> = std::sync::Mutex::new(());

/// Per case, where the open fit violations and the extruder's excesses
/// over the limits it was planned under depart from the pinned ones, and
/// every retract and prime the plan moves outside the toolhead's stop.
fn audited(cases: Vec<(String, String, PathBuf)>) -> Vec<String> {
    let _auditing = AUDITING
        .lock()
        .expect("an audited replay panicked while auditing");
    let mut failures = Vec::new();
    for (name, gcode, config) in cases {
        take_violations();
        let snapshot = corpus::plan(&name, &gcode, &config);
        let open: Vec<FitViolation> = take_violations()
            .into_iter()
            .filter(|violation| violation.exemption.is_none())
            .collect();
        if let Some(departure) = corpus::known_fit_violations::fit_violations(&name, &open) {
            failures.push(format!(
                "{name}: {departure}; of its {} open pieces, {} are follower pieces across a \
                 break in their edge sliver:\n{}",
                open.len(),
                assembled_across_edge_break(&open),
                table(&name, &open)
            ));
        }
        let Some(snapshot) = snapshot else {
            continue;
        };
        if let Some(placement) =
            corpus::retract_placement::retract_placement(&snapshot, &gcode, &config)
        {
            println!("{name}: {}", placement.summary());
            if let Some(misplaced) = placement.misplaced() {
                failures.push(format!("{name}: {misplaced}"));
            }
        }
        let Some(audit) = corpus::extruder::extruder_audit(&snapshot, &gcode, &config) else {
            continue;
        };
        println!("{name}: {}", audit.summary());
        if let Some(excess) = corpus::known_extruder_excesses::extruder_excess(&name, &audit) {
            failures.push(format!("{name}: {excess}"));
        }
    }
    failures
}

fn table(name: &str, violations: &[FitViolation]) -> String {
    let mut rows: BTreeMap<(&str, usize), (usize, f64, f64)> = BTreeMap::new();
    for violation in violations {
        let row = rows
            .entry((violation.fit_context, violation.axis))
            .or_insert((0, 0.0, 0.0));
        row.0 += 1;
        row.1 = row.1.max(violation.acceleration_over_tolerance);
        row.2 = row.2.max(violation.position_over_tolerance);
    }
    rows.into_iter()
        .map(|((context, axis), (count, acceleration, position))| {
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

fn assembled_across_edge_break(violations: &[FitViolation]) -> usize {
    violations
        .iter()
        .filter(|violation| {
            violation.break_in_edge_sliver && assembled_from_pieces(violation.fit_context)
        })
        .count()
}

/// Every accepted piece of `cases` holds the fit tolerance, bar the
/// exempt and the pinned, and the extruder holds the limits it was planned
/// under, bar the pinned. Where a span's edge acceleration changes by more
/// than the tolerance across its edge sliver, the ladder reads it past the
/// sliver; read at the seam instead, a break that seed coalescing hid there
/// pins the piece to the acceleration on the break's far side, which the
/// count of follower pieces across a break names. The shaper's unit tests
/// pin the read itself; this holds the whole pipeline to it.
fn assert_every_accepted_fit_piece_holds(cases: Vec<(String, String, PathBuf)>) {
    let failures = audited(cases);
    assert!(
        failures.is_empty(),
        "{} cases depart from their known fit violations and extruder excesses:\n{}",
        failures.len(),
        failures.join("\n")
    );
}

/// The snapshot cases; `scripts/review_baselines.py` reads this test's
/// table by name.
#[test]
fn every_accepted_fit_piece_holds_the_fit_tolerance_against_the_exact_signal() {
    assert_every_accepted_fit_piece_holds(corpus::snapshot_cases());
}

#[test]
fn every_fit_piece_of_the_dense_print_holds_the_fit_tolerance_on_the_fast_bench_config() {
    assert_every_accepted_fit_piece_holds(dense_cases(&[corpus::FAST_CONFIG]));
}

#[test]
fn every_fit_piece_of_the_dense_print_holds_the_fit_tolerance_on_the_printer_bench_config() {
    assert_every_accepted_fit_piece_holds(dense_cases(&[corpus::PRINTER_CONFIG]));
}

#[test]
fn every_fit_piece_of_the_dense_print_holds_the_fit_tolerance_on_the_neptune_config() {
    assert_every_accepted_fit_piece_holds(dense_cases(&[corpus::NEPTUNE_CONFIG]));
}

/// The first layers' slow perimeters, primes and retracts on `config`'s
/// kernel-free extruder chain.
fn first_layers_cases(config: &str) -> Vec<(String, String, PathBuf)> {
    corpus::print_cases("voron_first_layers", &[config])
}

#[test]
fn every_fit_piece_of_the_first_layers_holds_the_fit_tolerance_on_the_fast_bench_config() {
    assert_every_accepted_fit_piece_holds(first_layers_cases(corpus::FAST_CONFIG));
}

#[test]
fn every_fit_piece_of_the_first_layers_holds_the_fit_tolerance_on_the_printer_bench_config() {
    assert_every_accepted_fit_piece_holds(first_layers_cases(corpus::PRINTER_CONFIG));
}

#[test]
fn every_fit_piece_of_the_first_layers_holds_the_fit_tolerance_on_the_neptune_config() {
    assert_every_accepted_fit_piece_holds(first_layers_cases(corpus::NEPTUNE_CONFIG));
}

/// The Trident print that first broke the follower cache, on `config`'s
/// kernel-free extruder chain.
fn follower_cache_cases(config: &str) -> Vec<(String, String, PathBuf)> {
    corpus::print_cases("trident_follower_cache_regression", &[config])
}

#[test]
fn every_fit_piece_of_the_follower_cache_print_holds_the_fit_tolerance_on_the_fast_bench_config() {
    assert_every_accepted_fit_piece_holds(follower_cache_cases(corpus::FAST_CONFIG));
}

#[test]
fn every_fit_piece_of_the_follower_cache_print_holds_the_fit_tolerance_on_the_printer_bench_config()
{
    assert_every_accepted_fit_piece_holds(follower_cache_cases(corpus::PRINTER_CONFIG));
}

#[test]
fn every_fit_piece_of_the_follower_cache_print_holds_the_fit_tolerance_on_the_neptune_config() {
    assert_every_accepted_fit_piece_holds(follower_cache_cases(corpus::NEPTUNE_CONFIG));
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

#[test]
fn every_fit_piece_of_the_corner_flow_cases_holds_the_fit_tolerance_on_the_bench_configs() {
    assert_every_accepted_fit_piece_holds(corpus::corner_flow_cases(&corpus::SIM_CONFIGS));
}

/// [`corpus::FIRST_LAYERS_FROM_74_S`] on the Trident kernel chain.
#[test]
fn every_fit_piece_of_the_late_first_layers_holds_the_fit_tolerance_on_the_trident_kernel_chain() {
    assert_every_accepted_fit_piece_holds(corpus::print_slice_cases(
        &corpus::FIRST_LAYERS_FROM_74_S,
        &["tools/bench/configs/trident_kernel.cfg"],
    ));
}

/// The stretches of the sim prints in [`corpus::PROFILE_SLICES`] and the
/// corner_flow cases on the machine profile `config`.
fn profile_gate_cases(config: &str) -> Vec<(String, String, PathBuf)> {
    corpus::PROFILE_SLICES
        .iter()
        .flat_map(|slice| corpus::print_slice_cases(slice, &[config]))
        .chain(corpus::corner_flow_cases(&[config]))
        .collect()
}

#[test]
fn the_profile_slices_hold_the_fit_tolerance_on_the_p1_bowden_bedslinger_ramp_profile() {
    assert_every_accepted_fit_piece_holds(profile_gate_cases(
        "tools/bench/configs/profiles/p1_bowden_bedslinger_ramp.cfg",
    ));
}

#[test]
fn the_profile_slices_hold_the_fit_tolerance_on_the_p1_bowden_bedslinger_kernel_profile() {
    assert_every_accepted_fit_piece_holds(profile_gate_cases(
        "tools/bench/configs/profiles/p1_bowden_bedslinger_kernel.cfg",
    ));
}

#[test]
fn the_profile_slices_hold_the_fit_tolerance_on_the_p2_i3_direct_drive_ramp_profile() {
    assert_every_accepted_fit_piece_holds(profile_gate_cases(
        "tools/bench/configs/profiles/p2_i3_direct_drive_ramp.cfg",
    ));
}

#[test]
fn the_profile_slices_hold_the_fit_tolerance_on_the_p2_i3_direct_drive_kernel_profile() {
    assert_every_accepted_fit_piece_holds(profile_gate_cases(
        "tools/bench/configs/profiles/p2_i3_direct_drive_kernel.cfg",
    ));
}

#[test]
fn the_profile_slices_hold_the_fit_tolerance_on_the_p3_voron_direct_drive_ramp_profile() {
    assert_every_accepted_fit_piece_holds(profile_gate_cases(
        "tools/bench/configs/profiles/p3_voron_direct_drive_ramp.cfg",
    ));
}

#[test]
fn the_profile_slices_hold_the_fit_tolerance_on_the_p3_voron_direct_drive_kernel_profile() {
    assert_every_accepted_fit_piece_holds(profile_gate_cases(
        "tools/bench/configs/profiles/p3_voron_direct_drive_kernel.cfg",
    ));
}

#[test]
fn the_profile_slices_hold_the_fit_tolerance_on_the_p4_high_accel_ramp_profile() {
    assert_every_accepted_fit_piece_holds(profile_gate_cases(
        "tools/bench/configs/profiles/p4_high_accel_ramp.cfg",
    ));
}

#[test]
fn the_profile_slices_hold_the_fit_tolerance_on_the_p4_high_accel_kernel_profile() {
    assert_every_accepted_fit_piece_holds(profile_gate_cases(
        "tools/bench/configs/profiles/p4_high_accel_kernel.cfg",
    ));
}

#[test]
fn the_profile_slices_hold_the_fit_tolerance_on_the_trident_ramp_profile() {
    assert_every_accepted_fit_piece_holds(profile_gate_cases(
        "tools/bench/configs/trident_ramp.cfg",
    ));
}

#[test]
fn the_profile_slices_hold_the_fit_tolerance_on_the_trident_kernel_profile() {
    assert_every_accepted_fit_piece_holds(profile_gate_cases(
        "tools/bench/configs/trident_kernel.cfg",
    ));
}

/// Every case some audit runs: this test's sets and the corpus.
fn audited_case_names() -> BTreeSet<String> {
    let mut cases = corpus::all_cases();
    cases.extend(dense_cases(&[
        "tools/bench/configs/p1_bowden_bedslinger_kernel_cd0.04.cfg",
    ]));
    cases.extend(corpus::print_slice_cases(
        &corpus::FIRST_LAYERS_FROM_74_S,
        &["tools/bench/configs/trident_kernel.cfg"],
    ));
    for config in corpus::PROFILE_CONFIGS {
        cases.extend(profile_gate_cases(config));
    }
    cases.into_iter().map(|(name, ..)| name).collect()
}

/// A pin on a case no audit runs would never fail, nor ever be unpinned.
#[test]
fn every_pin_names_an_audited_case() {
    let audited = audited_case_names();
    let stale: Vec<&str> = corpus::known_fit_violations::KNOWN
        .iter()
        .map(|known| known.case)
        .chain(
            corpus::known_extruder_excesses::KNOWN
                .iter()
                .map(|known| known.case),
        )
        .filter(|case| !audited.contains(*case))
        .collect();
    assert!(stale.is_empty(), "pins on cases no audit runs: {stale:?}");
}
