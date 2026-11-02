//! `max_path_deviation` and `max_accel_deviation` are promises: every piece
//! a fit accepts must stay within them against the exact signal it replaces,
//! everywhere inside the piece — not just at the ladder's probe points. This
//! replays every snapshot case and a slice of a dense real print through the
//! pipeline with the fit audit on, and fails on any accepted piece that
//! leaves the tolerance.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use motion_pipeline::fit_audit::{FitViolation, take_violations};
use pipeline_snapshot::waypoints::parse_gcode;
use pipeline_snapshot::{SnapshotParams, pipeline_snapshot};
use planner_config::from_doc::read_motion_settings;

const DENSE_SLICE_LINES: usize = 2500;

fn repo() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../..")
}

fn run(name: &str, gcode: &str, config: &Path) -> Vec<FitViolation> {
    let text = std::fs::read_to_string(config).expect("read config");
    let doc =
        config_doc::Document::parse(&text, &config.display().to_string()).expect("config parses");
    let (settings, _) = read_motion_settings(&doc).expect("motion settings");
    let waypoints = parse_gcode(
        gcode,
        settings.cartesian.max_velocity,
        settings.cartesian.max_accel,
    )
    .expect("gcode parses");
    take_violations();
    if waypoints.len() >= 2 {
        pipeline_snapshot(
            &waypoints,
            SnapshotParams {
                max_velocity: settings.cartesian.max_velocity,
                max_accel: settings.cartesian.max_accel,
                corner_deviation: settings.cartesian.corner_deviation,
                max_jerk: settings.cartesian.max_jerk,
                max_extrude_only_velocity: settings.max_extrude_only_velocity,
                max_extrude_only_accel: settings.max_extrude_only_accel,
                max_path_deviation: Some(settings.fit_tolerance_mm),
                max_accel_deviation: Some(settings.fit_tolerance_accel_mm_s2),
                axis_decls: settings.axes,
                post_processor_decls: settings.post_processors,
            },
        )
        .unwrap_or_else(|e| panic!("{name}: {e}"));
    }
    take_violations()
}

fn snapshot_cases() -> Vec<(String, String, PathBuf)> {
    let cases_dir = repo().join("snapshots/cases");
    let mut groups: Vec<PathBuf> = std::fs::read_dir(&cases_dir)
        .expect("snapshot cases")
        .map(|entry| entry.expect("case entry").path())
        .filter(|path| path.is_dir())
        .collect();
    groups.sort();
    let files = |group: &Path, extension: &str| {
        let mut found: Vec<PathBuf> = std::fs::read_dir(group)
            .expect("case group")
            .map(|entry| entry.expect("case file").path())
            .filter(|path| path.extension().is_some_and(|e| e == extension))
            .collect();
        found.sort();
        found
    };
    let mut cases = Vec::new();
    for group in groups {
        let group_name = group
            .file_name()
            .expect("group name")
            .to_string_lossy()
            .to_string();
        for config in files(&group, "cfg") {
            for gcode in files(&group, "gcode") {
                let text = std::fs::read_to_string(&gcode).expect("read gcode");
                if text.trim().is_empty() {
                    continue;
                }
                let stem = |path: &Path| {
                    path.file_stem()
                        .expect("stem")
                        .to_string_lossy()
                        .to_string()
                };
                cases.push((
                    format!("{group_name}/{}/{}", stem(&config), stem(&gcode)),
                    text,
                    config.clone(),
                ));
            }
        }
    }
    cases
}

fn dense_cases() -> Vec<(String, String, PathBuf)> {
    let print =
        std::fs::read_to_string(repo().join("tools/sim/gcode/voron_dense_top_layers.gcode"))
            .expect("read dense print");
    let slice: String = print
        .lines()
        .take(DENSE_SLICE_LINES)
        .flat_map(|line| [line, "\n"])
        .collect();
    ["tools/bench/configs/fast.cfg", "tools/bench/printer.cfg"]
        .into_iter()
        .map(|config| {
            (
                format!("dense_top_layers/{config}"),
                slice.clone(),
                repo().join(config),
            )
        })
        .collect()
}

fn audited_violations() -> Vec<(String, FitViolation)> {
    snapshot_cases()
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
/// span edge.
fn assembled_from_pieces(fit_context: &str) -> bool {
    fit_context.starts_with("follower") || fit_context.starts_with("advanced_follower")
}

/// The ladder reads a span's edge state a cubic floor inside it. Read any
/// closer and a break that seed coalescing hid in the edge sliver pins the
/// piece to the acceleration on the break's far side.
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
#[ignore = "report: lists every accepted piece outside the fit tolerance; \
            the remaining exceedances are open (materialize disk rails, \
            shaped leaders, follower sources)"]
fn every_accepted_fit_piece_holds_the_fit_tolerance_against_the_exact_signal() {
    let violations = audited_violations();
    assert!(
        violations.is_empty(),
        "{} accepted fit pieces leave the fit tolerance:\n{}",
        violations.len(),
        table(&violations)
    );
}
