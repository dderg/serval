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

#[test]
fn every_accepted_fit_piece_holds_the_fit_tolerance_against_the_exact_signal() {
    let mut report = Vec::new();
    let mut total = 0usize;
    for (name, gcode, config) in snapshot_cases().into_iter().chain(dense_cases()) {
        let violations = run(&name, &gcode, &config);
        if violations.is_empty() {
            continue;
        }
        total += violations.len();
        let mut by_context: BTreeMap<(&str, usize), (usize, f64, f64)> = BTreeMap::new();
        for violation in &violations {
            let entry = by_context
                .entry((violation.fit_context, violation.axis))
                .or_insert((0, 0.0, 0.0));
            entry.0 += 1;
            entry.1 = entry.1.max(violation.acceleration_over_tolerance);
            entry.2 = entry.2.max(violation.position_over_tolerance);
        }
        for ((context, axis), (count, acceleration, position)) in by_context {
            report.push(format!(
                "{name}: {context} axis {axis}: {count} pieces, worst accel {acceleration:.2}x, \
                 worst position {position:.2}x"
            ));
        }
    }
    assert!(
        total == 0,
        "{total} accepted fit pieces leave the fit tolerance:\n{}",
        report.join("\n")
    );
}
