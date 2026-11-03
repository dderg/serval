//! The planner corpus: every snapshot case plus the four real sim prints on
//! both bench configs (51 + 4 x 2 = 59 cases). Shared by the corpus test, the
//! fit audit and ad-hoc probes (`scripts/corpus.sh --probe FILE.rs`).
//!
//! `CORPUS_FILTER=substr` keeps the cases whose name contains it;
//! `CORPUS_CONFIGS=fast,printer` keeps the cases run on those config stems.

#![allow(dead_code)]

use std::path::{Path, PathBuf};

use pipeline_snapshot::waypoints::parse_gcode;
use pipeline_snapshot::{SampleSide, SnapshotParams, pipeline_snapshot};
use planner_config::from_doc::read_motion_settings;

pub const SIM_GCODES: [&str; 4] = [
    "voron_first_layers",
    "voron_dense_top_layers",
    "trident_follower_cache_regression",
    "trident_mesh_clock_regression",
];
pub const SIM_CONFIGS: [&str; 2] = ["tools/bench/configs/fast.cfg", "tools/bench/printer.cfg"];

pub struct Case {
    pub name: String,
    pub gcode: String,
    pub config: PathBuf,
}

pub fn repo() -> PathBuf {
    source_tree::repo_root()
}

fn motion_settings(config: &Path) -> planner_config::from_doc::MotionSettings {
    let text = std::fs::read_to_string(config).expect("read config");
    let doc =
        config_doc::Document::parse(&text, &config.display().to_string()).expect("config parses");
    read_motion_settings(&doc).expect("motion settings").0
}

const Z_AXIS: usize = 2;
const Z_PROBES_PER_ROW: usize = 32;

/// Where the plan's z leaves the config's `max_z_velocity` / `max_z_accel`
/// (the latter within `max_accel_deviation`), probed over every z carrier.
pub fn z_caps_excess(snapshot: &pipeline_snapshot::Snapshot, config: &Path) -> Option<String> {
    let settings = motion_settings(config);
    let velocity_bound = settings.cartesian.max_z_velocity * (1.0 + 1e-6);
    let accel_bound = settings.cartesian.max_z_accel + settings.fit_tolerance_accel_mm_s2;
    let trajectory = &snapshot.trajectory;
    let mut peak = (0.0_f64, 0.0_f64, 0.0_f64);
    for row in trajectory.rows(Z_AXIS) {
        for k in 0..=Z_PROBES_PER_ROW {
            let t = row.t0 + (row.t1 - row.t0) * k as f64 / Z_PROBES_PER_ROW as f64;
            let side = if k == Z_PROBES_PER_ROW {
                SampleSide::Left
            } else {
                SampleSide::Right
            };
            let state = trajectory
                .eval_axis(Z_AXIS, t, side)
                .unwrap_or_else(|e| panic!("z at t={t} is not evaluable: {e}"));
            peak.0 = peak.0.max(state.velocity.abs());
            if state.acceleration.abs() > peak.1 {
                peak.1 = state.acceleration.abs();
                peak.2 = t;
            }
        }
    }
    (peak.0 > velocity_bound || peak.1 > accel_bound).then(|| {
        format!(
            "z reaches {:.3} mm/s and {:.1} mm/s² (at t={:.4}) against max_z_velocity {} and \
             max_z_accel {} + max_accel_deviation {}",
            peak.0,
            peak.1,
            peak.2,
            settings.cartesian.max_z_velocity,
            settings.cartesian.max_z_accel,
            settings.fit_tolerance_accel_mm_s2
        )
    })
}

pub fn plan(name: &str, gcode: &str, config: &Path) -> Option<pipeline_snapshot::Snapshot> {
    let settings = motion_settings(config);
    let waypoints = parse_gcode(gcode, &settings.cartesian).expect("gcode parses");
    if waypoints.len() < 2 {
        return None;
    }
    let snapshot = pipeline_snapshot(
        &waypoints,
        SnapshotParams {
            max_velocity: settings.cartesian.max_velocity,
            max_accel: settings.cartesian.max_accel,
            corner_deviation: settings.cartesian.corner_deviation,
            max_jerk: settings.cartesian.max_jerk,
            max_z_velocity: SnapshotParams::binding_z_cap(
                settings.cartesian.max_z_velocity,
                settings.cartesian.max_velocity,
            ),
            max_z_accel: SnapshotParams::binding_z_cap(
                settings.cartesian.max_z_accel,
                settings.cartesian.max_accel,
            ),
            max_extrude_only_velocity: settings.max_extrude_only_velocity,
            max_extrude_only_accel: settings.max_extrude_only_accel,
            max_path_deviation: Some(settings.fit_tolerance_mm),
            max_accel_deviation: Some(settings.fit_tolerance_accel_mm_s2),
            axis_decls: settings.axes,
            post_processor_decls: settings.post_processors,
        },
    )
    .unwrap_or_else(|e| panic!("{name}: {e}"));
    Some(snapshot)
}

pub fn snapshot_cases() -> Vec<(String, String, PathBuf)> {
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

pub fn sim_cases(max_lines: Option<usize>) -> Vec<(String, String, PathBuf)> {
    let mut cases = Vec::new();
    for gcode in SIM_GCODES {
        let print = std::fs::read_to_string(repo().join(format!("tools/sim/gcode/{gcode}.gcode")))
            .expect("read sim print");
        let text: String = match max_lines {
            Some(limit) => print
                .lines()
                .take(limit)
                .flat_map(|line| [line, "\n"])
                .collect(),
            None => print,
        };
        for config in SIM_CONFIGS {
            cases.push((
                format!("{gcode}/{config}"),
                text.clone(),
                repo().join(config),
            ));
        }
    }
    cases
}

pub fn corpus() -> Vec<Case> {
    let filter = std::env::var("CORPUS_FILTER").ok();
    let configs: Option<Vec<String>> = std::env::var("CORPUS_CONFIGS")
        .ok()
        .map(|list| list.split(',').map(str::to_string).collect());
    snapshot_cases()
        .into_iter()
        .chain(sim_cases(None))
        .filter(|(name, _, config)| {
            filter.as_deref().is_none_or(|f| name.contains(f))
                && configs.as_ref().is_none_or(|wanted| {
                    let stem = config.file_stem().expect("config stem").to_string_lossy();
                    wanted.iter().any(|w| *w == stem)
                })
        })
        .map(|(name, gcode, config)| Case {
            name,
            gcode,
            config,
        })
        .collect()
}
