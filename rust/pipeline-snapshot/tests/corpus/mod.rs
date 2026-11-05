//! The planner corpus: every snapshot case plus the four real sim prints on
//! both bench configs (51 + 4 x 2 = 59 cases). Shared by the corpus test, the
//! fit audit and ad-hoc probes (`scripts/corpus.sh --probe FILE.rs`).
//!
//! `CORPUS_FILTER=substr` keeps the cases whose name contains it;
//! `CORPUS_CONFIGS=fast,printer` keeps the cases run on those config stems.

#![allow(dead_code)]

use std::path::{Path, PathBuf};

use pipeline_snapshot::waypoints::parse_gcode;
use pipeline_snapshot::{SampleSide, pipeline_snapshot_for_config};
use planner_config::from_doc::{planner_config_from_text, read_motion_settings};

pub mod extruder;
pub mod known_extruder_excesses;

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

pub fn motion_settings(config: &Path) -> planner_config::from_doc::MotionSettings {
    let text = std::fs::read_to_string(config).expect("read config");
    let doc =
        config_doc::Document::parse(&text, &config.display().to_string()).expect("config parses");
    read_motion_settings(&doc).expect("motion settings").0
}

/// `bench_config` with its extruder kernel removed and the extruder's
/// acceleration limit set to `e_max_accel`; a kernel-free advance needs
/// finite extrude-only caps.
pub fn kernel_free(bench_config: &str, e_max_accel: f64) -> String {
    let text = std::fs::read_to_string(repo().join(bench_config))
        .unwrap_or_else(|e| panic!("read {bench_config}: {e}"));
    let replace = |text: &str, from: &str, to: &str| {
        assert!(text.contains(from), "{bench_config} has no `{from}`");
        text.replacen(from, to, 1)
    };
    let text = replace(
        &text,
        "post_processors: pa, st\n",
        &format!("post_processors: pa\nmax_accel: {e_max_accel}\n"),
    );
    replace(
        &text,
        "pid_kd: 124.081\n",
        "pid_kd: 124.081\nmax_extrude_only_velocity: 50\nmax_extrude_only_accel: 5000\n",
    )
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
    let waypoints = waypoints(gcode, config);
    if waypoints.len() < 2 {
        return None;
    }
    Some(plan_waypoints(name, &waypoints, config))
}

fn planner(name: &str, config: &Path) -> planner_config::PlannerConfig {
    let text = std::fs::read_to_string(config).expect("read config");
    planner_config_from_text(&text).unwrap_or_else(|e| panic!("{name}: {e}"))
}

pub fn waypoints(gcode: &str, config: &Path) -> Vec<pipeline_snapshot::waypoints::Waypoint> {
    parse_gcode(gcode, &planner("waypoints", config).cartesian).expect("gcode parses")
}

pub fn plan_waypoints(
    name: &str,
    waypoints: &[pipeline_snapshot::waypoints::Waypoint],
    config: &Path,
) -> pipeline_snapshot::Snapshot {
    pipeline_snapshot_for_config(waypoints, &planner(name, config), None)
        .unwrap_or_else(|e| panic!("{name}: {e}"))
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
