//! The planner corpus: every snapshot case, the sim prints and
//! `tools/corner_flow.py`'s cases on the bench configs and on every
//! machine profile. Shared by the corpus test, the fit audit and ad-hoc
//! probes (`scripts/corpus.sh --probe FILE.rs`).
//!
//! `CORPUS_FILTER=substr` keeps the cases whose name contains it;
//! `CORPUS_CONFIGS=fast,printer` keeps the cases run on those config stems.

#![allow(dead_code)]

use std::ops::RangeInclusive;
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
pub const SIM_CONFIGS: [&str; 3] = [
    "tools/bench/configs/fast.cfg",
    "tools/bench/printer.cfg",
    "rust/motion-core/tests/configs/neptune_sim_print.cfg",
];

/// The machine profiles, each class of printer with a ratio-ramped and a
/// kernel-smoothed extruder chain.
pub const PROFILE_CONFIGS: [&str; 10] = [
    "tools/bench/configs/profiles/p1_bowden_bedslinger_ramp.cfg",
    "tools/bench/configs/profiles/p1_bowden_bedslinger_kernel.cfg",
    "tools/bench/configs/profiles/p2_i3_direct_drive_ramp.cfg",
    "tools/bench/configs/profiles/p2_i3_direct_drive_kernel.cfg",
    "tools/bench/configs/profiles/p3_voron_direct_drive_ramp.cfg",
    "tools/bench/configs/profiles/p3_voron_direct_drive_kernel.cfg",
    "tools/bench/configs/profiles/p4_high_accel_ramp.cfg",
    "tools/bench/configs/profiles/p4_high_accel_kernel.cfg",
    "tools/bench/configs/trident_ramp.cfg",
    "tools/bench/configs/trident_kernel.cfg",
];

pub const CORNER_FLOW_CASES: &str = "tools/sim/gcode/corner_flow_cases.gcode";
const CORNER_FLOW_CASE_MARKER: &str = "; corner_flow case: ";

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

pub fn sim_print(print: &str) -> String {
    std::fs::read_to_string(repo().join(format!("tools/sim/gcode/{print}.gcode")))
        .unwrap_or_else(|e| panic!("read the sim print {print}: {e}"))
}

/// `print` on each of `configs`, each case named `<print>/<config>`.
pub fn print_cases(print: &str, configs: &[&str]) -> Vec<(String, String, PathBuf)> {
    let gcode = sim_print(print);
    configs
        .iter()
        .map(|config| (format!("{print}/{config}"), gcode.clone(), repo().join(config)))
        .collect()
}

/// Source lines `lines` (1-based, as the planner counts them) of `print`
/// on each of `configs`, each case named `<print>:<first>-<last>/<config>`
/// and planned by [`print_slice`].
pub fn print_slice_cases(
    print: &str,
    lines: RangeInclusive<usize>,
    configs: &[&str],
) -> Vec<(String, String, PathBuf)> {
    let gcode = print_slice(&sim_print(print), lines.clone());
    configs
        .iter()
        .map(|config| {
            (
                format!("{print}:{}-{}/{config}", lines.start(), lines.end()),
                gcode.clone(),
                repo().join(config),
            )
        })
        .collect()
}

/// Source lines `lines` (1-based) of `print`, started from rest where the
/// print stands before them: the G-code state the print set up and its
/// velocity limits replayed, the toolhead placed at the print's position
/// and its feedrate kept. Only the state the sim prints use is tracked;
/// any other command before the slice is refused.
pub fn print_slice(print: &str, lines: RangeInclusive<usize>) -> String {
    let all: Vec<&str> = print.lines().collect();
    assert!(
        *lines.start() >= 1 && lines.start() <= lines.end() && *lines.end() <= all.len(),
        "lines {lines:?} of a {}-line print",
        all.len()
    );
    let mut position: [Option<f64>; 3] = [None; 3];
    let mut feedrate: Option<f64> = None;
    let mut prelude = vec!["G21".to_string(), "G90".to_string(), "M83".to_string()];
    for (index, raw) in all[..lines.start() - 1].iter().enumerate() {
        let line = raw.split(';').next().expect("split yields a head").trim();
        let mut words = line.split_whitespace();
        let Some(command) = words.next() else {
            continue;
        };
        match command {
            "G0" | "G1" => {
                for word in words {
                    let (letter, value) = word.split_at(1);
                    let parse = || {
                        value.parse::<f64>().unwrap_or_else(|e| {
                            panic!("line {}: {word}: {e}", index + 1)
                        })
                    };
                    match letter {
                        "X" => position[0] = Some(parse()),
                        "Y" => position[1] = Some(parse()),
                        "Z" => position[2] = Some(parse()),
                        "F" => feedrate = Some(parse()),
                        "E" => {}
                        _ => panic!("line {}: {line}: no slice tracks {word}", index + 1),
                    }
                }
            }
            "G92" => assert!(
                words.all(|word| word.starts_with('E')),
                "line {}: {line}: a slice tracks no G92 but of E",
                index + 1
            ),
            "SET_VELOCITY_LIMIT" => prelude.push(line.to_string()),
            "G21" | "G90" | "M83" | "M73" | "M106" | "M107" | "M400" => {}
            _ => panic!("line {}: {line}: no slice tracks {command}", index + 1),
        }
    }
    let [Some(x), Some(y), Some(z)] = position else {
        panic!("the print states no position before line {}", lines.start());
    };
    prelude.push(format!("SET_KINEMATIC_POSITION X={x} Y={y} Z={z}"));
    if let Some(feedrate) = feedrate {
        prelude.push(format!("G1 F{feedrate}"));
    }
    prelude
        .iter()
        .map(String::as_str)
        .chain(all[lines.start() - 1..*lines.end()].iter().copied())
        .flat_map(|line| [line, "\n"])
        .collect()
}

/// `tools/corner_flow.py`'s synthetic corners, zigzag, flow change, travel
/// steps and retract at each of its speeds, each planned on its own from
/// rest as it measures them, on each of `configs`.
pub fn corner_flow_cases(configs: &[&str]) -> Vec<(String, String, PathBuf)> {
    let all = std::fs::read_to_string(repo().join(CORNER_FLOW_CASES))
        .unwrap_or_else(|e| panic!("read {CORNER_FLOW_CASES}: {e}"));
    let cases: Vec<(&str, &str)> = all
        .split(CORNER_FLOW_CASE_MARKER)
        .filter(|case| !case.is_empty())
        .map(|case| {
            case.split_once('\n')
                .unwrap_or_else(|| panic!("a corner_flow case without G-code: {case:?}"))
        })
        .collect();
    configs
        .iter()
        .flat_map(|config| {
            cases.iter().map(move |(name, gcode)| {
                (
                    format!("corner_flow {name}/{config}"),
                    (*gcode).to_string(),
                    repo().join(config),
                )
            })
        })
        .collect()
}

pub fn sim_cases() -> Vec<(String, String, PathBuf)> {
    SIM_GCODES
        .iter()
        .flat_map(|print| print_cases(print, &SIM_CONFIGS))
        .collect()
}

/// Every sim print on every machine profile.
pub fn profile_cases() -> Vec<(String, String, PathBuf)> {
    SIM_GCODES
        .iter()
        .flat_map(|print| print_cases(print, &PROFILE_CONFIGS))
        .collect()
}

pub fn corpus() -> Vec<Case> {
    let filter = std::env::var("CORPUS_FILTER").ok();
    let configs: Option<Vec<String>> = std::env::var("CORPUS_CONFIGS")
        .ok()
        .map(|list| list.split(',').map(str::to_string).collect());
    snapshot_cases()
        .into_iter()
        .chain(sim_cases())
        .chain(profile_cases())
        .chain(corner_flow_cases(&SIM_CONFIGS))
        .chain(corner_flow_cases(&PROFILE_CONFIGS))
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
