//! The shaped path's accuracy against the G-code polyline
//! (`corpus::path_accuracy`) on the bench and machine-profile configs:
//! the corner_flow cases and the dense and first-layer prints. Heavy, so
//! ignored by default: `scripts/corpus.sh --path-accuracy`.
//! `CORPUS_FILTER=substr` keeps the cases whose name contains it. The
//! `_cd005` tests hold profiles to an explicit `corner_deviation` of
//! 0.05 mm. A case fails where its vertices past their budget depart from
//! the pins in `corpus::known_corner_overs`.

use std::panic::{AssertUnwindSafe, catch_unwind};
use std::path::{Path, PathBuf};

mod corpus;

use corpus::known_corner_overs::corner_overs;

const PRINTS: [&str; 2] = ["voron_dense_top_layers", "voron_first_layers"];

/// `config`, or under `corner_deviation` a copy of it in the temp dir with
/// that budget in place of its own.
fn config_path(test: &str, config: &str, corner_deviation: Option<f64>) -> PathBuf {
    let source = corpus::repo().join(config);
    let Some(deviation) = corner_deviation else {
        return source;
    };
    let text = std::fs::read_to_string(&source).expect("read config");
    let budgeted: String = text
        .lines()
        .filter(|line| {
            !line.starts_with("square_corner_velocity:") && !line.starts_with("corner_deviation:")
        })
        .flat_map(|line| {
            let budget = (line == "[printer]").then(|| format!("corner_deviation: {deviation}"));
            std::iter::once(line.to_string()).chain(budget)
        })
        .flat_map(|line| [line, "\n".to_string()])
        .collect();
    assert!(
        budgeted.contains(&format!("corner_deviation: {deviation}")),
        "{config} has no [printer] section to budget"
    );
    let path = std::env::temp_dir().join(format!("path_accuracy_{test}.cfg"));
    std::fs::write(&path, budgeted).expect("write budgeted config");
    path
}

fn cases(test: &str, config: &Path) -> Vec<(String, String, PathBuf)> {
    let config_name = config.to_str().expect("a UTF-8 config path");
    let corner_flow =
        corpus::corner_flow_cases(&[config_name])
            .into_iter()
            .map(|(name, gcode, path)| {
                let case = name
                    .strip_suffix(&format!("/{config_name}"))
                    .expect("a corner_flow case names its config");
                (format!("{case}/{test}"), gcode, path)
            });
    let prints = PRINTS.iter().map(|print| {
        let gcode =
            std::fs::read_to_string(corpus::repo().join(format!("tools/sim/gcode/{print}.gcode")))
                .expect("read sim print");
        (format!("{print}/{test}"), gcode, config.to_path_buf())
    });
    let filter = std::env::var("CORPUS_FILTER").ok();
    corner_flow
        .chain(prints)
        .filter(|(name, _, _)| filter.as_deref().is_none_or(|f| name.contains(f)))
        .collect()
}

fn measure(test: &str, config: &str, corner_deviation: Option<f64>) {
    let config = config_path(test, config, corner_deviation);
    let mut failures = Vec::new();
    for (name, gcode, config) in &cases(test, &config) {
        let measured = catch_unwind(AssertUnwindSafe(|| {
            let snapshot = corpus::plan(name, gcode, config)
                .unwrap_or_else(|| panic!("fewer than two waypoints"));
            let report = corpus::path_accuracy::audit(&snapshot, gcode, config)
                .unwrap_or_else(|| panic!("no segment to measure"));
            (snapshot.traversal_time_s, report)
        }));
        match measured {
            Ok((time_s, report)) => {
                println!("{name} {time_s:.3} s: {}", report.summary());
                if !report.lost.is_empty() {
                    failures.push(format!(
                        "{name}: the path never nears {} vertices, first {:?}",
                        report.lost.len(),
                        report.lost[0]
                    ));
                }
                if let Some(departure) = corner_overs(name, &report) {
                    failures.push(format!("{name}: {departure}"));
                }
            }
            Err(payload) => failures.push(format!(
                "{name}: {}",
                payload
                    .downcast_ref::<String>()
                    .cloned()
                    .or_else(|| payload.downcast_ref::<&str>().map(|s| s.to_string()))
                    .unwrap_or_else(|| "non-string panic".to_string())
            )),
        }
    }
    assert!(
        failures.is_empty(),
        "path accuracy failed:\n{}",
        failures.join("\n")
    );
}

macro_rules! path_accuracy_of {
    ($($test:ident: $config:literal $(, corner_deviation $deviation:literal)?;)*) => {$(
        #[test]
        #[ignore = "full sim prints; run with scripts/corpus.sh --path-accuracy"]
        fn $test() {
            measure(stringify!($test), $config, None $(.or(Some($deviation)))?);
        }
    )*};
}

path_accuracy_of! {
    fast: "tools/bench/configs/fast.cfg";
    printer: "tools/bench/printer.cfg";
    neptune: "rust/motion-core/tests/configs/neptune_sim_print.cfg";
    p1_ramp: "tools/bench/configs/profiles/p1_bowden_bedslinger_ramp.cfg";
    p1_kernel: "tools/bench/configs/profiles/p1_bowden_bedslinger_kernel.cfg";
    p2_ramp: "tools/bench/configs/profiles/p2_i3_direct_drive_ramp.cfg";
    p2_kernel: "tools/bench/configs/profiles/p2_i3_direct_drive_kernel.cfg";
    p3_ramp: "tools/bench/configs/profiles/p3_voron_direct_drive_ramp.cfg";
    p3_kernel: "tools/bench/configs/profiles/p3_voron_direct_drive_kernel.cfg";
    p4_ramp: "tools/bench/configs/profiles/p4_high_accel_ramp.cfg";
    p4_kernel: "tools/bench/configs/profiles/p4_high_accel_kernel.cfg";
    trident_ramp: "tools/bench/configs/trident_ramp.cfg";
    trident_kernel: "tools/bench/configs/trident_kernel.cfg";
    p2_ramp_cd005: "tools/bench/configs/profiles/p2_i3_direct_drive_ramp.cfg", corner_deviation 0.05;
    p2_kernel_cd005: "tools/bench/configs/profiles/p2_i3_direct_drive_kernel.cfg", corner_deviation 0.05;
    p3_ramp_cd005: "tools/bench/configs/profiles/p3_voron_direct_drive_ramp.cfg", corner_deviation 0.05;
    p3_kernel_cd005: "tools/bench/configs/profiles/p3_voron_direct_drive_kernel.cfg", corner_deviation 0.05;
}
