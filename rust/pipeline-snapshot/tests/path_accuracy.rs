//! The shaped path's accuracy against the G-code polyline
//! (`corpus::path_accuracy`) on the bench and machine-profile configs:
//! the corner_flow cases and the dense and first-layer prints. Heavy, so
//! ignored by default: `scripts/corpus.sh --path-accuracy`.
//! `CORPUS_FILTER=substr` keeps the cases whose name contains it.

use std::panic::{AssertUnwindSafe, catch_unwind};
use std::path::PathBuf;

mod corpus;

const PRINTS: [&str; 2] = ["voron_dense_top_layers", "voron_first_layers"];

fn cases(config: &str) -> Vec<(String, String, PathBuf)> {
    let prints = PRINTS.iter().map(|print| {
        let gcode =
            std::fs::read_to_string(corpus::repo().join(format!("tools/sim/gcode/{print}.gcode")))
                .expect("read sim print");
        (
            format!("{print}/{config}"),
            gcode,
            corpus::repo().join(config),
        )
    });
    let filter = std::env::var("CORPUS_FILTER").ok();
    corpus::corner_flow_cases(&[config])
        .into_iter()
        .chain(prints)
        .filter(|(name, _, _)| filter.as_deref().is_none_or(|f| name.contains(f)))
        .collect()
}

fn measure(config: &str) {
    let mut failures = Vec::new();
    for (name, gcode, config) in &cases(config) {
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
    ($($test:ident: $config:literal,)*) => {$(
        #[test]
        #[ignore = "full sim prints; run with scripts/corpus.sh --path-accuracy"]
        fn $test() {
            measure($config);
        }
    )*};
}

path_accuracy_of! {
    fast: "tools/bench/configs/fast.cfg",
    printer: "tools/bench/printer.cfg",
    p1_ramp: "tools/bench/configs/profiles/p1_bowden_bedslinger_ramp.cfg",
    p1_kernel: "tools/bench/configs/profiles/p1_bowden_bedslinger_kernel.cfg",
    p2_ramp: "tools/bench/configs/profiles/p2_i3_direct_drive_ramp.cfg",
    p2_kernel: "tools/bench/configs/profiles/p2_i3_direct_drive_kernel.cfg",
    p3_ramp: "tools/bench/configs/profiles/p3_voron_direct_drive_ramp.cfg",
    p3_kernel: "tools/bench/configs/profiles/p3_voron_direct_drive_kernel.cfg",
    p4_ramp: "tools/bench/configs/profiles/p4_high_accel_ramp.cfg",
    p4_kernel: "tools/bench/configs/profiles/p4_high_accel_kernel.cfg",
    trident_ramp: "tools/bench/configs/trident_ramp.cfg",
    trident_kernel: "tools/bench/configs/trident_kernel.cfg",
}
