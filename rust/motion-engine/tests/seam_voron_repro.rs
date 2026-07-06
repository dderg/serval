// Offline reproduction harness for the intermittent live `OverCommitted` abort
// seen on the Neptune bench (Voron cube print). The streaming pipeline replays
// the whole file; repeated runs vary the stage interleaving (and so the
// emission boundaries) where the batch harness used to sweep commit caps.
//
// Point VORON_GCODE at the exact print file, then:
//   VORON_GCODE=/path/voron_cube.gcode cargo test -p _motion_engine \
//       --release --test seam_voron_repro -- --ignored --nocapture

use _motion_engine::seam_test_harness::{default_stream_config, parse_gcode_to_moves, run_moves};

#[test]
#[ignore = "needs VORON_GCODE env pointing at the bench print file"]
fn voron_seam_repeat_sweep() {
    let path = std::env::var("VORON_GCODE").expect("set VORON_GCODE to the gcode path");
    let src = std::fs::read_to_string(&path).expect("read gcode");

    // Faithful to the bench host StreamConfig (bridge.rs): default harness
    // already matches integration_tol=1e-4, buffer=512, jerk=100k, arc_fit off;
    // the only deltas are the printer.cfg scv and the host fit_tolerance default.
    let mut cfg = default_stream_config();
    cfg.limits.square_corner_velocity_mm_s = std::env::var("VORON_SCV")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(8.0);
    cfg.fit_tol_mm = 0.005;
    let moves = parse_gcode_to_moves(&src, cfg.limits);
    eprintln!("parsed {} moves from {path}", moves.len());

    let mut fatal_runs = 0usize;
    for run in 1usize..=10 {
        let report = run_moves(&moves, cfg);
        let fatal = report.fatal();
        eprintln!(
            "run={run:2}: segments={} fatal_c0_seams={fatal}",
            report.segments
        );
        if fatal > 0 {
            fatal_runs += 1;
        }
    }
    assert_eq!(fatal_runs, 0, "reproduced a fatal C0 seam offline");
}
