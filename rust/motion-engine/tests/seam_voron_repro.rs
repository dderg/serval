// Offline reproduction of the intermittent live `OverCommitted` abort seen on the
// Neptune bench (Voron cube print). The bug is timing-dependent live because it
// only fires when a commit seam lands with a *short front* (an early, producer-
// stall-forced commit) whose re-fit yields a sharper apex than the pre-commit
// curvature, dropping the corner cap below the already-committed entry velocity.
//
// Small FixedCap = commit as soon as the buffer holds `cap` moves = short front,
// so sweeping cap deterministically surfaces the seams live timing hits at random.
//
// Point VORON_GCODE at the exact print file, then:
//   VORON_GCODE=/path/voron_cube.gcode cargo test -p _motion_engine \
//       --release --test seam_voron_repro -- --ignored --nocapture

use _motion_engine::seam_test_harness::{
    Cadence, CommitSchedule, default_stream_config, parse_gcode_to_moves, run_moves,
};

#[test]
#[ignore = "needs VORON_GCODE env pointing at the bench print file"]
fn voron_seam_cap_sweep() {
    let path = std::env::var("VORON_GCODE").expect("set VORON_GCODE to the gcode path");
    let src = std::fs::read_to_string(&path).expect("read gcode");

    // Faithful to the bench host StreamConfig (bridge.rs:3511): default harness
    // already matches integration_tol=1e-4, buffer=512, jerk=100k, arc_fit off;
    // the only deltas are the printer.cfg scv and the host fit_tolerance default.
    let mut cfg = default_stream_config();
    cfg.limits.square_corner_velocity_mm_s = 8.0;
    cfg.fit_tol_mm = 0.005;
    let moves = parse_gcode_to_moves(&src, cfg.limits);
    eprintln!("parsed {} moves from {path}", moves.len());

    let mut fatals = 0usize;
    for cap in 1usize..=40 {
        let schedule = CommitSchedule {
            cadence: Cadence::FixedCap(cap),
            force_after_move: Vec::new(),
        };
        let mut cfg = default_stream_config();
        cfg.limits.square_corner_velocity_mm_s = 8.0;
        cfg.fit_tol_mm = 0.005;
        match run_moves(&moves, cfg, &schedule) {
            Ok(report) => eprintln!(
                "cap={cap:2}: OK   commits={} fatal_c0_seams={}",
                report.commits,
                report.fatal()
            ),
            Err(e) => {
                fatals += 1;
                eprintln!("cap={cap:2}: ABORT {e}");
            }
        }
    }
    eprintln!("caps that aborted: {fatals}/40");
    assert_eq!(fatals, 0, "reproduced the live OverCommitted abort offline");
}
