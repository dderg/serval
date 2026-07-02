use super::*;

const SQUARE: &str = "\
G90
G1 X0 Y0 F3000
G1 X20 Y0
G1 X20 Y20
G1 X0 Y20
G1 X0 Y0
";

#[test]
fn parse_drops_origin_and_zero_length_moves() {
    let limits = default_stream_config().limits;
    let moves = parse_gcode_to_moves(SQUARE, limits);
    assert_eq!(
        moves.len(),
        4,
        "origin-establishing move is consumed; four cornering moves remain"
    );
}

#[test]
fn relative_mode_is_honored() {
    let limits = default_stream_config().limits;
    let abs = "G90\nG1 X0 Y0 F3000\nG1 X10 Y0\nG1 X10 Y10\n";
    let rel = "G90\nG1 X0 Y0 F3000\nG91\nG1 X10 Y0\nG1 X0 Y10\n";
    assert_eq!(
        parse_gcode_to_moves(abs, limits).len(),
        parse_gcode_to_moves(rel, limits).len(),
        "relative and absolute encodings of the same path yield the same move count"
    );
}

#[test]
fn run_schedule_reports_sane_structure() {
    let report = run_schedule(SQUARE, default_stream_config());
    assert_eq!(report.moves, 4);
    assert!(report.segments > 0, "lowering must emit segments");
    assert!(report.worst() >= 0.0);
}

#[test]
fn pipeline_replay_is_seam_free() {
    let a = run_schedule(SQUARE, default_stream_config());
    let b = run_schedule(SQUARE, default_stream_config());
    for rep in [&a, &b] {
        assert_eq!(
            rep.fatal(),
            0,
            "square must replay without a fatal seam; worst {:?}",
            rep.worst_fatal()
        );
    }
}

const CRASH_VORON_CUBE: &str = include_str!("crash_voron_cube.gcode");

fn bench_config_arc_fit() -> StreamConfig {
    let mut cfg = default_stream_config();
    cfg.chain = ChainFitConfig::with_arc_fit(3);
    cfg.limits =
        VelocityLimits::try_new(500.0, 8000.0, 20.0, 100_000.0).expect("bench limits valid");
    cfg
}

#[test]
fn arc_fit_voron_cube_perimeter_is_c0() {
    let rep = run_schedule(CRASH_VORON_CUBE, bench_config_arc_fit());
    assert_eq!(
        rep.fatal(),
        0,
        "fatal junction discontinuity (worst={:.4} mm): {:?}",
        rep.worst(),
        rep.worst_fatal()
    );
}
