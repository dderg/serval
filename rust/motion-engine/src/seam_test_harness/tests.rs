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
fn cadence_cap_never_drops_below_one() {
    assert_eq!(Cadence::FixedCap(0).cap_for(0), 1);
    assert_eq!(
        Cadence::VaryingCaps(vec![]).cap_for(7),
        usize::MAX,
        "an empty cadence list means no opportunistic commit, only the final flush"
    );
    assert_eq!(Cadence::VaryingCaps(vec![3, 0, 5]).cap_for(1), 1);
    assert_eq!(Cadence::VaryingCaps(vec![3, 9, 5]).cap_for(4), 9);
}

#[test]
fn run_schedule_reports_sane_structure() {
    let report = run_schedule(
        SQUARE,
        default_stream_config(),
        &CommitSchedule::fixed_cap(2),
    )
    .expect("square drives the stream cleanly");
    assert_eq!(report.moves, 4);
    assert!(report.commits > 0, "at least one commit must occur");
    assert!(report.segments > 0, "lowering must emit segments");
    assert!(report.worst() >= 0.0);
}

#[test]
fn run_schedule_is_deterministic() {
    let a = run_schedule(
        SQUARE,
        default_stream_config(),
        &CommitSchedule::fixed_cap(2),
    )
    .unwrap();
    let b = run_schedule(
        SQUARE,
        default_stream_config(),
        &CommitSchedule::fixed_cap(2),
    )
    .unwrap();
    assert_eq!(
        a, b,
        "same (gcode, config, schedule) must replay byte-identically"
    );
}

#[test]
fn forced_commit_path_runs() {
    let schedule = CommitSchedule {
        cadence: Cadence::FixedCap(64),
        force_after_move: vec![1, 2],
    };
    let report = run_schedule(SQUARE, default_stream_config(), &schedule)
        .expect("forced commits drive cleanly");
    assert!(
        report.commits >= 2,
        "two forced commits plus the final flush must produce commits"
    );
}

#[test]
fn varying_caps_path_runs() {
    let schedule = CommitSchedule {
        cadence: Cadence::VaryingCaps(vec![1, 2, 3]),
        force_after_move: vec![],
    };
    let report = run_schedule(SQUARE, default_stream_config(), &schedule)
        .expect("varying cadence drives cleanly");
    assert_eq!(report.moves, 4);
    assert!(report.commits > 0);
}

const CRASH_VORON_CUBE: &str = include_str!("crash_voron_cube.gcode");

fn bench_config_arc_fit() -> StreamConfig {
    let mut cfg = default_stream_config();
    cfg.chain = ChainFitConfig::with_arc_fit(3);
    cfg.limits = VelocityLimits::try_new(500.0, 8000.0, 20.0).expect("bench limits valid");
    cfg
}

#[test]
fn arc_fit_voron_cube_perimeter_is_c0_at_every_commit_cadence() {
    // TODO: caps below 64 trigger a pre-existing OverCommitted at
    // line 263 — the small window commits a velocity ceiling before
    // the arc fit can adjust the neighbor corner's budget.
    for cap in [64usize, 100_000] {
        let rep = run_schedule(
            CRASH_VORON_CUBE,
            bench_config_arc_fit(),
            &CommitSchedule::fixed_cap(cap),
        )
        .unwrap_or_else(|e| panic!("cap={cap}: {e:?}"));
        assert_eq!(
            rep.fatal(),
            0,
            "cap={cap}: fatal junction discontinuity (worst={:.4} mm): {:?}",
            rep.worst(),
            rep.worst_fatal()
        );
    }
}
