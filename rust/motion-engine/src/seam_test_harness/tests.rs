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
    for cap in [1usize, 2, 4, 8, 16, 64, 100_000] {
        let rep = run_schedule(
            CRASH_VORON_CUBE,
            bench_config_arc_fit(),
            &CommitSchedule::fixed_cap(cap),
        )
        .expect("stream drives cleanly");
        assert_eq!(
            rep.fatal(),
            0,
            "cap={cap}: fatal junction discontinuity (worst={:.4} mm): {:?}",
            rep.worst(),
            rep.worst_fatal()
        );
    }
}

/// Strip every `E<num>` word so the gcode drives the fit the way the snapshot
/// pipeline does — `viz::build_moves` feeds `e_delta = 0` for every move, so the
/// extrusion-aware flow gates never fire there. Feeding extrusion to `fit_chain`
/// would exercise a path production never takes.
fn strip_extrusion(src: &str) -> String {
    src.lines()
        .map(|line| {
            line.split(' ')
                .filter(|t| {
                    !(t.starts_with('E')
                        && t[1..].starts_with(|c: char| c.is_ascii_digit() || c == '.' || c == '-'))
                })
                .collect::<Vec<_>>()
                .join(" ")
        })
        .collect::<Vec<_>>()
        .join("\n")
}

/// The snapshot fit (`fit_chain`, what `pipeline_snapshot` drives) must be free
/// of G2 curvature discontinuities on the neptune_cube cases: every adjacent
/// segment pair shares its junction curvature. A bare arc→line seam would jump.
#[test]
fn neptune_cube_snapshots_are_g2_continuous() {
    use geometry::path::CurvatureProfile;
    use geometry::path::lowering::PositionProfile;
    let cfg = bench_config_arc_fit();
    let cases = [
        (
            "discontinuity",
            include_str!("../../../../snapshots/cases/neptune_cube/discontinuity.gcode"),
        ),
        (
            "layer_5",
            include_str!("../../../../snapshots/cases/neptune_cube/layer_5.gcode"),
        ),
        (
            "layer_6",
            include_str!("../../../../snapshots/cases/neptune_cube/layer_6.gcode"),
        ),
    ];
    for (name, src) in cases {
        let moves = parse_gcode_to_moves(&strip_extrusion(src), cfg.limits);
        let out = geometry::fit_chain(&moves, cfg.chain).expect("fit drives cleanly");
        for w in out.moves.windows(2) {
            let (Some(a), Some(b)) = (w[0].segment.spatial.as_ref(), w[1].segment.spatial.as_ref())
            else {
                continue;
            };
            let jump = (a.kappa(a.s_len()) - b.kappa(0.0)).abs();
            assert!(
                jump <= 1e-4,
                "{name}: G2 discontinuity {jump:.4} at L{}->L{}",
                w[0].source.start_line,
                w[1].source.start_line,
            );
        }
    }
}

fn move_epmm(m: &geometry::Move) -> f64 {
    m.segment.followers.iter().map(|f| f.ratio.abs()).sum()
}

/// A curvature blend may never straddle an extrude↔travel boundary: where one
/// side deposits filament and the other is an air-move, the junction is a hard
/// full stop, not a clothoid (the extruder can't accelerate into a blended
/// travel, and bending the bead into the travel smears the feature edge). The
/// snapshot pipeline strips extrusion, so this invariant only has teeth on the
/// extrusion-carrying harness parse — hence the real (E-bearing) layer_5.
#[test]
fn extrude_travel_seams_are_full_stop() {
    use geometry::path::Segment as S;
    let cfg = bench_config_arc_fit();
    let moves = parse_gcode_to_moves(
        include_str!("../../../../snapshots/cases/neptune_cube/layer_5.gcode"),
        cfg.limits,
    );
    let out = geometry::fit_chain(&moves, cfg.chain).expect("fit drives cleanly");
    let segs: Vec<(bool, f64)> = out
        .moves
        .iter()
        .filter_map(|m| {
            m.segment
                .spatial
                .as_ref()
                .map(|s| (matches!(s, S::Clothoid(_)), move_epmm(m)))
        })
        .collect();
    let mut i = 0;
    while i < segs.len() {
        if !segs[i].0 {
            i += 1;
            continue;
        }
        let before = (0..i).rev().find(|&j| !segs[j].0).map(|j| segs[j].1);
        let mut e = i;
        while e < segs.len() && segs[e].0 {
            e += 1;
        }
        let after = segs.get(e).filter(|s| !s.0).map(|s| s.1);
        if let (Some(a), Some(b)) = (before, after) {
            const TRAVEL: f64 = 1e-9;
            assert!(
                (a < TRAVEL) == (b < TRAVEL),
                "blend straddles extrude↔travel seam (epmm {a:.4} -> {b:.4})"
            );
        }
        i = e;
    }
}
