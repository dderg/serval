//! An advance acts on the extruder's velocity, which is zero at every rest,
//! so a run ends on exactly the material the chain lays without it: the
//! ratio over the shaped arc wherever the commanded toolhead moves, and over
//! the chord the shapers' spill covers where it rests (docs/rewrite/shaper.md,
//! "Followers"). The cases are `tools/corner_flow.py`'s: a 45° corner and a
//! 2 mm zigzag between 30 mm leads, extruding a 0.45 × 0.2 mm bead of
//! 1.75 mm filament, each starting and ending at rest with none between.

mod corpus;

use std::path::Path;

const FILAMENT_MM: f64 = 1.75;
const BEAD_WIDTH_MM: f64 = 0.45;
const BEAD_HEIGHT_MM: f64 = 0.2;
const CONSERVED_MM: f64 = 1e-9;

fn bead_ratio() -> f64 {
    BEAD_WIDTH_MM * BEAD_HEIGHT_MM / (std::f64::consts::PI * (FILAMENT_MM / 2.0).powi(2))
}

fn gcode(points: &[(f64, f64)], speed_mm_s: f64) -> String {
    let ratio = bead_ratio();
    let mut lines = vec![
        "G21".to_string(),
        "G90".to_string(),
        "M83".to_string(),
        format!("G1 X0 Y0 F{:.0}", speed_mm_s * 60.0),
    ];
    let (mut x, mut y) = (0.0, 0.0);
    for &(to_x, to_y) in points {
        let length = libm::hypot(to_x - x, to_y - y);
        lines.push(format!("G1 X{to_x:.12} Y{to_y:.12} E{:.12}", ratio * length));
        (x, y) = (to_x, to_y);
    }
    lines.join("\n") + "\n"
}

fn corner_45() -> Vec<(f64, f64)> {
    let turn = (180.0_f64 - 45.0).to_radians();
    vec![
        (60.0, 0.0),
        (60.0 + 60.0 * libm::cos(turn), 60.0 * libm::sin(turn)),
    ]
}

fn zigzag() -> Vec<(f64, f64)> {
    let rise = 2.0 / std::f64::consts::SQRT_2;
    let mut points = vec![(30.0, 0.0)];
    let (mut x, mut y) = (30.0, 0.0);
    for index in 0..=20 {
        x += rise;
        y = if index % 2 == 0 { rise } else { 0.0 };
        points.push((x, y));
    }
    points.push((x + 30.0, y));
    points
}

/// `config` with the extruder's chain set to `chain`, its smoothing kernel's
/// time to `smooth_time` and its extrude-only caps to 50 mm/s and
/// 5000 mm/s², written to a scratch file.
fn variant(config: &Path, chain: &str, smooth_time: f64, name: &str) -> std::path::PathBuf {
    let text = std::fs::read_to_string(config).expect("config reads");
    let mut section = String::new();
    let mut out = Vec::new();
    for line in text.lines() {
        let trimmed = line.trim();
        if trimmed.starts_with('[') {
            if section == "[extruder]" {
                out.push("max_extrude_only_velocity = 50".to_string());
                out.push("max_extrude_only_accel = 5000".to_string());
            }
            section = trimmed.to_string();
        }
        let key = trimmed.split([':', '=']).next().unwrap_or("").trim();
        if section == "[post_processor st]" && key == "smooth_time" {
            out.push(format!("smooth_time = {smooth_time}"));
            continue;
        }
        let skip = (section == "[axis e]" && key == "post_processors")
            || (section == "[extruder]"
                && (key == "max_extrude_only_velocity" || key == "max_extrude_only_accel"));
        if skip {
            continue;
        }
        out.push(line.to_string());
        if trimmed == "[axis e]" && !chain.is_empty() {
            out.push(format!("post_processors = {chain}"));
        }
    }
    let dir = std::env::temp_dir().join(format!("advance_conservation_{}", std::process::id()));
    std::fs::create_dir_all(&dir).expect("scratch dir");
    let path = dir.join(format!("{name}.cfg"));
    std::fs::write(&path, out.join("\n") + "\n").expect("variant writes");
    path
}

const PATH_PANELS_PER_ROW: usize = 20000;

fn toolhead_xy(trajectory: &pipeline_snapshot::ExactTrajectory, row: usize, t: f64) -> (f64, f64) {
    let at = |axis: usize| {
        trajectory
            .eval_row(axis, row, t, pipeline_snapshot::SampleSide::Right)
            .expect("leader carrier")
            .position
    };
    (at(0), at(1))
}

/// The final extruder position, and the material the chord rule lays: the
/// bead ratio over the shaped arc of every row but the first and the last,
/// and over the chord from the start point to where the first row's spill
/// ends and from where the last row's spill starts to the end point. A run
/// from rest to rest with no rest between opens and closes on exactly one
/// spill row each, the shapers' reach before the first move and after the
/// last.
fn final_e_and_chord_rule(
    name: &str,
    program: &str,
    config: &Path,
    (start, end): ((f64, f64), (f64, f64)),
) -> (f64, f64) {
    let snapshot = corpus::plan(name, program, config).expect("the case plans");
    let trajectory = &snapshot.trajectory;
    let final_e = trajectory
        .eval_axis(
            pipeline_snapshot::EXTRUDER_AXIS,
            trajectory.t_end(),
            pipeline_snapshot::SampleSide::Left,
        )
        .expect("extruder carrier")
        .position;
    let rows = trajectory.rows(0);
    assert!(
        rows.len() >= 3,
        "{name}: a spill row on either side of the moves"
    );
    let distance = |a: (f64, f64), b: (f64, f64)| libm::hypot(a.0 - b.0, a.1 - b.1);
    let last = rows.len() - 1;
    let mut path = distance(start, toolhead_xy(trajectory, 0, rows[0].t1))
        + distance(toolhead_xy(trajectory, last, rows[last].t0), end);
    for (index, row) in rows.iter().enumerate().take(last).skip(1) {
        let speed = |t: f64| {
            (0..pipeline_snapshot::EXTRUDER_AXIS)
                .map(|axis| {
                    trajectory
                        .eval_row(axis, index, t, pipeline_snapshot::SampleSide::Right)
                        .expect("leader carrier")
                        .velocity
                        .powi(2)
                })
                .sum::<f64>()
                .sqrt()
        };
        let h = (row.t1 - row.t0) / PATH_PANELS_PER_ROW as f64;
        for panel in 0..PATH_PANELS_PER_ROW {
            let a = row.t0 + panel as f64 * h;
            path += h / 6.0 * (speed(a) + 4.0 * speed(a + 0.5 * h) + speed(a + h));
        }
    }
    (final_e, bead_ratio() * path)
}

struct Case<'a> {
    config: &'a str,
    chain: &'a str,
    plain_chain: &'a str,
    smooth_time: f64,
    name: &'a str,
    points: Vec<(f64, f64)>,
    speed_mm_s: f64,
}

/// Both chains lay exactly what the chord rule asks of their own plan. The
/// two plans need not share a path: with no kernel, or a short one, the
/// extruder caps budget the toolhead's acceleration for the advance, so the
/// planner moves it differently and the shapers cut the corners by a
/// different amount.
fn assert_advance_conserves(case: &Case) {
    let config = corpus::repo().join(case.config);
    let program = gcode(&case.points, case.speed_mm_s);
    let ends = ((0.0, 0.0), *case.points.last().expect("a case moves"));
    for chain in [case.chain, case.plain_chain] {
        let path = variant(
            &config,
            chain,
            case.smooth_time,
            &format!("{}_{}", case.name, chain.len()),
        );
        let (final_e, laid) = final_e_and_chord_rule(case.name, &program, &path, ends);
        assert!(
            (final_e - laid).abs() <= CONSERVED_MM,
            "{} at {} mm/s under `{chain}`: the extruder ends on {final_e}, {} mm off the {laid} \
             the chord rule lays",
            case.name,
            case.speed_mm_s,
            final_e - laid
        );
    }
}

#[test]
fn a_kernel_free_advance_conserves_material_through_a_corner() {
    assert_advance_conserves(&Case {
        config: "tools/bench/printer.cfg",
        chain: "pa",
        plain_chain: "",
        smooth_time: 0.04,
        name: "c45",
        points: corner_45(),
        speed_mm_s: 150.0,
    });
}

#[test]
fn a_kernel_free_advance_conserves_material_through_a_zigzag() {
    assert_advance_conserves(&Case {
        config: "tools/bench/configs/trident_ramp.cfg",
        chain: "pa",
        plain_chain: "",
        smooth_time: 0.04,
        name: "zigzag",
        points: zigzag(),
        speed_mm_s: 300.0,
    });
}

#[test]
fn an_advance_before_a_short_kernel_conserves_material_through_a_corner() {
    assert_advance_conserves(&Case {
        config: "tools/bench/printer.cfg",
        chain: "pa, st",
        plain_chain: "st",
        smooth_time: 0.005,
        name: "c45_short_kernel",
        points: corner_45(),
        speed_mm_s: 150.0,
    });
}

#[test]
fn an_advance_before_a_long_kernel_conserves_material_through_a_corner() {
    assert_advance_conserves(&Case {
        config: "tools/bench/printer.cfg",
        chain: "pa, st",
        plain_chain: "st",
        smooth_time: 0.04,
        name: "c45_long_kernel",
        points: corner_45(),
        speed_mm_s: 150.0,
    });
}

#[test]
fn a_kernel_free_advance_on_the_trident_conserves_material_through_a_corner() {
    assert_advance_conserves(&Case {
        config: "tools/bench/configs/trident_ramp.cfg",
        chain: "pa",
        plain_chain: "",
        smooth_time: 0.04,
        name: "c45_trident",
        points: corner_45(),
        speed_mm_s: 150.0,
    });
}
