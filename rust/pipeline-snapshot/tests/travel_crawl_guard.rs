//! #903: the turning guard sizes a crawl valley only where the follower
//! projection lays one, and a guarded follower lays none where it extrudes
//! nothing within the crawl's reach.

mod corpus;

use std::path::PathBuf;

const BENCH_PRINTER: &str = "tools/bench/printer.cfg";
const BENCH_SHAPERS: [&str; 2] = ["post_processors: shaper_x\n", "post_processors: shaper_y\n"];

/// `tools/bench/printer.cfg` with X and Y unshaped: the extruder keeps its
/// kernel-free linear advance, so the turning guard holds it.
fn unshaped_bench_printer(name: &str) -> PathBuf {
    let mut text =
        std::fs::read_to_string(corpus::repo().join(BENCH_PRINTER)).expect("read config");
    for shaper in BENCH_SHAPERS {
        assert!(text.contains(shaper), "{BENCH_PRINTER} no longer has {shaper:?}");
        text = text.replacen(shaper, "", 1);
    }
    printer_from(name, &text)
}

fn printer_from(name: &str, text: &str) -> PathBuf {
    let config = std::env::temp_dir().join(format!("{name}_{}.cfg", std::process::id()));
    std::fs::write(&config, text).expect("temp config");
    config
}

fn plans_within_the_extruder_limits(name: &str, gcode: &str, config: &std::path::Path) {
    let snapshot = corpus::plan(name, gcode, config).expect("the repro plans motion");
    let audit = corpus::extruder::extruder_audit(&snapshot, gcode, config)
        .expect("the config limits the extruder");
    let excesses: Vec<String> = audit.excesses.iter().map(ToString::to_string).collect();
    assert!(excesses.is_empty(), "{}: {excesses:#?}", audit.summary());
}

/// `lead_in`, then a zigzag of `count` moves `pitch_mm` long along X, each
/// `pitch_mm` across Y, at 100 mm/s; move `index` extrudes what
/// `extrude(index)` says.
fn zigzag(
    lead_in: &str,
    pitch_mm: f64,
    count: usize,
    extrude: impl Fn(usize) -> Option<f64>,
) -> String {
    let (x0, y0) = (ZIGZAG_START_X, ZIGZAG_Y);
    let mut gcode = format!("G90\nM83\n{lead_in}G1 X{x0} Y{y0} F6000\n");
    for index in 1..=count {
        let x = x0 + pitch_mm * index as f64;
        let y = if index % 2 == 1 { y0 + pitch_mm } else { y0 };
        let e = extrude(index).map_or(String::new(), |e| format!(" E{e:.6}"));
        gcode.push_str(&format!("G1 X{x:.6} Y{y:.6}{e}\n"));
    }
    gcode
}

const ZIGZAG_START_X: f64 = 100.0;
const ZIGZAG_Y: f64 = 100.0;

const TRIAGE_PITCH_MM: f64 = 0.0125;
const TRIAGE_MOVES: usize = 200;
const SWEEP_PITCH_MM: f64 = 0.01;
const SWEEP_MOVES: usize = 1000;
const ZIGZAG_E_PER_MOVE: f64 = 0.0005;

/// #903's triage: 200 travel zigzags 12.5 µm apart on the bench chain
/// without shapers. The guard found a crawl at 2 mm/s along them with a
/// valley side up to 4.6 mm wide, past the 0.39 mm the leaders cover in the
/// lead budget, and backed them off to its floor.
#[test]
fn an_unshaped_travel_zigzag_plans() {
    let config = unshaped_bench_printer("unshaped_travel_zigzag");
    let gcode = zigzag("", TRIAGE_PITCH_MM, TRIAGE_MOVES, |_| None);
    corpus::plan("unshaped_travel_zigzag", &gcode, &config).expect("the zigzag plans");
}

/// The offline sweep's `syn_tiny_zigzag_0.01`, a thousand 10 µm travel
/// zigzags, on the same unshaped chain.
#[test]
fn an_unshaped_sweep_zigzag_plans() {
    let config = unshaped_bench_printer("unshaped_sweep_zigzag");
    let gcode = zigzag("", SWEEP_PITCH_MM, SWEEP_MOVES, |_| None);
    corpus::plan("unshaped_sweep_zigzag", &gcode, &config).expect("the zigzag plans");
}

/// The triage's zigzag extruding on every move: its crawls lay valleys,
/// which the guard sizes as before.
#[test]
fn an_unshaped_extruding_zigzag_fits_the_extruder() {
    let config = unshaped_bench_printer("unshaped_extruding_zigzag");
    let gcode = zigzag("", TRIAGE_PITCH_MM, TRIAGE_MOVES, |_| Some(ZIGZAG_E_PER_MOVE));
    plans_within_the_extruder_limits("unshaped_extruding_zigzag", &gcode, &config);
}

/// The triage's travel zigzag after an extruding line and a straight
/// travel `TRAVEL_PAST_REACH_MM` long, past the 4.6 mm reach of its crawls
/// and the ramp of the line's ratio step (`0.5·v·RATIO_RAMP_MAX_TIME_S`,
/// 1.5 mm at 100 mm/s): its crawls lay no valley, and the line extrudes
/// within the extruder's limits.
#[test]
fn an_unshaped_travel_zigzag_past_an_extrusion_fits_the_extruder() {
    const LINE_MM: f64 = 4.0;
    const TRAVEL_PAST_REACH_MM: f64 = 8.0;
    const LINE_E: f64 = 0.15;
    let config = unshaped_bench_printer("unshaped_zigzag_past_extrusion");
    let line_end = ZIGZAG_START_X - TRAVEL_PAST_REACH_MM;
    let line_start = line_end - LINE_MM;
    let lead_in = format!(
        "G1 X{line_start} Y{ZIGZAG_Y} F6000\nG1 X{line_end} Y{ZIGZAG_Y} E{LINE_E}\n"
    );
    let gcode = zigzag(&lead_in, TRIAGE_PITCH_MM, TRIAGE_MOVES, |_| None);
    plans_within_the_extruder_limits("unshaped_zigzag_past_extrusion", &gcode, &config);
}

/// #903's minimized scenario fuzz case (seed 8): the `heaters` sim base,
/// no shaper, a kernel-free linear advance, and four travel moves, two of
/// them at 15 mm/s between travels at 462 mm/s. The guard laid a 14 mm
/// valley side nothing extrudes on.
const FUZZ_SEED_8_PRINTER: &str = "[printer]
max_velocity: 700.0
max_accel: 30710.0
max_jerk: 0
square_corner_velocity: 22.4
max_z_velocity: 25.0
max_z_accel: 30.0
[kinematics]
type: cartesian
axis_x: x
axis_y: y
axis_z: z
x_motors: x
y_motors: y
z_motors: z
[axis x]
position_min: 0
position_max: 200
[axis y]
position_min: 0
position_max: 200
[axis z]
position_min: -5
position_max: 200
[axis e]
follows: x, y, z
motors: extruder
post_processors: fz_pa
max_accel: 20000
max_velocity: 120
[post_processor fz_pa]
type: linear_pressure_advance
k: 0.0771
[motor x]
drive: stepper
[motor y]
drive: stepper
[motor z]
drive: stepper
[motor extruder]
drive: stepper
[extruder]
axis: e
nozzle_diameter: 0.400
filament_diameter: 1.750
";

const FUZZ_SEED_8_PRINT: &str = "G90
M83
G1 X100 Y100 Z10 F600
G1 Z1.95 F600
G1 X181.215749 Y11 F27720
G1 X181.215749 Y34.822483 F900
G1 X126.986427 Y65.215068 F900
G1 X154.984 Y128.758 F27720
";

#[test]
fn the_minimized_fuzz_travels_plan() {
    let config = printer_from("fuzz_seed_8", FUZZ_SEED_8_PRINTER);
    corpus::plan("fuzz_seed_8", FUZZ_SEED_8_PRINT, &config).expect("the travels plan");
}
