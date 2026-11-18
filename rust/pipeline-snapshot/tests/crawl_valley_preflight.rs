//! Prints the offline preflight sweep planned to a panic around crawl
//! valleys of a ratio-ramped advance, each cut to the few lines that still
//! panicked on 7250ffd13, on the config that found it.

mod corpus;

use std::path::{Path, PathBuf};

const BENCH_PRINTER: &str = "tools/bench/printer.cfg";
const BENCH_CORNER_DEVIATION: &str = "corner_deviation: 0.21";

/// `tools/bench/printer.cfg` with its corner budget line replaced by
/// `corner`, written where the planner reads it.
fn bench_printer_with(name: &str, corner: &str) -> PathBuf {
    let text = std::fs::read_to_string(corpus::repo().join(BENCH_PRINTER)).expect("read config");
    assert!(
        text.contains(BENCH_CORNER_DEVIATION),
        "{BENCH_PRINTER} no longer sets {BENCH_CORNER_DEVIATION}"
    );
    let config = std::env::temp_dir().join(format!("{name}_{}.cfg", std::process::id()));
    std::fs::write(&config, text.replacen(BENCH_CORNER_DEVIATION, corner, 1)).expect("temp config");
    config
}

fn printer_from(name: &str, text: &str) -> PathBuf {
    let config = std::env::temp_dir().join(format!("{name}_{}.cfg", std::process::id()));
    std::fs::write(&config, text).expect("temp config");
    config
}

fn plans_within_the_extruder_limits(name: &str, gcode: &str, config: &Path) {
    let snapshot = corpus::plan(name, gcode, config).expect("the repro plans motion");
    let audit = corpus::extruder::extruder_audit(&snapshot, gcode, config)
        .expect("the config limits the extruder");
    let excesses: Vec<String> = audit.excesses.iter().map(ToString::to_string).collect();
    assert!(excesses.is_empty(), "{}: {excesses:#?}", audit.summary());
}

/// #886: `voron_dense_top_layers.gcode` under a 1 mm corner budget. Its
/// blends slope the ratio over millimetres, and a crawl valley that owns
/// a ratio step owns that sloped run too; the step's flow was read at the
/// speed the leaders reach at the run's far end, 52675 mm/s² against
/// 20000.
const LONG_BLEND_RUN_INTO_A_CRAWL: &str = "G90
M83
G1 X120.123 Y150.439 Z40.4 F60000
G1 X117.142 Y149.487 E.01826
G1 X118.166 Y150.208 E.01864
G1 X118.603 Y150.302 E.0176
G1 X118.948 Y150.313 E.01354
G1 X119.27 Y150.276 E.01274
G1 X119.753 Y150.119 E.01994
G1 X120.115 Y149.909 E.01649
G1 X120.489 Y149.575 E.01967
G1 X120.515 Y149.539 E.00178
G1 X120.119 Y150.449 Z40.6 F60000
G1 X119.857 Y150.576 E.00916
G1 X119.6 Y150.667 E.00917
G1 X119.366 Y150.725 E.00812
";

#[test]
fn a_crawl_valley_owning_a_long_sloped_run_reads_each_step_where_it_ramps() {
    let config = bench_printer_with("long_blend_run", "corner_deviation: 1.0");
    plans_within_the_extruder_limits("long_blend_run", LONG_BLEND_RUN_INTO_A_CRAWL, &config);
}

/// #887: `voron_dense_top_layers_zhop.gcode` on the slow Trident profile.
/// The crawl at the turn from the z hop into the travel read the ratio of
/// the extrusion before the retract, across the rest, at 0.0385:
/// 5338 mm/s² against 5000 for a side whose leaders lay nothing.
const EXTRUSION_RETRACT_HOP_TRAVEL: &str = "G90
M83
G1 X134.928 Y157.499 Z40.2 F60000
G1 X135.619 Y139.703 E.6861
G1 E-.3 F3600
G1 Z40.6
G1 X120.126 Y150.428 F60000
";

#[test]
fn a_crawl_side_past_a_rest_reads_no_ratio_from_before_it() {
    let config = corpus::repo().join("rust/motion-core/tests/configs/p4_trident_ramp_slow.cfg");
    plans_within_the_extruder_limits("retract_hop_travel", EXTRUSION_RETRACT_HOP_TRAVEL, &config);
}

/// #888: `voron_dense_top_layers.gcode` on `bench_ramp.cfg`: a ratio step
/// ramped over 1.66 mm either side reached into the side of a crawl
/// valley 1.45 mm on, asking 123262 mm/s² against 5000.
const STEP_RAMPING_INTO_A_CRAWL_SIDE: &str = "M83
G1 X120.123 Y150.439 Z40.4 F60000
G1 X120.119 Y150.449 Z40.6 F60000
G1 X119.857 Y150.576 E.00916
G1 X121.366 Y147.337 E.00825
G1 X120.153 Y150.428 E.00708
";

#[test]
fn a_ratio_step_whose_ramp_reaches_a_crawl_side_keeps_the_extruder_within_its_limit() {
    let config = corpus::repo().join("rust/motion-core/tests/configs/bench_ramp.cfg");
    plans_within_the_extruder_limits(
        "step_into_crawl_side",
        STEP_RAMPING_INTO_A_CRAWL_SIDE,
        &config,
    );
}

/// #889: `voron_first_layers.gcode` with `square_corner_velocity: 0`: the
/// crawl valley at the reversal into the travel reached 6.36 mm back, but
/// the projection had taken the path to 5.74 mm before it.
const EXTRUSIONS_REVERSING_INTO_A_TRAVEL: &str = "G90
M83
G1 X118.78 Y118.345 E.02586
G1 X119.312 Y117.714 E.02507
G1 X119.832 Y116.953 E.02803
G1 X120.203 Y116.258 E.02394
G1 X120.569 Y115.412 E.02801
G1 X120.838 Y114.528 E.02808
G1 X121.062 Y113.02 E.02396
G1 X121.075 Y112.257 E.02317
G1 X119.699 Y124.517 F9000
";

#[test]
fn a_crawl_valley_reaching_back_to_a_rest_finds_that_path_untaken() {
    let config = bench_printer_with("reversal_into_travel", "square_corner_velocity: 0");
    plans_within_the_extruder_limits(
        "reversal_into_travel",
        EXTRUSIONS_REVERSING_INTO_A_TRAVEL,
        &config,
    );
}

/// A corexy with fast shapers and a kernel-free advance, drawn by the
/// print-like planner fuzz.
const FAST_SHAPED_ADVANCE: &str = "[printer]
max_velocity: 736
max_accel: 52700
max_z_velocity: 25
max_z_accel: 100
[kinematics]
type: corexy
axis_x: x
axis_y: y
axis_z: z
a_motors: a
b_motors: b
z_motors: z
[axis x]
post_processors: shaper_x
[axis y]
post_processors: shaper_y
[axis z]
[axis e]
follows: x, y, z
motors: extruder
post_processors: pa
max_accel: 18600
[post_processor pa]
type: linear_pressure_advance
k: 0.047
[post_processor shaper_x]
type: smooth_mzv
frequency_hz: 101.57
[post_processor shaper_y]
type: smooth_zv
frequency_hz: 114.82
[motor a]
drive: stepper
[motor b]
drive: stepper
[motor z]
drive: stepper
[motor extruder]
drive: stepper
[extruder]
axis: e
max_extrude_only_velocity: 50
max_extrude_only_accel: 5000
";

/// A reversal, a retract and a hop: the crawl valley at the reversal
/// keeps its floor flat past the rest valley the retract lays at its end,
/// and the hop's slowest point, 3e-5 mm on, lies on that floor. It was
/// taken for a crawl laid behind the rest valley (the hop on
/// `voron_dense_top_layers_zhop.gcode` and `p4_trident_ramp_slow.cfg` once
/// #887 no longer stopped it earlier).
const REVERSAL_RETRACT_HOP: &str = "G90
M83
G1 X150.629 Y150.542 E0.03447 F5342
G1 X161.535 Y155.766 E0.60584 F14064
G1 X161.486 Y155.755 E0.00140 F15840
G1 E-0.5 F2400
G1 Z0.600 F600
G1 X168.016 Y149.008 F30000
";

#[test]
fn a_crawl_on_a_valleys_floor_past_a_rest_needs_no_valley_of_its_own() {
    let config =
        std::env::temp_dir().join(format!("fast_shaped_advance_{}.cfg", std::process::id()));
    std::fs::write(&config, FAST_SHAPED_ADVANCE).expect("temp config");
    plans_within_the_extruder_limits("reversal_retract_hop", REVERSAL_RETRACT_HOP, &config);
}

/// #896: a fast corexy with a kernel-free advance, drawn by the print-like
/// planner fuzz.
const OPENING_CRAWL_PRINTER: &str = "[printer]
max_velocity: 694.719250661841
max_accel: 60718.597334361424
corner_deviation: 0.05729858307047112
max_z_velocity: 25
max_z_accel: 100
[kinematics]
type: corexy
axis_x: x
axis_y: y
axis_z: z
a_motors: a
b_motors: b
z_motors: z
[axis x]
post_processors: shaper_x
[axis y]
post_processors: shaper_y
[axis z]
[axis e]
follows: x, y, z
motors: extruder
post_processors: pa
max_accel: 15578.478844820105
[post_processor pa]
type: linear_pressure_advance
k: 0.014758274092604825
[post_processor shaper_x]
type: smooth_mzv
frequency_hz: 92.69466276064261
[post_processor shaper_y]
type: smooth_zv
frequency_hz: 104.10850563404021
[motor a]
drive: stepper
[motor b]
drive: stepper
[motor z]
drive: stepper
[motor extruder]
drive: stepper
[extruder]
axis: e
max_extrude_only_velocity: 50
max_extrude_only_accel: 5000
";

/// #896: a crawl valley at the stream's opening holds zero back to where
/// the leaders rested there and steps its weight at that rest's end; once
/// the committed path passes the rest, its valley is pruned and the step
/// was taken for one no valley holds.
const OPENING_REVERSAL: &str = "G90
M83
G1 X150.0 Y150.0 Z0.2 F30000
G1 X148.608 Y150.215 E0.04361 F13823
G1 X149.069 Y150.277 E0.01973 F17083
G1 X142.482 Y148.522 E0.22351 F4413
G1 X153.429 Y137.328 F30000
G1 X154.837 Y131.732 E0.22790 F3471
";

#[test]
fn a_valley_step_behind_the_committed_path_is_read_by_nothing() {
    let config =
        std::env::temp_dir().join(format!("opening_crawl_printer_{}.cfg", std::process::id()));
    std::fs::write(&config, OPENING_CRAWL_PRINTER).expect("temp config");
    plans_within_the_extruder_limits("opening_reversal", OPENING_REVERSAL, &config);
}

/// #898: a corexy with a kernel-free advance and a 0.74 mm corner budget,
/// drawn by the print-like planner fuzz.
const WAITING_CRAWL_PRINTER: &str = "[printer]
max_velocity: 942.3988288103301
max_accel: 47403.38670336128
corner_deviation: 0.7382658358164043
max_z_velocity: 25
max_z_accel: 100
[kinematics]
type: corexy
axis_x: x
axis_y: y
axis_z: z
a_motors: a
b_motors: b
z_motors: z
[axis x]
post_processors: shaper_x
[axis y]
post_processors: shaper_y
[axis z]
[axis e]
follows: x, y, z
motors: extruder
post_processors: pa
max_accel: 12568.097981509703
[post_processor pa]
type: linear_pressure_advance
k: 0.034813571169232026
[post_processor shaper_x]
type: smooth_mzv
frequency_hz: 54.08761953978757
[post_processor shaper_y]
type: smooth_zv
frequency_hz: 66.50291608902367
[motor a]
drive: stepper
[motor b]
drive: stepper
[motor z]
drive: stepper
[motor extruder]
drive: stepper
[extruder]
axis: e
max_extrude_only_velocity: 50
max_extrude_only_accel: 5000
";

/// #898: extrusions, a retract and hop, a travel, and extrusions again:
/// the projection took the path through a crawl whose valley was still
/// waiting on the spans past it.
const CRAWL_WAITING_PAST_A_HOP: &str = "G90
M83
G1 X150.0 Y150.0 Z0.2 F30000
G1 X162.520 Y148.170 F17051
G1 X161.935 Y148.180 E0.02696 F9159
G1 X164.740 Y154.177 E0.30497 F15202
G1 X163.435 Y145.208 E0.00660 F8121
G1 X163.466 Y145.716 E0.01341 F4821
G1 X163.193 Y146.504 E0.03146 F16205
G1 E-0.5 F2400
G1 Z0.600 F600
G1 X146.809 Y122.606 F30000
G1 Z0.200 F600
G1 E0.5 F2400
G1 X144.519 Y122.373 E0.06849 F15916
G1 X157.333 Y126.537 E0.28132 F10331
";

#[test]
fn a_waiting_crawl_keeps_its_path_untaken() {
    let config =
        std::env::temp_dir().join(format!("waiting_crawl_printer_{}.cfg", std::process::id()));
    std::fs::write(&config, WAITING_CRAWL_PRINTER).expect("temp config");
    plans_within_the_extruder_limits(
        "crawl_waiting_past_a_hop",
        CRAWL_WAITING_PAST_A_HOP,
        &config,
    );
}

fn plans_its_draw(draw: &corpus::crawl_cascade::Draw) {
    plans_within_the_extruder_limits(draw.name, draw.gcode, &draw.config());
}

#[test]
fn steps_far_past_a_reversal_keep_the_extruder_within_its_limit() {
    plans_its_draw(&corpus::crawl_cascade::STEPS_FAR_PAST_A_REVERSAL);
}

#[test]
fn a_crawl_among_a_streams_steps_keeps_the_extruder_within_its_limit() {
    plans_its_draw(&corpus::crawl_cascade::CRAWL_OWNING_THE_STREAM);
}

#[test]
fn a_crawl_among_close_steps_plans() {
    let draw = &corpus::crawl_cascade::CRAWL_AMONG_CLOSE_STEPS;
    corpus::plan(draw.name, draw.gcode, &draw.config()).expect("the draw plans motion");
}

#[test]
fn a_crawl_among_close_steps_keeps_the_extruder_within_its_limit() {
    plans_its_draw(&corpus::crawl_cascade::CRAWL_AMONG_CLOSE_STEPS);
}

/// #895: a corexy with a kernel-free advance limited to 12492 mm/s², drawn
/// by the print-like planner fuzz.
const SLIVER_LEG_PRINTER: &str = "[printer]
max_velocity: 968.4864196022259
max_accel: 11985.120692588014
square_corner_velocity: 2.631337096765715
max_z_velocity: 25
max_z_accel: 100
[kinematics]
type: corexy
axis_x: x
axis_y: y
axis_z: z
a_motors: a
b_motors: b
z_motors: z
[axis x]
post_processors: shaper_x
[axis y]
post_processors: shaper_y
[axis z]
[axis e]
follows: x, y, z
motors: extruder
post_processors: pa
max_accel: 12491.664179298827
[post_processor pa]
type: linear_pressure_advance
k: 0.018760200306420192
[post_processor shaper_x]
type: smooth_mzv
frequency_hz: 103.34334564648618
[post_processor shaper_y]
type: smooth_zv
frequency_hz: 63.85558791682546
[motor a]
drive: stepper
[motor b]
drive: stepper
[motor z]
drive: stepper
[motor extruder]
drive: stepper
[extruder]
axis: e
max_extrude_only_velocity: 50
max_extrude_only_accel: 5000
";

/// #895: short extrusions reversing into a slow travel. The crawl side
/// read the leaders' jerk as the mean over a 9e-6 mm probe interval of a
/// sliver leg, the fit's own: 40633 mm/s² against 12492.
const SLIVER_LEGS_INTO_A_CRAWL: &str = "G90
M83
G1 Z0.200 F600
G1 X188.228 Y132.858 E0.01540 F8236
G1 X188.480 Y132.995 E0.00615 F6687
G1 X188.231 Y132.863 E0.00765 F17733
G1 X187.986 Y132.765 E0.00544 F4988
G1 X187.919 Y132.776 E0.00120 F8358
G1 X191.618 Y133.199 F1866
";

#[test]
fn a_crawl_side_reads_a_sliver_legs_jerk_across_the_node_spacing() {
    let config =
        std::env::temp_dir().join(format!("sliver_leg_printer_{}.cfg", std::process::id()));
    std::fs::write(&config, SLIVER_LEG_PRINTER).expect("temp config");
    plans_within_the_extruder_limits(
        "sliver_legs_into_a_crawl",
        SLIVER_LEGS_INTO_A_CRAWL,
        &config,
    );
}

/// #895: `voron_first_layers_zhop.gcode` cut to an extrusion, a hop, a
/// travel, a prime and the extrusion back: on the bowden profile the crawl
/// side read 2238 mm/s² against 2000 over a 0.01 mm interval.
const BOWDEN_HOP_PRIME_AND_BACK: &str = "G90
M83
G1 X103.97 Y98.711 E.24344
G1 Z0.8
G1 X121.55 Y126.414 F9000
G1 Z0.6
G1 E.8 F1500
G1 X103.45 Y126.414 E.6089
";

#[test]
fn a_bowden_crawl_side_after_a_prime_fits_its_limit() {
    let config = corpus::repo().join("rust/motion-core/tests/configs/p1_bowden_ramp.cfg");
    plans_within_the_extruder_limits(
        "bowden_hop_prime_and_back",
        BOWDEN_HOP_PRIME_AND_BACK,
        &config,
    );
}

/// #895: a corexy with 30 Hz shapers and a kernel-free advance limited to
/// 2000 mm/s², shrunk by the print-like planner fuzz.
const TWO_REVERSALS_PRINTER: &str = "[printer]
max_velocity: 300
max_accel: 3000
square_corner_velocity: 2.7763543996128295
max_z_velocity: 25
max_z_accel: 100
[kinematics]
type: corexy
axis_x: x
axis_y: y
axis_z: z
a_motors: a
b_motors: b
z_motors: z
[axis x]
post_processors: shaper_x
[axis y]
post_processors: shaper_y
[axis z]
[axis e]
follows: x, y, z
motors: extruder
post_processors: pa
max_accel: 2000
[post_processor pa]
type: linear_pressure_advance
k: 0.027557459600702584
[post_processor shaper_x]
type: smooth_mzv
frequency_hz: 30
[post_processor shaper_y]
type: smooth_zv
frequency_hz: 30
[motor a]
drive: stepper
[motor b]
drive: stepper
[motor z]
drive: stepper
[motor extruder]
drive: stepper
[extruder]
axis: e
max_extrude_only_velocity: 50
max_extrude_only_accel: 5000
";

/// #895: two reversals 0.5 mm apart: the after side of the first crawl
/// reached over the second and read its turning, ~2e6 mm/s³, at the first
/// side's weight: 3566 mm/s² against 2000.
const TWO_REVERSALS: &str = "G90
M83
G1 X150.0 Y150.0 Z0.2 F30000
G1 X149.646 Y149.065 F30000
G1 X149.510 Y148.074 E0.04338 F1200
G1 X149.512 Y147.074 E0.04338 F1200
G1 X148.521 Y146.938 E0.04338 F1200
G1 X149.324 Y147.172 E0.05215 F5434
G1 X148.381 Y146.838 E0.04764 F8351
";

#[test]
fn a_crawl_side_leaves_the_next_crawls_turning_to_its_valley() {
    let config = printer_from("two_reversals_printer", TWO_REVERSALS_PRINTER);
    plans_within_the_extruder_limits("two_reversals", TWO_REVERSALS, &config);
}

/// #912: a corexy with 55 Hz smooth shapers and a kernel-free advance,
/// drawn by the print-like planner fuzz.
const REVERSAL_PAST_A_RETRACT_PRINTER: &str = "[printer]
max_velocity: 640.5295844275342
max_accel: 19421.576176581028
square_corner_velocity: 19.3295380619992
max_z_velocity: 25
max_z_accel: 100
[kinematics]
type: corexy
axis_x: x
axis_y: y
axis_z: z
a_motors: a
b_motors: b
z_motors: z
[axis x]
post_processors: shaper_x
[axis y]
post_processors: shaper_y
[axis z]
[axis e]
follows: x, y, z
motors: extruder
post_processors: pa
max_accel: 17616.127335386354
[post_processor pa]
type: linear_pressure_advance
k: 0.044418270385394816
[post_processor shaper_x]
type: smooth_mzv
frequency_hz: 55.16682954926499
[post_processor shaper_y]
type: smooth_zv
frequency_hz: 55.52886882081576
[motor a]
drive: stepper
[motor b]
drive: stepper
[motor z]
drive: stepper
[motor extruder]
drive: stepper
[extruder]
axis: e
max_extrude_only_velocity: 50
max_extrude_only_accel: 5000
";

/// #912: the reversal from the travel back over it, past a retract and a
/// z hop, widened the crawl valley of the turn before the retract back to
/// that valley's end, over the extruding move the projection had already
/// read at the valley's after side: the advance stepped its position by
/// 0.0035 mm where the move hands over to the retract.
const REVERSAL_PAST_A_RETRACT: &str = "G90
M83
G1 X150.0 Y150.0 Z0.2 F30000
G1 X152.723 Y154.752 E0.27212 F3571
G1 X152.359 Y154.137 E0.01266 F16890
G1 E-0.5 F2400
G1 Z0.600 F600
G1 X168.940 Y162.366 F12013
G1 X166.127 Y162.161 E0.00136 F6996
G1 X160.259 Y164.980 E0.26930 F6423
";

#[test]
fn a_crawl_valley_widened_past_a_retract_holds_zero_on_untaken_path() {
    let config = printer_from(
        "reversal_past_a_retract_printer",
        REVERSAL_PAST_A_RETRACT_PRINTER,
    );
    plans_within_the_extruder_limits("reversal_past_a_retract", REVERSAL_PAST_A_RETRACT, &config);
}

/// #912's widening check, drawn by the print-like planner fuzz: an 88 Hz
/// corexy with no square-corner velocity and a kernel-free advance.
const WIDENED_FROM_THE_TAKEN_EDGE_PRINTER: &str = "[printer]
max_velocity: 315.12426332023136
max_accel: 5396.416678781325
square_corner_velocity: 0
max_z_velocity: 25
max_z_accel: 100
[kinematics]
type: corexy
axis_x: x
axis_y: y
axis_z: z
a_motors: a
b_motors: b
z_motors: z
[axis x]
post_processors: shaper_x
[axis y]
post_processors: shaper_y
[axis z]
[axis e]
follows: x, y, z
motors: extruder
post_processors: pa
max_accel: 10069.342545867317
[post_processor pa]
type: linear_pressure_advance
k: 0.03820311461480342
[post_processor shaper_x]
type: smooth_mzv
frequency_hz: 88.15871560075092
[post_processor shaper_y]
type: smooth_zv
frequency_hz: 87.77439227405594
[motor a]
drive: stepper
[motor b]
drive: stepper
[motor z]
drive: stepper
[motor extruder]
drive: stepper
[extruder]
axis: e
max_extrude_only_velocity: 50
max_extrude_only_accel: 5000
";

/// The projection took the path to 3e-14 mm past the zero-wide crawl
/// valley at s=25.115, which the next crawl widens.
const WIDENED_FROM_THE_TAKEN_EDGE: &str = "G90
M83
G1 X150.0 Y150.0 Z0.2 F30000
G1 X150.075 Y149.980 E0.00336 F11517
G1 X166.285 Y147.318 F14701
G1 E-0.5 F2400
G1 Z0.600 F600
G1 X158.077 Y147.036 F30000
G1 Z0.200 F600
G1 E0.5 F2400
G1 X158.140 Y147.040 E0.00191 F7700
G1 X158.635 Y147.446 E0.00638 F10130
G1 X158.483 Y147.069 E0.02095 F12356
G1 X158.568 Y147.517 F3615
G1 E-0.5 F2400
";

#[test]
fn a_crawl_valley_widens_from_the_edge_of_the_taken_path() {
    let config = printer_from(
        "widened_from_the_taken_edge_printer",
        WIDENED_FROM_THE_TAKEN_EDGE_PRINTER,
    );
    plans_within_the_extruder_limits(
        "widened_from_the_taken_edge",
        WIDENED_FROM_THE_TAKEN_EDGE,
        &config,
    );
}

/// `voron_first_layers_zhop.gcode` on the straight-line snapshot printer,
/// whose extruder has no advance: a crawl here widens the last crawl
/// valley over path the projection has taken, which nothing reads.
const PRIME_AFTER_A_HOP: &str = "G90
M83
G1 X119.827 Y125.753 Z2 F30000
G1 X124.431 Y101.534 F9000
G1 Z2.2
G1 E.8 F1500
G1 X124.414 Y101.674 E.00476
G1 X124.427 Y102.682 E.03391
";

#[test]
fn a_follower_without_an_advance_widens_crawl_valleys_over_taken_path() {
    let config = corpus::repo().join("snapshots/cases/straight_line/printer.cfg");
    corpus::plan("prime_after_a_hop", PRIME_AFTER_A_HOP, &config).expect("the print plans");
}

/// #964 case 2: lines 10955-10998 of a stock Benchy, entered by a travel
/// from rest. On the bowden profile the crawl at s=24.13 widened its valley
/// back over path the projection had taken.
const BENCHY_WALLS_AFTER_A_RETRACT: &str = "G90
G21
M83
SET_VELOCITY_LIMIT VELOCITY=1000
SET_VELOCITY_LIMIT ACCEL=100000
SET_VELOCITY_LIMIT CORNER_DEVIATION=0.02
SET_VELOCITY_LIMIT ACCEL=15000
G1 X153.9270 Y135.5090 Z6.4500 F30000
G1 X153.927 Y154.496 E.8544
G1 X155.235 Y153.884 E.06497
G1 X155.944 Y153.506 E.03614
G1 X156.826 Y152.99 E.04602
G1 X157.51 Y152.549 E.03659
G1 X158.39 Y151.923 E.04864
G1 X158.39 Y138.077 E.6231
G1 X158.974 Y138.519 E.03295
G1 X159.439 Y138.898 E.02697
G1 X160.362 Y139.716 E.05554
G1 X161.364 Y140.718 E.06374
G1 X162.174 Y141.632 E.05495
G1 X162.854 Y142.497 E.04954
G1 X162.854 Y147.84 E.24041
G1 E-.5 F3600
G1 X127.57 Y140.447 Z6.7 F42000
G1 E.5 F3600
SET_VELOCITY_LIMIT ACCEL=20000
G1 F24000
G1 X127.472 Y140.566 E.00601
G1 X127.404 Y140.839 E.01096
G1 X127.412 Y149.254 E.3278
G1 X127.533 Y149.526 E.01158
G1 X127.725 Y149.68 E.00957
G1 X127.983 Y149.741 E.01036
G1 X135.348 Y149.741 E.28687
G1 X135.639 Y149.639 E.012
G1 X135.827 Y149.425 E.01108
G1 X135.89 Y149.161 E.01055
G1 X135.887 Y140.781 E.32643
G1 X135.781 Y140.501 E.01167
G1 X135.542 Y140.308 E.01198
G1 X135.31 Y140.259 E.00921
G1 X127.983 Y140.259 E.2854
G1 X127.725 Y140.32 E.01035
G1 X127.618 Y140.389 E.00497
G1 X127.57 Y140.447 E.0029
G1 X127.878 Y140.71 F42000
G1 F24000
G1 X127.817 Y140.839 E.00556
G1 X127.817 Y149.161 E.32417
G1 X127.854 Y149.266 E.00432
G1 X127.983 Y149.328 E.00558
G1 X135.31 Y149.328 E.2854
";

#[test]
fn benchy_walls_after_a_retract_keep_the_crawl_valley_off_taken_path() {
    let config = corpus::repo().join("rust/motion-core/tests/configs/p1_bowden_ramp.cfg");
    plans_within_the_extruder_limits(
        "benchy_walls_after_a_retract",
        BENCHY_WALLS_AFTER_A_RETRACT,
        &config,
    );
}

/// #903 (guard-limits' sweep, `syn_tiny_zigzag_0.01`): a thousand 10 µm
/// travel zigzags on the bench printer, nothing extruded. Each crawl laid
/// a valley the advance never reads, and one widened the last over path
/// the projection had taken (`place_valley`'s widening check).
#[test]
fn a_travel_zigzag_lays_no_crawl_valleys() {
    const ZIGZAGS: usize = 1000;
    let mut gcode = String::from("G90\nM83\nG1 X10 Y10 F6000\nG1 X10 Y10 F6000\n");
    for step in 1..=ZIGZAGS {
        let y = if step % 2 == 1 { 10.01 } else { 10.0 };
        gcode.push_str(&format!("G1 X{:.2} Y{y}\n", 10.0 + 0.01 * step as f64));
    }
    let config = corpus::repo().join(BENCH_PRINTER);
    corpus::plan("travel_zigzag", &gcode, &config).expect("the zigzag plans");
}

/// #936: a corexy with a kernel-free advance and a 0.32 mm corner budget,
/// drawn by the print-like planner fuzz.
const BLEND_INTO_TRAVEL_PRINTER: &str = "[printer]
max_velocity: 340.52936014829606
max_accel: 59251.052612795094
corner_deviation: 0.32044791936796385
max_z_velocity: 25
max_z_accel: 100
[kinematics]
type: corexy
axis_x: x
axis_y: y
axis_z: z
a_motors: a
b_motors: b
z_motors: z
[axis x]
post_processors: shaper_x
[axis y]
post_processors: shaper_y
[axis z]
[axis e]
follows: x, y, z
motors: extruder
post_processors: pa
max_accel: 8713.664834637058
[post_processor pa]
type: linear_pressure_advance
k: 0.024137113777108306
[post_processor shaper_x]
type: smooth_mzv
frequency_hz: 43.42076982567384
[post_processor shaper_y]
type: smooth_zv
frequency_hz: 101.40184248844695
[motor a]
drive: stepper
[motor b]
drive: stepper
[motor z]
drive: stepper
[motor extruder]
drive: stepper
[extruder]
axis: e
max_extrude_only_velocity: 50
max_extrude_only_accel: 5000
";

/// #936: the blend from the 0.24 mm extrusion into the travel after it
/// ramped the ratio to −6.9e-18, a rounding retract: the segment reaching
/// it read the advance table and the next one the laid table, smoothed
/// apart around that fake step, and the advance stepped the extruder by
/// −0.00057 mm at their join.
const BLEND_INTO_TRAVEL: &str = "G90
M83
G1 Z0.600 F600
G1 X138.977 Y148.155 F30000
G1 Z0.200 F600
G1 E0.5 F2400
G1 X138.675 Y156.557 E0.32563 F11737
G1 X138.590 Y157.984 E0.08804 F6400
G1 X138.908 Y158.559 E0.02579 F8274
G1 X138.947 Y158.613 E0.00308 F6024
G1 X138.958 Y158.373 E0.01434 F11348
G1 X138.998 Y159.555 F11337
G1 X139.064 Y160.022 E0.02354 F10643
G1 X139.170 Y161.580 E0.04421 F4369
G1 X137.339 Y176.872 E1.02258 F4212
";

#[test]
fn the_advance_reads_one_ratio_where_a_blend_ramps_into_a_travel() {
    let config = printer_from("blend_into_travel_printer", BLEND_INTO_TRAVEL_PRINTER);
    plans_within_the_extruder_limits("blend_into_travel", BLEND_INTO_TRAVEL, &config);
}
