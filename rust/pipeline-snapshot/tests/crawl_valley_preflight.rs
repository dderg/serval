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
    std::fs::write(&config, text.replacen(BENCH_CORNER_DEVIATION, corner, 1))
        .expect("temp config");
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
    plans_within_the_extruder_limits("crawl_waiting_past_a_hop", CRAWL_WAITING_PAST_A_HOP, &config);
}

/// #897: a corexy with a kernel-free advance limited to 6359 mm/s² and
/// `square_corner_velocity: 0`, drawn by the print-like planner fuzz.
const FAR_STEPS_PRINTER: &str = "[printer]
max_velocity: 386.46480970262087
max_accel: 51666.60727129722
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
max_accel: 6358.545634714979
[post_processor pa]
type: linear_pressure_advance
k: 0.025743938421182125
[post_processor shaper_x]
type: smooth_mzv
frequency_hz: 105.76857975056504
[post_processor shaper_y]
type: smooth_zv
frequency_hz: 80.640952705959
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

/// #897: a reversal's crawl valley owns the ratio steps 0.9 and 2.5 mm
/// after it, where the leaders may cross at ~206 mm/s; capped to the
/// crawl's 0.24 mm step window their flow asked 6734 mm/s².
const STEPS_FAR_PAST_A_REVERSAL: &str = "G90
M83
G1 X148.223 Y158.619 F10632
G1 X150.159 Y151.491 E0.41688 F13620
G1 X150.794 Y146.569 E0.23578 F11562
G1 X155.997 Y145.112 E0.14745 F7538
G1 X155.077 Y145.258 E0.02331 F4309
G1 X153.500 Y145.541 E0.06676 F6878
G1 X145.159 Y142.668 F14889
";

#[test]
fn a_crawl_valley_widens_far_owned_steps_to_fit_their_flow() {
    let config =
        std::env::temp_dir().join(format!("far_steps_printer_{}.cfg", std::process::id()));
    std::fs::write(&config, FAR_STEPS_PRINTER).expect("temp config");
    plans_within_the_extruder_limits("steps_far_past_a_reversal", STEPS_FAR_PAST_A_REVERSAL, &config);
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
    plans_within_the_extruder_limits("sliver_legs_into_a_crawl", SLIVER_LEGS_INTO_A_CRAWL, &config);
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
    plans_within_the_extruder_limits("bowden_hop_prime_and_back", BOWDEN_HOP_PRIME_AND_BACK, &config);
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
    let config =
        std::env::temp_dir().join(format!("two_reversals_printer_{}.cfg", std::process::id()));
    std::fs::write(&config, TWO_REVERSALS_PRINTER).expect("temp config");
    plans_within_the_extruder_limits("two_reversals", TWO_REVERSALS, &config);
}
