use planner_config::from_doc::default_extrude_ratio;

use super::backoff_stream_tests::{UnsoundPlanner, line};

const ADVANCE_K: f64 = 0.03;
const NOZZLE_MM: f64 = 0.4;
const FILAMENT_MM: f64 = 1.75;

/// The bench printer's motion (`tools/bench/printer.cfg`): a kernel-free
/// advance behind smooth X/Y shapers, `[axis e] max_accel` left to fill,
/// and an `[extruder]` naming its nozzle and filament where `diameters`.
fn bench(e_max_accel: f64, diameters: bool) -> String {
    let diameters = if diameters {
        format!("nozzle_diameter: {NOZZLE_MM}\nfilament_diameter: {FILAMENT_MM}\n")
    } else {
        String::new()
    };
    format!(
        "[printer]\nmax_velocity: 1000\nmax_accel: 10000\nmax_jerk: 0\ncorner_deviation: 0.21\n\
         max_z_velocity: 25\nmax_z_accel: 100\n\
         [kinematics]\ntype: corexy\naxis_x: x\naxis_y: y\naxis_z: z\na_motors: a\nb_motors: b\n\
         z_motors: z\n\
         [axis x]\npost_processors: shaper_x\n[axis y]\npost_processors: shaper_y\n[axis z]\n\
         [axis e]\nfollows: x, y, z\nmotors: extruder\npost_processors: pa\n\
         max_accel: {e_max_accel}\n\
         [post_processor pa]\ntype: linear_pressure_advance\nk: {ADVANCE_K}\n\
         [post_processor shaper_x]\ntype: smooth_mzv\nfrequency_hz: 50\n\
         [post_processor shaper_y]\ntype: smooth_zv\nfrequency_hz: 44\n\
         [motor a]\ndrive: stepper\n[motor b]\ndrive: stepper\n[motor z]\ndrive: stepper\n\
         [motor extruder]\ndrive: stepper\n\
         [extruder]\naxis: e\n{diameters}max_extrude_only_velocity: 50\n\
         max_extrude_only_accel: 5000\n"
    )
}

fn largest_ratio() -> f64 {
    default_extrude_ratio(NOZZLE_MM, FILAMENT_MM)
}

/// A 0.45 × 0.2 mm bead's extrusion ratio on the bench's filament.
fn printing_ratio() -> f64 {
    0.45 * 0.2 / (std::f64::consts::PI * 0.25 * FILAMENT_MM * FILAMENT_MM)
}

/// The floor the config holds the bench's `[axis e] max_accel` to.
fn floor() -> f64 {
    ADVANCE_K * largest_ratio() * geometry::valley_speed_crawl_rebuild(0.005, 50.0)
}

/// A 50 mm extrusion at `ratio` at the valley speed between two travels,
/// through the guard.
fn crawl(ratio: f64, e_max_accel: f64, diameters: bool) {
    let planner = UnsoundPlanner::new(&bench(e_max_accel, diameters), 1.0);
    let lines = [
        line(1, [0.0, 0.0], [10.0, 10.0], 0.0, 200.0),
        line(2, [10.0, 10.0], [60.0, 10.0], 50.0 * ratio, 2.0),
        line(3, [60.0, 10.0], [60.0, 60.0], 0.0, 200.0),
    ];
    planner.guard(planner.plan(&lines));
}

#[test]
fn the_guard_plans_a_crawl_at_a_printing_ratio_at_the_floor() {
    crawl(printing_ratio(), floor(), true);
}

#[test]
#[should_panic(expected = "turning guard: the cell at")]
fn the_guard_stops_a_crawl_at_the_largest_ratio_under_the_floor() {
    crawl(largest_ratio(), 0.85 * floor(), false);
}
