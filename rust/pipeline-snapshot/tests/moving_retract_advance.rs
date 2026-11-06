//! A retract the toolhead makes while it moves — a negative extrusion ratio
//! along the path, a wipe — is a retract, and a pressure advance leaves it
//! alone as Klipper does: the extruder retracts at `r·v` with no `k·r·a`
//! lead. Over unshaped leaders, whose acceleration steps a kernel-free
//! advance turns into extruder velocity steps, advancing it stepped the
//! extruder by `k·|r|·Δa` at each of its trapezoid's corners, far past the
//! instantaneous corner velocity, and the shaper refused it.

use pipeline_snapshot::waypoints::parse_gcode;
use pipeline_snapshot::{
    AxisDecl, EXTRUDER_AXIS, ExactTrajectory, PostProcessorDecl, SampleSide, SnapshotParams,
    pipeline_snapshot,
};

const MAX_VELOCITY: f64 = 300.0;
const MAX_ACCEL: f64 = 3000.0;
const EXTRUDER_MAX_VELOCITY: f64 = 120.0;
const EXTRUDER_MAX_ACCEL: f64 = 20_000.0;
const ADVANCE_K: f64 = 0.025;
const INSTANTANEOUS_CORNER_VELOCITY: f64 = 1.0;
const SAMPLE_DT_S: f64 = 1e-5;
const MOVING_MM_S: f64 = 0.5;
/// How far two fits of the same extruder motion may read apart in
/// velocity; the lead an advanced retract takes, `k·|r|·a`, is tens of
/// millimetres per second here.
const FIT_VELOCITY_ROUNDING_MM_S: f64 = 0.1;
/// What a sample's own acceleration adds to the velocity change across it.
const SAMPLE_VELOCITY_CHANGE_MM_S: f64 = 0.1;

/// `tools/sim/tests/test_unlimited_jerk.py`'s
/// `test_extruding_moves_resume_across_host_round_trips` from X20 Y20:
/// each `M400` a rest, then a moving retract at ratio −7.5/√50 ≈ −1.06.
const RETRACT_AFTER_ROUND_TRIPS: &str = "G90
M83
G1 X20 Y20 F6000
G4 P100
G1 X25 Y25 E7.5 F1500
G4 P100
G1 X30 Y30 E0.5
G1 X25 Y25
G4 P100
G1 X20 Y20 E-7.5
";
const RETRACT_AFTER_ROUND_TRIPS_RATIO: f64 = -1.060_660_171_779_821_2;

/// An extrusion at ratio 0.05 straight into a wipe retracting at −0.05
/// over the next 30 mm of the line, then a travel on.
const EXTRUSION_INTO_WIPE: &str = "G90
M83
G1 X0 Y0 F6000
G1 X30 Y0 E1.5 F6000
G1 X60 Y0 E-1.5
G1 X80 Y0
";
const WIPE_RATIO: f64 = -0.05;
/// The wipe's stretch the ratio ramp reaches no further into than half the
/// path the leaders cover in its longest ramp, 30 ms at 100 mm/s.
const WIPE_INTERIOR_X: (f64, f64) = (35.0, 55.0);

fn axis(name: &str, follows: &[&str], post_processors: &[&str]) -> AxisDecl {
    let follower = !follows.is_empty();
    AxisDecl {
        name: name.into(),
        follows: follows.iter().map(|s| (*s).into()).collect(),
        motors: Vec::new(),
        post_processors: post_processors.iter().map(|s| (*s).into()).collect(),
        max_velocity: follower.then_some(EXTRUDER_MAX_VELOCITY),
        max_accel: follower.then_some(EXTRUDER_MAX_ACCEL),
        instantaneous_corner_velocity: follower.then_some(INSTANTANEOUS_CORNER_VELOCITY),
    }
}

/// The sim's `heaters_config` with `max_jerk: 0`: unshaped x and y, and a
/// kernel-free linear advance on e.
fn unshaped_kernel_free_advance() -> SnapshotParams {
    SnapshotParams {
        max_velocity: MAX_VELOCITY,
        max_accel: MAX_ACCEL,
        corner: geometry::CornerBudget::Shaped(0.04),
        max_jerk: f64::INFINITY,
        max_z_velocity: None,
        max_z_accel: None,
        max_extrude_only_velocity: Some(EXTRUDER_MAX_VELOCITY),
        max_extrude_only_accel: Some(EXTRUDER_MAX_ACCEL),
        max_path_deviation: Some(0.005),
        max_accel_deviation: Some(50.0),
        axis_decls: vec![
            axis("x", &[], &[]),
            axis("y", &[], &[]),
            axis("z", &[], &[]),
            axis("e", &["x", "y", "z"], &["pa"]),
        ],
        post_processor_decls: vec![PostProcessorDecl {
            name: "pa".into(),
            ty: "linear_pressure_advance".into(),
            params: vec![("k".into(), ADVANCE_K)],
        }],
    }
}

fn plan(gcode: &str) -> ExactTrajectory {
    let waypoints = parse_gcode(
        gcode,
        &pipeline_snapshot::CartesianLimits::without_z_caps(
            MAX_VELOCITY,
            MAX_ACCEL,
            f64::INFINITY,
            geometry::CornerBudget::Shaped(0.04),
        ),
    )
    .expect("parse");
    pipeline_snapshot(&waypoints, unshaped_kernel_free_advance())
        .expect("snapshot")
        .trajectory
}

/// One probe of the toolhead and the extruder.
struct Probe {
    t: f64,
    x: f64,
    speed: f64,
    extruder_velocity: f64,
}

/// The probes of every stretch the trajectory covers, a dwell's gap
/// between two.
fn probe_runs(trajectory: &ExactTrajectory) -> Vec<Vec<Probe>> {
    let (start, end) = (trajectory.t_start(), trajectory.t_end());
    let count = ((end - start) / SAMPLE_DT_S) as usize;
    let mut runs: Vec<Vec<Probe>> = vec![Vec::new()];
    for k in 0..=count {
        let t = start + k as f64 * SAMPLE_DT_S;
        let states: Result<Vec<_>, _> = (0..=EXTRUDER_AXIS)
            .map(|axis| trajectory.eval_axis(axis, t, SampleSide::Right))
            .collect();
        let Ok(states) = states else {
            if runs.last().is_some_and(|run| !run.is_empty()) {
                runs.push(Vec::new());
            }
            continue;
        };
        runs.last_mut().expect("a run is open").push(Probe {
            t,
            x: states[0].position,
            speed: states[..EXTRUDER_AXIS]
                .iter()
                .map(|state| state.velocity.powi(2))
                .sum::<f64>()
                .sqrt(),
            extruder_velocity: states[EXTRUDER_AXIS].velocity,
        });
    }
    runs.retain(|run| !run.is_empty());
    runs
}

fn assert_unadvanced(probe: &Probe, ratio: f64) {
    let unadvanced = ratio * probe.speed;
    assert!(
        (probe.extruder_velocity - unadvanced).abs() <= FIT_VELOCITY_ROUNDING_MM_S,
        "at t={} the toolhead retracts along the path at {} mm/s, yet the extruder runs at {} \
         mm/s: the advance leads a retract",
        probe.t,
        unadvanced,
        probe.extruder_velocity
    );
}

fn assert_no_velocity_step(runs: &[Vec<Probe>]) {
    for pair in runs.iter().flat_map(|run| run.windows(2)) {
        let change = (pair[1].extruder_velocity - pair[0].extruder_velocity).abs();
        assert!(
            change <= INSTANTANEOUS_CORNER_VELOCITY + SAMPLE_VELOCITY_CHANGE_MM_S,
            "the extruder velocity steps by {change} mm/s at t={}, past the \
             {INSTANTANEOUS_CORNER_VELOCITY} mm/s instantaneous corner velocity",
            pair[1].t
        );
    }
}

#[test]
fn a_moving_retract_after_a_rest_is_not_advanced_over_unshaped_leaders() {
    let runs = probe_runs(&plan(RETRACT_AFTER_ROUND_TRIPS));
    let last = runs.last().expect("the plan covers some time");
    let retract: Vec<&Probe> = last
        .iter()
        .filter(|probe| probe.speed > MOVING_MM_S)
        .collect();
    assert!(
        retract.len() > 10_000,
        "the retract must be probed through its trapezoid, yet only {} probes move",
        retract.len()
    );
    for probe in retract {
        assert_unadvanced(probe, RETRACT_AFTER_ROUND_TRIPS_RATIO);
    }
    assert_no_velocity_step(&runs);
}

#[test]
fn an_extrusion_into_a_wipe_keeps_the_extruder_continuous_and_leaves_the_wipe_unadvanced() {
    let runs = probe_runs(&plan(EXTRUSION_INTO_WIPE));
    let interior: Vec<&Probe> = runs
        .iter()
        .flatten()
        .filter(|probe| (WIPE_INTERIOR_X.0..=WIPE_INTERIOR_X.1).contains(&probe.x))
        .collect();
    assert!(
        interior.len() > 1000,
        "the wipe's interior must be probed, yet only {} probes lie in it",
        interior.len()
    );
    for probe in interior {
        assert_unadvanced(probe, WIPE_RATIO);
    }
    assert_no_velocity_step(&runs);
}
