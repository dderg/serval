//! A pressure advance models the nozzle pressure the flow along the path
//! builds, so on a chain whose kernel carries the advance it advances only
//! that extrusion: a retract or prime the toolhead rests through keeps its
//! own motion, within the extrude-only caps it was planned under.

use pipeline_snapshot::waypoints::parse_gcode;
use pipeline_snapshot::{
    AxisDecl, EXTRUDER_AXIS, PostProcessorDecl, SampleSide, SnapshotParams, pipeline_snapshot,
};

const MAX_VELOCITY: f64 = 300.0;
const MAX_ACCEL: f64 = 3000.0;
const EXTRUDE_ONLY_VELOCITY: f64 = 45.0;
const EXTRUDE_ONLY_ACCEL: f64 = 2000.0;
const ACCEL_DEVIATION: f64 = 50.0;
const UNBINDING_EXTRUDER_LIMIT: f64 = 1e12;
const SMOOTH_TIME: f64 = 0.002;
const PROBES: usize = 20_000;
const TOOLHEAD_REST_MM_S: f64 = 1e-9;
/// How far two fits of the same extruder motion may read apart in velocity,
/// the fit audit's seam velocity resolution; the advance a resting extruder
/// must not take, `k·a`, is 1000 mm/s here.
const FIT_VELOCITY_ROUNDING_MM_S: f64 = 0.1;

const RETRACT_AND_PRIME_AT_REST: &str = "M83
G1 X10 Y0 E0.4 F3000
G1 E-0.8 F2700
G1 X20 Y5 F6000
G1 E0.8 F2400
G1 X30 Y5 E0.4 F3000
";

fn axis(name: &str, follows: &[&str], post_processors: &[&str]) -> AxisDecl {
    let limit = (!follows.is_empty()).then_some(UNBINDING_EXTRUDER_LIMIT);
    AxisDecl {
        name: name.into(),
        follows: follows.iter().map(|s| (*s).into()).collect(),
        motors: Vec::new(),
        post_processors: post_processors.iter().map(|s| (*s).into()).collect(),
        max_velocity: limit,
        max_accel: limit,
        instantaneous_corner_velocity: None,
    }
}

fn post_processor(name: &str, ty: &str, params: &[(&str, f64)]) -> PostProcessorDecl {
    PostProcessorDecl {
        name: name.into(),
        ty: ty.into(),
        params: params.iter().map(|(k, v)| ((*k).into(), *v)).collect(),
    }
}

fn bowden_kernel_chain(e_chain: &[&str]) -> SnapshotParams {
    SnapshotParams {
        max_velocity: MAX_VELOCITY,
        max_accel: MAX_ACCEL,
        corner: geometry::CornerBudget::Shaped(0.04),
        max_jerk: f64::INFINITY,
        max_z_velocity: None,
        max_z_accel: None,
        max_extrude_only_velocity: Some(EXTRUDE_ONLY_VELOCITY),
        max_extrude_only_accel: Some(EXTRUDE_ONLY_ACCEL),
        max_path_deviation: Some(0.005),
        max_accel_deviation: Some(ACCEL_DEVIATION),
        axis_decls: vec![
            axis("x", &[], &["shaper_x"]),
            axis("y", &[], &["shaper_y"]),
            axis("z", &[], &[]),
            axis("e", &["x", "y", "z"], e_chain),
        ],
        post_processor_decls: vec![
            post_processor("shaper_x", "smooth_mzv", &[("frequency_hz", 50.0)]),
            post_processor("shaper_y", "smooth_mzv", &[("frequency_hz", 50.0)]),
            post_processor("pa", "linear_pressure_advance", &[("k", 0.5)]),
            post_processor("st", "smooth_triangle", &[("smooth_time", SMOOTH_TIME)]),
        ],
    }
}

/// The extruder's velocity and acceleration at every probe where the
/// toolhead rests and has rested, and will rest, for a whole kernel width.
fn extruder_at_rest(e_chain: &[&str]) -> Vec<(f64, f64, f64)> {
    let waypoints = parse_gcode(
        RETRACT_AND_PRIME_AT_REST,
        &pipeline_snapshot::CartesianLimits::without_z_caps(
            MAX_VELOCITY,
            MAX_ACCEL,
            f64::INFINITY,
            geometry::CornerBudget::Shaped(0.0),
        ),
    )
    .expect("parse");
    let trajectory = pipeline_snapshot(&waypoints, bowden_kernel_chain(e_chain))
        .expect("snapshot")
        .trajectory;
    let (start, end) = (trajectory.t_start(), trajectory.t_end());
    let resting = |t: f64| {
        let t = t.clamp(start, end);
        (0..EXTRUDER_AXIS).all(|axis| {
            trajectory
                .eval_axis(axis, t, SampleSide::Right)
                .expect("leader carrier")
                .velocity
                .abs()
                <= TOOLHEAD_REST_MM_S
        })
    };
    let step = (end - start) / PROBES as f64;
    let window = (SMOOTH_TIME / step).ceil() as usize;
    let rests: Vec<bool> = (0..=PROBES)
        .map(|k| resting(start + k as f64 * step))
        .collect();
    (window..=PROBES - window)
        .filter(|&k| rests[k - window..=k + window].iter().all(|&rest| rest))
        .map(|k| {
            let t = start + k as f64 * step;
            let state = trajectory
                .eval_axis(EXTRUDER_AXIS, t, SampleSide::Right)
                .expect("extruder carrier");
            (t, state.velocity, state.acceleration)
        })
        .collect()
}

#[test]
fn a_kernel_chain_advances_neither_the_retract_nor_the_prime_the_toolhead_rests_through() {
    let unadvanced = extruder_at_rest(&["st"]);
    let advanced = extruder_at_rest(&["pa", "st"]);
    assert_eq!(advanced.len(), unadvanced.len());
    let moving = advanced
        .iter()
        .filter(|(_, velocity, _)| velocity.abs() > 1.0)
        .count();
    assert!(
        moving > PROBES / 100,
        "the retract and the prime must be probed at rest, yet only {moving} probes move e"
    );
    for (&(t, velocity, accel), &(_, plain_velocity, plain_accel)) in
        advanced.iter().zip(&unadvanced)
    {
        assert!(
            velocity.abs() <= EXTRUDE_ONLY_VELOCITY * (1.0 + 1e-6)
                && accel.abs() <= EXTRUDE_ONLY_ACCEL + ACCEL_DEVIATION,
            "at t={t} the extruder runs at {velocity} mm/s and {accel} mm/s² against the \
             extrude-only caps {EXTRUDE_ONLY_VELOCITY} mm/s and {EXTRUDE_ONLY_ACCEL} mm/s²"
        );
        assert!(
            (velocity - plain_velocity).abs() <= FIT_VELOCITY_ROUNDING_MM_S,
            "at t={t} the advance moves the resting extruder at {velocity} mm/s where the \
             unadvanced chain moves it at {plain_velocity} mm/s ({accel} against \
             {plain_accel} mm/s²)"
        );
    }
}
