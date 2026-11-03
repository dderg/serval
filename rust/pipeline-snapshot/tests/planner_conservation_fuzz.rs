use pipeline_snapshot::waypoints::Waypoint;
use pipeline_snapshot::{
    ExactTrajectory, SampleSide, SnapshotParams, TRAJECTORY_FIT_TOL_MM, pipeline_snapshot,
};
use proptest::prelude::*;

const AXES: usize = 4;
const E_AXIS: usize = 3;
const FIT_SLACK_MM: f64 = 4.0 * TRAJECTORY_FIT_TOL_MM;
const START: [f64; AXES] = [150.0, 150.0, 0.2, 0.0];

#[derive(Debug, Clone, Copy)]
struct Limits {
    max_velocity: f64,
    max_accel: f64,
    square_corner_velocity: f64,
}

#[derive(Debug, Clone, Copy)]
struct Step {
    heading: f64,
    length_mm: f64,
    feed_mm_s: f64,
    extrude_ratio: f64,
    dwell_before_s: Option<f64>,
}

fn arb_limits() -> impl Strategy<Value = Limits> {
    let corner = prop_oneof![1 => Just(0.0), 9 => 0.5..20.0_f64];
    (20.0..600.0_f64, 500.0..50_000.0_f64, corner).prop_map(|(v, a, scv)| Limits {
        max_velocity: v,
        max_accel: a,
        square_corner_velocity: scv.min(v / 4.0),
    })
}

fn arb_step() -> impl Strategy<Value = Step> {
    let heading = prop_oneof![
        2 => Just(0.0),
        4 => 0.0..std::f64::consts::TAU,
        1 => Just(std::f64::consts::PI),
    ];
    let dwell = prop_oneof![8 => Just(None), 1 => (0.001..0.5_f64).prop_map(Some)];
    (heading, -3.5..1.9_f64, 1.0..700.0_f64, 0.0..0.1_f64, dwell).prop_map(
        |(heading, log_len, feed_mm_s, extrude_ratio, dwell_before_s)| Step {
            heading,
            length_mm: libm::pow(10.0, log_len),
            feed_mm_s,
            extrude_ratio,
            dwell_before_s,
        },
    )
}

fn waypoints(steps: &[Step]) -> Vec<Waypoint> {
    let mut position = START;
    let mut points = vec![Waypoint::at(position, 0.0)];
    for step in steps {
        position[0] += step.length_mm * libm::cos(step.heading);
        position[1] += step.length_mm * libm::sin(step.heading);
        position[E_AXIS] += step.extrude_ratio * step.length_mm;
        let mut point = Waypoint::at(position, step.feed_mm_s);
        point.dwell_before_s = step.dwell_before_s;
        points.push(point);
    }
    points
}

fn plan(limits: Limits, points: &[Waypoint]) -> ExactTrajectory {
    let params = SnapshotParams {
        max_velocity: limits.max_velocity,
        max_accel: limits.max_accel,
        corner_deviation: geometry::corner_deviation_from_scv(
            limits.square_corner_velocity,
            limits.max_accel,
        ),
        max_jerk: f64::INFINITY,
        max_z_velocity: None,
        max_z_accel: None,
        max_extrude_only_velocity: None,
        max_extrude_only_accel: None,
        max_path_deviation: None,
        max_accel_deviation: None,
        axis_decls: Vec::new(),
        post_processor_decls: Vec::new(),
    };
    pipeline_snapshot(points, params)
        .expect("valid fuzz input")
        .trajectory
}

fn position(
    trajectory: &ExactTrajectory,
    axis: usize,
    t: f64,
    side: SampleSide,
) -> Result<f64, TestCaseError> {
    trajectory
        .eval_axis(axis, t, side)
        .map(|state| state.position)
        .map_err(|error| TestCaseError::fail(format!("axis {axis} at t={t}: {error}")))
}

fn sampled_positions(
    trajectory: &ExactTrajectory,
    axis: usize,
) -> Result<Vec<(f64, f64)>, TestCaseError> {
    let mut samples = Vec::new();
    for (index, row) in trajectory.rows(axis).iter().enumerate() {
        let inside = [
            (row.t0, SampleSide::Right),
            (0.5 * (row.t0 + row.t1), SampleSide::Right),
            (row.t1, SampleSide::Left),
        ];
        for (t, side) in inside {
            let state = trajectory.eval_row(axis, index, t, side).map_err(|error| {
                TestCaseError::fail(format!("axis {axis} row {index}: {error}"))
            })?;
            samples.push((t, state.position));
        }
    }
    Ok(samples)
}

proptest! {
    #[test]
    fn the_toolhead_starts_and_ends_on_the_commanded_waypoints(
        limits in arb_limits(),
        steps in prop::collection::vec(arb_step(), 1..30),
    ) {
        let points = waypoints(&steps);
        let trajectory = plan(limits, &points);
        let last = points.last().expect("at least the start waypoint").position;
        for axis in 0..E_AXIS {
            let start = position(&trajectory, axis, trajectory.t_start(), SampleSide::Right)?;
            let end = position(&trajectory, axis, trajectory.t_end(), SampleSide::Left)?;
            prop_assert!(
                (start - START[axis]).abs() <= FIT_SLACK_MM,
                "axis {axis} starts at {start}, commanded {}", START[axis]
            );
            prop_assert!(
                (end - last[axis]).abs() <= FIT_SLACK_MM,
                "axis {axis} ends at {end}, commanded {}", last[axis]
            );
        }
    }

    #[test]
    #[ignore = "#549: corner blends under-extrude"]
    fn the_extruder_ends_on_the_commanded_material(
        limits in arb_limits(),
        steps in prop::collection::vec(arb_step(), 1..30),
    ) {
        let points = waypoints(&steps);
        let trajectory = plan(limits, &points);
        let commanded = points.last().expect("at least the start waypoint").position[E_AXIS];
        let end = position(&trajectory, E_AXIS, trajectory.t_end(), SampleSide::Left)?;
        prop_assert!(
            (end - commanded).abs() <= FIT_SLACK_MM,
            "extruder ends at {end}, commanded {commanded}"
        );
    }

    #[test]
    #[ignore = "#549: corner blends over-extrude mid-path"]
    fn extruded_material_never_runs_backwards_and_never_exceeds_the_command(
        limits in arb_limits(),
        steps in prop::collection::vec(arb_step(), 1..30),
    ) {
        let points = waypoints(&steps);
        let trajectory = plan(limits, &points);
        let commanded = points.last().expect("at least the start waypoint").position[E_AXIS];
        let mut previous = f64::NEG_INFINITY;
        for (t, e) in sampled_positions(&trajectory, E_AXIS)? {
            prop_assert!(
                e >= previous - FIT_SLACK_MM,
                "extruder runs backwards at t={t}: {previous} then {e}"
            );
            prop_assert!(
                e <= commanded + FIT_SLACK_MM,
                "extruder at t={t} is {e}, past the commanded {commanded}"
            );
            previous = e;
        }
    }

    #[test]
    #[ignore = "#550: blend overshoots the waypoint hull"]
    fn the_toolhead_never_leaves_the_bounding_box_of_its_waypoints(
        limits in arb_limits(),
        steps in prop::collection::vec(arb_step(), 1..30),
    ) {
        let points = waypoints(&steps);
        let trajectory = plan(limits, &points);
        for axis in 0..2 {
            let lo = points.iter().map(|p| p.position[axis]).fold(f64::INFINITY, f64::min);
            let hi = points.iter().map(|p| p.position[axis]).fold(f64::NEG_INFINITY, f64::max);
            for (t, x) in sampled_positions(&trajectory, axis)? {
                prop_assert!(
                    x >= lo - FIT_SLACK_MM && x <= hi + FIT_SLACK_MM,
                    "axis {axis} at t={t} is {x}, outside [{lo}, {hi}]"
                );
            }
        }
    }

    #[test]
    fn the_trajectory_is_slower_than_the_velocity_limit_allows_end_to_end(
        limits in arb_limits(),
        steps in prop::collection::vec(arb_step(), 1..30),
    ) {
        let points = waypoints(&steps);
        let trajectory = plan(limits, &points);
        let first = points[0].position;
        let last = points[points.len() - 1].position;
        let chord = libm::hypot(last[0] - first[0], last[1] - first[1]);
        let duration = trajectory.t_end() - trajectory.t_start();
        prop_assert!(
            duration * limits.max_velocity >= chord - FIT_SLACK_MM,
            "{chord} mm in {duration} s outruns max_velocity {}", limits.max_velocity
        );
    }
}
