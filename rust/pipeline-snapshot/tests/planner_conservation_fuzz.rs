use pipeline_snapshot::waypoints::Waypoint;
use pipeline_snapshot::{ExactTrajectory, SampleSide, SnapshotParams, pipeline_snapshot};
use proptest::prelude::*;

const AXES: usize = 4;
const E_AXIS: usize = 3;
const FIT_TOL_MM: f64 = 0.005;
const FIT_SLACK_MM: f64 = 4.0 * FIT_TOL_MM;
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

fn corner_deviation(limits: Limits) -> f64 {
    geometry::corner_deviation_from_scv(limits.square_corner_velocity, limits.max_accel)
}

fn plan(limits: Limits, points: &[Waypoint]) -> ExactTrajectory {
    let params = SnapshotParams {
        max_velocity: limits.max_velocity,
        max_accel: limits.max_accel,
        corner_deviation: corner_deviation(limits),
        max_jerk: f64::INFINITY,
        max_z_velocity: None,
        max_z_accel: None,
        max_extrude_only_velocity: None,
        max_extrude_only_accel: None,
        max_path_deviation: Some(FIT_TOL_MM),
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

/// One point of the planned toolhead path: where it is, how much path it
/// has covered since the start, and how much material the extruder has laid.
#[derive(Debug, Clone, Copy)]
struct PathSample {
    t: f64,
    xy: [f64; 2],
    path_mm: f64,
    e: f64,
}

const PATH_PANELS_PER_ROW: usize = 32;

fn row_state(
    trajectory: &ExactTrajectory,
    axis: usize,
    index: usize,
    t: f64,
    side: SampleSide,
) -> Result<pipeline_snapshot::Pvaj, TestCaseError> {
    trajectory
        .eval_row(axis, index, t, side)
        .map_err(|error| TestCaseError::fail(format!("axis {axis} row {index}: {error}")))
}

fn speed(
    trajectory: &ExactTrajectory,
    index: usize,
    t: f64,
    side: SampleSide,
) -> Result<f64, TestCaseError> {
    let mut squared = 0.0;
    for axis in 0..E_AXIS {
        squared += row_state(trajectory, axis, index, t, side)?
            .velocity
            .powi(2);
    }
    Ok(squared.sqrt())
}

const PATH_LENGTH_TOL_MM: f64 = 1e-9;
const PATH_LENGTH_MAX_DEPTH: u32 = 48;

struct RowSpeed<'a> {
    trajectory: &'a ExactTrajectory,
    index: usize,
    t1: f64,
}

impl RowSpeed<'_> {
    fn at(&self, t: f64) -> Result<f64, TestCaseError> {
        let side = if t >= self.t1 {
            SampleSide::Left
        } else {
            SampleSide::Right
        };
        speed(self.trajectory, self.index, t, side)
    }

    /// Adaptive Simpson: a panel whose halves agree with it is exact for the
    /// piecewise polynomials the rows carry, whatever phase knots it spans.
    fn path_mm(&self, a: f64, b: f64, ends: [f64; 2], depth: u32) -> Result<f64, TestCaseError> {
        let mid = 0.5 * (a + b);
        let at_mid = self.at(mid)?;
        let whole = (b - a) / 6.0 * (ends[0] + 4.0 * at_mid + ends[1]);
        let left_mid = self.at(0.5 * (a + mid))?;
        let right_mid = self.at(0.5 * (mid + b))?;
        let left = (mid - a) / 6.0 * (ends[0] + 4.0 * left_mid + at_mid);
        let right = (b - mid) / 6.0 * (at_mid + 4.0 * right_mid + ends[1]);
        if (left + right - whole).abs() <= PATH_LENGTH_TOL_MM {
            return Ok(left + right);
        }
        if depth == PATH_LENGTH_MAX_DEPTH {
            return Err(TestCaseError::fail(format!(
                "row {} path length does not converge on [{a}, {b}]",
                self.index
            )));
        }
        Ok(self.path_mm(a, mid, [ends[0], at_mid], depth + 1)?
            + self.path_mm(mid, b, [at_mid, ends[1]], depth + 1)?)
    }
}

fn walk_path(trajectory: &ExactTrajectory) -> Result<Vec<PathSample>, TestCaseError> {
    let sample = |index: usize, t: f64, side: SampleSide, path_mm: f64| {
        Ok::<_, TestCaseError>(PathSample {
            t,
            xy: [
                row_state(trajectory, 0, index, t, side)?.position,
                row_state(trajectory, 1, index, t, side)?.position,
            ],
            path_mm,
            e: row_state(trajectory, E_AXIS, index, t, side)?.position,
        })
    };
    let mut samples = Vec::new();
    let mut path_mm = 0.0;
    for (index, row) in trajectory.rows(0).iter().enumerate() {
        let row_speed = RowSpeed {
            trajectory,
            index,
            t1: row.t1,
        };
        samples.push(sample(index, row.t0, SampleSide::Right, path_mm)?);
        let h = (row.t1 - row.t0) / PATH_PANELS_PER_ROW as f64;
        for panel in 0..PATH_PANELS_PER_ROW {
            let a = row.t0 + panel as f64 * h;
            let (b, end_side) = if panel + 1 == PATH_PANELS_PER_ROW {
                (row.t1, SampleSide::Left)
            } else {
                (a + h, SampleSide::Right)
            };
            path_mm += row_speed.path_mm(a, b, [row_speed.at(a)?, row_speed.at(b)?], 0)?;
            samples.push(sample(index, b, end_side, path_mm)?);
        }
    }
    Ok(samples)
}

/// The extruder's own fit tolerance: the path deviation scaled by the
/// largest extrusion ratio, clamped the way the planner clamps it.
fn extruder_fit_tol_mm(steps: &[Step]) -> f64 {
    let largest_ratio = steps.iter().map(|s| s.extrude_ratio).fold(0.0, f64::max);
    FIT_TOL_MM * largest_ratio.clamp(0.01, 1.0)
}

fn distance_to_polyline(p: [f64; 2], points: &[Waypoint]) -> f64 {
    points
        .windows(2)
        .map(|w| {
            let a = [w[0].position[0], w[0].position[1]];
            let b = [w[1].position[0], w[1].position[1]];
            let d = [b[0] - a[0], b[1] - a[1]];
            let len_sq = d[0] * d[0] + d[1] * d[1];
            let along = if len_sq > 0.0 {
                (((p[0] - a[0]) * d[0] + (p[1] - a[1]) * d[1]) / len_sq).clamp(0.0, 1.0)
            } else {
                0.0
            };
            libm::hypot(p[0] - a[0] - along * d[0], p[1] - a[1] - along * d[1])
        })
        .fold(f64::INFINITY, f64::min)
}

fn uniform_ratio(steps: Vec<Step>, ratio: f64) -> Vec<Step> {
    steps
        .into_iter()
        .map(|step| Step {
            extrude_ratio: ratio,
            ..step
        })
        .collect()
}

/// The design: the extruder lays `ratio` per mm of the path the toolhead
/// actually travels, not per mm of G-code — a corner blend that cuts the
/// corner lays less than the G-code commands, a path longer than its
/// polyline lays more.
fn extruder_lays_its_ratio_along_the_path(
    limits: Limits,
    steps: &[Step],
) -> Result<(), TestCaseError> {
    let ratio = steps[0].extrude_ratio;
    let trajectory = plan(limits, &waypoints(steps));
    let tol = extruder_fit_tol_mm(steps);
    for sample in walk_path(&trajectory)? {
        let laid = ratio * sample.path_mm;
        prop_assert!(
            (sample.e - laid).abs() <= tol,
            "at t={} the extruder is at {}, {ratio} along {} mm of path lays {laid}",
            sample.t,
            sample.e,
            sample.path_mm
        );
    }
    Ok(())
}

/// With mixed ratios a blend ramps between its neighbors' ratios, so between
/// any two instants the material laid per mm of path travelled stays within
/// the commanded ratios — which also rules out the extruder running
/// backwards when nothing commands a retract.
fn material_per_mm_of_path_stays_within_the_ratios(
    limits: Limits,
    steps: &[Step],
) -> Result<(), TestCaseError> {
    let trajectory = plan(limits, &waypoints(steps));
    let least = steps
        .iter()
        .map(|s| s.extrude_ratio)
        .fold(f64::INFINITY, f64::min);
    let most = steps.iter().map(|s| s.extrude_ratio).fold(0.0, f64::max);
    let tol = 2.0 * extruder_fit_tol_mm(steps);
    let mut least_lead = f64::INFINITY;
    let mut most_lag = f64::NEG_INFINITY;
    for sample in walk_path(&trajectory)? {
        let over_most = sample.e - most * sample.path_mm;
        let over_least = sample.e - least * sample.path_mm;
        prop_assert!(
            over_most <= least_lead + tol,
            "by t={} the extruder laid more than {most} per mm of path",
            sample.t
        );
        prop_assert!(
            over_least >= most_lag - tol,
            "by t={} the extruder laid less than {least} per mm of path",
            sample.t
        );
        least_lead = least_lead.min(over_most);
        most_lag = most_lag.max(over_least);
    }
    Ok(())
}

/// Corner blends and arc runs stay within `corner_deviation` of the G-code
/// polyline and the lowering within the fit tolerance of the fitted path. The
/// waypoints' bounding box is no bound: a blend between two short moves may
/// leave it by anything up to the corner budget.
fn toolhead_stays_within_the_corner_budget(
    limits: Limits,
    steps: &[Step],
) -> Result<(), TestCaseError> {
    let points = waypoints(steps);
    let trajectory = plan(limits, &points);
    let budget = corner_deviation(limits) + FIT_TOL_MM;
    for sample in walk_path(&trajectory)? {
        let off = distance_to_polyline(sample.xy, &points);
        prop_assert!(
            off <= budget,
            "at t={} the toolhead is {off} mm off the polyline, budget {budget}",
            sample.t
        );
    }
    Ok(())
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
    fn the_extruder_lays_its_ratio_along_the_path_the_toolhead_travels(
        limits in arb_limits(),
        steps in prop::collection::vec(arb_step(), 1..30),
        ratio in 0.0..0.1_f64,
    ) {
        extruder_lays_its_ratio_along_the_path(limits, &uniform_ratio(steps, ratio))?;
    }

    #[test]
    fn extruded_material_per_mm_of_path_stays_within_the_commanded_ratios(
        limits in arb_limits(),
        steps in prop::collection::vec(arb_step(), 1..30),
    ) {
        material_per_mm_of_path_stays_within_the_ratios(limits, &steps)?;
    }

    #[test]
    fn the_toolhead_stays_within_the_corner_budget_of_its_polyline(
        limits in arb_limits(),
        steps in prop::collection::vec(arb_step(), 1..30),
    ) {
        toolhead_stays_within_the_corner_budget(limits, &steps)?;
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

    #[test]
    fn every_axis_is_continuous_across_every_row_seam_and_hold_gap(
        limits in arb_limits(),
        steps in prop::collection::vec(arb_step(), 1..30),
    ) {
        let points = waypoints(&steps);
        let trajectory = plan(limits, &points);
        for axis in 0..AXES {
            let rows = trajectory.rows(axis);
            for (index, pair) in rows.windows(2).enumerate() {
                let before = trajectory
                    .eval_row(axis, index, pair[0].t1, SampleSide::Left)
                    .map_err(|error| TestCaseError::fail(format!("axis {axis} row {index}: {error}")))?;
                let after = trajectory
                    .eval_row(axis, index + 1, pair[1].t0, SampleSide::Right)
                    .map_err(|error| TestCaseError::fail(format!("axis {axis} row {}: {error}", index + 1)))?;
                prop_assert!(
                    (after.position - before.position).abs() <= FIT_SLACK_MM,
                    "axis {axis} jumps {} mm between row {index} ending t={} and row {} starting t={}",
                    after.position - before.position,
                    pair[0].t1,
                    index + 1,
                    pair[1].t0
                );
            }
        }
    }
}

fn step(heading: f64, length_mm: f64, feed_mm_s: f64, extrude_ratio: f64) -> Step {
    Step {
        heading,
        length_mm,
        feed_mm_s,
        extrude_ratio,
        dwell_before_s: None,
    }
}

/// #549: ten laps of a 1 mm square under a 0.21 mm corner budget. The
/// blended loop is about a tenth longer than the G-code's, so the extruder
/// ends that far past the commanded E — by design, it lays the ratio along
/// the path the nozzle travels.
#[test]
fn a_square_loop_lays_its_ratio_along_a_path_longer_than_its_gcode() {
    let limits = Limits {
        max_velocity: 300.0,
        max_accel: 500.0,
        square_corner_velocity: 16.0,
    };
    let steps: Vec<Step> = (0..40)
        .map(|i| step(f64::from(i) * std::f64::consts::FRAC_PI_2, 1.0, 100.0, 0.05))
        .collect();
    extruder_lays_its_ratio_along_the_path(limits, &steps).unwrap();
}

/// #549: a blended corner between two ratios ends short of the commanded E
/// by about the material the cut corner would have carried, laying between
/// the two ratios per mm of the path it travels.
#[test]
fn a_cut_corner_between_two_ratios_lays_between_them() {
    let limits = Limits {
        max_velocity: 450.71,
        max_accel: 500.0,
        square_corner_velocity: 18.2536,
    };
    let steps = [
        step(0.0, 43.8259, 1.0, 0.097627),
        step(4.743415, 53.5629, 1.0, 0.0771263),
    ];
    material_per_mm_of_path_stays_within_the_ratios(limits, &steps).unwrap();
}

/// #550: a 0.31 mm corner budget around 1 mm moves. The blends leave the
/// waypoints' bounding box by up to 51 µm, inside the budget they may spend.
#[test]
fn blends_between_short_moves_stay_within_the_corner_budget() {
    let limits = Limits {
        max_velocity: 486.231,
        max_accel: 500.0,
        square_corner_velocity: 19.3652,
    };
    let steps = [
        step(3.596239734616089, 1.0, 225.7938525874038, 0.05),
        step(1.6143925280647562, 1.0, 1.0, 0.05),
        step(0.8337332662322786, 1.0, 1.0, 0.05),
        step(0.0, 1.0, 1.0, 0.05),
    ];
    toolhead_stays_within_the_corner_budget(limits, &steps).unwrap();
}

/// #550: an arc run of three short facets after a 60° corner. Its head
/// spiral eased the long line into the run's circle across that corner,
/// cutting it 0.14 mm deep under a 0.032 mm budget: the easing refit only
/// held the run's interior vertices to the budget. An eased attempt that
/// strays past the budget from the polyline it replaces is now rejected.
#[test]
fn an_eased_arc_run_keeps_the_corner_budget_at_its_head() {
    let limits = Limits {
        max_velocity: 166.17838514016015,
        max_accel: 3078.0393014472274,
        square_corner_velocity: 15.329776316075177,
    };
    let steps = [
        step(
            5.031109235008767,
            37.564376486678306,
            1.0,
            0.059945549338687856,
        ),
        step(
            6.080414159127166,
            0.35620099389938975,
            1.0,
            0.05112626556293348,
        ),
        step(
            6.241205568117417,
            0.025574446227275976,
            1.0,
            0.06112902090337584,
        ),
        step(0.0, 0.0020136098647930614, 1.0, 0.051993597253804556),
    ];
    toolhead_stays_within_the_corner_budget(limits, &steps).unwrap();
}

/// Found once every follower ramps its ratio: a fast 1 mm line and a
/// 4.6 µm move nearly reversing off it, at one ratio. The follower rides
/// the leaders' corner, where their speed dips sharply, and its position
/// must follow its velocity there for the source fit to close.
#[test]
fn a_near_reversal_off_a_fast_line_lays_its_ratio_along_the_path() {
    let limits = Limits {
        max_velocity: 477.1372504755238,
        max_accel: 46370.99632569863,
        square_corner_velocity: 13.517812626216859,
    };
    let steps = [
        step(std::f64::consts::PI, 1.0, 593.262836305538, 0.06452893755225318),
        step(
            0.1662771977788236,
            0.004608638732611155,
            278.9544117803534,
            0.06452893755225318,
        ),
    ];
    extruder_lays_its_ratio_along_the_path(limits, &steps).unwrap();
}
