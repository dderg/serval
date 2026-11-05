mod corpus;

use std::path::Path;

use pipeline_snapshot::waypoints::Waypoint;
use pipeline_snapshot::{ExactTrajectory, SampleSide};

const E_AXIS: usize = 3;
const STOP_SPEED_MM_S: f64 = 1.0;
const PROBE_S: f64 = 1e-3;
const BALL_EDGE_BISECTIONS: usize = 48;
const REST_MM_S: f64 = 1e-9;

#[derive(Debug, Clone, Copy)]
struct RestRun {
    waypoint: usize,
    at: [f64; 3],
    net_mm: f64,
    travel_mm: f64,
}

fn rest_runs(waypoints: &[Waypoint]) -> Vec<RestRun> {
    let mut runs: Vec<RestRun> = Vec::new();
    let mut extending = false;
    for (index, pair) in waypoints.windows(2).enumerate() {
        let [from, to] = [pair[0].position, pair[1].position];
        let at = [from[0], from[1], from[2]];
        let de = to[E_AXIS] - from[E_AXIS];
        if [to[0], to[1], to[2]] != at {
            extending = false;
            continue;
        }
        if de == 0.0 {
            continue;
        }
        match runs.last_mut() {
            Some(run) if extending => {
                run.net_mm += de;
                run.travel_mm += de.abs();
            }
            _ => runs.push(RestRun {
                waypoint: index + 1,
                at,
                net_mm: de,
                travel_mm: de.abs(),
            }),
        }
        extending = true;
    }
    runs
}

fn state(trajectory: &ExactTrajectory, axis: usize, t: f64) -> pipeline_snapshot::Pvaj {
    trajectory
        .eval_axis(axis, t, SampleSide::Right)
        .or_else(|_| trajectory.eval_axis(axis, t, SampleSide::Left))
        .unwrap_or_else(|e| panic!("axis {axis} at t={t} is not evaluable: {e}"))
}

fn distance(trajectory: &ExactTrajectory, t: f64, at: [f64; 3]) -> f64 {
    (0..E_AXIS)
        .map(|axis| state(trajectory, axis, t).position - at[axis])
        .fold(0.0, |sum: f64, d| d.mul_add(d, sum))
        .sqrt()
}

fn speed(trajectory: &ExactTrajectory, t: f64) -> f64 {
    (0..E_AXIS)
        .map(|axis| state(trajectory, axis, t).velocity)
        .fold(0.0, |sum: f64, v| v.mul_add(v, sum))
        .sqrt()
}

/// The trajectory's breakpoints, each interval split to probes at most
/// [`PROBE_S`] apart.
fn probe_grid(trajectory: &ExactTrajectory) -> Vec<f64> {
    let breakpoints = trajectory.breakpoints();
    let mut grid = Vec::with_capacity(breakpoints.len());
    for interval in breakpoints.windows(2) {
        let (t0, t1) = (interval[0], interval[1]);
        let probes = ((t1 - t0) / PROBE_S).ceil().max(1.0) as usize;
        grid.extend((0..probes).map(|k| t0 + (t1 - t0) * k as f64 / probes as f64));
    }
    grid.extend(breakpoints.last());
    grid
}

/// The instant between `inside` and `outside` where the toolhead crosses
/// the ball of radius `radius` around `at`.
fn ball_edge(
    trajectory: &ExactTrajectory,
    at: [f64; 3],
    radius: f64,
    (mut inside, mut outside): (f64, f64),
) -> f64 {
    for _ in 0..BALL_EDGE_BISECTIONS {
        let mid = 0.5 * (inside + outside);
        if distance(trajectory, mid, at) <= radius {
            inside = mid;
        } else {
            outside = mid;
        }
    }
    inside
}

/// The first stop at `at` from grid index `from` on, as its window and the
/// grid index past it.
fn next_stop(
    trajectory: &ExactTrajectory,
    grid: &[f64],
    from: usize,
    at: [f64; 3],
    radius: f64,
) -> Option<((f64, f64), usize)> {
    let mut index = from;
    while index < grid.len() {
        if distance(trajectory, grid[index], at) > radius {
            index += 1;
            continue;
        }
        let first = index;
        let mut slowest = f64::INFINITY;
        while index < grid.len() && distance(trajectory, grid[index], at) <= radius {
            slowest = slowest.min(speed(trajectory, grid[index]));
            index += 1;
        }
        if slowest > STOP_SPEED_MM_S {
            continue;
        }
        let t0 = match first.checked_sub(1) {
            Some(before) => ball_edge(trajectory, at, radius, (grid[first], grid[before])),
            None => grid[first],
        };
        let t1 = match grid.get(index) {
            Some(&after) => ball_edge(trajectory, at, radius, (grid[index - 1], after)),
            None => grid[index - 1],
        };
        return Some(((t0, t1), index));
    }
    None
}

#[test]
#[ignore]
fn probe() {
    for corpus::Case {
        name,
        gcode,
        config,
    } in corpus::corpus()
    {
        let Some(snapshot) = corpus::plan(&name, &gcode, Path::new(&config)) else {
            continue;
        };
        let waypoints = corpus::waypoints(&gcode, Path::new(&config));
        let radius = corpus::motion_settings(Path::new(&config)).fit_tolerance_mm;
        let trajectory = snapshot.toolhead.as_ref().unwrap_or(&snapshot.trajectory);
        let grid = probe_grid(trajectory);
        let mut cursor = 0;
        let (mut before, mut after, mut runs, mut missing) = (0.0, 0.0, 0, 0);
        for run in rest_runs(&waypoints) {
            let Some(((s0, s1), past)) = next_stop(trajectory, &grid, cursor, run.at, radius)
            else {
                missing += 1;
                continue;
            };
            let lo = grid.partition_point(|&t| t < s0);
            let resting: Vec<f64> = grid[lo..past]
                .iter()
                .copied()
                .filter(|&t| t <= s1 && speed(trajectory, t) <= REST_MM_S)
                .collect();
            cursor = past;
            runs += 1;
            if let (Some(r0), Some(r1)) = (resting.first(), resting.last()) {
                before += r0 - s0;
                after += s1 - r1;
            }
        }
        let total = snapshot.traversal_time_s;
        println!(
            "SLACK {name} runs={runs} missing={missing} before={before:.4} after={after:.4} total={total:.3} pct={:.2}",
            100.0 * (before + after) / total
        );
    }
}
