//! Every retract and prime — a G-code move of e alone — happens inside the
//! toolhead's stop at its point: while the shaped toolhead stays within
//! `max_path_deviation` of the point the G-code rests at. A run of
//! extrude-only moves at one point shares one stop. The toolhead stops
//! where it holds within that ball and slows below [`STOP_SPEED_MM_S`];
//! a pass through the point at speed is no stop. Besides the run's own e,
//! the extruder moves inside the stop the extrusion the path lays inside
//! the ball on its way in and out: at most the print's extrusion ratio over
//! the ball's diameter, plus a kernel-free advance's `k` times that ratio
//! times the toolhead's speed where it crosses the ball's edge.
//!
//! An extruder kernel smooths the advance of the motion on either side of
//! a stop into it, which no ball edge bounds; on such a chain the e inside
//! the stop does not tell the run's own apart, and nothing is measured.

use std::path::Path;

use pipeline_snapshot::waypoints::{Waypoint, extrusion_ratio_bound};
use pipeline_snapshot::{ExactTrajectory, SampleSide};

use super::extruder::{e_has_kernel, kernel_free_advance_k};
use super::{motion_settings, waypoints};

const E_AXIS: usize = 3;
const STOP_SPEED_MM_S: f64 = 1.0;
const PROBE_S: f64 = 1e-3;
const E_PROBES_PER_INTERVAL: usize = 16;
const BALL_EDGE_BISECTIONS: usize = 48;
const E_ROUNDING_MM: f64 = 1e-9;

/// A run of extrude-only moves at one point.
#[derive(Debug, Clone, Copy)]
struct RestRun {
    waypoint: usize,
    at: [f64; 3],
    net_mm: f64,
    travel_mm: f64,
}

/// What the plan does with one [`RestRun`]: the e it moves inside the
/// toolhead's stop there, or `stop: None` where the toolhead never stops,
/// and the e of the path's own extrusion the stop may hold besides.
#[derive(Debug, Clone, Copy)]
struct Placement {
    run: RestRun,
    stop: Option<(f64, f64)>,
    net_mm: f64,
    travel_mm: f64,
    allowance_mm: f64,
}

impl Placement {
    /// The run's e the plan moves outside the stop, beyond the allowance.
    fn misplaced_mm(&self) -> f64 {
        let outside = (self.run.travel_mm - self.travel_mm)
            .max(0.0)
            .max((self.net_mm - self.run.net_mm).abs());
        (outside - self.allowance_mm).max(0.0)
    }

    fn inside_share(&self) -> f64 {
        1.0 - self.misplaced_mm() / self.run.travel_mm
    }
}

/// Every rest run of the case and where its plan places it.
pub struct RetractPlacement {
    placements: Vec<Placement>,
}

impl RetractPlacement {
    pub fn summary(&self) -> String {
        let least = self
            .placements
            .iter()
            .map(Placement::inside_share)
            .fold(1.0, f64::min);
        format!(
            "{} retracts and primes, at least {:.4} % inside the toolhead's stop",
            self.placements.len(),
            100.0 * least
        )
    }

    /// The runs the plan moves partly outside the toolhead's stop, one
    /// line each; `None` where every run is inside.
    pub fn misplaced(&self) -> Option<String> {
        let lines: Vec<String> = self
            .placements
            .iter()
            .filter(|placement| placement.stop.is_none() || placement.misplaced_mm() > 0.0)
            .map(|placement| {
                let run = placement.run;
                let at = format!(
                    "waypoint {} at ({:.3}, {:.3}, {:.3})",
                    run.waypoint, run.at[0], run.at[1], run.at[2]
                );
                match placement.stop {
                    None => format!(
                        "{at}: e {:+.4} mm, the toolhead never stops there",
                        run.net_mm
                    ),
                    Some((t0, t1)) => format!(
                        "{at}: e {:+.4} mm over {:.4} mm of travel; the stop t={t0:.5}..{t1:.5} \
                         moves e {:+.4} mm over {:.4} mm, {:.2e} mm of the run outside it beyond \
                         the allowance {:.2e} mm ({:.3} % inside)",
                        run.net_mm,
                        run.travel_mm,
                        placement.net_mm,
                        placement.travel_mm,
                        placement.misplaced_mm(),
                        placement.allowance_mm,
                        100.0 * placement.inside_share()
                    ),
                }
            })
            .collect();
        (!lines.is_empty()).then(|| {
            format!(
                "{} of {} retracts and primes leave the toolhead's stop:\n{}",
                lines.len(),
                self.placements.len(),
                lines.join("\n")
            )
        })
    }
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

/// The e the extruder moves over `(t0, t1)`: its net and its travel.
fn e_moved(trajectory: &ExactTrajectory, (t0, t1): (f64, f64)) -> (f64, f64) {
    let mut instants = vec![t0, t1];
    instants.extend(
        trajectory
            .axis_breakpoints(E_AXIS)
            .into_iter()
            .filter(|&t| t > t0 && t < t1),
    );
    instants.sort_by(f64::total_cmp);
    let mut probes = Vec::with_capacity(instants.len() * E_PROBES_PER_INTERVAL);
    for interval in instants.windows(2) {
        let (a, b) = (interval[0], interval[1]);
        probes.extend(
            (0..E_PROBES_PER_INTERVAL)
                .map(|k| a + (b - a) * k as f64 / E_PROBES_PER_INTERVAL as f64),
        );
    }
    probes.push(t1);
    let e: Vec<f64> = probes
        .iter()
        .map(|&t| state(trajectory, E_AXIS, t).position)
        .collect();
    let travel = e.windows(2).map(|pair| (pair[1] - pair[0]).abs()).sum();
    (e[e.len() - 1] - e[0], travel)
}

/// Where the plan of `gcode` under `config` places each retract and prime
/// against the toolhead's stop; `None` where e has a kernel.
pub fn retract_placement(
    snapshot: &pipeline_snapshot::Snapshot,
    gcode: &str,
    config: &Path,
) -> Option<RetractPlacement> {
    if e_has_kernel(config) {
        return None;
    }
    let waypoints = waypoints(gcode, config);
    let radius = motion_settings(config).fit_tolerance_mm;
    let ratio = extrusion_ratio_bound(&waypoints);
    let advance_k = kernel_free_advance_k(config).unwrap_or(0.0);
    let trajectory = snapshot.toolhead.as_ref().unwrap_or(&snapshot.trajectory);
    let grid = probe_grid(trajectory);
    let mut cursor = 0;
    let placements = rest_runs(&waypoints)
        .into_iter()
        .map(
            |run| match next_stop(trajectory, &grid, cursor, run.at, radius) {
                Some((stop, past)) => {
                    cursor = past;
                    let (net_mm, travel_mm) = e_moved(trajectory, stop);
                    let edge_speeds = speed(trajectory, stop.0) + speed(trajectory, stop.1);
                    Placement {
                        run,
                        stop: Some(stop),
                        net_mm,
                        travel_mm,
                        allowance_mm: ratio * advance_k.mul_add(edge_speeds, 2.0 * radius)
                            + E_ROUNDING_MM,
                    }
                }
                None => Placement {
                    run,
                    stop: None,
                    net_mm: 0.0,
                    travel_mm: 0.0,
                    allowance_mm: 0.0,
                },
            },
        )
        .collect();
    Some(RetractPlacement { placements })
}
