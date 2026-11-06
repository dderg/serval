//! How far the shaped toolhead path strays from the G-code polyline: at
//! every vertex the closest the path comes to it (its miss), and between
//! them the farthest a sample lies from the polyline (its excursion), both
//! grouped by how sharply the polyline turns there and whether the path
//! extrudes into and out of it. Each extruding vertex the corner budget
//! holds is held to its share of it ([`pipeline_snapshot::corner_budget`]),
//! the carriers' `max_path_deviation` on top.

use std::fmt::Write;
use std::path::Path;

use geometry::CornerBudget;
use pipeline_snapshot::corner_budget::{Point, Polyline, corner_budgets, distance};
use pipeline_snapshot::{ExactTrajectory, SampleSide, Snapshot};

use super::{planner, waypoints};

const SPATIAL_AXES: usize = 3;
const PLANAR_AXES: [usize; 2] = [0, 1];
const SAMPLE_STEP_MM: f64 = 0.01;
const SAMPLE_DT_MIN_S: f64 = 1e-6;
const SAMPLE_DT_MAX_S: f64 = 1e-3;
const CURSOR_REACH_MM: f64 = 3.0;
const CURSOR_SKIP_COST: f64 = 0.25;
/// The least share of the shaped path's progress the cursor's arc takes:
/// where a later stretch retraces an earlier one (a travel back over the
/// line it left), the cursor still moves on with the path.
const CURSOR_PACE: f64 = 0.5;
/// A vertex reads every sample the cursor places within this much polyline
/// of it.
const VERTEX_WINDOW_MM: f64 = 3.0;
/// A vertex the path never comes within this of has been lost: the cursor
/// or the plan went astray.
const LOST_MM: f64 = 2.0;
const BUDGET_SLACK: f64 = 1e-3;
pub const TURN_BANDS_DEG: [f64; 5] = [0.0, 20.0, 60.0, 120.0, 170.0];

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Motion {
    Extruding,
    Travel,
    Rest,
}

#[derive(Debug, Clone, Copy)]
pub struct Worst {
    pub mm: f64,
    pub at: Point,
}

#[derive(Debug, Clone, Copy)]
pub struct Band {
    pub vertices: usize,
    pub miss: Worst,
    pub miss_sum: f64,
    pub excursion: Worst,
}

impl Band {
    const EMPTY: Self = Self {
        vertices: 0,
        miss: Worst {
            mm: 0.0,
            at: [0.0; SPATIAL_AXES],
        },
        miss_sum: 0.0,
        excursion: Worst {
            mm: 0.0,
            at: [0.0; SPATIAL_AXES],
        },
    };

    fn add_miss(&mut self, mm: f64, at: Point) {
        self.vertices += 1;
        self.miss_sum += mm;
        if mm > self.miss.mm {
            self.miss = Worst { mm, at };
        }
    }

    fn add_excursion(&mut self, mm: f64, at: Point) {
        if mm > self.excursion.mm {
            self.excursion = Worst { mm, at };
        }
    }
}

#[derive(Debug, Clone)]
pub struct PathAccuracy {
    pub budget: CornerBudget,
    /// The extruding vertex whose miss takes the largest share of its
    /// budget: the share and where.
    pub worst_share: (f64, Point),
    /// The largest share of its budget an extruding vertex's miss takes,
    /// per turn band.
    pub worst_share_by_band: [f64; TURN_BANDS_DEG.len()],
    /// Indexed `[motion][turn band]`.
    pub bands: [[Band; TURN_BANDS_DEG.len()]; 3],
    pub lost: Vec<Point>,
    /// Extruding vertices the budget holds missed by more than it: where,
    /// the miss, the budget and the vertex's own turn in degrees.
    pub over_budget: Vec<(Point, f64, f64, f64)>,
    /// The largest share of its budget an extruding corner's miss takes,
    /// and a curve's: a vertex inside a fitted arc run or cluster.
    pub worst_corner_share: f64,
    pub worst_curve_share: f64,
    /// The largest miss of an extruding corner and of a curve vertex.
    pub worst_corner_miss_mm: f64,
    pub worst_curve_miss_mm: f64,
}

impl PathAccuracy {
    pub fn band(&self, motion: Motion, band: usize) -> &Band {
        &self.bands[motion as usize][band]
    }

    pub fn worst_budgeted_miss(&self) -> f64 {
        self.bands[Motion::Extruding as usize]
            .iter()
            .map(|band| band.miss.mm)
            .fold(0.0, f64::max)
    }

    pub fn summary(&self) -> String {
        let (share, at) = self.worst_share;
        let mut out = format!(
            "corner budget {:?}, worst extruding miss {:.4} mm, worst share of budget {:.2} at \
             ({:.3}, {:.3}, {:.3}), {} over budget, {} lost; corners miss {:.4} mm (share \
             {:.2}), curves miss {:.4} mm (share {:.2})",
            self.budget,
            self.worst_budgeted_miss(),
            share,
            at[0],
            at[1],
            at[2],
            self.over_budget.len(),
            self.lost.len(),
            self.worst_corner_miss_mm,
            self.worst_corner_share,
            self.worst_curve_miss_mm,
            self.worst_curve_share,
        );
        for (motion, name) in [
            (Motion::Extruding, "extrude"),
            (Motion::Travel, "travel"),
            (Motion::Rest, "rest"),
        ] {
            for (index, low) in TURN_BANDS_DEG.iter().enumerate() {
                let band = self.band(motion, index);
                if band.vertices == 0 {
                    continue;
                }
                let at = band.miss.at;
                write!(
                    out,
                    "\n  {name:<7} turn>={low:>3.0}: {:>6} vertices, miss mean {:.4} max {:.4} at \
                     ({:.3}, {:.3}, {:.3}), excursion max {:.4}",
                    band.vertices,
                    band.miss_sum / band.vertices as f64,
                    band.miss.mm,
                    at[0],
                    at[1],
                    at[2],
                    band.excursion.mm
                )
                .expect("write to string");
            }
        }
        out
    }
}

/// Follows the samples along the polyline: the segment a sample lies
/// nearest to, searched forward to [`CURSOR_REACH_MM`] of arc past the farthest
/// the samples have come along it (at least [`CURSOR_PACE`] of the shaped
/// path's own progress) and never backward, each mm of polyline it
/// would skip costing [`CURSOR_SKIP_COST`] mm of distance. A later pass over
/// the same place, or a neighbouring patch that a cut corner comes closer
/// to, never claims a sample from the stretch the path is on.
struct Cursor {
    segment: usize,
    arc: f64,
}

impl Cursor {
    fn advance(&mut self, line: &Polyline, q: Point, step_mm: f64) -> f64 {
        let here = (self.arc + CURSOR_PACE * step_mm).max(line.arc_along(self.segment, q));
        let own = line.distance_to_segment(self.segment, q);
        let mut best = (own, own, self.segment);
        let mut candidate = self.segment + 1;
        while candidate < line.segments() && line.arc_start(candidate) <= here + CURSOR_REACH_MM {
            let d = line.distance_to_segment(candidate, q);
            let cost = d + CURSOR_SKIP_COST * (line.arc_start(candidate) - here).max(0.0);
            if cost < best.0 {
                best = (cost, d, candidate);
            }
            candidate += 1;
        }
        self.segment = best.2;
        self.arc = here.max(line.arc_start(self.segment));
        best.1
    }
}

pub fn audit(snapshot: &Snapshot, gcode: &str, config: &Path) -> Option<PathAccuracy> {
    let waypoints = waypoints(gcode, config);
    let line = Polyline::of(&waypoints);
    if line.segments() < 1 {
        return None;
    }
    let budgets = corner_budgets(&line, &waypoints, &planner("path accuracy", config))
        .unwrap_or_else(|e| panic!("{}: {e}", config.display()));
    let trajectory = snapshot.toolhead.as_ref().unwrap_or(&snapshot.trajectory);
    let mut nearest = vec![f64::INFINITY; line.vertices.len()];
    let mut excursion = vec![(0.0_f64, [0.0; SPATIAL_AXES]); line.segments()];
    let mut cursor = Cursor {
        segment: 0,
        arc: 0.0,
    };
    let mut previous: Option<Point> = None;
    let mut window = 0..0;
    for_each_sample(trajectory, |q| {
        let step = previous.map_or(0.0, |p| distance(p, q));
        let off = cursor.advance(&line, q, step);
        let segment = cursor.segment;
        if off > excursion[segment].0 {
            excursion[segment] = (off, q);
        }
        let chord_from = previous.unwrap_or(q);
        while window.end < line.vertices.len()
            && line.vertex_arc(window.end) <= cursor.arc + VERTEX_WINDOW_MM
        {
            window.end += 1;
        }
        while line.vertex_arc(window.start) < cursor.arc - VERTEX_WINDOW_MM {
            window.start += 1;
        }
        for vertex in window.clone() {
            nearest[vertex] = nearest[vertex].min(line.planar_distance(vertex, (chord_from, q)));
        }
        previous = Some(q);
    });
    let mut report = PathAccuracy {
        budget: budgets.budget,
        worst_share: (0.0, [0.0; SPATIAL_AXES]),
        worst_share_by_band: [0.0; TURN_BANDS_DEG.len()],
        bands: [[Band::EMPTY; TURN_BANDS_DEG.len()]; 3],
        lost: Vec::new(),
        over_budget: Vec::new(),
        worst_corner_share: 0.0,
        worst_curve_share: 0.0,
        worst_corner_miss_mm: 0.0,
        worst_curve_miss_mm: 0.0,
    };
    for vertex in 1..line.vertices.len() - 1 {
        let at = line.vertices[vertex];
        if nearest[vertex] > LOST_MM {
            report.lost.push(at);
            continue;
        }
        let motion = motion_at(&line, vertex);
        let turn_deg = line.turn_deg(vertex);
        let band = turn_band(turn_deg);
        report.bands[motion as usize][band].add_miss(nearest[vertex], at);
        let (off, off_at) = excursion[vertex];
        report.bands[motion as usize][band].add_excursion(off, off_at);
        if let (Motion::Extruding, Some(budget)) = (motion, budgets.vertices[vertex]) {
            let granted = budget.mm;
            let share = nearest[vertex] / granted;
            report.worst_share_by_band[band] = report.worst_share_by_band[band].max(share);
            if share > report.worst_share.0 {
                report.worst_share = (share, at);
            }
            let (worst_share, worst_miss) = if budget.corner {
                (
                    &mut report.worst_corner_share,
                    &mut report.worst_corner_miss_mm,
                )
            } else {
                (
                    &mut report.worst_curve_share,
                    &mut report.worst_curve_miss_mm,
                )
            };
            *worst_share = worst_share.max(share);
            *worst_miss = worst_miss.max(nearest[vertex]);
            if budget.held
                && nearest[vertex] > granted * (1.0 + BUDGET_SLACK) + budgets.carrier_slack_mm
            {
                report
                    .over_budget
                    .push((at, nearest[vertex], granted, turn_deg));
            }
        }
    }
    Some(report)
}

fn turn_band(turn_deg: f64) -> usize {
    TURN_BANDS_DEG
        .iter()
        .rposition(|low| turn_deg >= *low)
        .expect("a turn is at least 0 degrees")
}

/// Every spatial position of `trajectory` a [`SAMPLE_STEP_MM`] apart along
/// its path, the ends of every carrier row included; hold gaps, where the
/// toolhead stays put, are stepped over.
fn for_each_sample(trajectory: &ExactTrajectory, mut visit: impl FnMut(Point)) {
    let mut t = trajectory.t_start();
    let t_end = trajectory.t_end();
    while t < t_end {
        let Some((q, speed)) = state_at(trajectory, t) else {
            t = after_hold_gap(trajectory, t);
            continue;
        };
        visit(q);
        t += (SAMPLE_STEP_MM / speed).clamp(SAMPLE_DT_MIN_S, SAMPLE_DT_MAX_S);
    }
    let (q, _) = state_at(trajectory, t_end)
        .unwrap_or_else(|| panic!("the shaped path is not evaluable at its end t={t_end}"));
    visit(q);
}

fn after_hold_gap(trajectory: &ExactTrajectory, t: f64) -> f64 {
    PLANAR_AXES
        .iter()
        .filter_map(|&axis| {
            let rows = trajectory.rows(axis);
            let next = rows.partition_point(|row| row.t0 <= t);
            rows.get(next).map(|row| row.t0)
        })
        .reduce(f64::min)
        .unwrap_or_else(|| panic!("the shaped path is not evaluable at t={t} nor after it"))
}

/// The toolhead's position and speed at `t`; `None` where X or Y carries no
/// row (a hold gap). Z reads 0 where it carries none: misses read in XY.
fn state_at(trajectory: &ExactTrajectory, t: f64) -> Option<(Point, f64)> {
    let mut q = [0.0; SPATIAL_AXES];
    let mut speed_squared = 0.0;
    for (axis, coordinate) in q.iter_mut().enumerate() {
        let state = trajectory
            .eval_axis(axis, t, SampleSide::Right)
            .or_else(|_| trajectory.eval_axis(axis, t, SampleSide::Left));
        match state {
            Ok(state) => {
                *coordinate = state.position;
                speed_squared += state.velocity * state.velocity;
            }
            Err(_) if PLANAR_AXES.contains(&axis) => return None,
            Err(_) => {}
        }
    }
    Some((q, speed_squared.sqrt()))
}

fn motion_at(line: &Polyline, vertex: usize) -> Motion {
    if line.rests[vertex] {
        Motion::Rest
    } else if line.extrudes_through(vertex) {
        Motion::Extruding
    } else {
        Motion::Travel
    }
}
