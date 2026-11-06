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
use pipeline_snapshot::Snapshot;
use pipeline_snapshot::corner_budget::{Point, Polyline, Walk, corner_budgets, walk};

use super::{planner, waypoints};

const SPATIAL_AXES: usize = 3;
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

pub fn audit(snapshot: &Snapshot, gcode: &str, config: &Path) -> Option<PathAccuracy> {
    let waypoints = waypoints(gcode, config);
    let line = Polyline::of(&waypoints);
    if line.segments() < 1 {
        return None;
    }
    let budgets = corner_budgets(&line, &waypoints, &planner("path accuracy", config))
        .unwrap_or_else(|e| panic!("{}: {e}", config.display()));
    let trajectory = snapshot.toolhead.as_ref().unwrap_or(&snapshot.trajectory);
    let Walk {
        approaches,
        excursion,
    } = walk(trajectory, &line);
    let nearest: Vec<f64> = approaches.iter().map(|approach| approach.miss_mm).collect();
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

fn motion_at(line: &Polyline, vertex: usize) -> Motion {
    if line.rests[vertex] {
        Motion::Rest
    } else if line.extrudes_through(vertex) {
        Motion::Extruding
    } else {
        Motion::Travel
    }
}
