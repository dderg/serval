//! How far the shaped toolhead path strays from the G-code polyline: at
//! every vertex the closest the path comes to it (its miss), and between
//! them the farthest a sample lies from the polyline (its excursion), both
//! grouped by how sharply the polyline turns there and whether the path
//! extrudes into and out of it. The corner budget bounds the miss of every
//! extruding vertex: an explicit `corner_deviation` outright, an implicit
//! budget as the shaper's floor at the vertex's turn plus its allowance,
//! the floor taken at `max_accel` (the planner takes it at each corner's
//! own, lower or equal, acceleration, so the audit's bound is the looser).
//! A vertex within the shaper's reach of a sharper corner takes that
//! corner's budget, and corners closer together than that reach turn as
//! one: the shaper rounds them as one. Misses read across the
//! bed (XY), and the shaped carriers may stray from the exact shaped path
//! by `max_path_deviation` on top of any budget.

use std::fmt::Write;
use std::path::Path;

use geometry::{CornerBudget, ShapedCorners};
use pipeline_snapshot::waypoints::Waypoint;
use pipeline_snapshot::{ExactTrajectory, SampleSide, Snapshot};

use super::{motion_settings, planner, waypoints};

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
const COINCIDENT_MM: f64 = 1e-9;
const EXTRUDING_E_MM: f64 = 1e-9;
/// A vertex the path never comes within this of has been lost: the cursor
/// or the plan went astray.
const LOST_MM: f64 = 2.0;
const BUDGET_SLACK: f64 = 1e-3;
pub const TURN_BANDS_DEG: [f64; 5] = [0.0, 20.0, 60.0, 120.0, 170.0];

type Point = [f64; SPATIAL_AXES];

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

/// The shaped miss a config's corner budget grants a vertex by its turn.
#[derive(Debug, Clone, Copy)]
pub struct VertexBudget {
    pub budget: CornerBudget,
    max_accel: f64,
    corners: Option<&'static ShapedCorners>,
    /// How far the shaped carriers may stray from the exact shaped path
    /// (`max_path_deviation`), on top of any budget.
    fit_tolerance_mm: f64,
}

impl VertexBudget {
    pub fn of(config: &Path) -> Self {
        let settings = motion_settings(config);
        let cartesian = settings.cartesian;
        let chains = planner("path accuracy", config)
            .compile_active_chains()
            .unwrap_or_else(|e| panic!("{}: {e}", config.display()));
        let kernel = |axis: usize| chains.chains.get(axis).and_then(|chain| chain.kernel());
        Self {
            budget: cartesian.corner,
            max_accel: cartesian.max_accel,
            corners: ShapedCorners::interned([kernel(0), kernel(1)]),
            fit_tolerance_mm: settings.fit_tolerance_mm,
        }
    }

    /// How far along the path the shaper carries one corner's rounding
    /// onto its neighbours: the distance a stop at `max_accel` covers in
    /// two of the wider kernel's standard deviations. A vertex that close
    /// to a sharper corner takes that corner's budget.
    fn reach_mm(&self) -> f64 {
        self.corners
            .map_or(0.0, |corners| 0.25 * corners.approach_mm(self.max_accel))
    }

    pub fn mm(&self, turn_rad: f64) -> f64 {
        self.corners.map_or_else(
            || self.budget.unshaped_cut_mm(self.max_accel),
            |corners| corners.budget_mm(self.budget, turn_rad, self.max_accel),
        )
    }
}

#[derive(Debug, Clone)]
pub struct PathAccuracy {
    pub budget: CornerBudget,
    /// The extruding vertex whose miss takes the largest share of its
    /// budget: the share and where.
    pub worst_share: (f64, Point),
    /// Indexed `[motion][turn band]`.
    pub bands: [[Band; TURN_BANDS_DEG.len()]; 3],
    pub lost: Vec<Point>,
    /// Extruding vertices missed by more than their budget: where, the
    /// miss and the budget.
    pub over_budget: Vec<(Point, f64, f64)>,
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
             ({:.3}, {:.3}, {:.3}), {} over budget, {} lost",
            self.budget,
            self.worst_budgeted_miss(),
            share,
            at[0],
            at[1],
            at[2],
            self.over_budget.len(),
            self.lost.len()
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

struct Polyline {
    vertices: Vec<Point>,
    arc: Vec<f64>,
    extruding: Vec<bool>,
    rests: Vec<bool>,
}

impl Polyline {
    fn of(waypoints: &[Waypoint]) -> Self {
        let mut line = Self {
            vertices: Vec::new(),
            arc: Vec::new(),
            extruding: Vec::new(),
            rests: Vec::new(),
        };
        for (index, waypoint) in waypoints.iter().enumerate() {
            let point = spatial(waypoint.position);
            let rest_here = index > 0 && waypoint.dwell_before_s.is_some();
            let e_step = index.checked_sub(1).map_or(0.0, |previous| {
                waypoint.position[3] - waypoints[previous].position[3]
            });
            match line.vertices.last() {
                Some(last) if distance(*last, point) <= COINCIDENT_MM => {
                    let rest_at = line.rests.len() - 1;
                    line.rests[rest_at] |= rest_here || e_step != 0.0;
                }
                Some(last) => {
                    let rest_at = line.rests.len() - 1;
                    line.rests[rest_at] |= rest_here;
                    line.arc
                        .push(line.arc.last().copied().unwrap_or(0.0) + distance(*last, point));
                    line.extruding.push(e_step > EXTRUDING_E_MM);
                    line.vertices.push(point);
                    line.rests.push(false);
                }
                None => {
                    line.vertices.push(point);
                    line.rests.push(true);
                }
            }
        }
        if let Some(last) = line.rests.last_mut() {
            *last = true;
        }
        line
    }

    fn segments(&self) -> usize {
        self.vertices.len().saturating_sub(1)
    }

    fn distance_to_segment(&self, segment: usize, q: Point) -> f64 {
        distance_to_segment(
            planar(q),
            planar(self.vertices[segment]),
            planar(self.vertices[segment + 1]),
        )
    }

    fn arc_along(&self, segment: usize, q: Point) -> f64 {
        let (a, b) = (self.vertices[segment], self.vertices[segment + 1]);
        let ab = sub(b, a);
        let length = norm(ab);
        self.arc_start(segment) + (dot(sub(q, a), ab) / length).clamp(0.0, length)
    }

    /// How far `q` lies from vertex `vertex` across the bed: misses read in
    /// XY, where a bed mesh or a z-hop leaves the corner a G-code draws.
    fn planar_distance(&self, vertex: usize, chord: (Point, Point)) -> f64 {
        distance_to_segment(
            planar(self.vertices[vertex]),
            planar(chord.0),
            planar(chord.1),
        )
    }

    fn vertex_arc(&self, vertex: usize) -> f64 {
        self.arc_start(vertex)
    }

    fn arc_start(&self, segment: usize) -> f64 {
        if segment == 0 {
            0.0
        } else {
            self.arc[segment - 1]
        }
    }

    fn turn_deg(&self, vertex: usize) -> f64 {
        let incoming = sub(self.vertices[vertex], self.vertices[vertex - 1]);
        let outgoing = sub(self.vertices[vertex + 1], self.vertices[vertex]);
        let cosine = dot(incoming, outgoing) / (norm(incoming) * norm(outgoing));
        cosine.clamp(-1.0, 1.0).acos().to_degrees()
    }

    /// The turn the path makes over `reach_mm` either side of `vertex`: the
    /// chords from where it stands that far before to the vertex and on to
    /// where it stands that far after. Corners closer together than the
    /// shaper's reach turn as one.
    fn turn_over_deg(&self, vertex: usize, reach_mm: f64) -> f64 {
        let here = self.vertices[vertex];
        let incoming = sub(here, self.point_at_arc(self.vertex_arc(vertex) - reach_mm));
        let outgoing = sub(self.point_at_arc(self.vertex_arc(vertex) + reach_mm), here);
        let lengths = norm(incoming) * norm(outgoing);
        if lengths <= 0.0 {
            return 0.0;
        }
        (dot(incoming, outgoing) / lengths)
            .clamp(-1.0, 1.0)
            .acos()
            .to_degrees()
    }

    fn point_at_arc(&self, arc: f64) -> Point {
        let total = self.arc.last().copied().unwrap_or(0.0);
        let arc = arc.clamp(0.0, total);
        let segment = self
            .arc
            .partition_point(|&end| end < arc)
            .min(self.segments() - 1);
        let start = self.arc_start(segment);
        let length = self.arc[segment] - start;
        let along = if length > 0.0 {
            (arc - start) / length
        } else {
            0.0
        };
        let (a, b) = (self.vertices[segment], self.vertices[segment + 1]);
        [
            a[0] + along * (b[0] - a[0]),
            a[1] + along * (b[1] - a[1]),
            a[2] + along * (b[2] - a[2]),
        ]
    }

    fn motion_at(&self, vertex: usize) -> Motion {
        if self.rests[vertex] {
            Motion::Rest
        } else if self.extruding[vertex - 1] && self.extruding[vertex] {
            Motion::Extruding
        } else {
            Motion::Travel
        }
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
    let budget = VertexBudget::of(config);
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
        budget: budget.budget,
        worst_share: (0.0, [0.0; SPATIAL_AXES]),
        bands: [[Band::EMPTY; TURN_BANDS_DEG.len()]; 3],
        lost: Vec::new(),
        over_budget: Vec::new(),
    };
    let reach = budget.reach_mm();
    let own: Vec<f64> = (0..line.vertices.len())
        .map(|vertex| {
            if vertex == 0 || vertex + 1 == line.vertices.len() {
                0.0
            } else {
                let turn = line.turn_deg(vertex).max(line.turn_over_deg(vertex, reach));
                budget.mm(turn.to_radians())
            }
        })
        .collect();
    let granted_at = |vertex: usize| {
        let here = line.vertex_arc(vertex);
        let lo = (0..=vertex)
            .rev()
            .take_while(|&j| here - line.vertex_arc(j) <= reach)
            .last()
            .unwrap_or(vertex);
        let hi = (vertex..line.vertices.len())
            .take_while(|&j| line.vertex_arc(j) - here <= reach)
            .last()
            .unwrap_or(vertex);
        own[lo..=hi].iter().copied().fold(0.0, f64::max)
    };
    for vertex in 1..line.vertices.len() - 1 {
        let at = line.vertices[vertex];
        if nearest[vertex] > LOST_MM {
            report.lost.push(at);
            continue;
        }
        let motion = line.motion_at(vertex);
        let turn_deg = line.turn_deg(vertex);
        let band = turn_band(turn_deg);
        report.bands[motion as usize][band].add_miss(nearest[vertex], at);
        let (off, off_at) = excursion[vertex];
        report.bands[motion as usize][band].add_excursion(off, off_at);
        if motion == Motion::Extruding {
            let granted = granted_at(vertex);
            let share = nearest[vertex] / granted;
            if share > report.worst_share.0 {
                report.worst_share = (share, at);
            }
            if nearest[vertex] > granted * (1.0 + BUDGET_SLACK) + budget.fit_tolerance_mm {
                report.over_budget.push((at, nearest[vertex], granted));
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

fn spatial(position: [f64; 4]) -> Point {
    [position[0], position[1], position[2]]
}

fn planar(p: Point) -> Point {
    [p[0], p[1], 0.0]
}

fn sub(a: Point, b: Point) -> Point {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn dot(a: Point, b: Point) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn norm(a: Point) -> f64 {
    dot(a, a).sqrt()
}

fn distance(a: Point, b: Point) -> f64 {
    norm(sub(a, b))
}

fn distance_to_segment(q: Point, a: Point, b: Point) -> f64 {
    let ab = sub(b, a);
    let length_squared = dot(ab, ab);
    if length_squared == 0.0 {
        return distance(q, a);
    }
    let along = (dot(sub(q, a), ab) / length_squared).clamp(0.0, 1.0);
    distance(
        q,
        [
            a[0] + along * ab[0],
            a[1] + along * ab[1],
            a[2] + along * ab[2],
        ],
    )
}
