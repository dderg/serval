//! What the corner budget holds the shaped path to at each vertex of a
//! print's G-code polyline. An explicit `corner_deviation` bounds a vertex
//! outright; an implicit budget is the shaper's floor at the vertex's turn
//! plus its allowance, the floor taken at `max_accel` (the planner takes it
//! at each corner's own, lower or equal, acceleration, so this bound is the
//! looser). A vertex within the shaper's reach of a sharper corner takes
//! that corner's budget, and corners closer together than that reach turn
//! as one. The default budget holds corners alone: the vertices the fitter
//! keeps as corners, a stop or a corner blend between the moves either
//! side; a vertex inside a fitted arc run or a consumed cluster of facets
//! traces a curve, which it leaves to the shaper. The shaped carriers may
//! stray from the exact shaped path by `max_path_deviation` on top of any
//! budget.

use std::collections::HashSet;

use geometry::path::Segment;
use geometry::shaped_corner::blend_apex;
use geometry::{CornerBudget, CornerFitConfig, Move, ShapedCorners, seam_requires_stop};
use motion_pipeline::StreamInput;
use motion_pipeline::fit_stage::FitStage;
use planner_config::PlannerConfig;

use crate::waypoints::Waypoint;
use crate::{ExactTrajectory, SampleSide, SnapshotError, stream_inputs};

pub type Point = [f64; 3];

const COINCIDENT_MM: f64 = 1e-9;
const EXTRUDING_E_MM: f64 = 1e-9;
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

/// The G-code path as the toolhead's polyline: its distinct vertices, the
/// arc length at each segment's end, whether each segment extrudes, which
/// vertices the toolhead rests at, and the G-code moves into and out of
/// each vertex by their index in the stream the waypoints make.
pub struct Polyline {
    pub vertices: Vec<Point>,
    pub arc: Vec<f64>,
    pub extruding: Vec<bool>,
    pub rests: Vec<bool>,
    pub seams: Vec<(usize, usize)>,
}

impl Polyline {
    #[must_use]
    pub fn of(waypoints: &[Waypoint]) -> Self {
        let mut line = Self {
            vertices: Vec::new(),
            arc: Vec::new(),
            extruding: Vec::new(),
            rests: Vec::new(),
            seams: Vec::new(),
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
                    let arriving = index - 1;
                    line.seams.last_mut().expect("a vertex before").1 = arriving;
                    line.seams.push((arriving, usize::MAX));
                }
                None => {
                    line.vertices.push(point);
                    line.rests.push(true);
                    line.seams.push((usize::MAX, usize::MAX));
                }
            }
        }
        if let Some(last) = line.rests.last_mut() {
            *last = true;
        }
        line
    }

    #[must_use]
    pub fn segments(&self) -> usize {
        self.vertices.len().saturating_sub(1)
    }

    #[must_use]
    pub fn distance_to_segment(&self, segment: usize, q: Point) -> f64 {
        distance_to_segment(
            planar(q),
            planar(self.vertices[segment]),
            planar(self.vertices[segment + 1]),
        )
    }

    #[must_use]
    pub fn arc_along(&self, segment: usize, q: Point) -> f64 {
        let (a, b) = (self.vertices[segment], self.vertices[segment + 1]);
        let ab = sub(b, a);
        let length = norm(ab);
        self.arc_start(segment) + (dot(sub(q, a), ab) / length).clamp(0.0, length)
    }

    /// How far the chord from `chord.0` to `chord.1` passes from vertex
    /// `vertex` across the bed: misses read in XY, where a bed mesh or a
    /// z-hop leaves the corner a G-code draws.
    #[must_use]
    pub fn planar_distance(&self, vertex: usize, chord: (Point, Point)) -> f64 {
        distance_to_segment(
            planar(self.vertices[vertex]),
            planar(chord.0),
            planar(chord.1),
        )
    }

    #[must_use]
    pub fn vertex_arc(&self, vertex: usize) -> f64 {
        self.arc_start(vertex)
    }

    #[must_use]
    pub fn arc_start(&self, segment: usize) -> f64 {
        if segment == 0 {
            0.0
        } else {
            self.arc[segment - 1]
        }
    }

    #[must_use]
    pub fn turn_deg(&self, vertex: usize) -> f64 {
        let incoming = sub(self.vertices[vertex], self.vertices[vertex - 1]);
        let outgoing = sub(self.vertices[vertex + 1], self.vertices[vertex]);
        let cosine = dot(incoming, outgoing) / (norm(incoming) * norm(outgoing));
        libm::acos(cosine.clamp(-1.0, 1.0)).to_degrees()
    }

    /// The turn the path makes over `reach_mm` either side of `vertex`: the
    /// chords from where it stands that far before to the vertex and on to
    /// where it stands that far after.
    #[must_use]
    pub fn turn_over_deg(&self, vertex: usize, reach_mm: f64) -> f64 {
        let here = self.vertices[vertex];
        let incoming = sub(here, self.point_at_arc(self.vertex_arc(vertex) - reach_mm));
        let outgoing = sub(self.point_at_arc(self.vertex_arc(vertex) + reach_mm), here);
        let lengths = norm(incoming) * norm(outgoing);
        if lengths <= 0.0 {
            return 0.0;
        }
        libm::acos((dot(incoming, outgoing) / lengths).clamp(-1.0, 1.0)).to_degrees()
    }

    #[must_use]
    pub fn point_at_arc(&self, arc: f64) -> Point {
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

    /// Whether the toolhead extrudes into and out of the interior vertex
    /// `vertex` without resting there.
    #[must_use]
    pub fn extrudes_through(&self, vertex: usize) -> bool {
        !self.rests[vertex] && self.extruding[vertex - 1] && self.extruding[vertex]
    }
}

/// Where the shaped path comes closest to a vertex: how close, across the
/// bed, and when.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Approach {
    pub miss_mm: f64,
    pub t: f64,
}

/// The shaped path read against the polyline: each vertex's closest
/// approach and, per segment, the farthest sample the cursor places on it
/// and where.
pub struct Walk {
    pub approaches: Vec<Approach>,
    pub excursion: Vec<(f64, Point)>,
}

/// Walks `trajectory`'s samples along `line` with a [`Cursor`], each vertex
/// reading the chords the cursor places within [`VERTEX_WINDOW_MM`] of it.
#[must_use]
pub fn walk(trajectory: &ExactTrajectory, line: &Polyline) -> Walk {
    let mut approaches = vec![
        Approach {
            miss_mm: f64::INFINITY,
            t: f64::NAN,
        };
        line.vertices.len()
    ];
    let mut excursion = vec![(0.0_f64, [0.0; SPATIAL_AXES]); line.segments()];
    let mut cursor = Cursor {
        segment: 0,
        arc: 0.0,
    };
    let mut previous: Option<Point> = None;
    let mut window = 0..0;
    for_each_sample(trajectory, |t, q| {
        let step = previous.map_or(0.0, |p| distance(p, q));
        let off = cursor.advance(line, q, step);
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
            let miss_mm = line.planar_distance(vertex, (chord_from, q));
            if miss_mm < approaches[vertex].miss_mm {
                approaches[vertex] = Approach { miss_mm, t };
            }
        }
        previous = Some(q);
    });
    Walk {
        approaches,
        excursion,
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

/// Every spatial position of `trajectory` a [`SAMPLE_STEP_MM`] apart along
/// its path, the ends of every carrier row included; hold gaps, where the
/// toolhead stays put, are stepped over.
fn for_each_sample(trajectory: &ExactTrajectory, mut visit: impl FnMut(f64, Point)) {
    let mut t = trajectory.t_start();
    let t_end = trajectory.t_end();
    while t < t_end {
        let Some((q, speed)) = state_at(trajectory, t) else {
            t = after_hold_gap(trajectory, t);
            continue;
        };
        visit(t, q);
        t += (SAMPLE_STEP_MM / speed).clamp(SAMPLE_DT_MIN_S, SAMPLE_DT_MAX_S);
    }
    let (q, _) = state_at(trajectory, t_end)
        .unwrap_or_else(|| panic!("the shaped path is not evaluable at its end t={t_end}"));
    visit(t_end, q);
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

/// What the budget grants one interior vertex: the most its shaped miss
/// may be, whether the fitter keeps it as a corner, and whether the budget
/// holds it (a corner, or any vertex under a budget that holds curves).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VertexBudget {
    pub mm: f64,
    pub corner: bool,
    pub held: bool,
}

/// A print's corner budget, how far the shaped carriers may stray on top
/// of it, and each vertex's share of it (`None` at the polyline's ends).
#[derive(Debug, Clone)]
pub struct CornerBudgets {
    pub budget: CornerBudget,
    pub carrier_slack_mm: f64,
    pub vertices: Vec<Option<VertexBudget>>,
}

/// Each vertex of `line`, the polyline of `waypoints`, under `planner`'s
/// corner budget.
pub fn corner_budgets(
    line: &Polyline,
    waypoints: &[Waypoint],
    planner: &PlannerConfig,
) -> Result<CornerBudgets, SnapshotError> {
    let (stream, chains, steps) = stream_inputs(waypoints, planner, None)?;
    let kernel = |axis: usize| chains.chains.get(axis).and_then(|chain| chain.kernel());
    let shaped = ShapedCorners::interned([kernel(0), kernel(1)]);
    let fit = CornerFitConfig {
        shaped_corners: shaped,
        ..stream.corner
    };
    let corners = fitted_corners(&steps, fit);
    let budget = planner.cartesian.corner;
    let max_accel = planner.cartesian.max_accel;
    let reach = shaped.map_or(0.0, |shaped| shaped.reach_mm(max_accel));
    let budget_mm = |turn_deg: f64| {
        shaped.map_or_else(
            || budget.unshaped_cut_mm(max_accel),
            |shaped| shaped.budget_mm(budget, turn_deg.to_radians(), max_accel),
        )
    };
    let last = line.vertices.len().saturating_sub(1);
    let own: Vec<f64> = (0..line.vertices.len())
        .map(|vertex| {
            if vertex == 0 || vertex == last {
                0.0
            } else {
                budget_mm(line.turn_deg(vertex).max(line.turn_over_deg(vertex, reach)))
            }
        })
        .collect();
    let granted = |vertex: usize| {
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
    let vertices = (0..line.vertices.len())
        .map(|vertex| {
            (vertex != 0 && vertex != last).then(|| {
                let corner = corners.contains(&line.seams[vertex]);
                VertexBudget {
                    mm: granted(vertex),
                    corner,
                    held: corner || ShapedCorners::holds_curves(budget),
                }
            })
        })
        .collect();
    Ok(CornerBudgets {
        budget,
        carrier_slack_mm: planner.fit_tolerance_mm,
        vertices,
    })
}

/// The seams the fitter keeps as corners, by the G-code moves either side:
/// a stop, a corner blend whose cut leaves room in its budget, or a bare
/// junction of two lines. A seam inside a fitted arc run or a consumed
/// cluster of facets, or easing into an arc, is none.
fn fitted_corners(steps: &[crate::PlanStep], config: CornerFitConfig) -> HashSet<(usize, usize)> {
    let mut fitted: Vec<Move> = Vec::new();
    let mut fit = FitStage::new(config).into_driver();
    let mut keep = |item: StreamInput| {
        if let StreamInput::Move(m) = item {
            if m.segment.spatial.is_some() {
                fitted.push(m);
            }
        }
        true
    };
    for step in steps.iter().cloned() {
        assert!(step.feed(|input| fit.feed(input, &mut keep)));
    }
    assert!(fit.finish(&mut keep));
    fitted
        .windows(2)
        .filter(|pair| is_corner(&pair[0], &pair[1], config))
        .map(|pair| {
            (
                pair[0].source.end_line as usize,
                pair[1].source.start_line as usize,
            )
        })
        .collect()
}

fn is_corner(before: &Move, after: &Move, config: CornerFitConfig) -> bool {
    if seam_requires_stop(before, after, config) {
        return true;
    }
    if let (Some(Segment::Line(_)), Some(Segment::Line(_))) =
        (&before.segment.spatial, &after.segment.spatial)
    {
        return true;
    }
    let Some(corners) = config.shaped_corners else {
        return blend_apex(before, after).is_some();
    };
    blend_apex(before, after).is_some_and(|(theta, cut_mm)| {
        let accel = before.limits.accel_mm_s2.max(after.limits.accel_mm_s2);
        let budget = before.limits.corner.stricter(
            after.limits.corner,
            before.limits.accel_mm_s2.min(after.limits.accel_mm_s2),
        );
        cut_mm < corners.budget_mm(budget, theta, accel)
    })
}

fn spatial(position: [f64; 4]) -> Point {
    [position[0], position[1], position[2]]
}

#[must_use]
pub fn planar(p: Point) -> Point {
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

#[must_use]
pub fn distance(a: Point, b: Point) -> f64 {
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
