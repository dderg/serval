use geometry::path::CurvatureProfile;
use geometry::path::lowering::PositionProfile;
use geometry::{
    ADVANCE_VALLEY_SPEED_MM_S, AdvanceValley, CrawlSide, VALLEY_LEAD_BUDGET_S, ValleyKind,
    crawl_floor, crawl_side_width, side_weight, valley_reach,
};
use nurbs::algebra::PiecewisePolynomialKernel;

use super::backoff_stream_tests::{UnsoundPlanner, line};
use super::test_motion::{cross, gauss, norm, value};
use crate::live_config::GcodeLine;
use crate::types::{PlannedItem, PlannedMove};

const P1_RAMP: &str = "[printer]\n\
    max_velocity: 150\n\
    max_accel: 3000\n\
    max_jerk: 0\n\
    square_corner_velocity: 5\n\
    max_z_velocity: 5\n\
    max_z_accel: 100\n\
    [kinematics]\n\
    type: cartesian\n\
    axis_x: x\n\
    axis_y: y\n\
    axis_z: z\n\
    x_motors: mx\n\
    y_motors: my\n\
    z_motors: z\n\
    [axis x]\n\
    post_processors: shaper_x\n\
    [axis y]\n\
    post_processors: shaper_y\n\
    [axis z]\n\
    post_processors: shaper_z\n\
    [axis e]\n\
    follows: x, y, z\n\
    motors: extruder\n\
    post_processors: pa\n\
    max_accel: 5000\n\
    [post_processor pa]\n\
    type: linear_pressure_advance\n\
    k: 0.5\n\
    [post_processor shaper_x]\n\
    type: smooth_mzv\n\
    frequency_hz: 42.0\n\
    [post_processor shaper_y]\n\
    type: smooth_mzv\n\
    frequency_hz: 36.0\n\
    [post_processor shaper_z]\n\
    type: smooth_mzv\n\
    frequency_hz: 100.0\n\
    [motor mx]\n\
    drive: stepper\n\
    [motor my]\n\
    drive: stepper\n\
    [motor z]\n\
    drive: stepper\n\
    [motor extruder]\n\
    drive: stepper\n\
    [extruder]\n\
    axis: e\n\
    max_extrude_only_velocity: 45\n\
    max_extrude_only_accel: 2000\n";

/// Zigzag infill turning nearly back on itself through short steps along
/// the perimeter, at 98 mm/s (`voron_first_layers` lines 1263–1275).
fn near_reversal_zigzag() -> Vec<GcodeLine> {
    let points = [
        (1263, [121.546, 112.298], 0.008_67),
        (1264, [121.535, 112.645], 0.011_75),
        (1265, [126.102, 117.212], 0.218_3),
        (1266, [126.102, 117.804], 0.020_03),
        (1267, [121.516, 113.219], 0.219_19),
        (1268, [121.451, 113.747], 0.017_98),
        (1269, [126.102, 118.397], 0.222_29),
        (1270, [126.102, 118.99], 0.020_03),
        (1271, [121.37, 114.258], 0.226_17),
        (1272, [121.257, 114.738], 0.016_66),
        (1273, [126.102, 119.582], 0.231_56),
        (1274, [126.102, 120.175], 0.020_03),
        (1275, [121.126, 115.199], 0.237_85),
    ];
    let mut start = [121.524, 112.042];
    points
        .into_iter()
        .map(|(seq, end, de)| {
            let move_line = line(seq, start, end, de, 98.401_5);
            start = end;
            move_line
        })
        .collect()
}

/// The planned motion of a guarded stream, from rest at `t = 0`.
struct PlannedPath<'a> {
    moves: Vec<(f64, &'a PlannedMove)>,
    breaks: Vec<f64>,
    end: f64,
}

impl<'a> PlannedPath<'a> {
    fn new(emitted: &'a [PlannedMove]) -> Self {
        let mut t = 0.0;
        let mut moves = Vec::new();
        let mut breaks = vec![0.0];
        for planned in emitted {
            moves.push((t, planned));
            for phase in &planned.velocity.phases {
                t += phase.dt;
                breaks.push(t);
            }
        }
        Self {
            moves,
            breaks,
            end: t,
        }
    }

    /// The planned velocity and acceleration at `t`.
    fn state(&self, t: f64) -> ([f64; 3], [f64; 3]) {
        if t < 0.0 || t >= self.end {
            return ([0.0; 3], [0.0; 3]);
        }
        let (t0, planned) = self.moves[self.moves.partition_point(|(t0, _)| *t0 <= t) - 1];
        let Some(segment) = &planned.geometry.segment.spatial else {
            return ([0.0; 3], [0.0; 3]);
        };
        let phases = &planned.velocity.phases;
        let origin = phases[0].s0;
        let mut elapsed = t0;
        let mut state = phases
            .last()
            .expect("a planned move has phases")
            .end_state();
        for phase in phases {
            if t < elapsed + phase.dt {
                state = phase.state_at(phase.t0 + (t - elapsed));
                break;
            }
            elapsed += phase.dt;
        }
        let (s, v, a) = state;
        let [heading, turning, _] =
            segment.heading_frame_at((s - origin).clamp(0.0, segment.s_len()));
        (
            heading.map(|component| component * v),
            std::array::from_fn(|axis| heading[axis] * a + turning[axis] * v * v),
        )
    }

    /// The ratio the laid table holds at most around `t`: every move whose
    /// planned time the kernels at `t` read bounds it.
    fn ratio_near(&self, (from, to): (f64, f64)) -> f64 {
        self.moves
            .iter()
            .filter(|(t0, planned)| {
                let t1 = t0 + planned.velocity.phases.iter().map(|p| p.dt).sum::<f64>();
                t1 > from && *t0 < to
            })
            .map(|(_, planned)| planned.budget.ratio_in_reach)
            .fold(f64::INFINITY, f64::min)
    }
}

/// The ideal shaped motion of a planned path through the leaders' kernels.
struct Shaped<'a> {
    path: PlannedPath<'a>,
    kernels: [Option<&'a PiecewisePolynomialKernel>; 3],
}

impl Shaped<'_> {
    fn reach(&self) -> (f64, f64) {
        self.kernels
            .iter()
            .flatten()
            .map(|kernel| kernel.support())
            .fold((0.0, 0.0), |(lo, hi), (a, b)| {
                (f64::min(lo, a), f64::max(hi, b))
            })
    }

    fn velocity_and_accel(&self, t: f64) -> ([f64; 3], [f64; 3]) {
        let mut out = ([0.0; 3], [0.0; 3]);
        for (axis, kernel) in self.kernels.iter().enumerate() {
            let Some(kernel) = kernel else {
                continue;
            };
            let (lo, hi) = kernel.support();
            let mut cuts = vec![lo, hi];
            cuts.extend(kernel.pieces.iter().map(|piece| piece.u_start));
            let breaks = &self.path.breaks;
            let first = breaks.partition_point(|&b| b <= t - hi);
            let last = breaks.partition_point(|&b| b < t - lo);
            cuts.extend(breaks[first..last].iter().map(|b| t - b));
            cuts.retain(|&u| u >= lo && u <= hi);
            cuts.sort_by(f64::total_cmp);
            cuts.dedup();
            for span in cuts.windows(2) {
                out.0[axis] += gauss(span[0], span[1], |u| {
                    value(kernel, u) * self.path.state(t - u).0[axis]
                });
                out.1[axis] += gauss(span[0], span[1], |u| {
                    value(kernel, u) * self.path.state(t - u).1[axis]
                });
            }
        }
        out
    }

    fn speed(&self, t: f64) -> f64 {
        norm(self.velocity_and_accel(t).0)
    }

    /// Velocity, acceleration and jerk, the jerk by a central difference.
    fn motion(&self, t: f64) -> [[f64; 3]; 3] {
        const STEP: f64 = 1e-7;
        let (velocity, accel) = self.velocity_and_accel(t);
        let (_, before) = self.velocity_and_accel(t - STEP);
        let (_, after) = self.velocity_and_accel(t + STEP);
        [
            velocity,
            accel,
            std::array::from_fn(|axis| (after[axis] - before[axis]) / (2.0 * STEP)),
        ]
    }

    /// The slowest instant between `from` and `to`, by golden-section.
    fn slowest_between(&self, from: f64, to: f64) -> f64 {
        let ratio = 0.5 * (5.0_f64.sqrt() - 1.0);
        let (mut lo, mut hi) = (from, to);
        for _ in 0..60 {
            let (left, right) = (hi - ratio * (hi - lo), lo + ratio * (hi - lo));
            if self.speed(left) <= self.speed(right) {
                hi = right;
            } else {
                lo = left;
            }
        }
        0.5 * (lo + hi)
    }
}

/// The advance's demand on the extruder along a guarded stream, read the
/// way the follower projection lays its valleys: at every seam the shaped
/// leaders cross under [`ADVANCE_VALLEY_SPEED_MM_S`] and at every move's
/// slowest point inside under it, each a valley from the shared rule, each
/// side as wide as its rebuild needs ([`crawl_side_width`]) where the
/// leaders' own demand leaves room, merged into the last where their sides
/// overlap, whether or not the projection would find reading the path
/// through too steep there.
struct ValleyReference {
    times: Vec<f64>,
    arcs: Vec<f64>,
    motions: Vec<[[f64; 3]; 3]>,
    valleys: Vec<AdvanceValley>,
}

const REFERENCE_STEP_S: f64 = 5e-5;

impl ValleyReference {
    fn new(shaped: &Shaped, caps: &geometry::ExtruderCaps) -> Self {
        let count = (shaped.path.end / REFERENCE_STEP_S).ceil() as usize;
        let times: Vec<f64> = (0..=count).map(|i| i as f64 * REFERENCE_STEP_S).collect();
        let motions: Vec<[[f64; 3]; 3]> = times.iter().map(|&t| shaped.motion(t)).collect();
        let mut arcs = vec![0.0];
        for pair in motions.windows(2) {
            let last = arcs[arcs.len() - 1];
            arcs.push(last + 0.5 * REFERENCE_STEP_S * (norm(pair[0][0]) + norm(pair[1][0])));
        }
        let mut reference = Self {
            times,
            arcs,
            motions,
            valleys: Vec::new(),
        };
        for (t0, planned) in &shaped.path.moves {
            if planned.geometry.segment.spatial.is_none() {
                continue;
            }
            let t1 = t0 + planned.velocity.phases.iter().map(|p| p.dt).sum::<f64>();
            if *t0 > 0.0 {
                reference.lay(shaped, caps, *t0);
            }
            let first = (*t0 / REFERENCE_STEP_S).ceil() as usize;
            let last = ((t1 / REFERENCE_STEP_S).floor() as usize).min(count);
            let Some(slowest) = (first..=last).min_by(|&i, &j| {
                norm(reference.motions[i][0]).total_cmp(&norm(reference.motions[j][0]))
            }) else {
                continue;
            };
            let turn = shaped.slowest_between(
                (reference.times[slowest] - REFERENCE_STEP_S).max(*t0),
                (reference.times[slowest] + REFERENCE_STEP_S).min(t1),
            );
            if turn - t0 > REFERENCE_STEP_S && t1 - turn > REFERENCE_STEP_S {
                reference.lay(shaped, caps, turn);
            }
        }
        reference
    }

    fn arc_at(&self, shaped: &Shaped, t: f64) -> f64 {
        let i = ((t / REFERENCE_STEP_S).floor() as usize).min(self.times.len() - 1);
        let from = self.times[i];
        self.arcs[i] + gauss(from, t, |tau| shaped.speed(tau))
    }

    /// Lays the valley at the crawl the shaped leaders cross at `t`, as the
    /// projection lays it: a before side reaching no further back than the
    /// rest the stream starts from, the weight held at zero back to it
    /// where none fits, and failing loudly where the side reaches back
    /// past the [`VALLEY_LEAD_BUDGET_S`] of motion before the crawl, which
    /// the projection has taken.
    fn lay(&mut self, shaped: &Shaped, caps: &geometry::ExtruderCaps, t: f64) {
        let [velocity, accel, _] = shaped.motion(t);
        let (speed, rate) = (norm(velocity), norm(accel));
        if speed >= ADVANCE_VALLEY_SPEED_MM_S {
            return;
        }
        let turn = self.arc_at(shaped, t);
        let reach = valley_reach(speed, rate).min(turn);
        if reach <= 0.0
            || self
                .valleys
                .last()
                .is_some_and(|valley| valley.holds_zero_at(turn, 1e-6))
        {
            return;
        }
        let floor = crawl_floor(speed, rate);
        let crawl = (turn, speed, rate, floor);
        let after = crawl_side_width(reach, |width| {
            self.rebuild_fits(shaped, caps, crawl, (turn, turn + width), width)
        })
        .unwrap_or_else(|| panic!("no width fits the after side of the crawl at s={turn}"));
        let last_reach_end = self
            .valleys
            .last()
            .map_or(f64::NEG_INFINITY, AdvanceValley::reach_end);
        let (start, before) = if last_reach_end > turn - reach {
            (turn, reach)
        } else {
            crawl_side_width(reach, |width| {
                width > turn || self.rebuild_fits(shaped, caps, crawl, (turn - width, turn), width)
            })
            .filter(|&width| width <= turn)
            .map_or((0.0, 0.0), |width| (turn, width))
        };
        match self.valleys.last_mut() {
            Some(last) if start - before < last.reach_end() => {
                last.end = turn;
                last.after = after;
                last.floor_after = floor;
            }
            _ => {
                let budget = turn - self.arc_at(shaped, (t - VALLEY_LEAD_BUDGET_S).max(0.0));
                assert!(
                    turn - start + before <= budget,
                    "the crawl at s={turn} lays a valley reaching {} mm back, past the {budget} mm \
                     the leaders cover in the lead budget before it",
                    turn - start + before
                );
                self.valleys.push(AdvanceValley {
                    start,
                    end: turn,
                    before,
                    after,
                    floor_before: floor,
                    floor_after: floor,
                    kind: ValleyKind::Crawl,
                });
            }
        }
    }

    /// Whether a side `width` wide over the path `window` of the crawl
    /// `(turn, speed, rate, floor)` rebuilds the lead within what the
    /// leaders' own demand over it leaves of the extruder's limit, read as
    /// the projection reads it off the leaders: `r·max(|s̈|, |s̈ + k·W·s⃛|)`
    /// with the side's weight `W` there, and [`CrawlSide::rebuild`].
    fn rebuild_fits(
        &self,
        shaped: &Shaped,
        caps: &geometry::ExtruderCaps,
        (turn, speed, rate, floor): (f64, f64, f64, f64),
        (lo, hi): (f64, f64),
        width: f64,
    ) -> bool {
        let (back, ahead) = shaped.reach();
        let k = caps.advance_slope_s;
        let (mut base, mut ratio, mut accel, mut fastest) = (0.0_f64, 0.0_f64, rate, speed);
        let mut outward = (f64::INFINITY, f64::NEG_INFINITY);
        for ((&t, &s), &[velocity, acceleration, jerk]) in
            self.times.iter().zip(&self.arcs).zip(&self.motions)
        {
            if s < lo || s > hi {
                continue;
            }
            let here = shaped.path.ratio_near((t - ahead, t - back));
            let v = norm(velocity);
            if !here.is_finite() || v == 0.0 {
                continue;
            }
            let along =
                |x: [f64; 3]| (x[0] * velocity[0] + x[1] * velocity[1] + x[2] * velocity[2]) / v;
            let turning = norm(cross(acceleration, velocity)).powi(2)
                / (v * v)
                / v.max(ADVANCE_VALLEY_SPEED_MM_S);
            let path_accel = along(acceleration);
            let weight = side_weight((s - turn).abs(), width, floor).0;
            let own = path_accel
                .abs()
                .max((path_accel + k * weight * (along(jerk) + turning)).abs());
            base = base.max(here * own);
            ratio = ratio.max(here);
            accel = accel.max(path_accel.abs());
            let away = (s - turn).signum() * path_accel;
            outward = (outward.0.min(away), outward.1.max(away));
            fastest = fastest.max(v);
        }
        let room = caps.max_accel_mm_s2 - base;
        assert!(
            room > 0.0,
            "the leaders ask {base} mm/s² of the extruder within {width} mm of the crawl at s={turn}"
        );
        let leaders = CrawlSide {
            speed,
            accel,
            fastest,
            floor: (floor, floor),
            outward: if outward.0 <= outward.1 {
                outward
            } else {
                (-accel, accel)
            },
            signed_from: 0.0,
        };
        k * ratio * leaders.rebuild(width) <= room
    }

    fn weight(&self, s: f64) -> (f64, f64, f64) {
        self.valleys
            .iter()
            .fold((1.0, 0.0, 0.0), |(w, w1, w2), valley| {
                let (v, v1, v2) = valley.weight(s);
                (w * v, w1 * v + w * v1, w2 * v + 2.0 * w1 * v1 + w * v2)
            })
    }

    /// The worst share of the extruder's limit the advance asks for, and
    /// when: `r·(σ̈ + k·(W·σ⃛ + 3W′·σ̇·σ̈ + W″·σ̇³)) + c·(r·σ̈)²`, the turning
    /// part of `σ⃛` read at the speed a valley leaves the advance reading
    /// (see [`super::shadow::CellMotion::turning_ceiling`]).
    fn worst_share(&self, shaped: &Shaped, caps: &geometry::ExtruderCaps) -> (f64, f64) {
        let (lo, hi) = shaped.reach();
        let (k, c) = (caps.advance_slope_s, caps.advance_curvature_s2_per_mm);
        let mut worst = (0.0, 0.0);
        for ((&t, &s), &[velocity, accel, jerk]) in
            self.times.iter().zip(&self.arcs).zip(&self.motions)
        {
            let ratio = shaped.path.ratio_near((t - hi, t - lo));
            let speed = norm(velocity);
            if ratio <= 0.0 || !ratio.is_finite() || speed == 0.0 {
                continue;
            }
            let along = |x: [f64; 3]| {
                (x[0] * velocity[0] + x[1] * velocity[1] + x[2] * velocity[2]) / speed
            };
            let turning_speed = speed.max(ADVANCE_VALLEY_SPEED_MM_S);
            let turning = norm(cross(accel, velocity)).powi(2) / speed.powi(2) / turning_speed;
            let (path_accel, path_jerk) = (along(accel), along(jerk) + turning);
            let (w, w1, w2) = self.weight(s);
            let demand = (ratio
                * (path_accel
                    + k * (w * path_jerk
                        + 3.0 * w1 * speed * path_accel
                        + w2 * speed * speed * speed)))
                .abs()
                + c * (ratio * path_accel).powi(2);
            let share = demand / caps.max_accel_mm_s2;
            if share > worst.0 {
                worst = (share, t);
            }
        }
        worst
    }
}

#[test]
fn a_valley_rebuild_under_an_under_estimated_jerk_gain_stays_within_the_extruder_limit() {
    let planner = UnsoundPlanner::new(P1_RAMP, 0.45);
    let planned = planner.plan(&near_reversal_zigzag());
    let moves = planned
        .iter()
        .filter(|item| matches!(item, PlannedItem::Move(_)))
        .count();
    let emitted = planner.guard(planned);
    assert_eq!(emitted.len(), moves);
    let leaders: [usize; 3] = [0, 1, 2];
    let shaped = Shaped {
        path: PlannedPath::new(&emitted),
        kernels: leaders.map(|axis| planner.chains.chains[axis].kernel()),
    };
    let reference = ValleyReference::new(&shaped, &planner.real);
    assert!(
        !reference.valleys.is_empty(),
        "the zigzag lays no advance valley"
    );
    let (share, at) = reference.worst_share(&shaped, &planner.real);
    assert!(
        share <= 1.0 + 1e-9,
        "the advance asks {share:.4}× the extruder's limit at t={at:.6} s, by {} valleys",
        reference.valleys.len()
    );
}

/// The guard holds back the motion within [`VALLEY_LEAD_BUDGET_S`] of what
/// has arrived, where a crawl still to come may lay a valley side, and a
/// drain releases all of it: every move leaves, in order, by the drain.
#[test]
fn a_drain_releases_the_moves_the_lead_budget_held_back() {
    let planner = UnsoundPlanner::new(P1_RAMP, 1.0);
    let planned = planner.plan(&near_reversal_zigzag());
    let lines: Vec<u32> = planned
        .iter()
        .filter_map(|item| match item {
            PlannedItem::Move(planned) => Some(planned.geometry.source.start_line),
            _ => None,
        })
        .collect();
    let (emitted, at_drains) = planner.guard_through_drains(planned);
    let emitted_lines: Vec<u32> = emitted
        .iter()
        .map(|planned| planned.geometry.source.start_line)
        .collect();
    assert_eq!(
        emitted_lines, lines,
        "a drained stream left moves held, or emitted them out of order"
    );
    let &[before_drain] = at_drains.as_slice() else {
        panic!("the zigzag drains once, at its end: {at_drains:?}");
    };
    let held: f64 = emitted[before_drain..]
        .iter()
        .flat_map(|planned| planned.velocity.phases.iter())
        .map(|phase| phase.dt)
        .sum();
    assert!(
        held >= VALLEY_LEAD_BUDGET_S,
        "the guard held back {held} s of motion before the drain, under the lead budget"
    );
}
