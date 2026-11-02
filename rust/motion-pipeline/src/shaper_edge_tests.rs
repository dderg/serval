//! How the ladder reads a span's anchoring edge state, and how coalescing
//! reports the seeds it absorbs.

use std::sync::Arc;

use geometry::path::{Arc as PathArc, PathSegment, Segment};
use geometry::{LawSegment, Move, ScalarLaw, SourceRange, VelocityLimits};
use trajectory::{AnalyticMoveSpan, SurfaceMode};

use super::{
    CrowdedVelocityBreaks, EdgeSlivers, FitTol, SINGULAR_SEED_GRADING_FLOOR_S, TrackSignal,
    VelocityBreak, cap_graded_seeds, coalesce_degenerate_seeds, edge_inset,
    edge_velocity_and_acceleration, grade_seeds_toward, velocity_breaks_among,
};

const FLOOR: f64 = 2e-9;
const POSITION_TOL: f64 = 1e-4;
const TOL: FitTol = FitTol {
    pos_mm: 1e-3,
    accel_mm_s2: 50.0,
};

/// Constant acceleration that steps from `before` to `after` at `step`.
struct AccelerationStep {
    step: f64,
    before: f64,
    after: f64,
}

impl TrackSignal for AccelerationStep {
    fn eval_pva(&self, t: f64) -> (f64, f64, f64) {
        let acceleration = if t < self.step {
            self.before
        } else {
            self.after
        };
        (0.5 * acceleration * t * t, acceleration * t, acceleration)
    }
}

/// Acceleration stepping by `step_by` at `step` on top of a linear rise at
/// `jerk`.
struct JerkWithStep {
    jerk: f64,
    step: f64,
    step_by: f64,
}

impl TrackSignal for JerkWithStep {
    fn eval_pva(&self, t: f64) -> (f64, f64, f64) {
        let stepped = if t < self.step { 0.0 } else { self.step_by };
        let j = self.jerk;
        (j * t * t * t / 6.0, 0.5 * j * t * t, j * t + stepped)
    }
}

/// Stored-acceleration slivers far from the origin reach this wide, where a
/// smooth signal's jerk alone moves its acceleration past the tolerance.
const WIDE_SLIVER_S: f64 = 7e-7;
const STEEP_JERK: f64 = 1e9;
const SPAN_S: f64 = 1e-3;

/// Acceleration rising linearly at `jerk`.
struct ConstantJerk {
    jerk: f64,
}

impl TrackSignal for ConstantJerk {
    fn eval_pva(&self, t: f64) -> (f64, f64, f64) {
        let j = self.jerk;
        (j * t * t * t / 6.0, 0.5 * j * t * t, j * t)
    }
}

#[test]
fn a_break_hidden_in_the_edge_sliver_is_read_past() {
    let t0 = 1.0;
    let sig = AccelerationStep {
        step: t0 + 0.5 * FLOOR,
        before: 0.0,
        after: 5000.0,
    };
    let (_, acceleration) = edge_velocity_and_acceleration(
        &sig,
        0,
        trajectory::next_toward(t0, 2.0),
        t0 + edge_inset(1e-3, FLOOR, 0.0),
        TOL,
    )
    .unwrap();
    assert_eq!(
        acceleration, 5000.0,
        "a step half a floor inside the span must not pin the piece to the acceleration \
         before it"
    );
}

#[test]
fn a_smooth_edge_keeps_its_seam_acceleration() {
    let t0 = 1.0;
    let sig = ConstantJerk { jerk: 1e9 };
    let seam = trajectory::next_toward(t0, 2.0);
    let (_, acceleration) =
        edge_velocity_and_acceleration(&sig, 0, seam, t0 + edge_inset(1e-3, FLOOR, 0.0), TOL)
            .unwrap();
    assert_eq!(
        acceleration,
        sig.eval_pva(seam).2,
        "where the signal is smooth both sides of a seam must anchor to the same \
         acceleration, or the seam steps"
    );
}

#[test]
fn the_edge_inset_never_reaches_past_a_quarter_of_the_span() {
    assert_eq!(edge_inset(1e-3, FLOOR, 0.0), FLOOR);
    assert_eq!(edge_inset(1e-3, FLOOR, 3.0 * FLOOR), 3.0 * FLOOR);
    assert_eq!(edge_inset(4e-9, FLOOR, 0.0), 1e-9);
}

#[test]
fn a_coalesced_chain_reports_the_full_width_it_absorbed() {
    let mut seeds = vec![
        0.0,
        1.0,
        1.0 + 0.6 * FLOOR,
        1.0 + 1.2 * FLOOR,
        1.0 + 1.8 * FLOOR,
        3.0,
    ];
    let slivers = coalesce_degenerate_seeds(&mut seeds, &[], FLOOR, POSITION_TOL)
        .expect("no velocity breaks");
    assert_eq!(seeds, vec![0.0, 1.0 + 1.8 * FLOOR, 3.0]);
    assert_eq!(slivers.len(), 2);
    assert!(
        (slivers[0].end - 1.8 * FLOOR).abs() <= 1e-15,
        "the chain reaches {} back from its kept seed, reported {:?}",
        1.8 * FLOOR,
        slivers[0]
    );
    assert_eq!(slivers[1], EdgeSlivers::default());
}

#[test]
fn seeds_absorbed_at_the_pinned_ends_reach_into_the_end_spans() {
    let mut seeds = vec![0.0, 0.5 * FLOOR, 1.0, 2.0 - 0.7 * FLOOR, 2.0];
    let slivers = coalesce_degenerate_seeds(&mut seeds, &[], FLOOR, POSITION_TOL)
        .expect("no velocity breaks");
    assert_eq!(seeds, vec![0.0, 1.0, 2.0]);
    assert_eq!(slivers[0].start, 0.5 * FLOOR);
    assert!(
        (slivers[1].end - 0.7 * FLOOR).abs() <= 1e-15,
        "{:?}",
        slivers[1]
    );
}

#[test]
fn a_chain_of_sub_floor_seeds_never_rolls_past_a_velocity_break() {
    let end = 2.0;
    let ratio_step = end - 7.0 * FLOOR;
    let mut seeds = vec![0.0, 1.0, ratio_step];
    seeds.extend((1..16).map(|k| ratio_step + f64::from(k) * 7.0 * FLOOR / 16.0));
    seeds.push(end);
    let breaks = [VelocityBreak {
        t: ratio_step,
        step: -3.4,
    }];
    let slivers = coalesce_degenerate_seeds(&mut seeds, &breaks, FLOOR, POSITION_TOL)
        .expect("the ratio step is a floor clear of the end");
    assert!(
        seeds.contains(&ratio_step),
        "the ratio step at {ratio_step} was coalesced away: {seeds:?}"
    );
    assert!(
        seeds.windows(2).all(|span| span[1] - span[0] >= FLOOR),
        "a kept span is narrower than the floor: {seeds:?}"
    );
    assert_eq!(slivers.len(), seeds.len() - 1);
}

#[test]
fn velocity_breaks_within_the_floor_of_an_anchor_move_onto_it() {
    let (crowding, near_end) = (1.0 + 0.5 * FLOOR, 2.0 - 0.5 * FLOOR);
    let mut seeds = vec![0.0, 1.0, crowding, near_end, 2.0];
    let breaks = [
        VelocityBreak { t: 1.0, step: 3.4 },
        VelocityBreak {
            t: crowding,
            step: -1.0,
        },
        VelocityBreak {
            t: near_end,
            step: 2.0,
        },
    ];
    let slivers = coalesce_degenerate_seeds(&mut seeds, &breaks, FLOOR, POSITION_TOL)
        .expect("each moved break shifts position by nanometres");
    assert_eq!(seeds, vec![0.0, 1.0, 2.0]);
    assert_eq!(slivers[0].end_break, None);
    assert_eq!(slivers[1].start_break, Some(crowding));
    assert_eq!(slivers[1].end_break, Some(near_end));
}

#[test]
fn moving_a_velocity_break_past_the_position_tolerance_fails_loudly() {
    let crowding = 1.0 + 0.5 * FLOOR;
    let mut seeds = vec![0.0, 1.0, crowding, 2.0];
    let step = 1e6;
    let breaks = [
        VelocityBreak { t: 1.0, step: 1.0 },
        VelocityBreak { t: crowding, step },
    ];
    assert_eq!(
        coalesce_degenerate_seeds(&mut seeds, &breaks, FLOOR, POSITION_TOL),
        Err(CrowdedVelocityBreaks {
            break_at: crowding,
            moved_onto: 1.0,
            velocity_step: step,
            position_shift: step * (crowding - 1.0),
        })
    );
}

struct LeftValuedVelocityStep {
    at: f64,
    before: f64,
    after: f64,
}

impl TrackSignal for LeftValuedVelocityStep {
    fn eval_pva(&self, t: f64) -> (f64, f64, f64) {
        let velocity = if t <= self.at {
            self.before
        } else {
            self.after
        };
        (velocity * (t - self.at), velocity, 0.0)
    }
}

#[test]
fn a_velocity_break_that_reads_its_left_value_on_the_seed_is_found() {
    let sig = LeftValuedVelocityStep {
        at: 0.01,
        before: 7.2673,
        after: 4.5839,
    };
    assert_eq!(
        velocity_breaks_among(&sig, &[0.0, sig.at, 0.02], 1e-6),
        vec![VelocityBreak {
            t: sig.at,
            step: sig.after - sig.before,
        }]
    );
}

#[test]
fn seeds_grade_geometrically_toward_a_cap_contact_from_both_sides() {
    let contact = 4e-4;
    let mut seeds = vec![0.0, 1e-3];
    grade_seeds_toward(&mut seeds, &[contact]);
    seeds.sort_by(f64::total_cmp);
    let at = seeds
        .iter()
        .position(|&seed| seed == contact)
        .expect("the contact is a seed");
    for side in [&seeds[..at], &seeds[at + 1..]] {
        let nearest = side
            .iter()
            .map(|&seed| (seed - contact).abs())
            .fold(f64::INFINITY, f64::min);
        assert!(
            nearest < 2.0 * SINGULAR_SEED_GRADING_FLOOR_S,
            "the grading stops {nearest} s short of the contact: {seeds:?}"
        );
    }
    assert!(
        seeds.windows(2).all(|pair| pair[1] > pair[0]),
        "graded seeds repeat: {seeds:?}"
    );
}

/// A circular arc travelled at the speed whose centripetal load is the
/// whole acceleration budget: the rail is on its curvature cap throughout,
/// so it starts and ends in contact. The span ends on the last absolute
/// time not past its rail's end, as the lowerer's spans do wherever adding
/// the duration rounds up.
fn arc_on_the_curvature_cap() -> AnalyticMoveSpan {
    let (accel, radius, sweep): (f64, f64, f64) = (5000.0, 2.0, 0.5);
    let length = radius * sweep;
    let rail = LawSegment::until_arc(
        0.0,
        0.0,
        (accel * radius).sqrt(),
        ScalarLaw::DiskRail {
            accel,
            kappa0: 1.0 / radius,
            sigma: 0.0,
            brake: false,
        },
        length,
    )
    .expect("a rail on the cap still advances");
    let t_start = 0.211_7;
    let rounded_end = t_start + rail.end_time();
    let t_end = if rounded_end - t_start > rail.end_time() {
        trajectory::next_toward(rounded_end, t_start)
    } else {
        rounded_end
    };
    let arc = PathArc::try_new(
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        radius,
        0.0,
        sweep,
    )
    .expect("arc");
    AnalyticMoveSpan::try_new(
        Move {
            segment: PathSegment::try_new(Segment::Arc(arc), Vec::new()).expect("path"),
            feedrate_mm_s: 200.0,
            limits: VelocityLimits::try_new(200.0, accel, 0.1, f64::INFINITY).expect("limits"),
            source: SourceRange {
                start_line: 1,
                end_line: 1,
            },
        },
        Arc::from([rail]),
        0.0,
        t_start,
        t_end,
        Arc::from([0.0, 0.0, 0.0]),
        SurfaceMode::None,
    )
    .expect("span")
}

#[test]
fn a_source_ending_on_the_curvature_cap_grades_its_seeds_toward_its_end() {
    let span = arc_on_the_curvature_cap();
    let (seeds, contacts) = cap_graded_seeds(&span);
    assert!(
        contacts.contains(&span.t_end),
        "the contact the span ends in must be one of its own: {contacts:?} in [{}, {}]",
        span.t_start,
        span.t_end
    );
    assert!(
        seeds
            .iter()
            .all(|&seed| seed >= span.t_start && seed <= span.t_end),
        "a seed left the span [{}, {}]: {seeds:?}",
        span.t_start,
        span.t_end
    );
    let nearest_below_end = seeds
        .iter()
        .filter(|&&seed| seed < span.t_end)
        .map(|&seed| span.t_end - seed)
        .fold(f64::INFINITY, f64::min);
    assert!(
        nearest_below_end < 2.0 * SINGULAR_SEED_GRADING_FLOOR_S,
        "the grading stops {nearest_below_end} s short of the end contact: {seeds:?}"
    );
}

#[test]
fn a_smooth_edge_keeps_its_seam_acceleration_across_a_wide_sliver() {
    let t0 = 1.0;
    let sig = ConstantJerk { jerk: STEEP_JERK };
    let seam = trajectory::next_toward(t0, 2.0);
    let past_sliver = t0 + edge_inset(SPAN_S, FLOOR, WIDE_SLIVER_S);
    assert!(
        (sig.eval_pva(past_sliver).2 - sig.eval_pva(seam).2).abs() > TOL.accel_mm_s2,
        "the sliver must be wide enough for jerk alone to move the acceleration past the \
         tolerance"
    );
    let (_, acceleration) =
        edge_velocity_and_acceleration(&sig, 0, seam, past_sliver, TOL).unwrap();
    assert_eq!(
        acceleration,
        sig.eval_pva(seam).2,
        "jerk across a wide sliver is no break: the edge must keep its seam acceleration"
    );
}

#[test]
fn a_break_in_a_wide_sliver_anchors_to_its_far_side_at_the_seam() {
    let t0 = 1.0;
    let step_by = 5000.0;
    let sig = JerkWithStep {
        jerk: STEEP_JERK,
        step: t0 + 0.5 * WIDE_SLIVER_S,
        step_by,
    };
    let seam = trajectory::next_toward(t0, 2.0);
    let past_sliver = t0 + edge_inset(SPAN_S, FLOOR, WIDE_SLIVER_S);
    let (_, acceleration) =
        edge_velocity_and_acceleration(&sig, 0, seam, past_sliver, TOL).unwrap();
    let far_side_at_seam = STEEP_JERK * seam + step_by;
    assert!(
        (acceleration - far_side_at_seam).abs() <= 1e-3 * TOL.accel_mm_s2,
        "the edge anchored to {acceleration}, the far side of the break reads \
         {far_side_at_seam} at the seam"
    );
}

#[test]
fn a_break_in_a_wide_sliver_at_the_span_end_anchors_to_its_near_side_at_the_seam() {
    let t1 = 1.0;
    let step_by = 5000.0;
    let sig = JerkWithStep {
        jerk: STEEP_JERK,
        step: t1 - 0.5 * WIDE_SLIVER_S,
        step_by,
    };
    let seam = trajectory::next_toward(t1, 0.0);
    let past_sliver = t1 - edge_inset(SPAN_S, FLOOR, WIDE_SLIVER_S);
    let (_, acceleration) =
        edge_velocity_and_acceleration(&sig, 0, seam, past_sliver, TOL).unwrap();
    let inside_side_at_seam = STEEP_JERK * seam;
    assert!(
        (acceleration - inside_side_at_seam).abs() <= 1e-3 * TOL.accel_mm_s2,
        "the end edge anchored to {acceleration}, the span's side of the break reads \
         {inside_side_at_seam} at the seam"
    );
}
