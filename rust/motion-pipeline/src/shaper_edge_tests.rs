//! How the ladder reads a span's anchoring edge state, and how coalescing
//! reports the seeds it absorbs.

use std::sync::Arc;

use geometry::path::{Arc as PathArc, PathSegment, Segment};
use geometry::{LawSegment, Move, ScalarLaw, SourceRange, VelocityLimits};
use trajectory::{AnalyticMoveSpan, SurfaceMode};

use super::{
    AxisFit, CrowdedVelocityBreaks, EdgeSlivers, Enclosure, FitTol, LadderFloors,
    PostProcessError, SignalBreak, TrackSignal, breaks_among, cap_contact_seeds,
    coalesce_degenerate_seeds, edge_inset, edge_velocity_and_acceleration, fit_axis_from_signal,
    plan_over_seeds, seed_floors, test_signals,
};
use crate::lowering::SeamContinuity;

const FLOOR: f64 = 2e-9;
const POSITION_TOL: f64 = 1e-4;
const TOL: FitTol = FitTol::absolute(1e-3, 50.0);

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

/// Accelerating at `accel` from rest at the origin until `reversal`, then
/// braking at `accel`: the accelerate-then-brake profile a planner ends a
/// move on when its two rails cross just short of the end.
struct AccelerateThenBrake {
    reversal: f64,
    accel: f64,
}

impl TrackSignal for AccelerateThenBrake {
    fn eval_pva(&self, t: f64) -> (f64, f64, f64) {
        let (r, a) = (self.reversal, self.accel);
        if t < r {
            return (0.5 * a * t * t, a * t, a);
        }
        let d = t - r;
        (
            0.5 * a * r * r + a * r * d - 0.5 * a * d * d,
            a * r - a * d,
            -a,
        )
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
    let acceleration = edge_velocity_and_acceleration(
        &sig,
        0,
        trajectory::next_toward(t0, 2.0),
        t0 + edge_inset(1e-3, FLOOR, 0.0),
        false,
        false,
        TOL,
    )
    .unwrap()
    .acceleration;
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
    let acceleration = edge_velocity_and_acceleration(
        &sig,
        0,
        seam,
        t0 + edge_inset(1e-3, FLOOR, 0.0),
        false,
        false,
        TOL,
    )
    .unwrap()
    .acceleration;
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
    let breaks = [SignalBreak {
        t: ratio_step,
        velocity_step: -3.4,
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
        SignalBreak {
            t: 1.0,
            velocity_step: 3.4,
        },
        SignalBreak {
            t: crowding,
            velocity_step: -1.0,
        },
        SignalBreak {
            t: near_end,
            velocity_step: 2.0,
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
        SignalBreak {
            t: 1.0,
            velocity_step: 1.0,
        },
        SignalBreak {
            t: crowding,
            velocity_step: step,
        },
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

/// The extruder's track where the toolhead turns around at `at`: slowing at
/// `accel` into the reversal and speeding up at `accel` out of it, its
/// velocity `accel·|t − at|` continuous and its acceleration stepping by
/// `2·accel`.
struct ReversingSpeed {
    at: f64,
    accel: f64,
    position: f64,
}

impl TrackSignal for ReversingSpeed {
    fn eval_pva(&self, t: f64) -> (f64, f64, f64) {
        let since = t - self.at;
        let acceleration = if since < 0.0 {
            -self.accel
        } else {
            self.accel
        };
        (
            0.5 * self.accel * since * since.abs() + self.position,
            self.accel * since.abs(),
            acceleration,
        )
    }

    fn enclosure_on(&self, lo: f64, hi: f64, accuracy: [f64; 3]) -> Option<Enclosure<'_>> {
        test_signals::modelled((lo, hi), accuracy, &[self.at], |window| {
            let half_acceleration = if window.center < self.at {
                -0.5 * self.accel
            } else {
                0.5 * self.accel
            };
            test_signals::polynomial(window, self.at, &[self.position, 0.0, half_acceleration])
        })
    }
}

/// The extruder's reversal on the first layers of the bench's advance-free
/// chain (voron_first_layers on bench_no_advance.cfg at 125.63 s), its fit
/// span ending on the support's end 24.6 ns past it.
const FIRST_LAYERS_REVERSAL: ReversingSpeed = ReversingSpeed {
    at: 125.630_145_291_922,
    accel: 168.125,
    position: 0.042_523_077_746,
};
const FIRST_LAYERS_SPAN: (f64, f64) = (125.625_357_523_8, 125.630_145_316_536);

/// The fit of [`FIRST_LAYERS_REVERSAL`] seeded at the reversal and at a
/// ratio crossing `crossing_floors` of a floor past it, where the crossing
/// time is the odometer's rounding and the signal does not break.
fn first_layers_seeds(crossing_floors: f64) -> (AxisFit<'static>, Vec<f64>) {
    let fit = AxisFit {
        axis: 3,
        t_start: FIRST_LAYERS_SPAN.0,
        t_end: FIRST_LAYERS_SPAN.1,
        seed_breakpoints: &[],
        fit_tol: TOL,
        fit_context: "follower_source",
        seam_continuity: SeamContinuity::Velocity,
    };
    let reversal = FIRST_LAYERS_REVERSAL.at;
    let mut seeds = vec![fit.t_start, reversal, fit.t_end];
    let floor = seed_floors(&fit, &FIRST_LAYERS_REVERSAL, &seeds).cubic;
    seeds.insert(2, reversal + crossing_floors * floor);
    (fit, seeds)
}

#[test]
fn a_seed_within_a_floor_past_an_acceleration_break_does_not_coalesce_the_break_away() {
    let (fit, seeds) = first_layers_seeds(0.85);
    let crossing = seeds[2];
    let plan = plan_over_seeds(&fit, &FIRST_LAYERS_REVERSAL, seeds).expect("the seeds plan");
    assert!(
        crossing - FIRST_LAYERS_REVERSAL.at < plan.floors.cubic,
        "the crossing must crowd the reversal within the floor {}",
        plan.floors.cubic
    );
    assert!(
        plan.seeds.contains(&FIRST_LAYERS_REVERSAL.at),
        "the reversal at {} steps the acceleration by {} and was coalesced onto the crossing \
         at {crossing}: {:?}",
        FIRST_LAYERS_REVERSAL.at,
        2.0 * FIRST_LAYERS_REVERSAL.accel,
        plan.seeds
    );
}

#[test]
fn the_fit_holds_the_acceleration_across_a_reversal_crowded_by_a_seed() {
    let (fit, seeds) = first_layers_seeds(0.85);
    let fit = AxisFit {
        seed_breakpoints: &seeds[1..3],
        ..fit
    };
    let track = fit_axis_from_signal(fit, &FIRST_LAYERS_REVERSAL)
        .expect("a reversal crowded by a seed fits");
    let around = (FIRST_LAYERS_REVERSAL.at - 25e-9, fit.t_end);
    let samples = 2000;
    let worst = (0..=samples)
        .map(|k| around.0 + (around.1 - around.0) * f64::from(k) / f64::from(samples))
        .filter(|&t| t != FIRST_LAYERS_REVERSAL.at)
        .map(|t| {
            let mut derivatives = [0.0; 3];
            nurbs::eval::eval_derivatives(
                track.control_points(),
                track.knots(),
                track.degree(),
                t,
                2,
                &mut derivatives,
            );
            (
                (derivatives[2] - FIRST_LAYERS_REVERSAL.eval_pva(t).2).abs(),
                t,
            )
        })
        .fold((0.0, 0.0), |worst: (f64, f64), miss| {
            if miss.0 > worst.0 { miss } else { worst }
        });
    assert!(
        worst.0 <= fit.fit_tol.accel_mm_s2,
        "the fit misses the acceleration by {} mm/s² at {} around the reversal at {}, past \
         the {} mm/s² tolerance; knots {:?}",
        worst.0,
        worst.1,
        FIRST_LAYERS_REVERSAL.at,
        fit.fit_tol.accel_mm_s2,
        track
            .knots()
            .iter()
            .filter(|&&knot| knot >= around.0)
            .collect::<Vec<_>>()
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
    let floors = LadderFloors {
        jerk_noise_cubed: 0.0,
        high_degree: FLOOR,
        cubic: FLOOR,
    };
    assert_eq!(
        breaks_among(&sig, &[0.0, sig.at, 0.02], TOL, floors),
        vec![SignalBreak {
            t: sig.at,
            velocity_step: sig.after - sig.before,
        }]
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
            limits: VelocityLimits::try_new(
                200.0,
                accel,
                geometry::CornerBudget::Shaped(0.1),
                f64::INFINITY,
            )
            .expect("limits"),
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
fn a_source_ending_on_the_curvature_cap_seeds_its_end_contact() {
    let span = arc_on_the_curvature_cap();
    let (seeds, contacts) = cap_contact_seeds(&span);
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
    assert!(
        contacts.iter().all(|contact| seeds.contains(contact)),
        "every contact must be a seed: {contacts:?} among {seeds:?}"
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
    let acceleration = edge_velocity_and_acceleration(&sig, 0, seam, past_sliver, true, false, TOL)
        .unwrap()
        .acceleration;
    assert_eq!(
        acceleration,
        sig.eval_pva(seam).2,
        "jerk across a wide sliver is no break: the edge must keep its seam acceleration"
    );
}

/// Acceleration rising at `jerk` until `kink`, at `jerk_after` from there.
struct JerkKink {
    kink: f64,
    jerk: f64,
    jerk_after: f64,
}

impl TrackSignal for JerkKink {
    fn eval_pva(&self, t: f64) -> (f64, f64, f64) {
        let past_kink = (t - self.kink).max(0.0);
        let extra = self.jerk_after - self.jerk;
        let j = self.jerk;
        (
            j * t * t * t / 6.0 + extra * past_kink * past_kink * past_kink / 6.0,
            0.5 * j * t * t + 0.5 * extra * past_kink * past_kink,
            j * t + extra * past_kink,
        )
    }
}

/// A rail barely off its curvature cap changes its jerk this way a
/// microsecond past a phase seam: the extrapolation through the reads past
/// the sliver misses the seam, but the acceleration is continuous.
#[test]
fn a_jerk_change_within_the_extrapolation_is_no_break() {
    let t0 = 1.0;
    let sig = JerkKink {
        kink: t0 + 1.4 * WIDE_SLIVER_S,
        jerk: 2e8,
        jerk_after: 4e8,
    };
    let seam = trajectory::next_toward(t0, 2.0);
    let past_sliver = t0 + edge_inset(SPAN_S, FLOOR, WIDE_SLIVER_S);
    let extrapolated = 2.0 * sig.eval_pva(past_sliver).2 - sig.eval_pva(2.0 * past_sliver - seam).2;
    assert!(
        (extrapolated - sig.eval_pva(seam).2).abs() > TOL.accel_mm_s2,
        "the jerk change must move the extrapolation past the tolerance"
    );
    let acceleration = edge_velocity_and_acceleration(&sig, 0, seam, past_sliver, true, false, TOL)
        .unwrap()
        .acceleration;
    assert_eq!(
        acceleration,
        sig.eval_pva(seam).2,
        "a continuous acceleration hides no break: the edge must keep its seam acceleration"
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
    let acceleration = edge_velocity_and_acceleration(&sig, 0, seam, past_sliver, true, false, TOL)
        .unwrap()
        .acceleration;
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
    let acceleration = edge_velocity_and_acceleration(&sig, 0, seam, past_sliver, true, false, TOL)
        .unwrap()
        .acceleration;
    let inside_side_at_seam = STEEP_JERK * seam;
    assert!(
        (acceleration - inside_side_at_seam).abs() <= 1e-3 * TOL.accel_mm_s2,
        "the end edge anchored to {acceleration}, the span's side of the break reads \
         {inside_side_at_seam} at the seam"
    );
}

#[test]
fn a_break_in_a_wide_sliver_carries_its_near_side_velocity_to_the_seam() {
    let t1 = 1.0;
    let accel = 15_000.0;
    let sig = AccelerateThenBrake {
        reversal: t1 - 0.6 * WIDE_SLIVER_S,
        accel,
    };
    let seam = trajectory::next_toward(t1, 0.0);
    let past_sliver = t1 - edge_inset(SPAN_S, FLOOR, WIDE_SLIVER_S);
    let anchor =
        edge_velocity_and_acceleration(&sig, 0, seam, past_sliver, true, false, TOL).unwrap();
    let near_side_at_seam = accel * seam;
    assert!(
        (anchor.velocity - near_side_at_seam).abs() <= 1e-9 * near_side_at_seam,
        "the end edge anchored to velocity {}, the span's side of the break reads {near_side_at_seam} \
         at the seam: the brake the sliver hides must move onto the seam, not bend the piece",
        anchor.velocity
    );
    assert_eq!(anchor.acceleration, accel);
}

#[test]
fn carrying_a_sliver_velocity_past_the_position_tolerance_fails_loudly() {
    let t1 = 1.0;
    let sig = AccelerateThenBrake {
        reversal: t1 - 0.6 * WIDE_SLIVER_S,
        accel: 15_000.0,
    };
    let seam = trajectory::next_toward(t1, 0.0);
    let past_sliver = t1 - edge_inset(SPAN_S, FLOOR, WIDE_SLIVER_S);
    let tight = FitTol::absolute(1e-12, TOL.accel_mm_s2);
    let anchor = edge_velocity_and_acceleration(&sig, 0, seam, past_sliver, true, false, tight);
    assert!(
        matches!(anchor, Err(PostProcessError::SliverVelocityCarry { .. })),
        "a hidden break whose velocity step shifts position past the tolerance must fail \
         loudly, got {anchor:?}"
    );
}
