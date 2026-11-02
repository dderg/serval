//! How the ladder reads a span's anchoring edge state, and how coalescing
//! reports the seeds it absorbs.

use super::{
    EdgeSlivers, FitTol, TrackSignal, coalesce_degenerate_seeds, edge_inset,
    edge_velocity_and_acceleration,
};

const FLOOR: f64 = 2e-9;
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
    let slivers = coalesce_degenerate_seeds(&mut seeds, FLOOR);
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
    let slivers = coalesce_degenerate_seeds(&mut seeds, FLOOR);
    assert_eq!(seeds, vec![0.0, 1.0, 2.0]);
    assert_eq!(slivers[0].start, 0.5 * FLOOR);
    assert!(
        (slivers[1].end - 0.7 * FLOOR).abs() <= 1e-15,
        "{:?}",
        slivers[1]
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
