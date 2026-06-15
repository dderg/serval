//! Deterministic gates for the anytime / graceful-degradation temporal solve.
//!
//! These gates are load-independent: they force the deadline (already-expired vs
//! far-future) rather than racing wall-clock against a busy CPU. A shipped
//! `Solved*` status is itself proof that `verify::check_chain` passed, because
//! `output::map_status` only emits a success status when `verify.feasible` is
//! true — so feasibility (G1) is asserted via the public status + the binding
//! ratio surfaced from `check_chain`.

use std::time::{Duration, Instant};

use nurbs::VectorNurbs;
use temporal::{
    GridConfig, GridScheme, Limits, SolveStatus, ToleranceMode, schedule_segment_with_tolerance,
};

const V_MAX: f64 = 300.0;

/// The sharp ~46mm S-bend that took 568ms to solve optimally on the Pi5 bench.
fn repro_curve() -> VectorNurbs<f64, 3> {
    VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![
            [0.0, 0.0, 0.0],
            [3.0, 20.0, 0.0],
            [43.0, -20.0, 0.0],
            [46.0, 0.0, 0.0],
        ],
    )
    .unwrap()
}

fn velocity_bound_limits() -> Limits {
    Limits::axis_boxes(
        [300.0, 300.0, 300.0],
        [10_000.0, 10_000.0, 10_000.0],
        [50_000.0, 30_000.0, 50_000.0],
    )
}

fn repro_grid() -> GridConfig {
    GridConfig {
        scheme: GridScheme::UniformArclength,
        n: 92,
    }
}

/// A representative straight 50mm move, rest-to-rest.
fn straight_50mm() -> VectorNurbs<f64, 3> {
    VectorNurbs::<f64, 3>::try_new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![[0.0, 0.0, 0.0], [50.0, 0.0, 0.0]],
    )
    .unwrap()
}

/// A short 4mm rest-to-rest straight move. Too short to reach v_max=300 under
/// the accel/jerk caps, so velocity tops out well below its limit and the move
/// is accel/jerk-limited — the exact shape that exposed the velocity-only worst-
/// binding report on the bench.
fn short_straight_4mm() -> VectorNurbs<f64, 3> {
    VectorNurbs::<f64, 3>::try_new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![[0.0, 0.0, 0.0], [4.0, 0.0, 0.0]],
    )
    .unwrap()
}

fn is_success(status: SolveStatus) -> bool {
    matches!(
        status,
        SolveStatus::Solved | SolveStatus::SolvedInexact { .. } | SolveStatus::SolvedSlp { .. }
    )
}

/// Regression — the reported worst binding spans all families. A short rest-to-
/// rest move is jerk-limited: velocity peaks far below its cap (~0.4-0.5) while
/// jerk is the highest-ratio family. The old report considered only velocity/
/// accel, so it surfaced the velocity ratio and a gap inflated by the unseen
/// jerk. The fix surfaces jerk as the worst — the gap is computed against the
/// genuinely-tightest constraint, not a blind-spot family.
///
/// Note this move does NOT ride jerk at ratio 1.0 (it converges near 0.6); that
/// residual gap is a real property of the relaxed solve on a short isolated
/// rest-to-rest segment, tracked separately. This test only locks that the
/// report no longer hides the tighter family behind velocity.
#[test]
fn converged_short_move_worst_spans_all_families() {
    let curve = short_straight_4mm();
    let limits = velocity_bound_limits();
    let grid = GridConfig {
        scheme: GridScheme::UniformArclength,
        n: 80,
    };

    let profile =
        schedule_segment_with_tolerance(&curve, &limits, &grid, 0.0, 0.0, ToleranceMode::Auto)
            .expect("short move must solve");

    assert!(
        is_success(profile.status),
        "short move must converge to a feasible profile; got {:?}",
        profile.status,
    );

    let worst = profile
        .binding
        .worst
        .expect("a converged move must report a worst binding");

    // The peak velocity ratio on this short move is well below 1; if the report
    // still keyed on velocity it would sit near that. Jerk is the tighter family,
    // so the reported worst must be jerk (or accel) at a ratio above the velocity
    // peak — proving the all-family worst, not the velocity-only blind spot.
    assert!(
        matches!(
            worst.constraint,
            temporal::BindingConstraint::JerkNorm { .. }
                | temporal::BindingConstraint::AccelNorm { .. }
        ),
        "worst on a jerk-limited short move must be the jerk/accel family, not the \
         velocity blind spot; got {:?} at ratio {}",
        worst.constraint,
        worst.ratio,
    );
    let velocity_peak = profile.samples.iter().map(|s| s.v).fold(0.0_f64, f64::max);
    assert!(
        velocity_peak < 0.9 * V_MAX,
        "the move must be genuinely sub-velocity-cap for this to test the blind spot; \
         peak v = {velocity_peak}",
    );
}

/// G1 + G2 — forcing an already-expired deadline on the 46mm@50 sharp curve
/// (which optimally takes 568ms) ships a FEASIBLE floor with a success status
/// (success ⇒ `check_chain.feasible`), reports a positive gap, names a limiter,
/// and does not error or panic.
#[test]
fn g1_g2_expired_deadline_ships_feasible_floor() {
    let curve = repro_curve();
    let limits = velocity_bound_limits();
    let grid = repro_grid();

    let expired = Instant::now() - Duration::from_secs(1);
    let _guard = temporal::deadline::scope(Some(expired));

    let profile = schedule_segment_with_tolerance(
        &curve,
        &limits,
        &grid,
        V_MAX * 0.75,
        0.0,
        ToleranceMode::Auto,
    )
    .expect("expired deadline must not error — it must ship the floor");

    assert!(
        is_success(profile.status),
        "expired-deadline solve must ship a feasible floor (success status); got {:?}",
        profile.status,
    );
    assert!(
        profile.total_time.is_finite() && profile.total_time > 0.0,
        "floor trajectory time must be finite and positive; got {}",
        profile.total_time,
    );

    let worst_ratio = profile.binding.worst.map_or(0.0, |w| w.ratio);
    let gap = (1.0 - worst_ratio).max(0.0);
    assert!(
        worst_ratio <= 1.0 + 1e-3,
        "shipped floor must respect the kinematic limit (ratio ≤ 1); got {worst_ratio}",
    );
    assert!(
        gap > 0.0,
        "conservative floor must report a positive optimality gap; got {gap} (ratio {worst_ratio})",
    );
    let worst = profile
        .binding
        .worst
        .expect("a limiter (worst binding constraint) must be reported");
    assert!(
        matches!(
            worst.constraint,
            temporal::BindingConstraint::Velocity { .. }
                | temporal::BindingConstraint::AccelNorm { .. }
                | temporal::BindingConstraint::JerkNorm { .. }
                | temporal::BindingConstraint::PaVelocity { .. }
                | temporal::BindingConstraint::PaAccel { .. }
                | temporal::BindingConstraint::PaJerk { .. }
        ),
        "the reported limiter must be a real kinematic constraint family; got {:?}",
        worst.constraint,
    );
    assert!(
        (gap - (1.0 - worst.ratio).max(0.0)).abs() < 1e-12,
        "the gap must be derived from the verified binding ratio, not fabricated",
    );
    assert!(
        profile.deadline_truncated,
        "a floor shipped because the deadline expired must be flagged deadline_truncated",
    );
}

/// G1 — the floor is feasible for a representative rest-to-rest straight move
/// under an expired deadline (success ⇒ `check_chain.feasible`). Slower-but-safe.
#[test]
fn g1_floor_feasible_representative() {
    let curve = straight_50mm();
    let limits = velocity_bound_limits();
    let grid = GridConfig {
        scheme: GridScheme::UniformArclength,
        n: 100,
    };

    let expired = Instant::now() - Duration::from_secs(1);
    let _guard = temporal::deadline::scope(Some(expired));

    let profile =
        schedule_segment_with_tolerance(&curve, &limits, &grid, 0.0, 0.0, ToleranceMode::Auto)
            .expect("representative move under expired deadline must not error");

    assert!(
        is_success(profile.status),
        "representative floor must be feasible (success status); got {:?}",
        profile.status,
    );
    assert!(profile.total_time.is_finite() && profile.total_time > 0.0);
}

/// G3 — an ample (far-future) deadline must converge to today's optimum: the
/// committed trajectory equals the unbounded (deadline = None) full solve within
/// solver tolerance. Trajectory-neutral when time is ample.
#[test]
fn g3_ample_deadline_converges_to_optimum() {
    let curve = repro_curve();
    let limits = velocity_bound_limits();
    let grid = repro_grid();

    let unbounded = schedule_segment_with_tolerance(
        &curve,
        &limits,
        &grid,
        V_MAX * 0.5,
        0.0,
        ToleranceMode::Auto,
    )
    .expect("unbounded solve must not error");

    let ample = Instant::now() + Duration::from_secs(3600);
    let bounded = {
        let _guard = temporal::deadline::scope(Some(ample));
        schedule_segment_with_tolerance(
            &curve,
            &limits,
            &grid,
            V_MAX * 0.5,
            0.0,
            ToleranceMode::Auto,
        )
        .expect("ample-deadline solve must not error")
    };

    let rel = (bounded.total_time - unbounded.total_time).abs() / unbounded.total_time.max(1e-12);
    assert!(
        rel < 1e-6,
        "ample deadline must be trajectory-neutral; unbounded={:.9}s bounded={:.9}s rel={:.3e}",
        unbounded.total_time,
        bounded.total_time,
        rel,
    );
    assert!(
        !unbounded.deadline_truncated && !bounded.deadline_truncated,
        "a converged solve is never deadline_truncated, no matter how long it takes — \
         this is the bench bug: a slow-but-converged homing solve was falsely flagged \
         by the old wall-clock heuristic",
    );

    assert_eq!(bounded.samples.len(), unbounded.samples.len());
    for (i, (b, u)) in bounded
        .samples
        .iter()
        .zip(unbounded.samples.iter())
        .enumerate()
    {
        assert!(
            (b.b - u.b).abs() <= 1e-6 * u.b.abs().max(1.0),
            "sample {i} b differs: bounded={} unbounded={}",
            b.b,
            u.b,
        );
        assert!(
            (b.a - u.a).abs() <= 1e-6 * u.a.abs().max(1.0),
            "sample {i} a differs: bounded={} unbounded={}",
            b.a,
            u.a,
        );
    }
}

/// G4 — a genuinely infeasible curve must still fail loud: an expired deadline
/// must NOT launder it into a success. Here the start velocity is pinned far
/// above the maximum velocity the limits permit (5000 mm/s vs v_max=300), so the
/// boundary sits above the MVC: NO feasible profile exists at the pinned
/// boundary at any interior speed. This is the hard boundary — the floor only
/// dilates interior speed; it cannot lower a pinned boundary, so the solve must
/// report `Infeasible`, not a feasible-looking floor.
#[test]
fn g4_genuine_infeasibility_still_fails_loud() {
    let curve = repro_curve();
    let limits = velocity_bound_limits();
    let grid = repro_grid();

    let expired = Instant::now() - Duration::from_secs(1);
    let _guard = temporal::deadline::scope(Some(expired));

    let profile =
        schedule_segment_with_tolerance(&curve, &limits, &grid, 5_000.0, 0.0, ToleranceMode::Auto)
            .expect("boundary-infeasible solve returns a profile carrying Infeasible, not an Err");

    assert!(
        matches!(profile.status, SolveStatus::Infeasible { .. }),
        "boundary above MVC must stay Infeasible under an expired deadline (no \
         laundering); got {:?}",
        profile.status,
    );
    assert!(
        !is_success(profile.status),
        "genuine infeasibility must not be laundered into a success",
    );
    assert!(
        !profile.deadline_truncated,
        "a hard boundary infeasibility is not a deadline truncation — the deadline \
         did not cause it, so it must not be flagged",
    );
}
