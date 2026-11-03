//! Property-based fuzz of the full fitter → planner → continuous-trajectory
//! pipeline: random adversarial move streams under random valid configs,
//! checked against the kinematic-invariant oracle in `pipeline_snapshot::audit`.
//!
//! Every case runs at infinite jerk, the configuration the continuous
//! pipeline ships. Acceleration steps are the plan, so the oracle's per-axis
//! acceleration and jerk rails do not apply. Carrier states must remain finite
//! over contiguous time, with no interval narrower than device step-time
//! resolution, no acceleration explosion, C0 position seams, and valid
//! velocity bounds.
//!
//! `hard_invariants_hold` draws a fresh seed per run and prints it
//! (`PROPTEST_RNG_SEED=<seed>` reproduces); `hard_invariants_hold_on_the_pinned_corpus`
//! is the same property on a fixed 256-case corpus. Hunting for the smoothness
//! budgets happens on the `#[ignore]`d `target_budgets_hold`, which fuzzes the
//! full input space (including the known-broken families the CI tier steers around), and additionally
//! asserts the intended smoothness budgets (seam C0/C1, velocity limits)
//! that current lowering is known to violate. Run it with
//! `cargo nextest run -p pipeline-snapshot --run-ignored all`, ideally with
//! a large `PROPTEST_CASES`. Every bug it finds gets pinned below as an
//! `#[ignore]`d regression test until fixed.

use motion_pipeline::live_config;
use pipeline_snapshot::audit::{
    AuditBudgets, AuditReport, Violation, ViolationKind, audit_trajectory,
};
use pipeline_snapshot::waypoints::Waypoint;
use pipeline_snapshot::{
    CartesianLimits, ExactTrajectory, PlannerConfig, SnapshotParams, pipeline_snapshot,
};
use proptest::prelude::*;
use proptest::test_runner::RngSeed;

#[derive(Debug, Clone, Copy)]
struct FuzzLimits {
    max_velocity: f64,
    max_accel: f64,
    square_corner_velocity: f64,
}

#[derive(Debug, Clone, Copy)]
enum Turn {
    Continue,
    Reverse,
    Absolute(f64),
}

#[derive(Debug, Clone, Copy)]
struct MoveSpec {
    turn: Turn,
    length_mm: f64,
    feed_mm_s: f64,
    extrude_ratio: f64,
    z_step_mm: f64,
}

fn limits_strategy() -> impl Strategy<Value = FuzzLimits> {
    let scv = prop_oneof![1 => Just(0.0), 3 => 1e-4..0.5_f64, 6 => 0.5..20.0_f64];
    (20.0..600.0_f64, 500.0..50_000.0_f64, scv).prop_map(|(v, a, scv)| FuzzLimits {
        max_velocity: v,
        max_accel: a,
        square_corner_velocity: scv.min(v / 4.0),
    })
}

fn turn_strategy() -> impl Strategy<Value = Turn> {
    prop_oneof![
        2 => Just(Turn::Continue),
        1 => Just(Turn::Reverse),
        4 => (0.0..std::f64::consts::TAU).prop_map(Turn::Absolute),
    ]
}

/// `full` includes z steps on spatial moves. The CI tier fuzzes planar-only:
/// z-carrying moves are a known-broken family (see
/// `feed_drop_with_z_step_escapes_profile_window` — even a collinear decel
/// with a z step escapes the lowering profile window).
fn move_strategy(full: bool) -> impl Strategy<Value = MoveSpec> {
    let z_step = if full {
        prop_oneof![9 => Just(0.0), 1 => -0.4..0.4_f64].boxed()
    } else {
        Just(0.0).boxed()
    };
    (
        turn_strategy(),
        -3.5..1.9_f64,
        1.0..700.0_f64,
        0.0..0.1_f64,
        z_step,
    )
        .prop_map(|(turn, log_len, feed, extrude_ratio, z_step_mm)| MoveSpec {
            turn,
            length_mm: libm::pow(10.0, log_len),
            feed_mm_s: feed,
            extrude_ratio,
            z_step_mm,
        })
}

fn waypoints(moves: &[MoveSpec]) -> Vec<Waypoint> {
    let (mut x, mut y, mut z, mut e) = (150.0, 150.0, 0.2, 0.0);
    let mut heading = 0.0_f64;
    let mut points = vec![Waypoint::at([x, y, z, e], 0.0)];
    for m in moves {
        heading = match m.turn {
            Turn::Continue => heading,
            Turn::Reverse => heading + std::f64::consts::PI,
            Turn::Absolute(a) => a,
        };
        x += m.length_mm * libm::cos(heading);
        y += m.length_mm * libm::sin(heading);
        z = (z + m.z_step_mm).max(0.0);
        e += m.extrude_ratio * m.length_mm;
        points.push(Waypoint::at([x, y, z, e], m.feed_mm_s));
    }
    points
}

fn run_case(limits: FuzzLimits, moves: &[MoveSpec]) -> (ExactTrajectory, AuditReport) {
    run_waypoints(limits, &waypoints(moves))
}

fn run_waypoints(limits: FuzzLimits, waypoints: &[Waypoint]) -> (ExactTrajectory, AuditReport) {
    let corner_deviation =
        geometry::corner_deviation_from_scv(limits.square_corner_velocity, limits.max_accel);
    let params = SnapshotParams {
        max_velocity: limits.max_velocity,
        max_accel: limits.max_accel,
        corner_deviation,
        max_jerk: f64::INFINITY,
        max_z_velocity: None,
        max_z_accel: None,
        max_extrude_only_velocity: None,
        max_extrude_only_accel: None,
        max_path_deviation: None,
        max_accel_deviation: None,
        axis_decls: Vec::new(),
        post_processor_decls: Vec::new(),
    };
    let snapshot = pipeline_snapshot(waypoints, params).expect("valid fuzz input");
    let traj = snapshot.trajectory;
    let config = live_config::stream_config(&PlannerConfig {
        cartesian: CartesianLimits::without_z_caps(
            limits.max_velocity,
            limits.max_accel,
            f64::INFINITY,
            corner_deviation,
        ),
        ..PlannerConfig::default()
    })
    .expect("strategy generates valid limits");
    let report = audit_trajectory(&traj, &config, &AuditBudgets::for_config(&config));
    (traj, report)
}

/// The smoothness budgets that still bind under infinite jerk: a seam
/// velocity step and the velocity rail. Per-axis accel and jerk rails are the
/// oracle's jerk-limited tier and say nothing here.
fn xy_smoothness_spikes(report: &AuditReport) -> Vec<&Violation> {
    report
        .target
        .iter()
        .filter(|v| {
            v.axis < 2
                && matches!(
                    v.kind,
                    ViolationKind::SeamVelocity | ViolationKind::Velocity
                )
        })
        .collect()
}

const PINNED_SEED: u64 = 20260708;

fn fresh_seed_config() -> ProptestConfig {
    let seed = std::env::var("PROPTEST_RNG_SEED")
        .ok()
        .and_then(|value| value.parse().ok())
        .unwrap_or_else(|| {
            let nanos = std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .expect("clock after the epoch")
                .as_nanos();
            (nanos as u64) ^ u64::from(std::process::id()).rotate_left(40)
        });
    eprintln!("planner_fuzz seed: reproduce with PROPTEST_RNG_SEED={seed}");
    ProptestConfig {
        rng_seed: RngSeed::Fixed(seed),
        ..ProptestConfig::default()
    }
}

proptest! {
    #![proptest_config(fresh_seed_config())]

    #[test]
    fn hard_invariants_hold(
        limits in limits_strategy(),
        moves in prop::collection::vec(move_strategy(false), 2..40),
    ) {
        let (traj, report) = run_case(limits, &moves);
        prop_assert!(!traj.rows(0).is_empty(), "pipeline produced no X trajectory");
        prop_assert!(report.hard_ok(), "{report}");
    }
}

proptest! {
    #![proptest_config(ProptestConfig {
        cases: 256,
        rng_seed: RngSeed::Fixed(PINNED_SEED),
        ..ProptestConfig::default()
    })]

    #[test]
    fn hard_invariants_hold_on_the_pinned_corpus(
        limits in limits_strategy(),
        moves in prop::collection::vec(move_strategy(false), 2..40),
    ) {
        let (traj, report) = run_case(limits, &moves);
        prop_assert!(!traj.rows(0).is_empty(), "pipeline produced no X trajectory");
        prop_assert!(report.hard_ok(), "{report}");
    }
}

proptest! {
    #[test]
    #[ignore = "#575: E seam velocity budget fails at every ratio step"]
    fn target_budgets_hold(
        limits in limits_strategy(),
        moves in prop::collection::vec(move_strategy(true), 2..40),
    ) {
        let (_, report) = run_case(limits, &moves);
        prop_assert!(report.hard_ok(), "{report}");
        prop_assert!(report.target_ok(), "{report}");
    }
}

/// Found by `hard_invariants_hold`: under a tiny square-corner velocity
/// (0.075 mm/s), an XY reversal with a simultaneous z step makes
/// `plan_velocity_stops` fail with `NonFinite { line_no: 0 }`. Same
/// tiny-positive-scv family as the two tests below — all three pass with
/// scv = 0 or scv >= 0.1.
#[test]
fn reversal_with_z_step_makes_velocity_plan_non_finite() {
    let limits = FuzzLimits {
        max_velocity: 20.0,
        max_accel: 47789.160095826584,
        square_corner_velocity: 0.07502855438839308,
    };
    let moves = [
        MoveSpec {
            turn: Turn::Continue,
            length_mm: 1.0,
            feed_mm_s: 1.0,
            extrude_ratio: 0.0,
            z_step_mm: 0.0,
        },
        MoveSpec {
            turn: Turn::Reverse,
            length_mm: 15.093502432273022,
            feed_mm_s: 1.0,
            extrude_ratio: 0.0,
            z_step_mm: -0.24971454457070866,
        },
    ];
    let (_, report) = run_case(limits, &moves);
    assert!(report.hard_ok(), "{report}");
}

/// Found by `hard_invariants_hold`: under a tiny square-corner velocity
/// (0.0105 mm/s), a micro reversal (2 µm) directly after a z-carrying move
/// tripped lowering's `ScalarProfile` window `debug_assert` — the quintic
/// arc-length profile escaped its position window. Fixed by the wall-aware
/// ride pass: the tiny-scv corner notch's exit is an ascending wall, whose
/// chord the pass no longer rides or lands on.
#[test]
fn z_step_then_micro_reversal_escapes_profile_window() {
    let limits = FuzzLimits {
        max_velocity: 20.0,
        max_accel: 7891.0876463579025,
        square_corner_velocity: 0.010488666819287094,
    };
    let moves = [
        MoveSpec {
            turn: Turn::Continue,
            length_mm: 1.0,
            feed_mm_s: 1.0,
            extrude_ratio: 0.0,
            z_step_mm: 0.1990060586102325,
        },
        MoveSpec {
            turn: Turn::Reverse,
            length_mm: 0.002026842467152674,
            feed_mm_s: 1.0,
            extrude_ratio: 0.0,
            z_step_mm: 0.0,
        },
    ];
    let (_, report) = run_case(limits, &moves);
    assert!(report.hard_ok(), "{report}");
}

/// Found by `hard_invariants_hold`: under a tiny square-corner velocity
/// (0.047 mm/s), a purely planar sharp corner (~179.5° turn between two 1 mm
/// moves) makes `plan_velocity_stops` fail with `NonFinite { line_no: 0 }`.
/// Sweeping the space: scv = 0 exactly is safe at every corner angle, scv of
/// 0.005 fails from 146° up, and no failures were found at scv >= 0.1. Same
/// tiny-positive-scv family as its neighbors.
///
/// Root cause: `solve_line_line` admitted a blend on its trim (> 1 nm) while
/// the clothoid halves came out 2.6e-10 mm long, below the length the velocity
/// plan accepts. Such a corner now stays sharp.
#[test]
fn near_reversal_planar_corner_makes_velocity_plan_non_finite() {
    let limits = FuzzLimits {
        max_velocity: 20.0,
        max_accel: 33192.99751381838,
        square_corner_velocity: 0.047081100007714954,
    };
    let moves = [
        MoveSpec {
            turn: Turn::Absolute(1.423698977372297),
            length_mm: 1.0,
            feed_mm_s: 1.0,
            extrude_ratio: 0.0,
            z_step_mm: 0.0,
        },
        MoveSpec {
            turn: Turn::Absolute(4.556944872947884),
            length_mm: 1.0,
            feed_mm_s: 1.0,
            extrude_ratio: 0.0,
            z_step_mm: 0.0,
        },
    ];
    let (_, report) = run_case(limits, &moves);
    assert!(report.hard_ok(), "{report}");
}

/// Found by `hard_invariants_hold`, and the only pinned failure with nothing
/// degenerate in the input: two collinear 1 mm moves, ordinary limits
/// (scv 16.4), a feed drop from 20.5 to 1 mm/s and a -0.125 mm z step on the
/// second move — vase-mode-like motion — drive lowering's quintic arc-length
/// profile negative, escaping its `[0, s_len]` window `debug_assert`. Until
/// fixed, the CI-tier generator fuzzes planar-only.
///
/// Root cause was the ride pass crossing the cap's step-down wall at the
/// seam as a single j=0 chord phase, so the brake chain carried an
/// instantaneous accel staircase (0 → -22147 → 0) and the seam sample pair
/// `(v=1, a=-22147)` was kinematically impossible for the lowering quintic.
/// Fixed by the wall-aware ride pass: unlandable descents detach/anchor
/// (jerk walls brake onto the wall's end node via the bang-bang
/// boundary-value maneuver; ascending walls detach to flight).
#[test]
fn feed_drop_with_z_step_escapes_profile_window() {
    let limits = FuzzLimits {
        max_velocity: 315.10075079011057,
        max_accel: 33143.074041397005,
        square_corner_velocity: 16.354159311305455,
    };
    let moves = [
        MoveSpec {
            turn: Turn::Continue,
            length_mm: 1.0,
            feed_mm_s: 20.496460264338538,
            extrude_ratio: 0.0,
            z_step_mm: 0.0,
        },
        MoveSpec {
            turn: Turn::Continue,
            length_mm: 1.0,
            feed_mm_s: 1.0,
            extrude_ratio: 0.0,
            z_step_mm: -0.12512212790340144,
        },
    ];
    let (_, report) = run_case(limits, &moves);
    assert!(report.hard_ok(), "{report}");
}

/// Found by `hard_invariants_hold`: purely planar, scv exactly 0, a 37 µm
/// move at feed 451 mm/s decelerating into a 1 mm/s move under a high accel
/// limit made the quintic arc-length profile dip below its window.
/// Root cause was the brake-chain splice rejecting on chord sag, leaving the
/// ride as per-cell chord phases whose staircase acceleration produced
/// kinematically impossible `(v, a)` sample pairs; fixed by deriving the
/// splice joint tolerance from the chord geometry.
#[test]
fn planar_micro_move_decel_escapes_profile_window() {
    let limits = FuzzLimits {
        max_velocity: 20.0,
        max_accel: 26083.903689944673,
        square_corner_velocity: 0.0,
    };
    let moves = [
        MoveSpec {
            turn: Turn::Continue,
            length_mm: 0.0375640321004116,
            feed_mm_s: 451.52187547094076,
            extrude_ratio: 0.0,
            z_step_mm: 0.0,
        },
        MoveSpec {
            turn: Turn::Continue,
            length_mm: 1.0,
            feed_mm_s: 1.0,
            extrude_ratio: 0.0,
            z_step_mm: 0.0,
        },
    ];
    let (_, report) = run_case(limits, &moves);
    assert!(report.hard_ok(), "{report}");
}

/// The corner-shoulder accel cliff (was defect 4's open residual in
/// ride-pass-known-defects.md): 90-degree corners at printer-scale limits
/// stepped the X/Y acceleration at the blend-boundary seams and the final
/// brake to rest by ~11, scaling linearly with feed. Root cause was the
/// straight lowering's micro-phase merge: absorbing a leading nanosecond
/// phase extended the host span without rebasing the host's coefficients,
/// time-shifting it by the merged duration; the resulting `v * lead` position
/// gap at the next joint was then welded shut by the NURBS packing, which
/// converts a C0 gap into a `6 * gap / h^2` acceleration corruption of the
/// short piece that follows. Fixed by Taylor-shifting the host coefficients
/// to the span start. Under infinite jerk that corruption shows up as the
/// welded C0 gap and the explosion it drives — both `hard` — plus the seam
/// velocity step; the per-axis accel rail is not asserted, since an
/// infinite-jerk plan bounds the scalar path's acceleration, not each axis'
/// reconstruction of it.
#[test]
fn perpendicular_corners_keep_seams_and_velocity_in_budget() {
    let limits = FuzzLimits {
        max_velocity: 100.0,
        max_accel: 1000.0,
        square_corner_velocity: 8.0,
    };
    let corner = std::f64::consts::FRAC_PI_2;
    let moves = [
        MoveSpec {
            turn: Turn::Continue,
            length_mm: 15.0,
            feed_mm_s: 100.0,
            extrude_ratio: 0.04,
            z_step_mm: 0.0,
        },
        MoveSpec {
            turn: Turn::Absolute(corner),
            length_mm: 2.0,
            feed_mm_s: 100.0,
            extrude_ratio: 0.04,
            z_step_mm: 0.0,
        },
        MoveSpec {
            turn: Turn::Absolute(0.0),
            length_mm: 2.0,
            feed_mm_s: 100.0,
            extrude_ratio: 0.04,
            z_step_mm: 0.0,
        },
        MoveSpec {
            turn: Turn::Absolute(corner),
            length_mm: 15.0,
            feed_mm_s: 100.0,
            extrude_ratio: 0.04,
            z_step_mm: 0.0,
        },
    ];
    let (_, report) = run_case(limits, &moves);
    assert!(report.hard_ok(), "{report}");
    let spikes = xy_smoothness_spikes(&report);
    assert!(
        spikes.is_empty(),
        "X/Y seam-velocity or velocity past budget: {spikes:?}\n{report}"
    );
}

/// A slow sharp corner whose blend exits with acceleration a fraction of a
/// mm/s² below the rail: the following straight move closes that gap with a
/// ramp lasting well under a microsecond (here ~130 ns for a 0.13 mm/s²
/// step). Lowered as its own Bézier piece, the NURBS pack/extract round trip
/// reconstructed that piece from Bernstein control points that carry the
/// absolute ~120 mm position, so ulp-level rounding divided by h³ came back
/// as ±10 mm/s² acceleration steps at both seams. Fixed by merging phases
/// below the Bernstein corruption floor (`MIN_PHASE_PIECE_S`) into a
/// neighbor. The waypoints are lifted verbatim from the neptune_cube layer_5
/// snapshot case; the epsilon-under-the-rail blend exit only occurs for
/// specific corner geometry, so the coordinates are load-bearing.
#[test]
fn slow_corner_rail_ramp_survives_nurbs_round_trip() {
    let limits = FuzzLimits {
        max_velocity: 100.0,
        max_accel: 1000.0,
        square_corner_velocity: 8.0,
    };
    let feed = 5932.264 / 60.0;
    let waypoints = [
        Waypoint::at([121.401, 114.125, 0.0, 0.0], feed),
        Waypoint::at([121.527, 113.034, 0.0, 0.03696], feed),
        Waypoint::at([126.102, 117.608, 0.0, 0.25458], feed),
    ];
    let (_, report) = run_waypoints(limits, &waypoints);
    assert!(report.hard_ok(), "{report}");
    let spikes = xy_smoothness_spikes(&report);
    assert!(
        spikes.is_empty(),
        "X/Y seam-velocity or velocity past budget: {spikes:?}\n{report}"
    );
}

fn planar(turn: Turn, length_mm: f64, feed_mm_s: f64, extrude_ratio: f64) -> MoveSpec {
    MoveSpec {
        turn,
        length_mm,
        feed_mm_s,
        extrude_ratio,
        z_step_mm: 0.0,
    }
}

/// Found by `hard_invariants_hold` (#562): a free clothoid-pair solve — the
/// facet-consumption chain split at the facet's midpoint, or an arc run's
/// boundary blend — sat at the edge of the pair's existence region, where one
/// half carries the whole turn and the other shrinks to nanometres, emitting
/// a row of a nanosecond or two. A pair half shorter than a device interval
/// at its fastest speed is now infeasible for every free solve.
#[test]
fn facet_consumption_split_leaves_no_sliver_row() {
    let limits = FuzzLimits {
        max_velocity: 20.0,
        max_accel: 42552.63125816086,
        square_corner_velocity: 5.0,
    };
    let moves = [
        planar(
            Turn::Absolute(2.968191818487306),
            0.002493439626094711,
            165.95540533434988,
            0.0,
        ),
        planar(
            Turn::Absolute(1.387277182399567),
            0.0006359771117086327,
            588.2719245715481,
            0.0,
        ),
        planar(Turn::Absolute(1.9715280945107174), 1.0, 1.0, 0.0),
    ];
    let (_, report) = run_case(limits, &moves);
    assert!(report.hard_ok(), "{report}");
}

#[test]
fn same_sense_facet_consumption_leaves_no_sliver_row() {
    let limits = FuzzLimits {
        max_velocity: 100.85130229005617,
        max_accel: 21853.28686118798,
        square_corner_velocity: 10.295157997623004,
    };
    let moves = [
        planar(Turn::Absolute(2.260652648254229), 1.0, 1.0, 0.0),
        planar(
            Turn::Absolute(0.8069452807702043),
            0.004896715143695845,
            1.0,
            0.0,
        ),
        planar(Turn::Absolute(4.586520500087806), 1.0, 1.0, 0.0),
    ];
    let (_, report) = run_case(limits, &moves);
    assert!(report.hard_ok(), "{report}");
}

#[test]
fn arc_run_tail_blend_leaves_no_sliver_row() {
    let limits = FuzzLimits {
        max_velocity: 579.9040202955272,
        max_accel: 45069.717321302596,
        square_corner_velocity: 17.587166725605982,
    };
    let moves = [
        planar(
            Turn::Absolute(3.942866479434897),
            1.0,
            1.0,
            0.09312678484514826,
        ),
        planar(
            Turn::Continue,
            0.0008990155284484794,
            1.0,
            0.059730024030819666,
        ),
        planar(
            Turn::Absolute(4.4081876990164295),
            0.0005882210496939083,
            1.0,
            0.073725797059201,
        ),
        planar(
            Turn::Absolute(5.384155481070599),
            0.0007041462012606629,
            543.1019473451835,
            0.06929877228620371,
        ),
        planar(
            Turn::Continue,
            1.0,
            158.99678961430908,
            0.047124637386670276,
        ),
    ];
    let (_, report) = run_case(limits, &moves);
    assert!(report.hard_ok(), "{report}");
}

/// Found by `target_budgets_hold`: once every follower ramps its ratio, a
/// slow z-carrying move, a fast 1 mm travel and a slow extrusion left the
/// follower's source fit facing a velocity step narrower than the
/// resolution floor at the travel → extrusion seam.
#[test]
fn slow_extrusion_after_a_short_fast_travel_ramps_its_ratio() {
    let limits = FuzzLimits {
        max_velocity: 369.8,
        max_accel: 37419.9,
        square_corner_velocity: 0.0,
    };
    let moves = [
        MoveSpec {
            z_step_mm: -0.185,
            ..planar(Turn::Continue, 1.0, 1.0, 0.0)
        },
        planar(Turn::Continue, 1.0, 472.3, 0.0),
        planar(Turn::Continue, 37.57, 1.0, 0.04166),
    ];
    let (_, report) = run_case(limits, &moves);
    assert!(report.hard_ok(), "{report}");
}

/// Found by `planner_conservation_fuzz`: a fast 1 mm extrusion, a
/// micrometre extrusion turning off it, and a slow travel; the follower's
/// plain source fit ran out of refinement splits on the ramp.
#[test]
fn a_micrometre_extrusion_between_a_fast_line_and_a_slow_travel_ramps_its_ratio() {
    let limits = FuzzLimits {
        max_velocity: 127.389385959449,
        max_accel: 39584.86189423528,
        square_corner_velocity: 7.296773245024728,
    };
    let moves = [
        planar(Turn::Absolute(2.9053886486487226), 1.0, 546.924938602993, 0.0765246174258583),
        planar(
            Turn::Absolute(5.682024081779053),
            0.0009566603280114744,
            90.87832523810361,
            0.09079521363142903,
        ),
        planar(Turn::Absolute(4.323424096437546), 1.0, 1.0, 0.0),
    ];
    let (_, report) = run_case(limits, &moves);
    assert!(report.hard_ok(), "{report}");
}
