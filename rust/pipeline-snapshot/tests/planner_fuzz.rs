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
//! is the same property on a fixed 256-case corpus. `target_budgets_hold`
//! fuzzes the same planar space and additionally asserts the smoothness
//! budgets (seam C0/C1, velocity limits). Every bug a fuzz finds gets pinned
//! below as an `#[ignore]`d regression test until fixed.
//!
//! `an_advanced_extruder_plans_print_like_streams` plans print-like G-code
//! (dense short extrusions whose ratio jitters, curves, reversals into
//! travels, retracts with a z hop) through X/Y shapers and a kernel-free
//! linear advance whose extruder limit binds, under every kind of corner
//! budget: the configuration whose crawl valleys the turning guard and the
//! follower projection lay. Planning must not fail.

mod corpus;

use std::fmt::Write as _;

use motion_pipeline::live_config;
use pipeline_snapshot::audit::{
    AuditBudgets, AuditReport, Violation, ViolationKind, audit_trajectory,
};
use pipeline_snapshot::waypoints::{Waypoint, extrusion_ratio_bound};
use pipeline_snapshot::{
    CartesianLimits, ExactTrajectory, PlannerConfig, SnapshotParams, pipeline_snapshot,
    pipeline_snapshot_for_config,
};
use planner_config::from_doc::planner_config_from_text;
use proptest::prelude::*;
use proptest::test_runner::{FailurePersistence, FileFailurePersistence, RngSeed};

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
        corner: geometry::CornerBudget::Shaped(corner_deviation),
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
    let planner = PlannerConfig {
        cartesian: CartesianLimits::without_z_caps(
            limits.max_velocity,
            limits.max_accel,
            f64::INFINITY,
            geometry::CornerBudget::Shaped(corner_deviation),
        ),
        ..PlannerConfig::default()
    };
    let config =
        live_config::stream_config(&planner, None).expect("strategy generates valid limits");
    let report = audit_trajectory(&traj, &config, &budgets(&config, waypoints));
    (traj, report)
}

const E_AXIS: usize = 3;

/// The audit's budgets for the chains the snapshot compiles when no axis
/// is declared (x, y, z, and e following them, with no post-processors),
/// the extruder's seam velocity under the derived ratio ramp: every ratio
/// step is ramped, so only rounding steps it, bar the velocity steps a
/// kernel-free advance makes of an unshaped leader's acceleration steps
/// ([`motion_pipeline::follower_velocity_step_allowance`]).
fn budgets(config: &motion_pipeline::StreamConfig, waypoints: &[Waypoint]) -> AuditBudgets {
    let axis = |name: &str, follows: &[&str]| planner_config::AxisDecl {
        name: name.to_string(),
        follows: follows.iter().map(ToString::to_string).collect(),
        motors: Vec::new(),
        post_processors: Vec::new(),
        max_velocity: None,
        max_accel: None,
        instantaneous_corner_velocity: None,
    };
    let registry = planner_config::AxisRegistry::try_new(vec![
        axis("x", &[]),
        axis("y", &[]),
        axis("z", &[]),
        axis("e", &["x", "y", "z"]),
    ])
    .expect("the fuzz's axes register");
    let chains = planner_config::PostProcessorSet::try_new(&registry, &[])
        .expect("no post-processors")
        .compile_active_chains(&registry, false)
        .expect("the fuzz's chains compile");
    let mut budgets = AuditBudgets::for_config(config);
    budgets.seam_vel_mm_s[E_AXIS] += motion_pipeline::follower_velocity_step_allowance(
        &chains,
        E_AXIS,
        extrusion_ratio_bound(waypoints),
        config.fit_tol_accel_mm_s2,
    );
    budgets
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

const REGRESSIONS: &str = "proptest-regressions/planner_fuzz.txt";

fn committed_regressions() -> Box<dyn FailurePersistence> {
    Box::new(FileFailurePersistence::Direct(REGRESSIONS))
}

/// A fresh seed's run saves its failures to the committed regressions file
/// and replays the file's seeds first. A run given its seed
/// (`PROPTEST_RNG_SEED`) explores that seed alone and leaves the file be:
/// the seed reproduces it, and replaying the file first would report a
/// known failure in place of the seed's own.
fn fresh_seed_config() -> ProptestConfig {
    let given = std::env::var("PROPTEST_RNG_SEED")
        .ok()
        .and_then(|value| value.parse().ok());
    let seed = given.unwrap_or_else(|| {
        let nanos = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .expect("clock after the epoch")
            .as_nanos();
        (nanos as u64) ^ u64::from(std::process::id()).rotate_left(40)
    });
    eprintln!("planner_fuzz seed: reproduce with PROPTEST_RNG_SEED={seed}");
    ProptestConfig {
        rng_seed: RngSeed::Fixed(seed),
        failure_persistence: given.is_none().then(committed_regressions),
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
        failure_persistence: Some(committed_regressions()),
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
    #![proptest_config(fresh_seed_config())]

    #[test]
    fn target_budgets_hold(
        limits in limits_strategy(),
        moves in prop::collection::vec(move_strategy(true), 2..40),
    ) {
        let (_, report) = run_case(limits, &moves);
        prop_assert!(report.hard_ok(), "{report}");
        prop_assert!(report.target_ok(), "{report}");
    }
}

/// The corner option of a `[printer]` section, every kind a config may set.
fn corner_option_strategy() -> impl Strategy<Value = String> {
    prop_oneof![
        1 => Just(String::new()),
        1 => Just("square_corner_velocity: 0\n".to_owned()),
        1 => (1.0..20.0_f64).prop_map(|scv| format!("square_corner_velocity: {scv}\n")),
        2 => (-2.3..0.7_f64)
            .prop_map(|log_mm| format!("corner_deviation: {}\n", libm::pow(10.0, log_mm))),
    ]
}

/// A corexy printer with X/Y smooth shapers and a linear advance no kernel
/// follows, its extruder limited: the follower the turning guard holds.
fn advanced_printer_strategy() -> impl Strategy<Value = String> {
    (
        (
            300.0..1000.0_f64,
            3000.0..70_000.0_f64,
            corner_option_strategy(),
        ),
        (30.0..120.0_f64, 30.0..120.0_f64),
        (0.01..0.06_f64, 2000.0..20_000.0_f64),
    )
        .prop_map(|((velocity, accel, corner), (fx, fy), (k, e_accel))| {
            format!(
                "[printer]\nmax_velocity: {velocity}\nmax_accel: {accel}\n{corner}\
                 max_z_velocity: 25\nmax_z_accel: 100\n\
                 [kinematics]\ntype: corexy\naxis_x: x\naxis_y: y\naxis_z: z\n\
                 a_motors: a\nb_motors: b\nz_motors: z\n\
                 [axis x]\npost_processors: shaper_x\n[axis y]\npost_processors: shaper_y\n\
                 [axis z]\n[axis e]\nfollows: x, y, z\nmotors: extruder\npost_processors: pa\n\
                 max_accel: {e_accel}\n\
                 [post_processor pa]\ntype: linear_pressure_advance\nk: {k}\n\
                 [post_processor shaper_x]\ntype: smooth_mzv\nfrequency_hz: {fx}\n\
                 [post_processor shaper_y]\ntype: smooth_zv\nfrequency_hz: {fy}\n\
                 [motor a]\ndrive: stepper\n[motor b]\ndrive: stepper\n\
                 [motor z]\ndrive: stepper\n[motor extruder]\ndrive: stepper\n\
                 [extruder]\naxis: e\nmax_extrude_only_velocity: 50\n\
                 max_extrude_only_accel: 5000\n"
            )
        })
}

/// One print move: an extrusion or travel turning `turn` off the last
/// heading, or a retract, z hop and travel to `heading` and back down.
#[derive(Debug, Clone, Copy)]
enum PrintMove {
    Line {
        turn: f64,
        length_mm: f64,
        feed_mm_s: f64,
        ratio_jitter: Option<f64>,
    },
    HopTravel {
        heading: f64,
        length_mm: f64,
    },
}

/// Slicer output: mostly short extrusions bending along an outline, their
/// ratio off the layer's by the width and rounding a slicer puts in each
/// line, reversals, travels, and hops.
fn print_move_strategy() -> impl Strategy<Value = PrintMove> {
    let turn = prop_oneof![
        6 => -0.6..0.6_f64,
        2 => (-0.3..0.3_f64).prop_map(|off| std::f64::consts::PI + off),
        2 => 0.0..std::f64::consts::TAU,
    ];
    let jitter = prop_oneof![4 => (-0.5..0.5_f64).prop_map(Some), 1 => Just(None)];
    prop_oneof![
        9 => (turn, -1.3..1.3_f64, 20.0..300.0_f64, jitter).prop_map(
            |(turn, log_len, feed_mm_s, ratio_jitter)| PrintMove::Line {
                turn,
                length_mm: libm::pow(10.0, log_len),
                feed_mm_s,
                ratio_jitter,
            }
        ),
        1 => (0.0..std::f64::consts::TAU, 1.0..30.0_f64)
            .prop_map(|(heading, length_mm)| PrintMove::HopTravel { heading, length_mm }),
    ]
}

const RETRACT_MM: f64 = 0.5;
const HOP_MM: f64 = 0.4;
const TRAVEL_FEED_MM_S: f64 = 500.0;

/// The G-code of `moves` from the bed's middle, at `ratio` mm of filament
/// per mm of path, each extrusion off it by its jitter; one without is a
/// travel.
fn print_gcode(ratio: f64, moves: &[PrintMove]) -> String {
    let (mut x, mut y, mut heading) = (150.0, 150.0, 0.0_f64);
    let z = 0.2_f64;
    let mut gcode = format!("G90\nM83\nG1 X{x} Y{y} Z{z} F{}\n", 60.0 * TRAVEL_FEED_MM_S);
    for m in moves {
        match *m {
            PrintMove::Line {
                turn,
                length_mm,
                feed_mm_s,
                ratio_jitter,
            } => {
                heading += turn;
                x += length_mm * libm::cos(heading);
                y += length_mm * libm::sin(heading);
                let extrude = ratio_jitter
                    .map(|jitter| format!(" E{:.5}", ratio * (1.0 + jitter) * length_mm))
                    .unwrap_or_default();
                writeln!(
                    gcode,
                    "G1 X{x:.3} Y{y:.3}{extrude} F{:.0}",
                    60.0 * feed_mm_s
                )
                .expect("a string takes G-code");
            }
            PrintMove::HopTravel {
                heading: to,
                length_mm,
            } => {
                heading = to;
                x += length_mm * libm::cos(heading);
                y += length_mm * libm::sin(heading);
                let feed = 60.0 * TRAVEL_FEED_MM_S;
                write!(
                    gcode,
                    "G1 E-{RETRACT_MM} F2400\nG1 Z{:.3} F600\nG1 X{x:.3} Y{y:.3} F{feed:.0}\n\
                     G1 Z{z:.3} F600\nG1 E{RETRACT_MM} F2400\n",
                    z + HOP_MM
                )
                .expect("a string takes G-code");
            }
        }
    }
    gcode
}

/// Why the print-like stream of `moves` at `ratio` on `printer` fails to
/// plan within the extruder's limits, or `None` where it plans within them.
fn print_like_failure(printer: &str, ratio: f64, moves: &[PrintMove]) -> Option<String> {
    let planner = planner_config_from_text(printer).expect("the strategy draws valid configs");
    let gcode = print_gcode(ratio, moves);
    let waypoints = pipeline_snapshot::waypoints::parse_gcode(&gcode, &planner.cartesian)
        .expect("the strategy draws valid G-code");
    let snapshot = match pipeline_snapshot_for_config(&waypoints, &planner, None) {
        Ok(snapshot) => snapshot,
        Err(error) => return Some(format!("{gcode}\n{printer}\n{error:?}")),
    };
    let config = std::env::temp_dir().join(format!("print_like_{}.cfg", std::process::id()));
    std::fs::write(&config, printer).expect("temp config");
    let audit = corpus::extruder::extruder_audit(&snapshot, &gcode, &config)
        .expect("the strategy limits the extruder");
    let excesses: Vec<String> = audit.excesses.iter().map(ToString::to_string).collect();
    (!excesses.is_empty()).then(|| format!("{gcode}\n{printer}\n{}: {excesses:?}", audit.summary()))
}

proptest! {
    #![proptest_config(fresh_seed_config())]

    #[test]
    #[ignore = "#936 #903 #946 #955: a ratio read changing across a join, the turning guard's valley sides, a velocity step near a retract and a ratio step crossed while braking fail on draws of this space"]
    fn an_advanced_extruder_plans_print_like_streams(
        printer in advanced_printer_strategy(),
        ratio in 0.02..0.06_f64,
        moves in prop::collection::vec(print_move_strategy(), 2..40),
    ) {
        let failure = print_like_failure(&printer, ratio, &moves);
        prop_assert!(failure.is_none(), "{}", failure.unwrap_or_default());
    }
}

/// A shrink step of the print-like fuzz's seed 6 that main plans within the
/// extruder's limits: a ratio step's ramp reached a crawl valley's side and
/// asked more than the follower's limit of it.
const CRAWL_SIDE_RAMP_PRINTER: &str = "[printer]
max_velocity: 300
max_accel: 53030.867600916
max_z_velocity: 25
max_z_accel: 100
[kinematics]
type: corexy
axis_x: x
axis_y: y
axis_z: z
a_motors: a
b_motors: b
z_motors: z
[axis x]
post_processors: shaper_x
[axis y]
post_processors: shaper_y
[axis z]
[axis e]
follows: x, y, z
motors: extruder
post_processors: pa
max_accel: 2000
[post_processor pa]
type: linear_pressure_advance
k: 0.05670768351151637
[post_processor shaper_x]
type: smooth_mzv
frequency_hz: 30
[post_processor shaper_y]
type: smooth_zv
frequency_hz: 30
[motor a]
drive: stepper
[motor b]
drive: stepper
[motor z]
drive: stepper
[motor extruder]
drive: stepper
[extruder]
axis: e
max_extrude_only_velocity: 50
max_extrude_only_accel: 5000
";

#[test]
fn a_ratio_step_ramping_into_a_crawl_side_plans_within_the_extruder_limits() {
    let moves = [
        PrintMove::Line {
            turn: 0.4453970580868697,
            length_mm: 0.08724551618549914,
            feed_mm_s: 126.92786675546716,
            ratio_jitter: Some(0.18307493377378248),
        },
        PrintMove::Line {
            turn: 0.5347926537076366,
            length_mm: 0.09906039716886528,
            feed_mm_s: 198.6486467843766,
            ratio_jitter: None,
        },
        PrintMove::Line {
            turn: 2.2181117011357845,
            length_mm: 0.07359850564771436,
            feed_mm_s: 210.42431979334052,
            ratio_jitter: Some(-0.38969626671081964),
        },
        PrintMove::Line {
            turn: 3.2233642992221734,
            length_mm: 2.297048752418555,
            feed_mm_s: 76.32399910279231,
            ratio_jitter: None,
        },
        PrintMove::Line {
            turn: 2.8829616812000536,
            length_mm: 2.8498657154555014,
            feed_mm_s: 235.01815766067898,
            ratio_jitter: Some(-0.43620889912452787),
        },
        PrintMove::Line {
            turn: 4.790200647295755,
            length_mm: 0.7716705902165792,
            feed_mm_s: 193.07661868237045,
            ratio_jitter: Some(0.2086437881617884),
        },
        PrintMove::Line {
            turn: 0.4595372567991594,
            length_mm: 0.6084310423948747,
            feed_mm_s: 165.87527517588973,
            ratio_jitter: Some(-0.04969426935122287),
        },
        PrintMove::Line {
            turn: 3.0393510283254703,
            length_mm: 1.4088092847739644,
            feed_mm_s: 211.59760300244022,
            ratio_jitter: Some(-0.2915940117931435),
        },
        PrintMove::Line {
            turn: -0.27501761138810854,
            length_mm: 2.709952080658379,
            feed_mm_s: 91.29068783003602,
            ratio_jitter: Some(-0.4497002682258164),
        },
        PrintMove::Line {
            turn: 3.3848823282406815,
            length_mm: 11.429376715718998,
            feed_mm_s: 103.00977455332931,
            ratio_jitter: Some(-0.48908015690039164),
        },
        PrintMove::Line {
            turn: 5.86657129843279,
            length_mm: 7.725149075843305,
            feed_mm_s: 263.24942444370487,
            ratio_jitter: Some(-0.031244275828838187),
        },
        PrintMove::Line {
            turn: 0.7498762824029065,
            length_mm: 0.12398279162189307,
            feed_mm_s: 179.41474766360452,
            ratio_jitter: Some(-0.19472465383242124),
        },
        PrintMove::Line {
            turn: -0.09099386419494253,
            length_mm: 0.500893076174017,
            feed_mm_s: 127.35284578104289,
            ratio_jitter: Some(-0.0009826218904185175),
        },
        PrintMove::Line {
            turn: 0.15327696249071596,
            length_mm: 9.061544232875576,
            feed_mm_s: 29.571066201657317,
            ratio_jitter: Some(-0.05612154219415402),
        },
        PrintMove::Line {
            turn: -0.4155228447993262,
            length_mm: 0.45079284238384304,
            feed_mm_s: 21.697257115884508,
            ratio_jitter: Some(0.2872380808136781),
        },
        PrintMove::Line {
            turn: -0.45393514190287637,
            length_mm: 0.46822326773093964,
            feed_mm_s: 91.97366349734762,
            ratio_jitter: Some(-0.29054247390490484),
        },
        PrintMove::Line {
            turn: 0.02074041762618198,
            length_mm: 0.9749543859069524,
            feed_mm_s: 256.90216662093474,
            ratio_jitter: Some(0.3935682218089948),
        },
        PrintMove::Line {
            turn: 0.5352951234153126,
            length_mm: 1.9675940767543143,
            feed_mm_s: 291.6742829113563,
            ratio_jitter: None,
        },
        PrintMove::Line {
            turn: 0.05727145462325999,
            length_mm: 17.550404945378162,
            feed_mm_s: 68.61138854039872,
            ratio_jitter: Some(-0.19325944268530412),
        },
        PrintMove::Line {
            turn: -0.11933006463934184,
            length_mm: 4.579901485406777,
            feed_mm_s: 105.22373412837642,
            ratio_jitter: None,
        },
        PrintMove::Line {
            turn: -0.044465756506873635,
            length_mm: 0.08310827404242764,
            feed_mm_s: 295.1441197835098,
            ratio_jitter: Some(0.2579972485136182),
        },
        PrintMove::HopTravel {
            heading: 5.628584364939775,
            length_mm: 19.019184452506614,
        },
        PrintMove::Line {
            turn: 0.5450908709851561,
            length_mm: 0.21023402783964062,
            feed_mm_s: 65.91953590383167,
            ratio_jitter: Some(-0.05917705071798234),
        },
        PrintMove::Line {
            turn: -0.47629377914997895,
            length_mm: 1.9246647461270798,
            feed_mm_s: 77.57153109790576,
            ratio_jitter: None,
        },
    ];
    let failure = print_like_failure(CRAWL_SIDE_RAMP_PRINTER, 0.056202335201128814, &moves);
    assert!(failure.is_none(), "{}", failure.unwrap_or_default());
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
        planar(
            Turn::Absolute(2.9053886486487226),
            1.0,
            546.924938602993,
            0.0765246174258583,
        ),
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

/// #619, found by `target_budgets_hold`: a junction turning 0.89 mrad, under
/// the fitter's collinear threshold, passed unblended at 207 mm/s under a
/// zero corner budget, and z stepped by 0.18 mm/s there. Travel only. A
/// junction now passes as collinear only when `v·θ` at the fastest its
/// pieces may cross it stays within the seam budget.
#[test]
fn a_nearly_collinear_z_junction_keeps_the_seam_velocity_budget() {
    let limits = FuzzLimits {
        max_velocity: 314.47805824987176,
        max_accel: 500.0,
        square_corner_velocity: 0.0,
    };
    let moves = [
        MoveSpec {
            turn: Turn::Continue,
            length_mm: 46.522523281975396,
            feed_mm_s: 604.447037922063,
            extrude_ratio: 0.0,
            z_step_mm: 0.04119073092929801,
        },
        MoveSpec {
            turn: Turn::Continue,
            length_mm: 39.702781729279174,
            feed_mm_s: 505.4380942226453,
            extrude_ratio: 0.0,
            z_step_mm: 0.0,
        },
    ];
    let (_, report) = run_case(limits, &moves);
    assert!(report.hard_ok() && report.target_ok(), "{report}");
}

/// #619 in the plane: a 0.6 mrad heading change between two fast extrusions
/// under a corner budget blends instead of stepping 0.18 mm/s.
#[test]
fn a_nearly_collinear_planar_junction_at_speed_keeps_the_seam_velocity_budget() {
    let limits = FuzzLimits {
        max_velocity: 300.0,
        max_accel: 5000.0,
        square_corner_velocity: 5.0,
    };
    let moves = [
        planar(Turn::Absolute(0.0), 40.0, 300.0, 0.05),
        planar(Turn::Absolute(6e-4), 40.0, 300.0, 0.05),
    ];
    let (_, report) = run_case(limits, &moves);
    assert!(report.hard_ok(), "{report}");
    let spikes = xy_smoothness_spikes(&report);
    assert!(spikes.is_empty(), "{spikes:?}\n{report}");
}

/// #618: under a 0.016 mm/s square-corner velocity, a travel reversal that
/// steps z planned a member its own rails could not join, and the planner
/// panicked with `Infeasible`.
#[test]
fn tiny_scv_reversal_with_z_step_plans() {
    let limits = FuzzLimits {
        max_velocity: 20.0,
        max_accel: 38_054.585_539_152_91,
        square_corner_velocity: 0.016_357_394_343_349_78,
    };
    let moves = [
        MoveSpec {
            turn: Turn::Absolute(0.0),
            length_mm: 1.0,
            feed_mm_s: 1.0,
            extrude_ratio: 0.0,
            z_step_mm: 0.0,
        },
        MoveSpec {
            turn: Turn::Absolute(3.791_313_142_220_391),
            length_mm: 1.0,
            feed_mm_s: 1.0,
            extrude_ratio: 0.0,
            z_step_mm: -0.189_761_367_961_591_32,
        },
    ];
    let (_, report) = run_case(limits, &moves);
    assert!(report.hard_ok(), "{report}");
}

/// #682, found by `target_budgets_hold`: a fast extrusion, a 1 mm travel
/// turning off it, a reversed extrusion and three travels slowing to 1 mm/s;
/// the follower's source fit ran out of refinement splits.
#[test]
fn an_extrusion_between_two_travels_keeps_its_source_fit() {
    let limits = FuzzLimits {
        max_velocity: 553.945_527_622_928_2,
        max_accel: 43_373.570_522_562_106,
        square_corner_velocity: 0.339_824_488_063_710_8,
    };
    let moves = [
        planar(
            Turn::Continue,
            59.968_641_825_493_34,
            582.359_961_523_245_7,
            0.087_711_029_672_142_36,
        ),
        planar(
            Turn::Absolute(0.927_233_164_578_025),
            1.0,
            235.130_400_227_618_9,
            0.0,
        ),
        planar(
            Turn::Absolute(4.058_019_275_991_085),
            1.0,
            527.270_397_943_680_9,
            0.085_164_978_667_546_02,
        ),
        planar(Turn::Continue, 1.0, 556.546_817_523_540_5, 0.0),
        planar(Turn::Continue, 1.0, 46.816_668_799_578_36, 0.0),
        planar(Turn::Continue, 1.0, 1.0, 0.0),
    ];
    let (_, report) = run_case(limits, &moves);
    assert!(report.hard_ok(), "{report}");
    assert!(report.target_ok(), "{report}");
}

/// Print-like fuzz seed 107, shrunk: a crawl valley whose before side
/// waited for a ratio step's final width while the projection took the
/// path behind it, then found no side on the path left and held the weight
/// at zero back to the stream start, which the projection had passed.
const SEED_107_PRINTER: &str = "[printer]
max_velocity: 300
max_accel: 7537.213174089488
square_corner_velocity: 0
max_z_velocity: 25
max_z_accel: 100
[kinematics]
type: corexy
axis_x: x
axis_y: y
axis_z: z
a_motors: a
b_motors: b
z_motors: z
[axis x]
post_processors: shaper_x
[axis y]
post_processors: shaper_y
[axis z]
[axis e]
follows: x, y, z
motors: extruder
post_processors: pa
max_accel: 19375.069920562964
[post_processor pa]
type: linear_pressure_advance
k: 0.05987164252539586
[post_processor shaper_x]
type: smooth_mzv
frequency_hz: 53.02952157777028
[post_processor shaper_y]
type: smooth_zv
frequency_hz: 106.43648156271938
[motor a]
drive: stepper
[motor b]
drive: stepper
[motor z]
drive: stepper
[motor extruder]
drive: stepper
[extruder]
axis: e
max_extrude_only_velocity: 50
max_extrude_only_accel: 5000
";

#[test]
fn a_crawl_waiting_on_a_step_keeps_its_before_side_on_untaken_path() {
    let moves = [
        PrintMove::Line { turn: 0.5581263465705404, length_mm: 1.0, feed_mm_s: 20.0, ratio_jitter: Some(0.0) },
        PrintMove::Line { turn: -0.25910234519181824, length_mm: 1.0, feed_mm_s: 20.0, ratio_jitter: Some(0.0) },
        PrintMove::Line { turn: -0.3732766402430854, length_mm: 1.0, feed_mm_s: 20.0, ratio_jitter: Some(0.0) },
        PrintMove::Line { turn: 0.0, length_mm: 8.634078274694607, feed_mm_s: 272.7737433730867, ratio_jitter: Some(0.3418686509066707) },
        PrintMove::Line { turn: 0.0, length_mm: 0.12460863258698181, feed_mm_s: 20.0, ratio_jitter: Some(0.0) },
        PrintMove::Line { turn: 3.141592653589793, length_mm: 1.0, feed_mm_s: 20.0, ratio_jitter: Some(0.0) },
        PrintMove::Line { turn: 0.0, length_mm: 1.0, feed_mm_s: 20.0, ratio_jitter: Some(0.0) },
        PrintMove::Line { turn: 0.0, length_mm: 1.0, feed_mm_s: 20.0, ratio_jitter: Some(0.0) },
    ];
    let failure = print_like_failure(SEED_107_PRINTER, 0.0506342928097271, &moves);
    assert!(failure.is_none(), "{}", failure.unwrap_or_default());
}

/// Print-like fuzz seed 116, shrunk: main plans it. Budgeting the ramps
/// of the unowned steps inside the before side asked a 21.2 mm side of a
/// crawl the leaders reach 1.70 mm around; the side is now held at the
/// lead budget and the ramps are the leaders' speed's to fit.
const SEED_116_PRINTER: &str = "[printer]
max_velocity: 300
max_accel: 66430.71505287588
corner_deviation: 1
max_z_velocity: 25
max_z_accel: 100
[kinematics]
type: corexy
axis_x: x
axis_y: y
axis_z: z
a_motors: a
b_motors: b
z_motors: z
[axis x]
post_processors: shaper_x
[axis y]
post_processors: shaper_y
[axis z]
[axis e]
follows: x, y, z
motors: extruder
post_processors: pa
max_accel: 18876.98144294022
[post_processor pa]
type: linear_pressure_advance
k: 0.03820863970495176
[post_processor shaper_x]
type: smooth_mzv
frequency_hz: 110.02050780375808
[post_processor shaper_y]
type: smooth_zv
frequency_hz: 93.88770551221835
[motor a]
drive: stepper
[motor b]
drive: stepper
[motor z]
drive: stepper
[motor extruder]
drive: stepper
[extruder]
axis: e
max_extrude_only_velocity: 50
max_extrude_only_accel: 5000
";

#[test]
fn step_ramps_inside_a_lead_held_crawl_side_fit_the_extruder() {
    let moves = [
        PrintMove::Line { turn: 0.0, length_mm: 1.0, feed_mm_s: 20.0, ratio_jitter: Some(0.0) },
        PrintMove::Line { turn: 0.0, length_mm: 1.0, feed_mm_s: 20.0, ratio_jitter: Some(0.0) },
        PrintMove::Line { turn: 0.0, length_mm: 1.0, feed_mm_s: 20.0, ratio_jitter: Some(-0.39332805590604747) },
        PrintMove::Line { turn: 0.0, length_mm: 2.989920269118111, feed_mm_s: 20.0, ratio_jitter: Some(0.0) },
        PrintMove::Line { turn: 0.0, length_mm: 5.0349722288363745, feed_mm_s: 20.0, ratio_jitter: Some(-0.49831958520602304) },
        PrintMove::Line { turn: 0.0, length_mm: 5.1184025187851425, feed_mm_s: 278.40796362043454, ratio_jitter: Some(0.20222715099830577) },
        PrintMove::Line { turn: 0.0, length_mm: 0.8140516725991794, feed_mm_s: 227.25216287748776, ratio_jitter: Some(0.0) },
        PrintMove::Line { turn: 3.141592653589793, length_mm: 1.0, feed_mm_s: 20.0, ratio_jitter: Some(0.0) },
        PrintMove::Line { turn: 0.0, length_mm: 1.0, feed_mm_s: 122.115339346924, ratio_jitter: Some(0.0) },
        PrintMove::Line { turn: 0.0, length_mm: 1.0, feed_mm_s: 20.0, ratio_jitter: Some(0.0) },
        PrintMove::Line { turn: 0.0, length_mm: 1.0, feed_mm_s: 20.0, ratio_jitter: Some(0.0) },
        PrintMove::Line { turn: 0.0, length_mm: 10.75845666965621, feed_mm_s: 20.0, ratio_jitter: Some(0.0) },
    ];
    let failure = print_like_failure(SEED_116_PRINTER, 0.045351793198012864, &moves);
    assert!(failure.is_none(), "{}", failure.unwrap_or_default());
}
