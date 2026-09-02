//! Property campaign over the exact scalar time laws and the phase chains the
//! velocity planner builds out of them.
//!
//! The oracles here are the laws' own definitions — `ds/dt = v`, `dv/dt = a`,
//! and, on the disk rail, `a² + (κ·v²)² = A²` — evaluated by quadrature and
//! finite differences that never touch the crate's interpolation.

use geometry::path::{Arc, Clothoid, Line, PathSegment, Segment};
use geometry::velocity::law::{LawSegment, ScalarLaw};
use geometry::{BoundaryState, Move, SourceRange, VelocityLimits, plan_velocity_stops};
use proptest::prelude::*;
use proptest::test_runner::FileFailurePersistence;

/// A rail is only a law while its normal load fits inside the budget; past
/// `|κ|·v² = A` the square root has no real branch. Generated rails stay this
/// far inside it so the segment under test is one the law actually describes.
const DISK_LOAD_MARGIN: f64 = 0.9;
const MIN_ACCEL: f64 = 100.0;
const MAX_ACCEL: f64 = 50_000.0;
const MAX_KAPPA: f64 = 50.0;
const MIN_KAPPA: f64 = 1e-3;
const MAX_ARC_MM: f64 = 20.0;
/// The dense rail carries one knot per `RAIL_KNOT_DS` of arc, so a planner
/// member's cost is linear in its length and a plan is replanned twice here.
const MAX_PLANNER_ARC_MM: f64 = 4.0;
const MIN_ARC_MM: f64 = 1e-3;
const MAX_FEED_MM_S: f64 = 600.0;
const INTEGRATION_TOL: f64 = 1e-9;
const EXTRUDE_ONLY_V: f64 = 25.0;
const EXTRUDE_ONLY_A: f64 = 1500.0;

fn log_uniform(min: f64, max: f64) -> impl Strategy<Value = f64> {
    (libm::log(min)..libm::log(max)).prop_map(libm::exp)
}

fn arb_signed_kappa() -> impl Strategy<Value = f64> {
    prop_oneof![
        1 => Just(0.0),
        9 => (log_uniform(MIN_KAPPA, MAX_KAPPA), prop::bool::ANY)
            .prop_map(|(k, negative)| if negative { -k } else { k }),
    ]
}

/// A rail segment plus the law parameters it was built from, so a property can
/// re-derive the disk without reaching into the crate.
#[derive(Debug, Clone)]
struct Rail {
    segment: LawSegment,
    accel: f64,
    kappa0: f64,
    sigma: f64,
    brake: bool,
    entry_v: f64,
    arc: f64,
}

impl Rail {
    fn kappa_at(&self, s: f64) -> f64 {
        self.kappa0 + self.sigma * (s - self.segment.s0)
    }

    fn law(&self) -> ScalarLaw {
        ScalarLaw::DiskRail {
            accel: self.accel,
            kappa0: self.kappa0,
            sigma: self.sigma,
            brake: self.brake,
        }
    }
}

/// Rails may be entered at rest or a hair above it: the dense solution's knot
/// durations are self-consistent with its interpolant at any speed ratio.
const MIN_RAIL_SPEED: f64 = 0.0;

fn arb_rail() -> impl Strategy<Value = Rail> {
    (
        log_uniform(MIN_ACCEL, MAX_ACCEL),
        arb_signed_kappa(),
        arb_signed_kappa(),
        prop::bool::ANY,
        0.0f64..1.0,
        0.0f64..1.0,
    )
        .prop_filter_map(
            "the rail must cover its arc without stalling",
            |(accel, kappa_entry, kappa_exit, brake, slow_fraction, span_fraction)| {
                let kappa_max = kappa_entry.abs().max(kappa_exit.abs());
                let cap = if kappa_max > 0.0 {
                    (DISK_LOAD_MARGIN * accel / kappa_max).sqrt()
                } else {
                    MAX_FEED_MM_S
                };
                let slow = MIN_RAIL_SPEED + slow_fraction * (cap - MIN_RAIL_SPEED);
                let fast = slow + span_fraction * (cap - slow);
                // A rail's tangential accel never exceeds the budget, so the
                // straight-line span at that budget covers at least as much
                // speed: sizing the arc from it keeps both ends inside the cap
                // whichever way the rail runs.
                let arc = ((fast * fast - slow * slow) / (2.0 * accel)).min(MAX_ARC_MM);
                if !(arc > 0.0) {
                    return None;
                }
                let entry_v = if brake { fast } else { slow };
                let sigma = (kappa_exit - kappa_entry) / arc;
                let law = ScalarLaw::DiskRail {
                    accel,
                    kappa0: kappa_entry,
                    sigma,
                    brake,
                };
                let segment = LawSegment::until_arc(0.0, 0.0, entry_v, law, arc)?;
                Some(Rail {
                    segment,
                    accel,
                    kappa0: kappa_entry,
                    sigma,
                    brake,
                    entry_v,
                    arc,
                })
            },
        )
}

fn arb_const_accel() -> impl Strategy<Value = LawSegment> {
    (
        log_uniform(MIN_ACCEL, MAX_ACCEL),
        prop_oneof![1 => Just(0.0), 4 => 1.0f64..MAX_FEED_MM_S],
        prop_oneof![1 => Just(0.0), 2 => Just(1.0), 2 => Just(-1.0)],
        log_uniform(MIN_ARC_MM, MAX_ARC_MM),
        -5.0f64..5.0,
    )
        .prop_map(|(accel, v0, direction, arc, t0)| {
            // A span at rest only moves under a positive budget, and a braking
            // span must stop short of its own rest arc.
            let direction = if v0 == 0.0 { 1.0 } else { direction };
            let a0 = direction * accel;
            let arc = if a0 < 0.0 {
                arc.min(DISK_LOAD_MARGIN * v0 * v0 / (2.0 * accel))
            } else {
                arc
            };
            LawSegment::until_arc(t0, 3.0, v0, ScalarLaw::ConstAccel { a0 }, arc)
                .expect("a constant-accel span always reaches an arc inside its own reach")
        })
}

/// `∫ v dt` over the whole span by composite Simpson.
fn integrated_velocity(segment: &LawSegment, panels: usize) -> f64 {
    let h = segment.dt / panels as f64;
    let mut total = 0.0;
    for panel in 0..panels {
        let a = segment.t0 + h * panel as f64;
        let b = a + h;
        total += (h / 6.0)
            * (segment.state_at(a).1
                + 4.0 * segment.state_at(0.5 * (a + b)).1
                + segment.state_at(b).1);
    }
    total
}

fn sample_times(segment: &LawSegment, count: usize) -> Vec<f64> {
    (0..=count)
        .map(|i| segment.t0 + segment.dt * i as f64 / count as f64)
        .collect()
}

fn fail(what: &str, got: f64, want: f64, tol: f64) -> Result<(), TestCaseError> {
    let error = (got - want).abs();
    if error <= tol {
        return Ok(());
    }
    Err(TestCaseError::fail(format!(
        "{what}: got {got:e}, want {want:e}, error {error:e} > tol {tol:e}"
    )))
}

fn at_most(what: &str, got: f64, bound: f64) -> Result<(), TestCaseError> {
    if got <= bound {
        return Ok(());
    }
    Err(TestCaseError::fail(format!(
        "{what}: {got:e} exceeds {bound:e}"
    )))
}

const UNIT_U: [f64; 3] = [1.0, 0.0, 0.0];
const UNIT_V: [f64; 3] = [0.0, 1.0, 0.0];

/// A planner move is consumed through its length and curvature profile alone,
/// so the members are placed independently; only the alphabet matters.
fn arb_planner_segment() -> impl Strategy<Value = Segment> {
    prop_oneof![
        2 => log_uniform(MIN_ARC_MM, MAX_PLANNER_ARC_MM)
            .prop_map(|len| Segment::Line(Line::try_new([0.0; 3], [len, 0.0, 0.0]).expect("a line"))),
        2 => (log_uniform(MIN_ARC_MM, MAX_PLANNER_ARC_MM), log_uniform(MIN_KAPPA, MAX_KAPPA), prop::bool::ANY)
            .prop_filter_map("an arc needs a representable sweep", |(len, kappa, negative)| {
                let radius = 1.0 / kappa;
                let sweep = if negative { -len * kappa } else { len * kappa };
                Arc::try_new([0.0; 3], UNIT_U, UNIT_V, radius, 0.0, sweep).ok().map(Segment::Arc)
            }),
        3 => (log_uniform(MIN_ARC_MM, MAX_PLANNER_ARC_MM), arb_signed_kappa(), arb_signed_kappa())
            .prop_filter_map("a clothoid needs a representable rate", |(len, k_in, k_out)| {
                Clothoid::try_new([0.0; 3], UNIT_U, UNIT_V, k_in, (k_out - k_in) / len, len)
                    .ok()
                    .map(Segment::Clothoid)
            }),
    ]
}

#[derive(Debug, Clone)]
struct PlanCase {
    moves: Vec<Move>,
    stop_before: Vec<bool>,
    entry: BoundaryState,
}

impl PlanCase {
    /// The same geometry replanned at a different acceleration budget.
    fn with_accel(&self, accel: f64) -> PlanCase {
        let moves = self
            .moves
            .iter()
            .map(|m| Move {
                segment: m.segment.clone(),
                feedrate_mm_s: m.feedrate_mm_s,
                limits: VelocityLimits {
                    accel_mm_s2: accel,
                    ..m.limits
                },
                source: m.source,
            })
            .collect();
        PlanCase {
            moves,
            stop_before: self.stop_before.clone(),
            entry: self.entry,
        }
    }

    fn plan(&self) -> Result<geometry::VelocityProfile, geometry::VelocityError> {
        plan_velocity_stops(
            &self.moves,
            &self.stop_before,
            INTEGRATION_TOL,
            EXTRUDE_ONLY_V,
            EXTRUDE_ONLY_A,
            self.entry,
        )
    }
}

fn arb_plan_case() -> impl Strategy<Value = PlanCase> {
    (
        prop::collection::vec(arb_planner_segment(), 1..4),
        log_uniform(MIN_ACCEL, MAX_ACCEL),
        log_uniform(10.0, MAX_FEED_MM_S),
        prop::collection::vec(prop::bool::weighted(0.25), 4),
        prop_oneof![3 => Just(0.0), 2 => 0.0f64..1.0],
    )
        .prop_map(|(segments, accel, feed, stops, entry_fraction)| {
            // The feed may exceed a member's curvature caps: the brake into a
            // tighter end then hugs the cap, which the rail must follow
            // smoothly enough for its seams to close.
            let limits = VelocityLimits::try_new(MAX_FEED_MM_S, accel, 0.02, f64::INFINITY)
                .expect("planner limits");
            let moves: Vec<Move> = segments
                .into_iter()
                .enumerate()
                .map(|(i, spatial)| Move {
                    segment: PathSegment::try_new(spatial, Vec::new()).expect("a path segment"),
                    feedrate_mm_s: feed,
                    limits,
                    source: SourceRange {
                        start_line: i as u32 + 1,
                        end_line: i as u32 + 1,
                    },
                })
                .collect();
            let stop_before = stops[..moves.len()].to_vec();
            let entry_ceiling = {
                let head = &moves[0];
                let kappa_entry = match &head.segment.spatial {
                    Some(Segment::Arc(a)) => 1.0 / a.radius,
                    Some(Segment::Clothoid(c)) => c.kappa_0.abs(),
                    _ => 0.0,
                };
                let curvature_cap = if kappa_entry > 0.0 {
                    (accel / kappa_entry).sqrt()
                } else {
                    f64::INFINITY
                };
                feed.min(curvature_cap)
                    .min((2.0 * accel * head.segment.s_len()).sqrt())
            };
            PlanCase {
                moves,
                stop_before,
                entry: BoundaryState {
                    v: entry_fraction * entry_ceiling,
                    a: 0.0,
                },
            }
        })
}

/// `SEAM_SLACK_REL·(1 + v) + 1e-6` — the slack `velocity/reconstruct.rs`
/// enforces on its own brake seam, and therefore the continuity the phase
/// chain promises.
fn seam_slack(speed_scale: f64) -> f64 {
    1e-6 * speed_scale + 1e-6
}

proptest! {
    #![proptest_config(ProptestConfig {
        cases: 256,
        failure_persistence: Some(Box::new(FileFailurePersistence::Direct(
            "proptest-regressions/law_fuzz.txt",
        ))),
        ..ProptestConfig::default()
    })]

    /// A phase's arc is the time integral of the velocity it reports.
    #[test]
    fn phase_arc_is_the_time_integral_of_its_velocity(
        rail in arb_rail(),
        span in arb_const_accel(),
    ) {
        const PANELS: usize = 4096;
        for segment in [&rail.segment, &span] {
            let covered = segment.end_distance() - segment.s0;
            fail(
                "integrated velocity",
                integrated_velocity(segment, PANELS),
                covered,
                1e-8 * covered.abs(),
            )?;
        }
    }

    /// The reported velocity is the time derivative of the reported arc, and
    /// the reported acceleration is the time derivative of the velocity.
    #[test]
    fn phase_state_derivatives_agree_with_the_state(
        rail in arb_rail(),
        span in arb_const_accel(),
        fractions in prop::collection::vec(0.02f64..0.98, 8),
    ) {
        for segment in [&rail.segment, &span] {
            let h = 1e-3 * segment.dt;
            for fraction in &fractions {
                let t = segment.t0 + segment.dt * fraction;
                let (_, v, a) = segment.state_at(t);
                let (s_lo, v_lo, _) = segment.state_at(t - h);
                let (s_hi, v_hi, _) = segment.state_at(t + h);
                let arc_scale = segment.end_distance() - segment.s0;
                let fd_noise = 8.0 * f64::EPSILON * (segment.s0.abs() + arc_scale) / h;
                fail(
                    "ds/dt against v",
                    (s_hi - s_lo) / (2.0 * h),
                    v,
                    3e-6 * (v.abs() + fd_noise),
                )?;
                fail(
                    "dv/dt against a",
                    (v_hi - v_lo) / (2.0 * h),
                    a,
                    1e-4 * a.abs().max(1.0),
                )?;
            }
        }
    }

    /// Every state of a rail sits on its acceleration disk, with the tangential
    /// branch the `brake` flag selected; the speed is therefore monotone and
    /// its extremes are the span's ends.
    ///
    /// The branch, budget and monotonicity are exact consequences of the law
    /// and are checked as such. The disk residual is not: `a` between knots is
    /// the derivative of the quintic in time, not a re-evaluation of the law,
    /// and on the fastest-turning rails (`|σ| ~ 1e4`) it drifts by up to
    /// ~2e-5 of the budget.
    #[test]
    fn disk_rail_states_stay_on_the_acceleration_disk(rail in arb_rail()) {
        const SAMPLES: usize = 512;
        let segment = &rail.segment;
        let budget = rail.accel;
        let (_, v_end, _) = segment.end_state();
        let (v_min, v_max) = (rail.entry_v.min(v_end), rail.entry_v.max(v_end));
        for t in sample_times(segment, SAMPLES) {
            let (s, v, a) = segment.state_at(t);
            let normal = rail.kappa_at(s).abs() * v * v;
            at_most("speed below zero", -v, 0.0)?;
            at_most("tangential accel over budget", a.abs(), budget * (1.0 + 1e-6))?;
            let signed = if rail.brake { a } else { -a };
            at_most("tangential accel on the wrong branch", signed, 1e-9 * budget)?;
            fail(
                "state off the disk",
                libm::hypot(a, normal),
                budget,
                1e-4 * budget,
            )?;
            at_most("speed above the span's ends", v - v_max, 1e-9 * (1.0 + v_max))?;
            at_most("speed below the span's ends", v_min - v, 1e-9 * (1.0 + v_max))?;
        }
        fail("min_velocity", segment.min_velocity(), v_min, 1e-9 * (1.0 + v_max))?;
    }

    /// The three constructors agree on the span they describe: `until_arc`
    /// covers exactly the arc asked of it, `reach_over` reports that span's
    /// exit speed, and `brake_to` lands its target speed at the same arc.
    #[test]
    fn rail_constructors_agree_on_the_span_they_cover(rail in arb_rail()) {
        let segment = &rail.segment;
        fail(
            "until_arc end arc",
            segment.end_distance() - segment.s0,
            rail.arc,
            1e-12 * rail.arc,
        )?;
        fail("until_arc entry speed", segment.state_at(segment.t0).1, rail.entry_v, 1e-12 * (1.0 + rail.entry_v))?;
        let (_, v_end, _) = segment.end_state();
        let reached = LawSegment::reach_over(rail.law(), rail.entry_v, rail.arc)
            .ok_or_else(|| TestCaseError::fail("reach_over refused a span until_arc covered"))?;
        fail("reach_over exit speed", reached, v_end, 1e-9 * (1.0 + v_end))?;

        let braking = ScalarLaw::DiskRail {
            accel: rail.accel,
            kappa0: rail.kappa0,
            sigma: rail.sigma,
            brake: true,
        };
        if let Some((landed, entry_v)) =
            LawSegment::brake_to(0.0, 0.0, braking, rail.arc, 0.5 * rail.entry_v)
        {
            let (arc, exit_v, _) = landed.end_state();
            fail("brake_to end arc", arc, rail.arc, 1e-12 * rail.arc)?;
            fail("brake_to landing speed", exit_v, 0.5 * rail.entry_v, 1e-9 * (1.0 + rail.entry_v))?;
            fail("brake_to entry speed", landed.state_at(0.0).1, entry_v, 1e-9 * (1.0 + entry_v))?;
            at_most("brake_to entry below its landing", entry_v, exit_v.max(entry_v))?;
        }
    }

    /// A planned move's phase chain is C1 in `(s, v)` across its seams, covers
    /// exactly the move, and lands the exit speed the plan chose. The planner
    /// itself must never report an internal failure on a well-formed chain.
    ///
    /// Speed seams are held to the slack the reconstruction declares for them
    /// (`SEAM_SLACK_REL·(1 + v) + 1e-6` in `velocity/reconstruct.rs`), which is
    /// the tolerance its own brake-seam check enforces; the seam and endpoint
    /// arcs are exact, since every phase is built to cover a computed arc, and
    /// every interior state stays inside its own phase's arc.
    #[test]
    fn planned_phase_chain_is_c1_and_lands_its_endpoints(case in arb_plan_case()) {
        let profile = match case.plan() {
            Ok(profile) => profile,
            Err(geometry::VelocityError::OverCommitted { .. }) => return Ok(()),
            Err(other) => {
                return Err(TestCaseError::fail(format!("planner failed: {other:?}")));
            }
        };
        for mv in &profile.moves {
            let phases = &mv.phases;
            let speed_scale = 1.0 + mv.peak_v;
            fail("chain starts at the move", phases[0].s0, 0.0, 1e-12 * mv.length)?;
            fail("chain starts at the entry speed", phases[0].v0, mv.entry_v, seam_slack(speed_scale))?;
            for pair in phases.windows(2) {
                let (before, after) = (&pair[0], &pair[1]);
                fail("phase seam time", after.t0, before.end_time(), 1e-12 * (1.0 + before.end_time().abs()))?;
                fail("phase seam arc", after.s0, before.end_distance(), 1e-9 * mv.length)?;
                fail("phase seam speed", after.v0, before.end_state().1, seam_slack(speed_scale))?;
            }
            let last = phases.last().expect("a move always carries a phase");
            fail("chain covers the move", last.end_distance(), mv.length, 1e-9 * mv.length)?;
            fail("chain lands the exit speed", last.end_state().1, mv.exit_v, seam_slack(speed_scale))?;
            for phase in phases {
                let (arc_lo, arc_hi) = (phase.s0, phase.end_distance());
                for t in sample_times(phase, 16) {
                    let (s, v, a) = phase.state_at(t);
                    at_most("phase speed below zero", -v, 1e-9 * speed_scale)?;
                    at_most("phase accel over the move limit", a.abs(), mv.accel * (1.0 + 1e-6))?;
                    at_most("phase arc before its start", arc_lo - s, 1e-9 * mv.length)?;
                    at_most("phase arc past its end", s - arc_hi, 1e-9 * mv.length)?;
                }
            }
        }
    }

    /// Raising the acceleration budget can only raise every reach and every
    /// curvature cap, so the same geometry is never traversed more slowly.
    #[test]
    fn a_larger_acceleration_budget_is_never_slower(
        case in arb_plan_case(),
        boost in 1.5f64..8.0,
    ) {
        let slow = match case.plan() {
            Ok(profile) => profile,
            Err(_) => return Ok(()),
        };
        let faster = case.with_accel(case.moves[0].limits.accel_mm_s2 * boost);
        let quick = match faster.plan() {
            Ok(profile) => profile,
            Err(other) => {
                return Err(TestCaseError::fail(format!(
                    "a larger budget broke a feasible plan: {other:?}"
                )));
            }
        };
        at_most(
            "traversal time grew with the acceleration budget",
            quick.report.traversal_time_s - slow.report.traversal_time_s,
            1e-9 * slow.report.traversal_time_s,
        )?;
    }
}
