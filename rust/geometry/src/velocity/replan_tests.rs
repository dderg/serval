use proptest::prelude::*;
use proptest::test_runner::FileFailurePersistence;

use super::*;
use crate::fitter::{CornerFitConfig, fit_corners, seam_requires_stop};
use crate::frontend::{ExtruderCaps, LeaderSpill, Move, MoveContext, VelocityLimits, line_move};
use crate::path::lowering::PositionProfile;
use nurbs::algebra::PiecewisePolynomialKernel;

const EXTRUDER_AXIS: usize = 3;
const EXTRUDE_ONLY_V: f64 = 40.0;
const EXTRUDE_ONLY_A: f64 = 2500.0;
const TOL: f64 = 1e-6;

/// One gcode move of a random print: a travel, an extrusion at some ratio,
/// or a retract or unretract in place; some climb.
#[derive(Debug, Clone, Copy)]
enum Step {
    Travel,
    Extrude(f64),
    InPlace(f64),
}

#[derive(Debug, Clone)]
struct Stream {
    moves: Vec<Move>,
    stop_before: Vec<bool>,
    budgets: Vec<ExtrusionBudget>,
    window: VelocityProfile,
}

fn params(entry_v: f64) -> VelocityPlanParams<'static> {
    VelocityPlanParams {
        integration_tol: 1e-7,
        max_extrude_only_velocity_mm_s: EXTRUDE_ONLY_V,
        max_extrude_only_accel_mm_s2: EXTRUDE_ONLY_A,
        entry_v,
        step_capacity: None,
    }
}

fn limits(accel: f64, (z_velocity, z_accel): (f64, f64), extruder: ExtruderCaps) -> VelocityLimits {
    VelocityLimits::try_new(500.0, accel, 0.05, f64::INFINITY)
        .and_then(|limits| limits.with_z_caps(z_velocity, z_accel))
        .and_then(|limits| limits.with_extruder_caps(Some(extruder)))
        .expect("valid limits")
}

fn extruder(max_velocity: f64, max_accel: f64, slope: f64) -> ExtruderCaps {
    ExtruderCaps {
        axis_index: EXTRUDER_AXIS,
        max_velocity_mm_s: max_velocity,
        max_accel_mm_s2: max_accel,
        advance_slope_s: slope,
        advance_curvature_s2_per_mm: 0.0,
        path_jerk_per_accel_hz: [0.0, 900.0, 700.0, 1100.0, 0.0, 900.0, 700.0, 1100.0],
        unshaped_leaders: [false, false, true],
        own_jerk_per_accel_hz: f64::INFINITY,
        instantaneous_corner_velocity_mm_s: 2.0,
        leader_spill: LeaderSpill::of_kernels(&[&PiecewisePolynomialKernel::single_poly(
            vec![125.0],
            (-0.004, 0.004),
        )]),
        leader_kernel_reach_s: 0.004,
    }
}

/// A step, the turn it takes, whether it climbs a layer, its feedrate and
/// the decimal logarithm of its length.
fn arb_step() -> impl Strategy<Value = (Step, f64, bool, f64, f64)> {
    (
        prop_oneof![
            2 => Just(Step::Travel),
            6 => (0.005f64..0.08).prop_map(Step::Extrude),
            1 => prop_oneof![Just(-0.8), Just(0.8)].prop_map(Step::InPlace),
        ],
        -std::f64::consts::PI..std::f64::consts::PI,
        prop::bool::weighted(0.1),
        20.0f64..600.0,
        -1.3f64..1.3,
    )
}

fn arb_stream() -> impl Strategy<Value = Stream> {
    (
        prop::collection::vec(arb_step(), 3..14),
        5_000.0f64..40_000.0,
        (25.0f64..400.0, 100.0f64..5_000.0),
        (30.0f64..60.0, 3_000.0f64..20_000.0, 0.0f64..0.05),
    )
        .prop_filter_map(
            "the stream fits and plans",
            |(steps, accel, z_caps, (e_velocity, e_accel, slope))| {
                let limits = limits(accel, z_caps, extruder(e_velocity, e_accel, slope));
                let mut at = [100.0, 100.0, 0.2];
                let mut heading = 0.0_f64;
                let mut raw = Vec::new();
                for (line, (step, turn, climbs, feed, log_length)) in steps.into_iter().enumerate()
                {
                    let context = MoveContext {
                        extruder_axis: EXTRUDER_AXIS,
                        feedrate_mm_s: feed,
                        limits,
                        source: SourceRange {
                            start_line: line as u32 + 1,
                            end_line: line as u32 + 1,
                        },
                    };
                    let m = match step {
                        Step::InPlace(e) => line_move(at, at, e, context),
                        Step::Travel | Step::Extrude(_) => {
                            heading += turn;
                            let length = libm::pow(10.0, log_length);
                            let end = [
                                at[0] + length * libm::cos(heading),
                                at[1] + length * libm::sin(heading),
                                at[2] + if climbs { 0.4 } else { 0.0 },
                            ];
                            let e = match step {
                                Step::Extrude(ratio) => ratio * crate::vec3::dist(at, end),
                                _ => 0.0,
                            };
                            let m = line_move(at, end, e, context);
                            at = end;
                            m
                        }
                    };
                    raw.push(m.ok()?);
                }
                let moves = fit_corners(&raw, CornerFitConfig::default()).ok()?.moves;
                let stop_before: Vec<bool> = (0..moves.len())
                    .map(|k| {
                        k > 0
                            && seam_requires_stop(
                                &moves[k - 1],
                                &moves[k],
                                CornerFitConfig::default(),
                            )
                    })
                    .collect();
                let budgets = extrusion_budgets(&moves, EXTRUDE_ONLY_V, EXTRUDE_ONLY_A);
                let window = plan_velocity_stops_select_prefix(
                    &moves,
                    &stop_before,
                    &budgets,
                    params(0.0),
                    &Serial,
                    |_| moves.len(),
                )
                .ok()?;
                Some(Stream {
                    moves,
                    stop_before,
                    budgets,
                    window,
                })
            },
        )
}

impl Stream {
    fn span(&self, (from, to): (f64, f64)) -> (usize, usize) {
        let n = self.moves.len();
        let lo = ((from * n as f64) as usize).min(n - 1);
        let hi = (lo + 1 + (to * (n - lo) as f64) as usize).min(n);
        (lo, hi)
    }

    fn replan(
        &self,
        (lo, hi): (usize, usize),
        scales: &[f64],
    ) -> Result<VelocityProfile, VelocityError> {
        replan_span(
            &self.moves[lo..hi],
            &self.stop_before[lo..hi],
            &self.budgets[lo..hi],
            scales,
            &self.window.boundary_speeds[lo..=hi],
            params(self.window.boundary_speeds[lo]),
            self.window.boundary_speeds[hi],
        )
    }

    /// The caps the planner holds each move of the span to, its budgets
    /// unscaled.
    fn caps(&self, (lo, hi): (usize, usize)) -> Vec<MoveCaps> {
        build_move_caps(
            &self.moves[lo..hi],
            &self.budgets[lo..hi],
            None,
            params(0.0),
            &mut VelocityReport::default(),
        )
        .expect("the window planned these caps")
    }
}

fn close(what: &str, got: f64, want: f64) -> Result<(), TestCaseError> {
    prop_assert!(
        (got - want).abs() <= TOL * (1.0 + want.abs()),
        "{what}: {got} against {want}"
    );
    Ok(())
}

fn within(what: &str, got: f64, bound: f64) -> Result<(), TestCaseError> {
    prop_assert!(
        got <= bound + TOL * (1.0 + bound.abs()),
        "{what}: {got} exceeds {bound}"
    );
    Ok(())
}

/// Every phase state of `velocity`, a few per phase: arc into the move,
/// speed, tangential acceleration.
fn states(velocity: &MoveVelocity) -> impl Iterator<Item = (f64, f64, f64)> + '_ {
    let origin = velocity.phases.first().map_or(0.0, |phase| phase.s0);
    velocity.phases.iter().flat_map(move |phase| {
        (0..=8).map(move |k| {
            let (s, v, a) = phase.state_at(phase.t0 + phase.dt * f64::from(k) / 8.0);
            (s - origin, v, a)
        })
    })
}

/// What the planner holds `m` to that it states physically: the path and
/// z speed caps, the extruder's velocity at its advance, and the
/// extrude-only caps on a move in place; the tangential acceleration within
/// the move's budget and, on a curve, the disk, both up to the defect a
/// disk rail's knots are proven within.
fn assert_physical_limits(m: &Move, velocity: &MoveVelocity) -> Result<(), TestCaseError> {
    let limits = &m.limits;
    let ratio = m
        .segment
        .followers
        .iter()
        .find(|follower| follower.axis_index == EXTRUDER_AXIS)
        .map_or(0.0, |follower| {
            follower.ratio.abs().max(follower.ratio_end.abs())
        });
    let budget = velocity.accel + law::RAIL_ACCELERATION_DEFECT;
    for (s, v, a) in states(velocity) {
        within("tangential acceleration", a.abs(), budget)?;
        match &m.segment.spatial {
            None => {
                within("extrude-only speed", v, EXTRUDE_ONLY_V)?;
                within("extrude-only acceleration", a.abs(), EXTRUDE_ONLY_A)?;
            }
            Some(segment) => {
                let s = s.clamp(0.0, segment.s_len());
                within(
                    "path speed",
                    v,
                    m.feedrate_mm_s.min(limits.max_velocity_mm_s),
                )?;
                within(
                    "z speed",
                    v * segment.heading_at(s)[2].abs(),
                    limits.max_z_velocity_mm_s,
                )?;
                let (kappa0, _) = segment.kappa_endpoints();
                let kappa = kappa0 + segment.dkappa_ds(0.0) * s;
                within(
                    "acceleration on the disk",
                    (a * a + (kappa * v * v).powi(2)).sqrt(),
                    budget,
                )?;
                if let Some(extruder) = &limits.extruder {
                    within(
                        "extruder velocity",
                        ratio * (v + extruder.advance_slope_s * velocity.accel),
                        extruder.max_velocity_mm_s,
                    )?;
                }
            }
        }
    }
    Ok(())
}

proptest! {
    #![proptest_config(ProptestConfig {
        cases: 128,
        failure_persistence: Some(Box::new(FileFailurePersistence::Direct(
            "proptest-regressions/replan_tests.txt",
        ))),
        ..ProptestConfig::default()
    })]

    /// With every budget kept, a span re-planned between the window's own
    /// seam speeds is the window's plan there: it reads the same caps,
    /// stops, budgets and seam ceilings as the planner.
    #[test]
    fn an_unscaled_replan_is_the_windows_plan(
        stream in arb_stream(),
        span in (0.0f64..1.0, 0.0f64..1.0),
    ) {
        let (lo, hi) = stream.span(span);
        let replanned = stream.replan((lo, hi), &vec![1.0; hi - lo]).expect("the window's own span replans");
        for (offset, (got, want)) in replanned.moves.iter().zip(&stream.window.moves[lo..hi]).enumerate() {
            let line = stream.moves[lo + offset].source.start_line;
            close(&format!("line {line} entry"), got.entry_v, want.entry_v)?;
            close(&format!("line {line} exit"), got.exit_v, want.exit_v)?;
            close(&format!("line {line} peak"), got.peak_v, want.peak_v)?;
            close(&format!("line {line} budget"), got.accel, want.accel)?;
        }
    }

    /// A span re-planned on lowered budgets keeps every limit the planner
    /// holds: the speed ceilings, the z, extruder and extrude-only caps, the
    /// disk, the stops, the seam ceilings (curvature and the ratio steps'
    /// at the lowered budgets) and both seam speeds it was cut at; and it
    /// raises no seam over the speed the window left there.
    #[test]
    fn a_replan_on_lowered_budgets_keeps_every_planner_limit(
        stream in arb_stream(),
        span in (0.0f64..1.0, 0.0f64..1.0),
        scales in prop::collection::vec(0.3f64..1.0, 64),
    ) {
        let (lo, hi) = stream.span(span);
        let scales = &scales[..hi - lo];
        let replanned = match stream.replan((lo, hi), scales) {
            Ok(replanned) => replanned,
            Err(VelocityError::OverCommitted { .. } | VelocityError::Infeasible { .. }) => {
                return Ok(());
            }
            Err(error) => return Err(TestCaseError::fail(format!("{error:?}"))),
        };
        let (entry, exit) = (stream.window.boundary_speeds[lo], stream.window.boundary_speeds[hi]);
        close("entry seam", replanned.boundary_speeds[0], entry)?;
        within("exit seam", replanned.boundary_speeds[hi - lo], exit)?;
        let caps = stream.caps((lo, hi));
        for (k, velocity) in replanned.moves.iter().enumerate() {
            let m = &stream.moves[lo + k];
            close("scaled budget", velocity.accel, caps[k].kin.accel * scales[k])?;
            within("move ceiling", velocity.peak_v, caps[k].kin.flat_ceiling)?;
            assert_physical_limits(m, velocity)?;
        }
        for k in 1..hi - lo {
            let seam = replanned.boundary_speeds[k];
            within("seam the window left", seam, stream.window.boundary_speeds[lo + k])?;
            if stream.stop_before[lo + k] {
                close("stop", seam, 0.0)?;
                continue;
            }
            let (up, dn) = (&caps[k - 1].kin, &caps[k].kin);
            let (up_accel, dn_accel) = (up.accel * scales[k - 1], dn.accel * scales[k]);
            let kappa_up = (up.kappa0 + up.sigma * up.length).abs();
            within("seam on the upstream disk", seam, disk::limit_speed(kappa_up, up_accel))?;
            within("seam on the downstream disk", seam, disk::limit_speed(dn.kappa0.abs(), dn_accel))?;
            within(
                "ratio step seam",
                seam,
                extruder_caps::ratio_step_ceiling(
                    &stream.moves[lo + k - 1],
                    &stream.moves[lo + k],
                    (&stream.budgets[lo + k - 1], &stream.budgets[lo + k]),
                    up_accel.max(dn_accel),
                ),
            )?;
        }
    }

    /// The seam the brake horizon finds stands whatever a later re-plan
    /// lowers at or past the far end: braking to rest there on the scaled
    /// budgets re-plans from that seam's own speed.
    #[test]
    fn a_rest_at_the_far_end_leaves_the_brake_horizon_standing(
        stream in arb_stream(),
        reach in 0.0f64..1.0,
        scales in prop::collection::vec(0.3f64..1.0, 64),
    ) {
        let n = stream.moves.len();
        let reach = 1 + ((reach * n as f64) as usize).min(n - 1);
        let scales = &scales[..reach];
        let seams = &stream.window.boundary_speeds[..=reach];
        let horizon = brake_horizon(
            &stream.moves[..reach],
            &stream.stop_before[..reach],
            &stream.budgets[..reach],
            scales,
            seams,
            params(0.0),
        )
        .expect("the window's moves have a brake envelope");
        let Some(standing) = horizon else {
            return Ok(());
        };
        prop_assert!(standing < reach);
        let braked = replan_span(
            &stream.moves[standing..reach],
            &stream.stop_before[standing..reach],
            &stream.budgets[standing..reach],
            &scales[standing..],
            &seams[standing..],
            params(seams[standing]),
            0.0,
        );
        prop_assert!(braked.is_ok(), "braking from seam {standing} to rest at {reach}: {braked:?}");
    }
}
