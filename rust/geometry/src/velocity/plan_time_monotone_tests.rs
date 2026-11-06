use nurbs::algebra::PiecewisePolynomialKernel;
use proptest::prelude::*;
use proptest::test_runner::FileFailurePersistence;

use super::*;
use crate::fitter::{CornerFitConfig, fit_corners, seam_requires_stop};
use crate::frontend::{
    ExtruderCaps, LeaderSpill, Move, MoveContext, RestSettle, VelocityLimits, line_move,
};

const EXTRUDER_AXIS: usize = 3;
const EXTRUDE_ONLY_V: f64 = 40.0;
const EXTRUDE_ONLY_A: f64 = 2500.0;
const TIME_TOLERANCE: f64 = 1e-9;
/// The largest relative error a slicer's rounding of the extrusion lays
/// on a held ratio: the trident first layers step by up to 3e-4 of it.
const RATIO_NOISE: f64 = 5e-4;
/// The straight extrusion each print starts and ends with, at the held
/// ratio: longer than the reach within which the stream's ends, rests of
/// the leaders, narrow a step's ramp at the fastest ceiling generated
/// ([`crate::ratio_ramp::rest_room_reach_mm`] of 500 mm/s, 37.5 mm), so
/// every noisy join lies clear of them.
const LEAD_MM: f64 = 50.0;
const LEAD_FEED_MM_S: f64 = 100.0;

/// A print extruding one ratio up to a slicer's rounding noise: per move a
/// heading turn, a feedrate, the decimal logarithm of its length and its
/// rounding in [−1, 1]; and its machine.
#[derive(Debug, Clone)]
struct Print {
    steps: Vec<(f64, f64, f64, f64)>,
    ratio: f64,
    machine: Machine,
}

#[derive(Debug, Clone, Copy)]
struct Machine {
    max_velocity: f64,
    accel: f64,
    e_velocity: f64,
    e_accel: f64,
    slope: f64,
    kernel_reach_s: f64,
}

fn arb_print() -> impl Strategy<Value = Print> {
    (
        prop::collection::vec(
            (-2.5f64..2.5, 20.0f64..500.0, -1.3f64..1.3, -1.0f64..1.0),
            3..16,
        ),
        0.01f64..0.06,
        (
            80.0f64..500.0,
            3_000.0f64..60_000.0,
            (20.0f64..60.0, 2_000.0f64..30_000.0, 0.0f64..0.05),
            0.002f64..0.02,
        ),
    )
        .prop_map(
            |(steps, ratio, (max_velocity, accel, (e_velocity, e_accel, slope), reach))| Print {
                steps,
                ratio,
                machine: Machine {
                    max_velocity,
                    accel,
                    e_velocity,
                    e_accel,
                    slope,
                    kernel_reach_s: reach,
                },
            },
        )
}

fn extruder(machine: &Machine) -> ExtruderCaps {
    let reach = machine.kernel_reach_s;
    ExtruderCaps {
        axis_index: EXTRUDER_AXIS,
        max_velocity_mm_s: machine.e_velocity,
        max_accel_mm_s2: machine.e_accel,
        advance_slope_s: machine.slope,
        advance_curvature_s2_per_mm: 0.0,
        path_jerk_per_accel_hz: [0.0, 900.0, 700.0, 1100.0, 0.0, 900.0, 700.0, 1100.0],
        leader_peak_hz: [550.0, 550.0, 0.0],
        unshaped_leaders: [false; 3],
        own_jerk_per_accel_hz: f64::INFINITY,
        instantaneous_corner_velocity_mm_s: 2.0,
        leader_spill: LeaderSpill::of_kernels(&[&PiecewisePolynomialKernel::single_poly(
            vec![0.5 / reach],
            (-reach, reach),
        )]),
        leader_kernel_reach_s: reach,
        rest_settle: RestSettle::of_kernels(
            &[&PiecewisePolynomialKernel::single_poly(
                vec![0.5 / reach],
                (-reach, reach),
            )],
            0.005,
        ),
    }
}

impl Print {
    /// The print's moves fitted under its machine, with their stops.
    fn fitted(&self) -> Option<(Vec<Move>, Vec<bool>)> {
        let limits = VelocityLimits::try_new(
            self.machine.max_velocity,
            self.machine.accel,
            crate::CornerBudget::Shaped(0.05),
            f64::INFINITY,
        )
        .and_then(|limits| limits.with_extruder_caps(Some(extruder(&self.machine))))
        .ok()?;
        let mut at = [100.0, 100.0, 0.2];
        let mut heading = 0.0_f64;
        let mut raw = Vec::new();
        let lead = (0.0, LEAD_FEED_MM_S, libm::log10(LEAD_MM), 0.0);
        let steps = std::iter::once(&lead)
            .chain(&self.steps)
            .chain(std::iter::once(&lead));
        for (line, &(turn, feed, log_length, rounding)) in steps.enumerate() {
            heading += turn;
            let length = libm::pow(10.0, log_length);
            let end = [
                at[0] + length * libm::cos(heading),
                at[1] + length * libm::sin(heading),
                at[2],
            ];
            let ratio = self.ratio * (1.0 + RATIO_NOISE * rounding);
            let context = MoveContext {
                extruder_axis: EXTRUDER_AXIS,
                feedrate_mm_s: feed,
                limits,
                source: SourceRange {
                    start_line: line as u32 + 1,
                    end_line: line as u32 + 1,
                },
            };
            raw.push(line_move(at, end, ratio * length, context).ok()?);
            at = end;
        }
        let moves = fit_corners(&raw, CornerFitConfig::default()).ok()?.moves;
        let stop_before = (0..moves.len())
            .map(|k| {
                k > 0 && seam_requires_stop(&moves[k - 1], &moves[k], CornerFitConfig::default())
            })
            .collect();
        Some((moves, stop_before))
    }
}

/// A plan's print time.
struct Plan {
    time_s: f64,
}

/// The planner's plan of `moves` with every move's limits loosened by
/// `loosen`.
fn plan(
    moves: &[Move],
    stop_before: &[bool],
    loosen: impl Fn(VelocityLimits) -> VelocityLimits,
) -> Plan {
    let loosened: Vec<Move> = moves
        .iter()
        .map(|m| Move {
            limits: loosen(m.limits),
            ..m.clone()
        })
        .collect();
    let budgets = extrusion_budgets(&loosened, EXTRUDE_ONLY_V, EXTRUDE_ONLY_A);
    let params = VelocityPlanParams {
        integration_tol: 1e-7,
        max_extrude_only_velocity_mm_s: EXTRUDE_ONLY_V,
        max_extrude_only_accel_mm_s2: EXTRUDE_ONLY_A,
        entry_v: 0.0,
        step_capacity: None,
        shaped_corners: None,
    };
    let time_s = plan_velocity_stops_select_prefix(
        &loosened,
        stop_before,
        &budgets,
        params,
        &Serial,
        |_| loosened.len(),
    )
    .expect("the fitted print plans")
    .report
    .traversal_time_s;
    Plan { time_s }
}

fn no_slower(what: &str, loosened: &Plan, held: &Plan) -> Result<(), TestCaseError> {
    prop_assert!(
        loosened.time_s <= held.time_s * (1.0 + TIME_TOLERANCE) + TIME_TOLERANCE,
        "a looser {what} takes {} s against {} s",
        loosened.time_s,
        held.time_s
    );
    Ok(())
}

proptest! {
    #![proptest_config(ProptestConfig {
        cases: 256,
        failure_persistence: Some(Box::new(FileFailurePersistence::Direct(
            "proptest-regressions/plan_time_monotone_tests.txt",
        ))),
        ..ProptestConfig::default()
    })]

    /// Loosening every move's acceleration never slows a print extruding
    /// one ratio up to rounding noise: no cap the planner derives at the
    /// joins, the noise steps' among them, tightens as the budget grows,
    /// and no move trades a speed its plan reaches for an acceleration it
    /// gives up.
    #[test]
    fn a_looser_acceleration_never_slows_a_held_ratio(print in arb_print(), scale in 1.0f64..2.0) {
        let Some((moves, stop_before)) = print.fitted() else {
            return Ok(());
        };
        let held = plan(&moves, &stop_before, |limits| limits);
        let loosened = plan(&moves, &stop_before, |limits| VelocityLimits {
            accel_mm_s2: scale * limits.accel_mm_s2,
            ..limits
        });
        no_slower("max_accel", &loosened, &held)?;
    }

    /// Loosening every move's speed limit never slows a print extruding one
    /// ratio: the extruder's velocity cap trades the budget only for speed
    /// the plan can reach, and the joins read the speeds the plan can reach.
    #[test]
    fn a_looser_speed_limit_never_slows_a_held_ratio(print in arb_print(), scale in 1.0f64..2.0) {
        let Some((moves, stop_before)) = print.fitted() else {
            return Ok(());
        };
        let held = plan(&moves, &stop_before, |limits| limits);
        let loosened = plan(&moves, &stop_before, |limits| VelocityLimits {
            max_velocity_mm_s: scale * limits.max_velocity_mm_s,
            ..limits
        });
        no_slower("max_velocity", &loosened, &held)?;
    }

    /// Loosening the extruder's speed and acceleration caps never slows a
    /// print extruding one ratio.
    #[test]
    fn looser_extruder_caps_never_slow_a_held_ratio(
        print in arb_print(),
        (velocity_scale, accel_scale) in (1.0f64..2.0, 1.0f64..2.0),
    ) {
        let Some((moves, stop_before)) = print.fitted() else {
            return Ok(());
        };
        let held = plan(&moves, &stop_before, |limits| limits);
        let loosened = plan(&moves, &stop_before, |limits| VelocityLimits {
            extruder: limits.extruder.map(|caps| ExtruderCaps {
                max_velocity_mm_s: velocity_scale * caps.max_velocity_mm_s,
                max_accel_mm_s2: accel_scale * caps.max_accel_mm_s2,
                ..caps
            }),
            ..limits
        });
        no_slower("extruder cap", &loosened, &held)?;
    }
}
