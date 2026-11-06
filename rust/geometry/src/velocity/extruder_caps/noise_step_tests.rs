use nurbs::algebra::PiecewisePolynomialKernel;

use crate::frontend::{
    ExtruderCaps, LeaderSpill, MoveContext, RestSettle, VelocityLimits, line_move,
};
use crate::segment::SourceRange;
use crate::velocity::{VelocityProfile, plan_velocity_stops, warm_start_params};

const PEAK_RATIO: f64 = 0.03116;
const FEED_MM_S: f64 = 120.0;
const ACCEL_MM_S2: f64 = 74_661.0;
const MOVE_MM: f64 = 5.0;
const KERNEL_REACH_S: f64 = 2.0 * 0.001_895;

fn trident_like_extruder() -> ExtruderCaps {
    ExtruderCaps {
        axis_index: 3,
        max_velocity_mm_s: 50.0,
        max_accel_mm_s2: 1.0e6,
        advance_slope_s: 0.02,
        advance_curvature_s2_per_mm: 0.0,
        path_jerk_per_accel_hz: [0.0; 8],
        path_jerk_per_accel_step_hz: [0.0; 8],
        unshaped_leaders: [false; 3],
        own_jerk_per_accel_hz: f64::INFINITY,
        instantaneous_corner_velocity_mm_s: 1.0,
        leader_spill: LeaderSpill::of_kernels(&[&PiecewisePolynomialKernel::single_poly(
            vec![0.5 / KERNEL_REACH_S],
            (-KERNEL_REACH_S, KERNEL_REACH_S),
        )]),
        leader_kernel_reach_s: KERNEL_REACH_S,
        rest_settle: RestSettle::of_kernels(
            &[&PiecewisePolynomialKernel::single_poly(
                vec![0.5 / KERNEL_REACH_S],
                (-KERNEL_REACH_S, KERNEL_REACH_S),
            )],
            0.005,
        ),
    }
}

fn plan_ratios(ratios: &[f64]) -> VelocityProfile {
    let limits = VelocityLimits::try_new(500.0, ACCEL_MM_S2, 0.0, f64::INFINITY)
        .and_then(|limits| limits.with_extruder_caps(Some(trident_like_extruder())))
        .expect("valid limits");
    let moves: Vec<_> = ratios
        .iter()
        .enumerate()
        .map(|(index, &ratio)| {
            let start = index as f64 * MOVE_MM;
            line_move(
                [start, 0.0, 0.0],
                [start + MOVE_MM, 0.0, 0.0],
                ratio * MOVE_MM,
                MoveContext {
                    extruder_axis: 3,
                    feedrate_mm_s: FEED_MM_S,
                    limits,
                    source: SourceRange {
                        start_line: index as u32 + 1,
                        end_line: index as u32 + 1,
                    },
                },
            )
            .expect("a line move")
        })
        .collect();
    plan_velocity_stops(&moves, &vec![false; moves.len()], warm_start_params(1e-7))
        .expect("the moves plan")
}

#[test]
fn a_noise_ratio_step_plans_as_if_the_ratio_held() {
    let held = plan_ratios(&[PEAK_RATIO, PEAK_RATIO]);
    let noise = plan_ratios(&[PEAK_RATIO, PEAK_RATIO - 1e-5]);
    assert_eq!(noise.report.ratio_step_bound, 0, "{:?}", noise.report);
    assert!(
        (noise.moves[0].exit_v - held.moves[0].exit_v).abs() <= 1e-9 * held.moves[0].exit_v,
        "a 1e-5 ratio step crosses at {} mm/s, the held ratio at {} mm/s",
        noise.moves[0].exit_v,
        held.moves[0].exit_v
    );
}

#[test]
fn the_seam_slows_monotonically_from_the_held_speed_as_the_ratio_step_grows() {
    let held = plan_ratios(&[PEAK_RATIO, PEAK_RATIO]).moves[0].exit_v;
    let seam_speeds: Vec<(f64, f64)> = (0..=60)
        .map(|index| {
            let step =
                (PEAK_RATIO * libm::pow(10.0, -9.0 + 0.15 * f64::from(index))).min(PEAK_RATIO);
            (
                step,
                plan_ratios(&[PEAK_RATIO, PEAK_RATIO - step]).moves[0].exit_v,
            )
        })
        .collect();
    assert!(
        (seam_speeds[0].1 - held).abs() <= 1e-9 * held,
        "the smallest step crosses at {} mm/s, the held ratio at {held} mm/s",
        seam_speeds[0].1
    );
    for pair in seam_speeds.windows(2) {
        let ((smaller_step, before), (larger_step, after)) = (pair[0], pair[1]);
        assert!(
            after <= before * (1.0 + 1e-9) && after > 0.0,
            "a step of {larger_step} crosses at {after} mm/s, one of {smaller_step} at {before} mm/s"
        );
    }
}
