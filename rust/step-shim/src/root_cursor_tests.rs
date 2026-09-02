use std::sync::Arc;

use nurbs::ScalarNurbs;
use trajectory::{
    ClockedMotorSpan, ContinuousAxis, MotorGroup, MotorSpan, MotorTerm, NudgeProfile,
};

use super::{EVAL_COUNT, Slope, StepRootCursor};
use crate::ring::SpanQueue;
use crate::{MotorConfig, StepEncoder};

#[test]
fn endpoint_roundoff_does_not_hide_a_monotonic_spline() {
    let duration = 0.0045;
    let curve = ScalarNurbs::try_new(
        2,
        vec![0.0, 0.0, 0.0, duration, duration, duration],
        vec![139.7, 140.000_000_000_000_5, 139.999_999_999_999_8],
    )
    .expect("a quadratic spline with endpoint-scale roundoff");
    let signal = Arc::new(
        MotorSpan::try_new(
            Arc::from([MotorGroup::Spline {
                curve: Arc::new(curve),
                summed_scale: 1.0,
            }]),
            0.0,
            duration,
            0,
            0,
            false,
        )
        .expect("a dispatchable motor span"),
    );
    let view =
        ClockedMotorSpan::try_new(signal, 0.0, duration, 0.0, duration, 1_000.0, 520_000_000.0)
            .expect("a clocked motor span");
    let config = MotorConfig {
        oid: 2,
        microstep_distance: 0.008,
        invert_dir: false,
        cycles_per_second: 520_000_000.0,
        encoder: StepEncoder::Classic {
            max_error_ticks: 13_000,
        },
        min_rearm_cycles: 0,
    };
    let cursor = StepRootCursor::new(&config);

    assert_eq!(
        cursor
            .certified_slope(0, &view, view.start_clock, view.end_clock)
            .unwrap(),
        Some(Slope::Rising)
    );
}

#[test]
fn a_c0_reversal_between_pieces_emits_every_root_on_both_slopes() {
    const FREQ: f64 = 1_000_000.0;
    const MICROSTEP_MM: f64 = 0.0025;
    const DEPTH_MM: f64 = 0.15;
    let duration = 0.001;
    let curve = ScalarNurbs::try_new(
        1,
        vec![0.0, 0.0, duration / 2.0, duration, duration],
        vec![0.0, -DEPTH_MM, 0.0],
    )
    .expect("a linear tent");
    let signal = Arc::new(
        MotorSpan::try_new(
            Arc::from([MotorGroup::Spline {
                curve: Arc::new(curve),
                summed_scale: 1.0,
            }]),
            0.0,
            duration,
            0,
            0,
            false,
        )
        .expect("a dispatchable motor span"),
    );
    let view = ClockedMotorSpan::try_new(signal, 0.0, duration, 0.0, duration, 0.0, FREQ)
        .expect("a clocked motor span");
    let config = MotorConfig {
        oid: 0,
        microstep_distance: MICROSTEP_MM,
        invert_dir: false,
        cycles_per_second: FREQ,
        encoder: StepEncoder::Classic { max_error_ticks: 0 },
        min_rearm_cycles: 0,
    };
    let mut queue = SpanQueue::new(1);
    queue.push(0, view).expect("an admissible view");
    let mut cursor = StepRootCursor::new(&config);
    let mut roots = Vec::new();

    cursor
        .advance(0, &config, &mut queue, u64::MAX, &mut roots, None)
        .expect("a drainable tent");

    let steps_per_slope = (DEPTH_MM / MICROSTEP_MM) as usize;
    let advances = roots.iter().map(|root| root.advance).collect::<Vec<i8>>();
    assert_eq!(
        advances,
        [vec![-1; steps_per_slope], vec![1; steps_per_slope]].concat(),
        "the tent descends {steps_per_slope} steps then climbs back"
    );
    assert_eq!(cursor.step_count(), 0);
}

fn constant_velocity_ramp(travel_mm: f64, speed_mm_s: f64, freq: f64) -> ClockedMotorSpan {
    let duration = travel_mm / speed_mm_s;
    let profile =
        NudgeProfile::try_new(travel_mm, speed_mm_s, 0.0, 0.0).expect("a constant-velocity nudge");
    let signal = Arc::new(
        MotorSpan::try_new(
            Arc::from([MotorGroup::Independent(MotorTerm {
                source_axis: 0,
                axis: ContinuousAxis::Nudge(profile),
                scale: 1.0,
            })]),
            0.0,
            duration,
            0,
            0,
            false,
        )
        .expect("a dispatchable motor span"),
    );
    ClockedMotorSpan::try_new(signal, 0.0, duration, 0.0, duration, 0.0, freq)
        .expect("a clocked motor span")
}

fn position_at(view: &ClockedMotorSpan, clock: u64) -> f64 {
    view.position_at_clock(clock)
        .expect("a clock inside the view")
}

fn least_clock_reaching(view: &ClockedMotorSpan, level: f64) -> u64 {
    let mut low = view.start_clock;
    let mut high = view.end_clock;
    assert!(
        position_at(view, low) < level && position_at(view, high) >= level,
        "the level must be bracketed by the view"
    );
    while high - low > 1 {
        let mid = low + (high - low) / 2;
        if position_at(view, mid) >= level {
            high = mid;
        } else {
            low = mid;
        }
    }
    high
}

#[test]
fn an_on_lattice_ramp_solves_each_root_within_a_constant_probe_budget() {
    const FREQ: f64 = 520_000_000.0;
    const MICROSTEP_MM: f64 = 0.01;
    const TRAVEL_MM: f64 = 4.0;
    const SPEED_MM_S: f64 = 40.0;
    const SPACING: u64 = 130_000;
    let steps = (TRAVEL_MM / MICROSTEP_MM) as u64;
    let view = constant_velocity_ramp(TRAVEL_MM, SPEED_MM_S, FREQ);
    assert_eq!(view.start_clock, 0);
    assert_eq!(view.end_clock, steps * SPACING);
    let level = |step: u64| step as f64 * MICROSTEP_MM;
    let expected = (1..=steps)
        .map(|step| least_clock_reaching(&view, level(step)))
        .collect::<Vec<u64>>();
    let landing_on_the_level = expected
        .iter()
        .zip(1..=steps)
        .filter(|&(&clock, step)| position_at(&view, clock) == level(step))
        .count();
    assert!(
        landing_on_the_level > 0,
        "no crossing lands bit-exactly on its level, so this ramp does not \
         exercise the degenerate bracket the probe budget bounds"
    );

    let config = MotorConfig {
        oid: 3,
        microstep_distance: MICROSTEP_MM,
        invert_dir: false,
        cycles_per_second: FREQ,
        encoder: StepEncoder::Classic { max_error_ticks: 0 },
        min_rearm_cycles: 0,
    };
    let mut queue = SpanQueue::new(4);
    queue.push(0, view.clone()).expect("an admissible view");
    let mut cursor = StepRootCursor::new(&config);
    cursor.reset_to(0, 0);
    let mut roots = Vec::new();
    let evals_before = EVAL_COUNT.with(std::cell::Cell::get);

    cursor
        .advance(0, &config, &mut queue, u64::MAX, &mut roots, None)
        .expect("a drainable ramp");

    let evals = EVAL_COUNT.with(std::cell::Cell::get) - evals_before;
    assert_eq!(
        roots.iter().map(|root| root.clock).collect::<Vec<u64>>(),
        expected,
        "every root must be the first clock whose position reaches its level"
    );
    assert!(
        roots.iter().all(|root| root.advance == 1 && root.dir == 1),
        "a rising ramp steps forward only"
    );
    assert!(
        evals <= 4 * steps,
        "{evals} evaluations for {steps} roots: the search is halving a bracket \
         whose far end is already the answer"
    );
}

#[test]
fn a_single_clock_window_keeps_the_last_root_of_a_decel_to_rest() {
    const FREQ: f64 = 1_000_000.0;
    const MICROSTEP_MM: f64 = 0.25;
    let travel_mm = -4.0 * MICROSTEP_MM;
    let profile =
        NudgeProfile::try_new(travel_mm, 100.0, 10_000.0, 0.0).expect("a triangular nudge");
    let duration = profile.duration();
    let cycles = (duration * FREQ).round() as u64;
    let signal = Arc::new(
        MotorSpan::try_new(
            Arc::from([MotorGroup::Independent(MotorTerm {
                source_axis: 0,
                axis: ContinuousAxis::Nudge(profile),
                scale: 1.0,
            })]),
            0.0,
            duration,
            0,
            0,
            false,
        )
        .expect("a dispatchable motor span"),
    );
    let view = |from: u64, to: u64| {
        ClockedMotorSpan::try_new(
            Arc::clone(&signal),
            from as f64 / FREQ,
            (to as f64 / FREQ).min(duration),
            0.0,
            0.0,
            from as f64,
            FREQ,
        )
        .expect("a clocked sub-view")
    };
    let config = MotorConfig {
        oid: 5,
        microstep_distance: MICROSTEP_MM,
        invert_dir: false,
        cycles_per_second: FREQ,
        encoder: StepEncoder::Classic { max_error_ticks: 0 },
        min_rearm_cycles: 0,
    };
    let solve = |views: &[ClockedMotorSpan]| {
        let mut queue = SpanQueue::new(4);
        for view in views {
            queue.push(0, view.clone()).expect("an admissible view");
        }
        let mut cursor = StepRootCursor::new(&config);
        cursor.reset_to(0, 0);
        let mut roots = Vec::new();
        cursor
            .advance(0, &config, &mut queue, u64::MAX, &mut roots, None)
            .expect("a drainable nudge");
        (roots, cursor.step_count())
    };
    let whole = view(0, cycles);
    assert_eq!(whole.eval_at_clock(cycles).unwrap().velocity, 0.0);
    assert!(position_at(&whole, cycles) <= travel_mm);

    let (whole_roots, whole_count) = solve(&[whole]);
    let (split_roots, split_count) = solve(&[view(0, cycles - 1), view(cycles - 1, cycles)]);

    assert_eq!(whole_count, -4);
    assert_eq!(whole_roots.last().map(|root| root.clock), Some(cycles));
    assert_eq!(
        split_roots, whole_roots,
        "the rest clock in its own view must still carry the fourth root"
    );
    assert_eq!(split_count, whole_count);
}
