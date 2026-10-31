use std::sync::Arc;

use nurbs::ScalarNurbs;
use proptest::prelude::*;
use proptest::test_runner::FileFailurePersistence;
use trajectory::{
    ClockedMotorSpan, ContinuousAxis, ContinuousError, MotorGroup, MotorSpan, MotorTerm, Pva,
    MAX_SPAN_SECS,
};

/// Two roundings of one shared stream instant: the end of a view and the start
/// of its successor are each `round(start_clock_exact + dt * freq)`, so a seam
/// may disagree by that much. Restated from `step_shim::ring`, which the
/// trajectory crate must not depend on.
const SEAM_ROUNDING_CYCLES: u64 = 2;
const SIGNAL_DEGREE: u8 = 4;
const SIGNAL_INTERIOR_KNOTS: usize = 3;
const SIGNAL_CONTROL_POINTS: usize = SIGNAL_INTERIOR_KNOTS + SIGNAL_DEGREE as usize + 1;
const SAMPLED_CLOCKS: usize = 24;
const ENUMERATION_LIMIT: u64 = 512;
const RELATIVE_SLACK: f64 = 1e-9;

/// A clamped degree-4 spline with simple interior knots: the signal is C3, so
/// two evaluations one ulp of stream time apart cannot disagree in position,
/// velocity, or acceleration by more than that ulp times the local jerk.
fn smooth_signal(t_start: f64, t_end: f64, offsets: &[f64], amplitude_mm: f64) -> Arc<MotorSpan> {
    let order = SIGNAL_DEGREE as usize + 1;
    let mut knots = vec![t_start; order];
    for index in 1..=SIGNAL_INTERIOR_KNOTS {
        knots.push(t_start + (t_end - t_start) * index as f64 / (SIGNAL_INTERIOR_KNOTS + 1) as f64);
    }
    knots.extend(std::iter::repeat_n(t_end, order));
    let control_points = offsets
        .iter()
        .map(|offset| amplitude_mm * offset)
        .collect::<Vec<f64>>();
    let curve = Arc::new(
        ScalarNurbs::try_new(SIGNAL_DEGREE, knots, control_points).expect("a clamped spline"),
    );
    Arc::new(
        MotorSpan::try_new(
            Arc::from([MotorGroup::Independent(MotorTerm {
                source_axis: 0,
                axis: ContinuousAxis::Spline(curve),
                scale: 1.0,
            })]),
            t_start,
            t_end,
            1,
            41,
            false,
        )
        .expect("a dispatchable motor span"),
    )
}

#[derive(Debug, Clone)]
struct ClockCase {
    frequency: f64,
    signal_t_start: f64,
    lead: f64,
    duration: f64,
    tail: f64,
    start_clock_exact: f64,
    start_host: f64,
    host_span: f64,
    offsets: Vec<f64>,
    amplitude_mm: f64,
    clock_fractions: Vec<f64>,
}

impl ClockCase {
    /// Rounding moves each end by at most half a cycle, so a window of four
    /// cycles always spans at least one whole clock.
    fn duration(&self) -> f64 {
        self.duration.max(4.0 / self.frequency)
    }

    fn window(&self) -> (f64, f64) {
        let start = self.signal_t_start + self.lead;
        (start, start + self.duration())
    }

    fn signal(&self) -> Arc<MotorSpan> {
        let (_, window_end) = self.window();
        smooth_signal(
            self.signal_t_start,
            window_end + self.tail,
            &self.offsets,
            self.amplitude_mm,
        )
    }

    fn clocked(&self) -> ClockedMotorSpan {
        let (start, end) = self.window();
        ClockedMotorSpan::try_new(
            self.signal(),
            start,
            end,
            self.start_host,
            self.start_host + self.host_span,
            self.start_clock_exact,
            self.frequency,
        )
        .expect("a clocked view over a positive window")
    }

    fn sampled_clocks(&self, first: u64, last: u64) -> Vec<u64> {
        if last - first <= ENUMERATION_LIMIT {
            return (first..=last).collect();
        }
        let mut clocks = vec![first, first + 1, last - 1, last];
        let span = (last - first) as f64;
        clocks.extend(
            self.clock_fractions
                .iter()
                .map(|fraction| first + (fraction * span) as u64),
        );
        clocks.sort_unstable();
        clocks.dedup();
        clocks
    }
}

fn arb_clock_case() -> impl Strategy<Value = ClockCase> {
    (
        prop_oneof![Just(1e6), Just(16e6), Just(72e6), Just(180e6), Just(520e6)],
        prop_oneof![Just(0.0), 0.0f64..1_000.0],
        prop_oneof![Just(0.0), 0.0f64..0.5],
        prop_oneof![1e-6f64..1e-4, 1e-4f64..MAX_SPAN_SECS, MAX_SPAN_SECS..0.3],
        prop_oneof![Just(0.0), 0.0f64..0.5],
        (
            prop_oneof![
                Just(0u64),
                0u64..1_000_000,
                0u64..(1u64 << 40),
                Just((1u64 << 40) - 1)
            ],
            prop_oneof![Just(0.0), Just(0.5), 0.0f64..1.0],
        ),
        prop_oneof![Just(0.0), 0.0f64..100_000.0],
        prop_oneof![Just(0.0), 1e-6f64..0.5],
        prop::collection::vec(-1.0f64..=1.0, SIGNAL_CONTROL_POINTS),
        prop_oneof![Just(0.0), 1e-6f64..1.0, 1.0f64..200.0],
        prop::collection::vec(0.0f64..=1.0, SAMPLED_CLOCKS),
    )
        .prop_map(
            |(
                frequency,
                signal_t_start,
                lead,
                duration,
                tail,
                (clock_whole, clock_fraction),
                start_host,
                host_span,
                offsets,
                amplitude_mm,
                clock_fractions,
            )| ClockCase {
                frequency,
                signal_t_start,
                lead,
                duration,
                tail,
                start_clock_exact: clock_whole as f64 + clock_fraction,
                start_host,
                host_span,
                offsets,
                amplitude_mm,
                clock_fractions,
            },
        )
}

fn magnitude_scale(samples: [&Pva; 2]) -> f64 {
    samples
        .into_iter()
        .flat_map(|pva| {
            [
                pva.position.abs(),
                pva.velocity.abs(),
                pva.acceleration.abs(),
            ]
        })
        .fold(1.0_f64, f64::max)
}

/// Two samples of one signal taken `[lo, hi]` apart: the signal's own bounds
/// over that bracket are what the mean value theorem allows them to differ by.
fn check_same_sample_across(
    left: &Pva,
    right: &Pva,
    signal: &MotorSpan,
    lo: f64,
    hi: f64,
) -> Result<(), TestCaseError> {
    let bracket = signal.pva_bounds(lo, hi).expect("bounds over the bracket");
    let slack = RELATIVE_SLACK * magnitude_scale([left, right]);
    let speed_ceiling = bracket.velocity_min.abs().max(bracket.velocity_max.abs());
    for (name, difference, allowance) in [
        (
            "position",
            left.position - right.position,
            speed_ceiling * (hi - lo),
        ),
        (
            "velocity",
            left.velocity - right.velocity,
            bracket.velocity_max - bracket.velocity_min,
        ),
        (
            "acceleration",
            left.acceleration - right.acceleration,
            2.0 * bracket.acceleration_abs_max,
        ),
    ] {
        prop_assert!(
            difference.abs() <= allowance + slack,
            "{name} differs by {difference} over [{lo}, {hi}], more than {allowance} + {slack}: {bracket:?}"
        );
    }
    Ok(())
}

proptest! {
    #![proptest_config(ProptestConfig {
        cases: 384,
        failure_persistence: Some(Box::new(FileFailurePersistence::Direct(
            "proptest-regressions/clocked_span_fuzz.txt",
        ))),
        ..ProptestConfig::default()
    })]

    #[test]
    fn every_clock_round_trips_through_stream_time(case in arb_clock_case()) {
        let clocked = case.clocked();
        let mut previous_time = f64::NEG_INFINITY;
        for clock in case.sampled_clocks(clocked.start_clock, clocked.end_clock) {
            let t = clocked
                .stream_time_at_clock(clock)
                .expect("a clock inside the view");
            prop_assert!(
                t >= clocked.stream_t_start && t <= clocked.stream_t_end,
                "clock {clock} mapped to {t} outside [{}, {}]",
                clocked.stream_t_start,
                clocked.stream_t_end
            );
            prop_assert!(
                t >= previous_time,
                "clock {clock} mapped to {t}, behind the previous {previous_time}"
            );
            previous_time = t;
            prop_assert_eq!(
                clocked.clock_at_stream_time(t).expect("a time inside the view"),
                clock,
                "clock {} did not survive the round trip through {}",
                clock,
                t
            );
        }
    }

    #[test]
    fn clocks_outside_the_view_fail_loudly(case in arb_clock_case()) {
        let clocked = case.clocked();
        let outside = |clock: u64| ContinuousError::ClockOutsideSpan {
            clock,
            start_clock: clocked.start_clock,
            end_clock: clocked.end_clock,
        };
        if let Some(before) = clocked.start_clock.checked_sub(1) {
            prop_assert_eq!(clocked.stream_time_at_clock(before).err(), Some(outside(before)));
            prop_assert_eq!(clocked.position_at_clock(before).err(), Some(outside(before)));
        }
        let after = clocked.end_clock + 1;
        prop_assert_eq!(clocked.stream_time_at_clock(after).err(), Some(outside(after)));
        prop_assert_eq!(clocked.eval_at_clock(after).err(), Some(outside(after)));
    }

    #[test]
    fn split_pieces_tile_the_original_view(case in arb_clock_case()) {
        let clocked = case.clocked();
        let pieces = clocked.split_max_duration().expect("a splittable view");
        let piece_ceiling =
            MAX_SPAN_SECS + SEAM_ROUNDING_CYCLES as f64 / case.frequency + 8.0 * f64::EPSILON;

        prop_assert!(!pieces.is_empty());
        let first = &pieces[0];
        let last = &pieces[pieces.len() - 1];
        prop_assert_eq!(first.start_clock, clocked.start_clock);
        prop_assert_eq!(first.stream_t_start, clocked.stream_t_start);
        prop_assert_eq!(first.start_host, clocked.start_host);
        prop_assert_eq!(first.start_clock_exact, clocked.start_clock_exact);
        prop_assert_eq!(last.stream_t_end, clocked.stream_t_end);
        prop_assert_eq!(last.end_host, clocked.end_host);
        prop_assert!(
            last.end_clock.abs_diff(clocked.end_clock) <= SEAM_ROUNDING_CYCLES,
            "tail clock {} is more than a seam rounding from {}",
            last.end_clock,
            clocked.end_clock
        );

        for piece in &pieces {
            prop_assert!(Arc::ptr_eq(&piece.signal, &clocked.signal));
            prop_assert_eq!(piece.clock_freq_hz, clocked.clock_freq_hz);
            prop_assert!(piece.stream_t_end > piece.stream_t_start);
            prop_assert!(piece.stream_t_start >= clocked.stream_t_start);
            prop_assert!(piece.stream_t_end <= clocked.stream_t_end);
            let duration = piece.stream_t_end - piece.stream_t_start;
            prop_assert!(
                duration <= piece_ceiling,
                "piece duration {duration} exceeds {piece_ceiling}"
            );
        }

        for window in pieces.windows(2) {
            prop_assert_eq!(window[1].stream_t_start, window[0].stream_t_end);
            prop_assert_eq!(window[1].start_host, window[0].end_host);
            prop_assert!(
                window[0].end_clock.abs_diff(window[1].start_clock) <= SEAM_ROUNDING_CYCLES,
                "seam clocks {} and {} disagree by more than a rounding",
                window[0].end_clock,
                window[1].start_clock
            );
        }
    }

    #[test]
    fn split_pieces_evaluate_the_original_signal(case in arb_clock_case()) {
        let clocked = case.clocked();
        let pieces = clocked.split_max_duration().expect("a splittable view");
        for piece in &pieces {
            let first = piece.start_clock.max(clocked.start_clock) + 1;
            let last = piece.end_clock.min(clocked.end_clock);
            if last <= first {
                continue;
            }
            for clock in case.sampled_clocks(first, last - 1) {
                let piece_time = piece
                    .stream_time_at_clock(clock)
                    .expect("a clock inside the piece");
                let whole_time = clocked
                    .stream_time_at_clock(clock)
                    .expect("a clock inside the original");
                let anchor_slack = 4.0 * f64::EPSILON * clock as f64 / case.frequency;
                let time_slack = anchor_slack + 8.0 * f64::EPSILON * whole_time.abs().max(1.0);
                prop_assert!(
                    (piece_time - whole_time).abs() <= time_slack,
                    "clock {clock} maps to {piece_time} in the piece and {whole_time} in the original (slack {time_slack})"
                );
                let piece_sample = piece.eval_at_clock(clock).expect("a piece sample");
                let whole_sample = clocked.eval_at_clock(clock).expect("an original sample");
                prop_assert_eq!(
                    piece.position_at_clock(clock).expect("a piece position"),
                    piece_sample.position
                );
                if piece_time == whole_time {
                    prop_assert_eq!(piece_sample, whole_sample);
                    continue;
                }
                check_same_sample_across(
                    &piece_sample,
                    &whole_sample,
                    &clocked.signal,
                    piece_time.min(whole_time),
                    piece_time.max(whole_time),
                )?;
            }
        }
    }

    #[test]
    fn degenerate_clocked_views_are_rejected(case in arb_clock_case()) {
        let signal = case.signal();
        let (start, end) = case.window();
        let build = |stream_t_start: f64, stream_t_end: f64, exact: f64, frequency: f64| {
            ClockedMotorSpan::try_new(
                Arc::clone(&signal),
                stream_t_start,
                stream_t_end,
                case.start_host,
                case.start_host + case.host_span,
                exact,
                frequency,
            )
        };

        let malformed = ContinuousError::InvalidSpan {
            reason: "clocked view requires finite positive ranges and frequency",
        };
        for rejected in [
            build(start, start, case.start_clock_exact, case.frequency),
            build(end, start, case.start_clock_exact, case.frequency),
            build(start, end, case.start_clock_exact, 0.0),
            build(start, end, case.start_clock_exact, -case.frequency),
        ] {
            prop_assert_eq!(rejected.err(), Some(malformed));
        }
        for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            for rejected in [
                build(bad, end, case.start_clock_exact, case.frequency),
                build(start, bad, case.start_clock_exact, case.frequency),
                build(start, end, bad, case.frequency),
                build(start, end, case.start_clock_exact, bad),
            ] {
                prop_assert_eq!(rejected.err(), Some(malformed));
            }
        }
        prop_assert_eq!(
            build(start, end, -1.0, case.frequency).err(),
            Some(ContinuousError::InvalidSpan {
                reason: "clock mapping is not representable",
            })
        );
        let sub_cycle = start + 0.4 / case.frequency;
        prop_assert_eq!(
            build(start, sub_cycle, case.start_clock_exact.floor(), case.frequency).err(),
            Some(ContinuousError::InvalidSpan {
                reason: "positive-duration clocked view must span at least one clock",
            })
        );
    }
}
