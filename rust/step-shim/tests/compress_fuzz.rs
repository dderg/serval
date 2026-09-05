use proptest::prelude::*;
use proptest::test_runner::FileFailurePersistence;
use step_shim::compress::{CLOCK_DIFF_MAX, StepMove, compress_with_max_error};

const MAX_STEPS: usize = 400;
const MAX_GAP: u64 = 1 << 20;

/// How one stretch of step clocks is spaced: a stepper cruising, ramping, or
/// jittering around its nominal interval.
#[derive(Debug, Clone)]
enum Stretch {
    Cruise {
        interval: u64,
        count: usize,
    },
    Ramp {
        interval: u64,
        add: i64,
        count: usize,
    },
    Jitter {
        interval: u64,
        spread: u64,
        seeds: Vec<u64>,
    },
}

impl Stretch {
    fn gaps(&self) -> Vec<u64> {
        match self {
            Self::Cruise { interval, count } => vec![*interval; *count],
            Self::Ramp {
                interval,
                add,
                count,
            } => (0..*count as i64)
                .map(|n| (*interval as i64 + add * n).clamp(1, MAX_GAP as i64) as u64)
                .collect(),
            Self::Jitter {
                interval,
                spread,
                seeds,
            } => seeds
                .iter()
                .map(|seed| {
                    (interval + seed % (2 * spread + 1))
                        .saturating_sub(*spread)
                        .max(1)
                })
                .collect(),
        }
    }
}

fn arb_stretch() -> impl Strategy<Value = Stretch> {
    let interval = prop_oneof![1u64..64, 64u64..4096, 4096u64..MAX_GAP];
    prop_oneof![
        (interval.clone(), 1usize..80)
            .prop_map(|(interval, count)| Stretch::Cruise { interval, count }),
        (interval.clone(), -300i64..300, 1usize..80).prop_map(|(interval, add, count)| {
            Stretch::Ramp {
                interval,
                add,
                count,
            }
        }),
        (
            interval,
            0u64..512,
            prop::collection::vec(any::<u64>(), 1..80)
        )
            .prop_map(|(interval, spread, seeds)| Stretch::Jitter {
                interval,
                spread,
                seeds,
            }),
    ]
}

#[derive(Debug, Clone)]
struct Case {
    last_step_clock: u64,
    first_gap: u64,
    stretches: Vec<Stretch>,
    max_error: u32,
}

impl Case {
    fn steps(&self) -> Vec<u64> {
        let mut clock = self.last_step_clock + self.first_gap;
        let mut steps = vec![clock];
        for gap in self.stretches.iter().flat_map(Stretch::gaps) {
            clock += gap;
            steps.push(clock);
        }
        steps.truncate(MAX_STEPS);
        steps
    }
}

fn arb_case() -> impl Strategy<Value = Case> {
    (
        0u64..(1 << 40),
        1u64..CLOCK_DIFF_MAX,
        prop::collection::vec(arb_stretch(), 1..6),
        prop_oneof![
            Just(0u32),
            1u32..64,
            64u32..1600,
            Just(1600u32),
            1600u32..50_000
        ],
    )
        .prop_map(|(last_step_clock, first_gap, stretches, max_error)| Case {
            last_step_clock,
            first_gap,
            stretches,
            max_error,
        })
}

struct Decoded {
    clock: u64,
    move_index: usize,
    first_in_move: bool,
}

fn decode(moves: &[StepMove], last_step_clock: u64) -> Vec<Decoded> {
    let mut decoded = Vec::new();
    let mut base = last_step_clock;
    for (move_index, step_move) in moves.iter().enumerate() {
        for nth in 1..=step_move.count {
            decoded.push(Decoded {
                clock: step_move.step_clock(base, nth),
                move_index,
                first_in_move: nth == 1,
            });
        }
        base = step_move.last_clock(base);
    }
    decoded
}

proptest! {
    #![proptest_config(ProptestConfig {
        cases: 512,
        failure_persistence: Some(Box::new(FileFailurePersistence::Direct(
            "proptest-regressions/compress_fuzz.txt",
        ))),
        ..ProptestConfig::default()
    })]

    #[test]
    fn every_encoded_step_lands_early_within_its_error_allowance(case in arb_case()) {
        let steps = case.steps();
        let (moves, consumed) = compress_with_max_error(&steps, case.last_step_clock, case.max_error)
            .expect("monotonic reachable steps compress");
        let decoded = decode(&moves, case.last_step_clock);

        prop_assert_eq!(consumed, decoded.len());
        prop_assert!(consumed >= 1, "the first step is within CLOCK_DIFF_MAX and must be encoded");
        let mut previous_decoded = case.last_step_clock;
        let mut previous_target = case.last_step_clock;
        for (index, (step, target)) in decoded.iter().zip(&steps).enumerate() {
            prop_assert!(
                step.clock > previous_decoded,
                "step {index} (move {}) at {} does not advance past {previous_decoded}",
                step.move_index,
                step.clock
            );
            prop_assert!(
                step.clock <= *target,
                "step {index} (move {}) at {} is late for its target {target}",
                step.move_index,
                step.clock
            );
            let anchor = if step.first_in_move { previous_decoded } else { previous_target };
            let allowance = u64::from(case.max_error).min((target - anchor) / 2);
            prop_assert!(
                target - step.clock <= allowance,
                "step {index} (move {}) at {} is {} early; allowance {allowance} from anchor {anchor}",
                step.move_index,
                step.clock,
                target - step.clock
            );
            previous_decoded = step.clock;
            previous_target = *target;
        }
        if consumed < steps.len() {
            prop_assert!(
                steps[consumed] - previous_decoded >= CLOCK_DIFF_MAX,
                "step {consumed} at {} was left unencoded while reachable from {previous_decoded}",
                steps[consumed]
            );
        }
    }

    #[test]
    fn a_repeated_or_reversed_clock_is_rejected(
        case in arb_case(),
        flaw in 0usize..MAX_STEPS,
        reverse in any::<bool>(),
    ) {
        let mut steps = case.steps();
        let flaw = flaw % steps.len();
        steps[flaw] = if reverse && flaw > 0 { steps[flaw - 1] - 1 } else if flaw > 0 { steps[flaw - 1] } else { case.last_step_clock };

        prop_assert!(compress_with_max_error(&steps, case.last_step_clock, case.max_error).is_err());
    }
}
