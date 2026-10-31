use proptest::prelude::*;
use proptest::test_runner::FileFailurePersistence;
use step_shim::compress::DEFAULT_MAX_ERROR_TICKS;
use step_shim::compress_hp::{HpScratch, StepMoveHp, compress_hp};

const MAX_STEPS: usize = 320;
const MAX_GAP: u64 = 1 << 20;
const WIRE_MAX_COUNT: u16 = 0x7FFF;
const WIRE_MAX_INTERVAL: u32 = 0x8000_0000;
const WIRE_MAX_ADD: i32 = 0x7FFF;
const WIRE_MAX_ADD2: i32 = 0xFFF;
const WIRE_MIN_SHIFT: i8 = -8;
const WIRE_MAX_SHIFT: i8 = 16;

/// `minmax_point` sizes a step's error window from the gap it follows, rounded
/// to the nearest 64th of a tick count.
const WINDOW_GAP_SHIFT: u32 = 6;
const MIN_STEP_ERR: u64 = 3;

/// The mcu's `queue_step_hp` decode: `command_queue_step_hp` pre-normalizes the
/// wire move, then `stepper_load_next` runs `add_interval`/`inc_interval` per
/// step in uint32 wrap arithmetic.
struct McuStepper {
    interval: u32,
    add: i32,
    add2: i32,
    low: u32,
    shift: u32,
}

impl McuStepper {
    fn load(m: &StepMoveHp) -> (u32, Self) {
        if m.shift <= 0 {
            let scale = 1_i64 << (-m.shift) as u32;
            let interval = m.interval.wrapping_shl((-m.shift) as u32);
            let add = (i64::from(m.add) * scale) as i32;
            let add2 = (i64::from(m.add2) * scale) as i32;
            (
                interval,
                Self {
                    interval: interval.wrapping_add(add as u32),
                    add: add.wrapping_add(add2),
                    add2,
                    low: 0,
                    shift: 0,
                },
            )
        } else {
            let shift = m.shift as u32;
            let seeded = m.interval.wrapping_add(1_u32 << (shift - 1));
            let first = seeded >> shift;
            (
                first,
                Self {
                    interval: m.interval.wrapping_add(m.add as u32),
                    add: i32::from(m.add).wrapping_add(i32::from(m.add2)),
                    add2: i32::from(m.add2),
                    low: seeded.wrapping_sub(first.wrapping_shl(shift)),
                    shift,
                },
            )
        }
    }

    fn next_delta(&mut self) -> u32 {
        let accumulated = self.interval.wrapping_add(self.low);
        let delta = accumulated >> self.shift;
        self.low = accumulated.wrapping_sub(delta.wrapping_shl(self.shift));
        self.interval = self.interval.wrapping_add(self.add as u32);
        self.add = self.add.wrapping_add(self.add2);
        delta
    }
}

/// Tick offsets of every step of one wire move from the pre-move step clock.
fn mcu_step_offsets(m: &StepMoveHp) -> Vec<u64> {
    let (first, mut stepper) = McuStepper::load(m);
    let mut time = first;
    let mut offsets = Vec::with_capacity(usize::from(m.count));
    offsets.push(u64::from(time));
    for _ in 1..m.count {
        time = time.wrapping_add(stepper.next_delta());
        offsets.push(u64::from(time));
    }
    offsets
}

fn wire_rejection(m: &StepMoveHp) -> Option<&'static str> {
    if m.count == 0 {
        return Some("count is zero");
    }
    if m.count > WIRE_MAX_COUNT {
        return Some("count is 0x8000 or greater");
    }
    if m.interval >= WIRE_MAX_INTERVAL {
        return Some("interval is at least 2^31");
    }
    if i32::from(m.add).abs() > WIRE_MAX_ADD {
        return Some("add is outside the wire range");
    }
    if i32::from(m.add2).abs() > WIRE_MAX_ADD2 {
        return Some("add2 is outside the wire range");
    }
    if !(WIRE_MIN_SHIFT..=WIRE_MAX_SHIFT).contains(&m.shift) {
        return Some("shift is outside the wire range");
    }
    if m.count > 1 && m.interval == 0 && m.add == 0 && m.add2 == 0 {
        return Some("zero interval and increments for multiple steps");
    }
    if m.shift < 0 && u64::from(m.interval) << (-m.shift) as u32 >= 1 << 32 {
        return Some("the shifted decode wraps the mcu's uint32 interval");
    }
    None
}

fn rounded_window_error(gap: u64) -> u64 {
    let scale = 1_u64 << WINDOW_GAP_SHIFT;
    if gap % scale >= scale / 2 {
        gap / scale + 1
    } else {
        gap / scale
    }
}

#[derive(Debug, Clone, Copy)]
struct Window {
    minp: i64,
    maxp: i64,
}

/// The allowance `minmax_point` grants the step at `index`: backward by the
/// rounded gap it follows (floored at three ticks, capped at the classic error
/// budget), forward by the rounded gap it precedes — and never forward at all
/// on the last step of the run, which may not pass its requested clock.
fn window(steps: &[u64], index: usize, move_start: usize, pre_move_clock: u64) -> Window {
    let point = steps[index] - pre_move_clock;
    let previous_gap = if index > move_start {
        steps[index] - steps[index - 1]
    } else {
        point
    };
    let mut backward = rounded_window_error(previous_gap)
        .max(MIN_STEP_ERR)
        .min(u64::from(DEFAULT_MAX_ERROR_TICKS));
    let mut forward = if index + 1 < steps.len() {
        rounded_window_error(steps[index + 1] - steps[index])
    } else {
        0
    };
    if forward != 0 {
        forward = backward.min(forward.max(MIN_STEP_ERR));
        backward = forward;
    }
    Window {
        minp: point as i64 - backward as i64,
        maxp: point as i64 + forward as i64,
    }
}

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
    let interval = prop_oneof![1u64..8, 8u64..64, 64u64..4096, 4096u64..MAX_GAP];
    prop_oneof![
        (interval.clone(), 1usize..90)
            .prop_map(|(interval, count)| Stretch::Cruise { interval, count }),
        (interval.clone(), -300i64..300, 1usize..90).prop_map(|(interval, add, count)| {
            Stretch::Ramp {
                interval,
                add,
                count,
            }
        }),
        (
            interval,
            0u64..512,
            prop::collection::vec(any::<u64>(), 1..90)
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
    seed_interval: u32,
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
        prop_oneof![1u64..8, 8u64..4096, 4096u64..MAX_GAP, MAX_GAP..(1 << 28)],
        prop::collection::vec(arb_stretch(), 1..5),
        prop_oneof![Just(0u32), 1u32..64, 64u32..40_000],
    )
        .prop_map(
            |(last_step_clock, first_gap, stretches, seed_interval)| Case {
                last_step_clock,
                first_gap,
                stretches,
                seed_interval,
            },
        )
}

#[derive(Debug)]
struct Decoded {
    clocks: Vec<u64>,
    end_clock: u64,
}

/// Walks the emitted moves the way the mcu does and checks every step against
/// the window the encoder declared for it.
fn decode_and_check_windows(
    steps: &[u64],
    last_step_clock: u64,
    moves: &[StepMoveHp],
) -> Result<Decoded, TestCaseError> {
    let mut clocks = Vec::with_capacity(steps.len());
    let mut cursor = last_step_clock;
    let mut move_start = 0usize;
    for (move_index, m) in moves.iter().enumerate() {
        prop_assert!(
            wire_rejection(m).is_none(),
            "move {move_index} {m:?} is not a legal wire move: {}",
            wire_rejection(m).unwrap_or_default()
        );
        let offsets = mcu_step_offsets(m);
        prop_assert_eq!(m.first_step, offsets[0], "move {} first_step", move_index);
        prop_assert_eq!(
            m.last_step,
            *offsets.last().expect("a move carries at least one step"),
            "move {} last_step",
            move_index
        );
        prop_assert!(
            move_start + offsets.len() <= steps.len(),
            "move {move_index} covers past the end of the run"
        );
        for (step_in_move, &offset) in offsets.iter().enumerate() {
            let index = move_start + step_in_move;
            let allowed = window(steps, index, move_start, cursor);
            prop_assert!(
                offset as i64 >= allowed.minp && offset as i64 <= allowed.maxp,
                "step {index} of move {move_index}: decoded offset {offset} outside \
                 {}:{} (requested offset {})",
                allowed.minp,
                allowed.maxp,
                steps[index] - cursor
            );
            clocks.push(cursor + offset);
        }
        cursor += m.last_step;
        move_start += offsets.len();
    }
    prop_assert_eq!(move_start, clocks.len());
    Ok(Decoded {
        clocks,
        end_clock: cursor,
    })
}

fn assert_strictly_increasing(clocks: &[u64], above: u64) -> Result<(), TestCaseError> {
    let mut previous = above;
    for (index, &clock) in clocks.iter().enumerate() {
        prop_assert!(
            clock > previous,
            "decoded step {index} at {clock} does not advance past {previous}"
        );
        previous = clock;
    }
    Ok(())
}

proptest! {
    #![proptest_config(ProptestConfig {
        cases: 256,
        failure_persistence: Some(Box::new(FileFailurePersistence::Direct(
            "proptest-regressions/compress_hp_fuzz.txt",
        ))),
        ..ProptestConfig::default()
    })]

    #[test]
    fn every_hp_step_decodes_inside_its_declared_window(case in arb_case()) {
        let steps = case.steps();
        let (moves, covered, _) =
            compress_hp(&mut HpScratch::new(), &steps, case.last_step_clock, case.seed_interval)
                .expect("a monotonic run of representable steps compresses");

        prop_assert_eq!(covered, steps.len(), "the encoder must consume the whole run");
        let decoded = decode_and_check_windows(&steps, case.last_step_clock, &moves)?;
        prop_assert_eq!(decoded.clocks.len(), covered, "decoded steps vs consumed steps");
        assert_strictly_increasing(&decoded.clocks, case.last_step_clock)?;
        prop_assert_eq!(
            decoded.end_clock,
            *decoded.clocks.last().expect("a non-empty run decodes"),
            "the carried step clock must be the last decoded step"
        );
        prop_assert!(
            decoded.end_clock <= *steps.last().expect("a non-empty run"),
            "the terminal step {} passes its requested clock {}",
            decoded.end_clock,
            steps.last().expect("a non-empty run")
        );
    }

    #[test]
    fn a_reused_scratch_is_indistinguishable_from_a_fresh_one(
        cases in prop::collection::vec(arb_case(), 1..4),
    ) {
        let mut scratch = HpScratch::new();
        for case in &cases {
            let steps = case.steps();
            let reused = compress_hp(&mut scratch, &steps, case.last_step_clock, case.seed_interval);
            let fresh = compress_hp(
                &mut HpScratch::new(),
                &steps,
                case.last_step_clock,
                case.seed_interval,
            );
            prop_assert_eq!(reused.is_ok(), fresh.is_ok());
            if let (Ok(reused), Ok(fresh)) = (reused, fresh) {
                prop_assert_eq!(reused, fresh, "a reused scratch changed the wire");
            }
        }
    }

    #[test]
    fn a_run_split_across_two_calls_continues_the_same_stream(
        case in arb_case(),
        cut in 1usize..MAX_STEPS,
    ) {
        let steps = case.steps();
        assert!(steps.len() >= 2, "every generated stretch contributes a gap");
        let cut = 1 + cut % (steps.len() - 1);

        let mut scratch = HpScratch::new();
        let (head_moves, head_covered, carry) =
            compress_hp(&mut scratch, &steps[..cut], case.last_step_clock, case.seed_interval)
                .expect("the head of a representable run compresses");
        prop_assert_eq!(head_covered, cut);
        let head = decode_and_check_windows(&steps[..cut], case.last_step_clock, &head_moves)?;

        let (tail_moves, tail_covered, _) =
            compress_hp(&mut scratch, &steps[cut..], head.end_clock, carry)
                .expect("the tail resumes from the head's last step clock");
        prop_assert_eq!(tail_covered, steps.len() - cut);
        let tail = decode_and_check_windows(&steps[cut..], head.end_clock, &tail_moves)?;

        let mut joined = head.clocks;
        joined.extend(tail.clocks);
        prop_assert_eq!(joined.len(), steps.len());
        assert_strictly_increasing(&joined, case.last_step_clock)?;
    }

    #[test]
    fn a_constant_cadence_packs_at_least_half_the_pending_steps_per_move(
        interval in prop_oneof![1u64..8, 8u64..64, 64u64..4096, 4096u64..MAX_GAP],
        count in 2usize..MAX_STEPS,
        last_step_clock in 0u64..(1 << 40),
        hint_carries_the_cadence in any::<bool>(),
    ) {
        let steps: Vec<u64> = (1..=count as u64)
            .map(|nth| last_step_clock + interval * nth)
            .collect();
        let hint = if hint_carries_the_cadence { interval as u32 } else { 0 };
        let (moves, covered, _) =
            compress_hp(&mut HpScratch::new(), &steps, last_step_clock, hint)
                .expect("a constant cadence compresses");

        prop_assert_eq!(covered, count);
        decode_and_check_windows(&steps, last_step_clock, &moves)?;
        let mut pending = count;
        for (move_index, m) in moves.iter().enumerate() {
            prop_assert!(
                2 * usize::from(m.count) >= pending,
                "move {move_index} packs {} of the {pending} steps still pending in a run \
                 of {count} steps spaced {interval} ticks apart",
                m.count
            );
            pending -= usize::from(m.count);
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
        let previous = if flaw > 0 { steps[flaw - 1] } else { case.last_step_clock };
        steps[flaw] = if reverse { previous.saturating_sub(1) } else { previous };

        let error = compress_hp(
            &mut HpScratch::new(),
            &steps,
            case.last_step_clock,
            case.seed_interval,
        )
        .expect_err("a clock that does not advance is a producer bug");
        prop_assert!(
            error.detail.contains("not after previous clock"),
            "{}",
            error.detail
        );
    }
}
