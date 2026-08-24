// Differential tests: the encoder's fixed-point walk must reproduce, step
// for step, the times the 32-bit MCU executes for a queue_step_hp wire move.
//
// The reference model below is a literal transcription of the C decode and
// execute path — command_queue_step_hp (src/stepper_classic.c:320-358),
// stepper_load_next (src/stepper_classic.c:57-189), add_interval /
// inc_interval (src/stepper_classic.c:30-43), with the struct fields from
// src/stepper.h:35-49 — using the C's exact uint32/int32 wrap arithmetic.
// The encoder's walk (fill_stepper_moves/add_interval/inc_interval in
// compress_hp.rs) must agree with this model for every wire move, including
// the wrap regimes where the C's decoded interval leaves the uint32 domain.

use super::*;

/// Literal transcription of the C decode+execute path. Returns the step
/// clock offsets (mod 2^32, as the MCU's `uint32_t waketime` accumulates).
fn c_mcu_walk(m: StepMoveHp) -> Vec<u64> {
    let interval = m.interval;
    let add = i32::from(m.add);
    let add2 = i32::from(m.add2);
    let shift = i32::from(m.shift);
    // command_queue_step_hp (src/stepper_classic.c:331-353): pre-normalize.
    let (first, next_interval, m_add, m_add2, m_low, m_shift) = if shift <= 0 {
        let amount = -shift;
        let interval = interval.wrapping_shl(amount as u32);
        let add = if add >= 0 {
            add << amount
        } else {
            -(-add << amount)
        };
        let add2 = if add2 >= 0 {
            add2 << amount
        } else {
            -(-add2 << amount)
        };
        (
            interval,
            interval.wrapping_add(add as u32),
            add.wrapping_add(add2),
            add2,
            0_u32,
            0_u32,
        )
    } else {
        let seed = 1_u32 << (shift - 1);
        let first = interval.wrapping_add(seed) >> shift;
        (
            first,
            interval.wrapping_add(add as u32),
            add.wrapping_add(add2),
            add2,
            interval.wrapping_add(seed) - (first << shift),
            shift as u32,
        )
    };
    // stepper_load_next (src/stepper_classic.c:155-189): s->interval =
    // m->next_interval, the first step fires at m->interval.
    let mut s_interval = next_interval;
    let mut s_add = m_add;
    let mut s_low = m_low;
    let mut time = 0_u32;
    let mut offsets = Vec::with_capacity(m.count as usize);
    time = time.wrapping_add(first);
    offsets.push(u64::from(time));
    // Per-step events (src/stepper_classic.c:30-43): add_interval then
    // inc_interval between steps.
    for _ in 1..m.count {
        let acc = s_interval.wrapping_add(s_low);
        let delta = acc >> m_shift;
        s_low = acc - (delta << m_shift);
        time = time.wrapping_add(delta);
        offsets.push(u64::from(time));
        s_interval = s_interval.wrapping_add(s_add as u32);
        s_add = s_add.wrapping_add(m_add2);
    }
    offsets
}

fn c_model_deltas(m: StepMoveHp) -> Vec<u64> {
    let offsets = c_mcu_walk(m);
    let mut deltas = Vec::with_capacity(offsets.len());
    let mut previous = 0_u64;
    for offset in offsets {
        deltas.push(offset.wrapping_sub(previous));
        previous = offset;
    }
    deltas
}

fn wire_valid(m: StepMoveHp) -> bool {
    validate_wire(&m).is_ok()
}

fn assert_walk_equals_c_model(m: StepMoveHp) {
    let c = c_mcu_walk(m);
    let encoder = mcu_walk_offsets(&m).unwrap_or_else(|detail| {
        panic!(
            "encoder walk rejected a move the C model walks: {detail} \
             (shift={} interval={} count={} add={} add2={})",
            m.shift, m.interval, m.count, m.add, m.add2
        )
    });
    assert_eq!(
        encoder, c,
        "encoder walk diverges from the C model: shift={} interval={} count={} add={} add2={}",
        m.shift, m.interval, m.count, m.add, m.add2
    );
}

struct XorShift64(u64);

impl XorShift64 {
    fn next(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.0 = x;
        x
    }

    fn below(&mut self, n: u64) -> u64 {
        self.next() % n
    }
}

fn random_move(rng: &mut XorShift64, shift: i8) -> StepMoveHp {
    let bucket = rng.below(3);
    let interval = match bucket {
        0 => rng.below(1_000),
        1 => rng.below(1 << 24),
        _ => (1 << 24) + rng.below((1 << 30) - (1 << 24)),
    };
    let add = rng.below(0x10000) as i64 - 0x8000;
    let add2 = rng.below(0x2000) as i64 - 0x1000;
    StepMoveHp {
        interval: interval as u32,
        count: (rng.below(400) + 1) as u16,
        add: add as i16,
        add2: add2 as i16,
        shift,
        first_step: 0,
        last_step: 0,
    }
}

/// The encoder walk must equal the C decode+execute model across the whole
/// wire shift range, including the wraps that only the C's uint32 arithmetic
/// exhibits (shift<=0 scale-up past 2^32, and shifted-domain intervals that
/// go negative mid-move).
#[test]
fn c_decode_walk_matches_encoder_walk_across_shift_range() {
    let mut rng = XorShift64(0x9E37_79B9_7F4A_7C15);
    for shift in MIN_SHIFT..=MAX_SHIFT {
        // Homing-like constant velocity, at the wire scale of each shift.
        for count in [1_u16, 700] {
            let interval = if shift <= 0 {
                (42_000_u32) << (-shift)
            } else {
                (42_000_u32 >> shift).max(1)
            };
            assert_walk_equals_c_model(StepMoveHp {
                interval,
                count,
                add: 0,
                add2: 0,
                shift,
                first_step: 0,
                last_step: 0,
            });
        }
        // Accel ramps: one that stays positive and one that drives the
        // shifted-domain interval negative mid-move (C uint32 wrap).
        for (interval, add, add2) in [(100, -3, 0), (100, -7, 0), (40, -1, 1), (2_000, -60, 3)] {
            assert_walk_equals_c_model(StepMoveHp {
                interval,
                count: 30,
                add,
                add2,
                shift,
                first_step: 0,
                last_step: 0,
            });
        }
        // High-precision small-interval runs.
        for interval in 1_u32..=8 {
            for add in [-2_i16, -1, 1, 2] {
                assert_walk_equals_c_model(StepMoveHp {
                    interval,
                    count: 24,
                    add,
                    add2: -1,
                    shift,
                    first_step: 0,
                    last_step: 0,
                });
            }
        }
        for _ in 0..300 {
            let m = random_move(&mut rng, shift);
            if wire_valid(m) {
                assert_walk_equals_c_model(m);
            }
        }
    }
}

/// The C shift<=0 decode scales the wire interval by 2^-shift in uint32; an
/// interval whose scaled value reaches 2^32 decodes to a wrapped (near-zero)
/// step rate on the MCU. These are exactly the moves the encoder must never
/// validate as normal — the walk must agree with the C model on the wrapped
/// times (this test), and the window check must reject them (storm guard
/// test below).
#[test]
fn c_decode_wrap_moves_agree_with_encoder_walk() {
    for shift in MIN_SHIFT..=-1 {
        let amount = -shift as u32;
        let wrap_interval = 1_u32 << (32 - amount);
        for offset in [0_u32, 1, 1 << 16, (1 << 16) - 1, (1 << 24) - 1] {
            for add in [-0x7FFF_i16, 0, 0x7FFF] {
                for count in [2_u16, 10, 64] {
                    let interval = wrap_interval.wrapping_add(offset);
                    let m = StepMoveHp {
                        interval,
                        count,
                        add,
                        add2: 0,
                        shift,
                        first_step: 0,
                        last_step: 0,
                    };
                    if wire_valid(m) {
                        assert_walk_equals_c_model(m);
                    }
                }
            }
        }
    }
}

fn source_delta(source: &[u64], index: usize, queue_pos: usize, cursor: u64) -> u64 {
    if index > queue_pos {
        source[index] - source[index - 1]
    } else {
        source[index] - cursor
    }
}

/// Emitted moves must decode, per the C model, to step times inside the
/// source windows the encoder was given — the window check runs in the
/// C-model domain — and the claimed first/last offsets must be the C-model
/// endpoints.
#[test]
fn emitted_moves_stay_in_windows_in_c_model_domain() {
    let sources: &[&[u64]] = &[
        &constant_interval(42_000, 1_500, 0),
        &accel_ramp(42_000, 300, 1_500),
        &constant_interval(64, 900, 0),
        &jerk_run(1_200, 900),
        &decel_to_zero(42_000, 4, 2_000),
    ];
    for source in sources {
        let (moves, covered, _) = compress_hp(&mut HpScratch::new(), source, 0, 0).unwrap();
        assert_eq!(covered, source.len());
        let mut cursor = 0_u64;
        let mut input_pos = 0usize;
        for m in &moves {
            let offsets = c_mcu_walk(*m);
            assert_eq!(
                m.first_step, offsets[0],
                "first_step is not the C-model first step"
            );
            assert_eq!(
                m.last_step,
                *offsets.last().unwrap(),
                "last_step is not the C-model last step"
            );
            for (step_in_move, &offset) in offsets.iter().enumerate() {
                let index = input_pos + step_in_move;
                let point = minmax_point(source, index, input_pos, cursor);
                let offset = offset as i64;
                assert!(
                    offset >= point.minp && offset <= point.maxp,
                    "step {index}: C-model time {offset} outside window {}:{} (requested {})",
                    point.minp,
                    point.maxp,
                    source[index] - cursor
                );
            }
            cursor += m.last_step;
            input_pos += usize::from(m.count);
        }
    }
}

/// No emitted move may decode, per the C model, to per-step intervals below
/// 2x a plausible step_pulse_ticks floor unless the source step times are
/// genuinely that fast (or within the encoder's window error budget of
/// them). The bench storm move decoded to ~0-delta steps while its source
/// was homing-speed.
#[test]
fn storm_guard_emitted_moves_never_decode_to_sub_floor_intervals() {
    // 2x EDGE_STEP_TICKS at 168 MHz (DIV_ROUND_UP(168MHz, 8MHz) = 21).
    const FLOOR: u64 = 42;
    let sources: &[&[u64]] = &[
        &constant_interval(42_000, 1_500, 0),
        &accel_ramp(42_000, 300, 1_500),
        &constant_interval(1_000, 1_200, 0),
        &jerk_run(1_200, 900),
        &decel_to_zero(42_000, 4, 2_000),
    ];
    for source in sources {
        let (moves, covered, _) = compress_hp(&mut HpScratch::new(), source, 0, 0).unwrap();
        assert_eq!(covered, source.len());
        let mut cursor = 0_u64;
        let mut input_pos = 0usize;
        for m in &moves {
            let deltas = c_model_deltas(*m);
            for (step_in_move, &delta) in deltas.iter().enumerate() {
                let index = input_pos + step_in_move;
                let src_delta = source_delta(source, index, input_pos, cursor);
                let back_err = rounded_window_error(src_delta)
                    .max(MIN_STEP_ERR)
                    .min(u64::from(DEFAULT_MAX_ERROR_TICKS));
                assert!(
                    delta >= FLOOR || src_delta < FLOOR || delta + 2 * back_err >= src_delta,
                    "move shift={} step {index}: C-model interval {delta} below floor {FLOOR} \
                     while the source interval {src_delta} is not sub-floor",
                    m.shift
                );
            }
            cursor += m.last_step;
            input_pos += usize::from(m.count);
        }
    }
}

/// A source whose steps sit at ~2^32 spacing cannot be represented: the
/// shift<=0 decode would wrap to a near-zero step rate on the MCU. The
/// encoder must reject it (fail loudly) rather than emit a move the C model
/// decodes to storm-speed intervals.
#[test]
fn storm_guard_wrap_decode_is_rejected_not_emitted() {
    let wrap_source: Vec<u64> = (1..=64).map(|i| i * (1_u64 << 32) + i as u64).collect();
    let result = compress_hp(&mut HpScratch::new(), &wrap_source, 0, 0);
    if let Ok((moves, covered, _)) = result {
        assert_eq!(covered, wrap_source.len());
        let mut cursor = 0_u64;
        let mut input_pos = 0usize;
        for m in &moves {
            let offsets = c_mcu_walk(*m);
            for (step_in_move, &offset) in offsets.iter().enumerate() {
                let index = input_pos + step_in_move;
                let point = minmax_point(&wrap_source, index, input_pos, cursor);
                assert!(
                    (offset as i64) >= point.minp && (offset as i64) <= point.maxp,
                    "step {index}: C-model time {offset} diverges from source window \
                     {}:{} — the MCU would execute a different step rate than requested",
                    point.minp,
                    point.maxp
                );
            }
            cursor += m.last_step;
            input_pos += usize::from(m.count);
        }
    }
}

fn constant_interval(interval: u64, count: usize, base: u64) -> Vec<u64> {
    (1..=count)
        .map(|index| base + interval * index as u64)
        .collect()
}

fn accel_ramp(start: u64, end: u64, count: usize) -> Vec<u64> {
    let mut steps = Vec::with_capacity(count);
    let mut clock = 0_u64;
    for index in 0..count as u64 {
        let interval = start - ((start - end) * index / count as u64);
        clock += interval;
        steps.push(clock);
    }
    steps
}

fn decel_to_zero(start: u64, end: u64, count: usize) -> Vec<u64> {
    let mut steps = Vec::with_capacity(count);
    let mut clock = 0_u64;
    for index in 0..count as u64 {
        let interval = start - ((start - end) * index / count as u64);
        clock += interval.max(1);
        steps.push(clock);
    }
    steps
}

fn jerk_run(center: u64, count: usize) -> Vec<u64> {
    let mut steps = Vec::with_capacity(count);
    let mut clock = 0_u64;
    for index in 0..count as u64 {
        let t = index as i64 - count as i64 / 2;
        let interval = center + (t * t * t / 2_000_000).unsigned_abs();
        clock += interval.max(200);
        steps.push(clock);
    }
    steps
}
