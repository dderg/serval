use std::sync::OnceLock;

use crate::compress::{CompressError, DEFAULT_MAX_ERROR_TICKS};

const MAX_ERR_2P: u32 = 6;
const MIN_STEP_ERR: u64 = 3;
const MAX_COUNT_LSM: usize = 1024;
const MAX_COUNT_BISECT: usize = 512;
const MAX_INTRVL: i64 = 0x3FF_FFFF;
const MAX_ADD: i64 = 0x7FFF;
const MAX_ADD2: i64 = 0xFFF;
const MAX_SHIFT: i8 = 16;
const MIN_SHIFT: i8 = -8;
const MAX_INT32: i64 = 0x7FFF_FFFF;
const FIRST_STEP_BIAS: f64 = 1.0;
const EXTRA_FIRST_STEP_BIAS: f64 = 19.0;
const A2_REGULARIZATION: f64 = 0.01;
const MAX_COUNT: usize = 0x7FFF;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct StepMoveHp {
    pub interval: u32,
    pub count: u16,
    pub add: i16,
    pub add2: i16,
    pub shift: i8,
    pub first_step: u64,
    pub last_step: u64,
}

impl StepMoveHp {
    pub fn first_clock(&self, last_step_clock: u64) -> u64 {
        last_step_clock.wrapping_add(self.first_step)
    }

    pub fn last_clock(&self, last_step_clock: u64) -> u64 {
        last_step_clock.wrapping_add(self.last_step)
    }
}

#[derive(Debug, Clone, Copy, Default)]
struct Matrix3x3 {
    a00: f64,
    a10: f64,
    a11: f64,
    a20: f64,
    a21: f64,
    a22: f64,
}

#[derive(Debug, Clone, Copy, Default)]
struct Rhs3 {
    b0: f64,
    b1: f64,
    b2: f64,
}

#[derive(Debug, Clone, Copy, Default)]
struct Points {
    minp: i64,
    maxp: i64,
}

static LEAST_SQUARES_LDL: OnceLock<Box<[Matrix3x3; MAX_COUNT_LSM]>> = OnceLock::new();
static LEAST_SQUARES_EFSB_LDL: OnceLock<Box<[Matrix3x3; MAX_COUNT_LSM]>> = OnceLock::new();

fn fill_least_squares_matrix(count: usize, extra_bias: bool) -> Matrix3x3 {
    let mut m = Matrix3x3::default();
    for i in 0..count {
        let c0 = (i + 1) as i64;
        let c1 = c0 * i as i64 / 2;
        let c2 = c1 * (i as i64 - 1) / 3;
        let c0 = c0 as f64;
        let c1 = c1 as f64;
        let c2 = c2 as f64;
        m.a00 += c0 * c0;
        m.a10 += c1 * c0;
        m.a11 += c1 * c1;
        m.a20 += c2 * c0;
        m.a21 += c2 * c1;
        m.a22 += c2 * c2;
    }
    m.a00 += FIRST_STEP_BIAS;
    if extra_bias {
        m.a00 += EXTRA_FIRST_STEP_BIAS;
    }
    m.a22 += A2_REGULARIZATION;
    if count < 2 {
        m.a11 = 1.0;
    }
    if count < 3 {
        m.a22 = 1.0;
    }
    m
}

fn compute_ldl(mut m: Matrix3x3) -> Matrix3x3 {
    let d0 = m.a00;
    m.a00 = 1.0 / d0;
    m.a10 *= m.a00;
    m.a20 *= m.a00;

    let d1 = m.a11 - d0 * m.a10 * m.a10;
    m.a11 = 1.0 / d1;
    m.a21 -= m.a20 * m.a10 * d0;
    m.a21 *= m.a11;

    let d2 = m.a22 - d0 * m.a20 * m.a20 - d1 * m.a21 * m.a21;
    m.a22 = 1.0 / d2;
    m
}

fn least_squares_ldl() -> &'static [Matrix3x3; MAX_COUNT_LSM] {
    LEAST_SQUARES_LDL.get_or_init(|| {
        Box::new(std::array::from_fn(|i| {
            compute_ldl(fill_least_squares_matrix(i + 1, false))
        }))
    })
}

fn least_squares_efsb_ldl() -> &'static [Matrix3x3; MAX_COUNT_LSM] {
    LEAST_SQUARES_EFSB_LDL.get_or_init(|| {
        Box::new(std::array::from_fn(|i| {
            compute_ldl(fill_least_squares_matrix(i + 1, true))
        }))
    })
}

fn solve_3x3(m: &Matrix3x3, f: &mut Rhs3) {
    f.b1 -= f.b0 * m.a10;
    f.b2 -= f.b0 * m.a20 + f.b1 * m.a21;

    f.b0 *= m.a00;
    f.b1 *= m.a11;
    f.b2 *= m.a22;

    f.b1 -= f.b2 * m.a21;
    f.b0 -= f.b1 * m.a10 + f.b2 * m.a20;
}

fn round_i64(value: f64) -> Option<i64> {
    if !value.is_finite() {
        return None;
    }
    let rounded = value.round();
    if rounded < i64::MIN as f64 || rounded > i64::MAX as f64 {
        None
    } else {
        Some(rounded as i64)
    }
}

fn step_move_encode(count: usize, f: &Rhs3) -> Option<StepMoveHp> {
    let mut result = StepMoveHp {
        interval: 0,
        count: 0,
        add: 0,
        add2: 0,
        shift: 0,
        first_step: 0,
        last_step: 0,
    };
    let mut interval = f.b0;
    let mut add = f.b1;
    let mut add2 = f.b2;
    if interval < 0.0 || count > MAX_COUNT {
        return None;
    }
    if count <= 1 {
        let interval = round_i64(interval)?;
        if !(0..=u32::MAX as i64).contains(&interval) {
            return None;
        }
        result.count = count as u16;
        result.interval = interval as u32;
        return Some(result);
    }

    let mut end_add = add + add2 * count as f64;
    let mut max_int_inc = count as f64 * add.abs().max(end_add.abs());
    let mut max_end_int = interval + max_int_inc;
    if add.abs() > MAX_ADD as f64
        || end_add.abs() > MAX_ADD as f64
        || add2.abs() > MAX_ADD2 as f64
        || max_end_int > MAX_INTRVL as f64
    {
        while result.shift >= MIN_SHIFT
            && (add.abs() > MAX_ADD as f64
                || end_add.abs() > MAX_ADD as f64
                || add2.abs() > MAX_ADD2 as f64
                || max_end_int > MAX_INTRVL as f64)
        {
            interval *= 0.5;
            add *= 0.5;
            add2 *= 0.5;
            end_add *= 0.5;
            max_int_inc *= 0.5;
            max_end_int *= 0.5;
            result.shift -= 1;
        }
        if result.shift < MIN_SHIFT {
            return None;
        }
    } else if max_int_inc >= 0.5 || count as f64 * (interval - interval.round()).abs() >= 0.5 {
        while result.shift < MAX_SHIFT {
            let next_interval = interval * 2.0;
            let next_add = add * 2.0;
            let next_add2 = add2 * 2.0;
            let next_end_add = end_add * 2.0;
            let next_max_end_int = max_end_int * 2.0;
            let next_shift = result.shift + 1;
            let extra_shift = if next_shift > 8 {
                (16 - next_shift) as u32
            } else {
                (8 - next_shift) as u32
            };
            let scale = (1_i64 << extra_shift) as f64;
            if next_add.abs() > MAX_ADD as f64
                || next_end_add.abs() > MAX_ADD as f64
                || next_add2.abs() > MAX_ADD2 as f64
                || next_max_end_int > MAX_INTRVL as f64
                || next_interval * scale > MAX_INT32 as f64
                || next_add.abs() * scale > MAX_INT32 as f64
                || next_add2.abs() * scale > MAX_INT32 as f64
            {
                break;
            }
            interval = next_interval;
            add = next_add;
            add2 = next_add2;
            end_add = next_end_add;
            max_end_int = next_max_end_int;

            result.shift = next_shift;
        }
    }

    let interval = round_i64(interval)?;
    let add = round_i64(add)?;
    let add2 = round_i64(add2)?;
    if !(0..=u32::MAX as i64).contains(&interval)
        || !(i16::MIN as i64..=i16::MAX as i64).contains(&add)
        || !(i16::MIN as i64..=i16::MAX as i64).contains(&add2)
    {
        return None;
    }
    result.count = count as u16;
    result.interval = interval as u32;
    result.add = add as i16;
    result.add2 = add2 as i16;
    Some(result)
}

fn points_error(detail: impl Into<String>) -> CompressError {
    CompressError {
        detail: detail.into(),
    }
}

fn rounded_window_error(delta: u64) -> u64 {
    if delta % (1 << MAX_ERR_2P) >= (1 << (MAX_ERR_2P - 1)) {
        delta / (1 << MAX_ERR_2P) + 1
    } else {
        delta / (1 << MAX_ERR_2P)
    }
}

fn minmax_point(steps: &[u64], index: usize, queue_pos: usize, last_step_clock: u64) -> Points {
    let point = steps[index]
        .checked_sub(last_step_clock)
        .expect("monotonic step clock validation precedes minmax_point");
    let previous_delta = if index > queue_pos {
        steps[index]
            .checked_sub(steps[index - 1])
            .expect("monotonic step clock validation precedes minmax_point")
    } else {
        point
    };
    let mut max_bck_error = rounded_window_error(previous_delta).max(MIN_STEP_ERR);
    max_bck_error = max_bck_error.min(u64::from(DEFAULT_MAX_ERROR_TICKS));

    let mut max_frw_error = if index + 1 < steps.len() {
        rounded_window_error(
            steps[index + 1]
                .checked_sub(steps[index])
                .expect("monotonic step clock validation precedes minmax_point"),
        )
    } else {
        0
    };
    if max_frw_error != 0 {
        max_frw_error = max_frw_error.max(MIN_STEP_ERR);
        let shared = max_bck_error.min(max_frw_error);
        max_bck_error = shared;
        max_frw_error = shared;
    } else {
        max_frw_error = MIN_STEP_ERR;
    }
    let point = i64::try_from(point).expect("step offset must fit signed arithmetic");
    let back = i64::try_from(max_bck_error).expect("window must fit signed arithmetic");
    let forward = i64::try_from(max_frw_error).expect("window must fit signed arithmetic");
    Points {
        minp: point - back,
        maxp: point + forward,
    }
}

#[derive(Debug, Clone, Copy)]
struct StepperMoves {
    interval: i64,
    add: i64,
    add2: i64,
    shift: u8,
    int_low_acc: i64,
}

fn fill_stepper_moves(m: &StepMoveHp) -> Result<StepperMoves, &'static str> {
    if m.shift <= 0 {
        let amount = (-m.shift) as u32;
        let scale = 1_i64 << amount;
        Ok(StepperMoves {
            interval: i64::from(m.interval) * scale,
            add: i64::from(m.add) * scale,
            add2: i64::from(m.add2) * scale,
            shift: 0,
            int_low_acc: 0,
        })
    } else {
        let extra_shift = if m.shift > 8 {
            (16 - m.shift) as u32
        } else {
            (8 - m.shift) as u32
        };
        let shift = if m.shift > 8 { 16 } else { 8 };
        let scale = 1_i64 << extra_shift;
        Ok(StepperMoves {
            interval: i64::from(m.interval) * scale,
            add: i64::from(m.add) * scale,
            add2: i64::from(m.add2) * scale,
            shift,
            int_low_acc: 1_i64 << (shift - 1),
        })
    }
}

fn add_interval(time: &mut i64, s: &mut StepperMoves) -> Result<(), &'static str> {
    let interval = s.interval + s.int_low_acc;
    let delta = if s.shift == 0 {
        interval
    } else {
        interval >> s.shift
    };
    *time = time.checked_add(delta).ok_or("step time overflow")?;
    if s.shift != 0 {
        s.int_low_acc = interval & ((1_i64 << s.shift) - 1);
    }
    Ok(())
}

fn inc_interval(s: &mut StepperMoves) -> Result<(), &'static str> {
    s.interval = s.interval.checked_add(s.add).ok_or("interval overflow")?;
    s.add = s.add.checked_add(s.add2).ok_or("add overflow")?;
    Ok(())
}

fn validate_wire(m: &StepMoveHp) -> Result<(), &'static str> {
    if m.count == 0 {
        return Err("count is zero");
    }
    if m.count as usize > MAX_COUNT {
        return Err("count is 0x8000 or greater");
    }
    if m.interval >= 0x8000_0000 {
        return Err("interval is at least 2^31");
    }
    if i64::from(m.add).abs() > MAX_ADD {
        return Err("add is outside the wire range");
    }
    if i64::from(m.add2).abs() > MAX_ADD2 {
        return Err("add2 is outside the wire range");
    }
    if !(MIN_SHIFT..=MAX_SHIFT).contains(&m.shift) {
        return Err("shift is outside the wire range");
    }
    if m.count > 1 && m.interval == 0 && m.add == 0 && m.add2 == 0 {
        return Err("zero interval and increments for multiple steps");
    }
    Ok(())
}

#[cfg(test)]
fn mcu_walk_offsets(m: &StepMoveHp) -> Result<Vec<u64>, &'static str> {
    validate_wire(m)?;
    let mut s = fill_stepper_moves(m)?;
    let mut time = 0_i64;
    let mut offsets = Vec::with_capacity(m.count as usize);
    for _ in 0..m.count {
        add_interval(&mut time, &mut s)?;
        if time < 0 {
            return Err("step time became negative");
        }
        offsets.push(time as u64);
        inc_interval(&mut s)?;
        if !(0..(1_i64 << 31)).contains(&s.interval) {
            return Err("interval overflow");
        }
    }
    Ok(offsets)
}

#[derive(Debug, Clone, Copy)]
struct WalkedMove {
    move_out: StepMoveHp,
    next_step_interval: u32,
}

#[derive(Debug, Clone, Copy)]
struct WalkFailure {
    step_index: usize,
    covered: usize,
    detail: &'static str,
}

struct Compressor<'a> {
    steps: &'a [u64],
    pos: usize,
    last_step_clock: u64,
    next_expected_interval: u32,
    rhs_cache: Box<[Rhs3; MAX_COUNT_LSM]>,
    errb_cache: Box<[Points; MAX_COUNT_LSM]>,
    cached_count: usize,
}

impl<'a> Compressor<'a> {
    fn new(steps: &'a [u64], last_step_clock: u64, next_expected_interval: u32) -> Self {
        Self {
            steps,
            pos: 0,
            last_step_clock,
            next_expected_interval,
            rhs_cache: Box::new([Rhs3::default(); MAX_COUNT_LSM]),
            errb_cache: Box::new([Points::default(); MAX_COUNT_LSM]),
            cached_count: 0,
        }
    }

    fn set_cursor(&mut self, pos: usize, last_step_clock: u64, next_expected_interval: u32) {
        self.pos = pos;
        self.last_step_clock = last_step_clock;
        self.next_expected_interval = next_expected_interval;
        self.cached_count = 0;
    }

    fn compute_rhs(&self, count: usize, previous: Option<Rhs3>) -> Rhs3 {
        let mut f = previous.unwrap_or(Rhs3 {
            b0: FIRST_STEP_BIAS * f64::from(self.next_expected_interval),
            b1: 0.0,
            b2: 0.0,
        });
        let d = (self.steps[self.pos + count - 1] - self.last_step_clock) as f64;
        let count_i = count as i64;
        f.b0 += d * count as f64;
        let c1 = count_i * (count_i - 1) / 2;
        f.b1 += d * c1 as f64;
        let c2 = c1 * (count_i - 2) / 3;
        f.b2 += d * c2 as f64;
        f
    }

    fn update_caches_to_count(&mut self, count: usize) {
        assert!(count <= MAX_COUNT_LSM);
        if self.cached_count == 0 {
            self.rhs_cache[0] = self.compute_rhs(1, None);
            self.errb_cache[0] = minmax_point(self.steps, self.pos, self.pos, self.last_step_clock);
            self.cached_count = 1;
        }
        for i in self.cached_count + 1..=count {
            let previous = self.rhs_cache[i - 2];
            self.rhs_cache[i - 1] = self.compute_rhs(i, Some(previous));
            self.errb_cache[i - 1] =
                minmax_point(self.steps, self.pos + i - 1, self.pos, self.last_step_clock);
        }
        self.cached_count = self.cached_count.max(count);
    }

    fn point_for(&self, index: usize) -> Points {
        if index < self.cached_count {
            self.errb_cache[index]
        } else {
            minmax_point(self.steps, self.pos + index, self.pos, self.last_step_clock)
        }
    }

    fn test_move(
        &self,
        mut move_out: StepMoveHp,
        trunc_move: bool,
    ) -> Result<WalkedMove, WalkFailure> {
        if let Err(detail) = validate_wire(&move_out) {
            return Err(WalkFailure {
                step_index: 0,
                covered: 0,
                detail,
            });
        }
        let mut s = fill_stepper_moves(&move_out).map_err(|detail| WalkFailure {
            step_index: 0,
            covered: 0,
            detail,
        })?;
        let mut cur_step = 0_i64;
        let mut prev_step = 0_i64;
        let mut trunc_pos = 0usize;
        let mut trunc_last_step = 0_i64;
        let mut trunc_err = i64::MAX;
        let mut next_step_interval = 0_u32;
        for i in 0..move_out.count as usize {
            add_interval(&mut cur_step, &mut s).map_err(|detail| WalkFailure {
                step_index: i,
                covered: i,
                detail,
            })?;
            let point = self.point_for(i);
            if cur_step < point.minp || cur_step > point.maxp {
                return Err(WalkFailure {
                    step_index: i,
                    covered: i,
                    detail: "step is outside its error window",
                });
            }
            if trunc_move
                && move_out.count > 3
                && move_out.count as usize - i <= (move_out.count as usize + 9) / 10
            {
                let requested = (self.steps[self.pos + i] - self.last_step_clock) as i64;
                let error = (cur_step - requested).abs();
                if error <= trunc_err || error <= 1 {
                    trunc_pos = i;
                    trunc_err = error;
                    let interval = cur_step - prev_step;
                    if !(0..=u32::MAX as i64).contains(&interval) {
                        return Err(WalkFailure {
                            step_index: i,
                            covered: i,
                            detail: "junction interval is outside u32 range",
                        });
                    }
                    next_step_interval = interval as u32;
                    trunc_last_step = prev_step;
                }
            }
            inc_interval(&mut s).map_err(|detail| WalkFailure {
                step_index: i,
                covered: i,
                detail,
            })?;
            if !(0..(1_i64 << 31)).contains(&s.interval) {
                return Err(WalkFailure {
                    step_index: i,
                    covered: i,
                    detail: "expanded interval overflow",
                });
            }
            if i == 0 {
                move_out.first_step = cur_step as u64;
            }
            move_out.last_step = cur_step as u64;
            prev_step = cur_step;
        }
        if trunc_move && trunc_pos != 0 {
            move_out.count = trunc_pos as u16;
            move_out.last_step = trunc_last_step as u64;
        }
        Ok(WalkedMove {
            move_out,
            next_step_interval,
        })
    }
    fn test_candidate(&self, mut candidate: StepMoveHp) -> StepMoveHp {
        match self.test_move(candidate, false) {
            Ok(walked) => walked.move_out,
            Err(failure) if failure.covered != 0 => {
                candidate.count = failure.covered as u16;
                candidate
            }
            Err(_) => StepMoveHp {
                interval: 0,
                count: 0,
                add: 0,
                add2: 0,
                shift: 0,
                first_step: 0,
                last_step: 0,
            },
        }
    }

    fn test_step_count(&self, count: usize) -> StepMoveHp {
        if count == 0 || count > MAX_COUNT_LSM || self.cached_count < count {
            return StepMoveHp {
                interval: 0,
                count: 0,
                add: 0,
                add2: 0,
                shift: 0,
                first_step: 0,
                last_step: 0,
            };
        }
        let mut rhs = self.rhs_cache[count - 1];
        solve_3x3(&least_squares_ldl()[count - 1], &mut rhs);
        let regular = step_move_encode(count, &rhs)
            .map(|candidate| self.test_candidate(candidate))
            .unwrap_or_default();
        if count > 20 && (regular.count as usize) < count / 4 {
            let mut extra_rhs = self.rhs_cache[count - 1];
            extra_rhs.b0 += EXTRA_FIRST_STEP_BIAS * f64::from(self.next_expected_interval);
            solve_3x3(&least_squares_efsb_ldl()[count - 1], &mut extra_rhs);
            let extra = step_move_encode(count, &extra_rhs)
                .map(|candidate| self.test_candidate(candidate))
                .unwrap_or_default();
            if extra.count > regular.count {
                return extra;
            }
        }
        regular
    }

    fn gen_avg_interval(&self, count: usize) -> StepMoveHp {
        let d = (self.steps[self.pos + count - 1] - self.last_step_clock) as f64
            + FIRST_STEP_BIAS * f64::from(self.next_expected_interval);
        let rhs = Rhs3 {
            b0: d / (count as f64 + FIRST_STEP_BIAS),
            b1: 0.0,
            b2: 0.0,
        };
        step_move_encode(count, &rhs).unwrap_or_default()
    }

    fn single_step_move(&self) -> Option<StepMoveHp> {
        let interval = self.steps[self.pos].checked_sub(self.last_step_clock)?;
        if interval >= 0x8000_0000 {
            return None;
        }
        Some(StepMoveHp {
            interval: interval as u32,
            count: 1,
            add: 0,
            add2: 0,
            shift: 0,
            first_step: interval,
            last_step: interval,
        })
    }

    fn compress_bisect_count(&mut self) -> Option<StepMoveHp> {
        let queue_size = (self.steps.len() - self.pos).min(MAX_COUNT);
        let mut best = StepMoveHp {
            interval: 0,
            count: 0,
            add: 0,
            add2: 0,
            shift: 0,
            first_step: 0,
            last_step: 0,
        };
        let mut left = 0usize;
        let mut right = 8usize;
        while right <= MAX_COUNT_LSM && right <= queue_size {
            self.update_caches_to_count(right);
            let current = self.test_step_count(right);
            if current.count > best.count {
                left = current.count as usize;
                best = current;
            } else {
                break;
            }
            right *= 2;
        }
        if right >= MAX_COUNT_BISECT {
            while right <= queue_size {
                let current = self.test_candidate(self.gen_avg_interval(right));
                if current.count > best.count {
                    best = current;
                } else {
                    break;
                }
                right *= 2;
            }
            return if best.count <= 1 {
                self.single_step_move()
            } else {
                Some(best)
            };
        }
        if right > queue_size {
            right = queue_size + 1;
        }
        self.update_caches_to_count(right - 1);
        while right - left > 1 {
            let count = (left + right) / 2;
            let current = self.test_step_count(count);
            if current.count as usize > best.count as usize {
                left = count;
                best = current;
            } else {
                right = count;
            }
        }
        if best.count <= 1 {
            self.single_step_move()
        } else {
            Some(best)
        }
    }
}

fn validate_monotonic(steps: &[u64], last_step_clock: u64) -> Result<(), CompressError> {
    let mut previous = last_step_clock;
    for (index, &clock) in steps.iter().enumerate() {
        if clock <= previous {
            return Err(points_error(format!(
                "stepcompress hp: step {index} clock {clock} not after previous clock {previous}"
            )));
        }
        previous = clock;
    }
    Ok(())
}

pub fn compress_hp(
    steps: &[u64],
    last_step_clock: u64,
    next_expected_interval: u32,
) -> Result<(Vec<StepMoveHp>, usize, u32), CompressError> {
    if steps.is_empty() {
        return Err(points_error("stepcompress hp: empty input"));
    }
    validate_monotonic(steps, last_step_clock)?;
    let mut compressor = Compressor::new(steps, last_step_clock, next_expected_interval);
    let mut moves = Vec::new();
    let mut pos = 0usize;
    let mut lsc = last_step_clock;
    let mut carry = next_expected_interval;
    while pos < steps.len() {
        if carry == 0 {
            let interval = steps[pos]
                .checked_sub(lsc)
                .ok_or_else(|| points_error("stepcompress hp: step clock moved backwards"))?;
            carry = u32::try_from(interval).map_err(|_| {
                points_error(format!(
                    "stepcompress hp move {} step 1: next interval {interval} is outside u32 range",
                    moves.len()
                ))
            })?;
        }
        compressor.set_cursor(pos, lsc, carry);
        let candidate = compressor.compress_bisect_count().ok_or_else(|| {
            points_error(format!(
                "stepcompress hp move {} step 1: first step cannot be represented",
                moves.len()
            ))
        })?;
        let walked = compressor.test_move(candidate, true).map_err(|failure| {
            points_error(format!(
                "stepcompress hp move {} step {}: {}",
                moves.len(),
                failure.step_index + 1,
                failure.detail
            ))
        })?;
        if walked.move_out.count == 0 {
            return Err(points_error(format!(
                "stepcompress hp move {} step 1: encoder covered zero steps",
                moves.len()
            )));
        }
        let covered = usize::from(walked.move_out.count);
        if pos + covered > steps.len() {
            return Err(points_error(format!(
                "stepcompress hp move {} step {}: covered count exceeds input",
                moves.len(),
                covered
            )));
        }
        lsc = lsc.checked_add(walked.move_out.last_step).ok_or_else(|| {
            points_error(format!(
                "stepcompress hp move {} step {}: last step clock overflow",
                moves.len(),
                covered
            ))
        })?;
        pos += covered;
        carry = walked.next_step_interval;
        moves.push(walked.move_out);
    }
    Ok((moves, pos, carry))
}

#[cfg(test)]
#[path = "compress_hp_tests.rs"]
mod tests;
