use core::fmt;

pub const DEFAULT_MAX_ERROR_TICKS: u32 = 1600;

const QUADRATIC_DEV: i64 = 11;
/// The widest offset a queue_step stream can carry from the clock the mcu
/// stepper is anchored on. A step further out than this is unreachable from
/// that anchor: the caller must re-anchor the stepper before encoding it.
pub const CLOCK_DIFF_MAX: u64 = 3 << 28;
const MAX_MOVE_STEPS: usize = 65535;
const MAX_INTERVAL: i64 = 0x8000_0000;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StepMove {
    pub interval: u32,
    pub count: u16,
    pub add: i16,
}

impl StepMove {
    pub fn step_clock(&self, last_step_clock: u64, nth: u16) -> u64 {
        assert!(nth >= 1 && nth <= self.count);
        let n = i64::from(nth);
        let ticks = i64::from(self.interval) * n + i64::from(self.add) * (n * (n - 1) / 2);
        last_step_clock.wrapping_add(ticks as u64)
    }

    pub fn last_clock(&self, last_step_clock: u64) -> u64 {
        self.step_clock(last_step_clock, self.count)
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CompressError {
    pub detail: String,
}

impl fmt::Display for CompressError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.detail)
    }
}

impl std::error::Error for CompressError {}

fn err<T>(detail: String) -> Result<T, CompressError> {
    Err(CompressError { detail })
}

fn idiv_up(n: i64, d: i64) -> i64 {
    if n >= 0 { (n + d - 1) / d } else { n / d }
}

fn idiv_down(n: i64, d: i64) -> i64 {
    if n >= 0 { n / d } else { (n - d + 1) / d }
}

#[derive(Clone, Copy)]
struct Points {
    minp: i64,
    maxp: i64,
}

struct Window<'a> {
    steps: &'a [u64],
    pos: usize,
    last_step_clock: u64,
    max_error: i64,
}

impl Window<'_> {
    fn minmax_point(&self, index: usize) -> Points {
        let point = (self.steps[index] - self.last_step_clock) as i64;
        let prevpoint = if index > self.pos {
            (self.steps[index - 1] - self.last_step_clock) as i64
        } else {
            0
        };
        let mut max_error = (point - prevpoint) / 2;
        if max_error > self.max_error {
            max_error = self.max_error;
        }
        Points {
            minp: point - max_error,
            maxp: point,
        }
    }
}

struct Candidate {
    interval: i64,
    count: usize,
    add: i64,
}

fn compress_bisect_add(window: &Window<'_>, qlast: usize) -> Candidate {
    let pos = window.pos;
    let point = window.minmax_point(pos);
    let mut outer_mininterval = point.minp;
    let mut outer_maxinterval = point.maxp;
    let mut add: i64 = 0;
    let mut minadd: i64 = -0x8000;
    let mut maxadd: i64 = 0x7fff;
    let mut bestinterval: i64 = 0;
    let mut bestcount: usize = 1;
    let mut bestadd: i64 = 1;
    let mut bestreach: i64 = i64::from(i32::MIN);
    let mut zerointerval: i64 = 0;
    let mut zerocount: usize = 0;

    loop {
        let mut nextmininterval = outer_mininterval;
        let mut nextmaxinterval = outer_maxinterval;
        let mut interval = nextmaxinterval;
        let mut nextcount: usize = 1;
        let nextpoint;
        loop {
            nextcount += 1;
            if pos + nextcount > qlast {
                return Candidate {
                    interval,
                    count: nextcount - 1,
                    add,
                };
            }
            let candidate_point = window.minmax_point(pos + nextcount - 1);
            let n = nextcount as i64;
            let nextaddfactor = n * (n - 1) / 2;
            let c = add * nextaddfactor;
            if nextmininterval * n < candidate_point.minp - c {
                nextmininterval = idiv_up(candidate_point.minp - c, n);
            }
            if nextmaxinterval * n > candidate_point.maxp - c {
                nextmaxinterval = idiv_down(candidate_point.maxp - c, n);
            }
            if nextmininterval > nextmaxinterval {
                nextpoint = candidate_point;
                break;
            }
            interval = nextmaxinterval;
        }

        let count = nextcount - 1;
        let cn = count as i64;
        let addfactor = cn * (cn - 1) / 2;
        let reach = add * addfactor + interval * cn;
        if reach > bestreach || (reach == bestreach && interval > bestinterval) {
            bestinterval = interval;
            bestcount = count;
            bestadd = add;
            bestreach = reach;
            if add == 0 {
                zerointerval = interval;
                zerocount = count;
            }
            if count > 0x200 {
                break;
            }
        }

        let n = nextcount as i64;
        let nextaddfactor = n * (n - 1) / 2;
        let nextreach = add * nextaddfactor + interval * n;
        if nextreach < nextpoint.minp {
            minadd = add + 1;
            outer_maxinterval = nextmaxinterval;
        } else {
            maxadd = add - 1;
            outer_mininterval = nextmininterval;
        }

        if count > 1 {
            let errdelta = window.max_error * QUADRATIC_DEV / (cn * cn);
            if minadd < add - errdelta {
                minadd = add - errdelta;
            }
            if maxadd > add + errdelta {
                maxadd = add + errdelta;
            }
        }

        let c = outer_maxinterval * n;
        if minadd * nextaddfactor < nextpoint.minp - c {
            minadd = idiv_up(nextpoint.minp - c, nextaddfactor);
        }
        let c = outer_mininterval * n;
        if maxadd * nextaddfactor > nextpoint.maxp - c {
            maxadd = idiv_down(nextpoint.maxp - c, nextaddfactor);
        }

        if minadd > maxadd {
            break;
        }
        add = maxadd - (maxadd - minadd) / 4;
    }

    if zerocount + zerocount / 16 >= bestcount {
        return Candidate {
            interval: zerointerval,
            count: zerocount,
            add: 0,
        };
    }
    Candidate {
        interval: bestinterval,
        count: bestcount,
        add: bestadd,
    }
}

fn check_line(
    window: &Window<'_>,
    candidate: &Candidate,
    move_out: &StepMove,
) -> Result<(), CompressError> {
    let describe = || {
        format!(
            "i={} c={} a={}",
            move_out.interval, move_out.count, move_out.add
        )
    };
    if candidate.count == 0
        || (candidate.interval == 0 && candidate.add == 0 && candidate.count > 1)
        || candidate.interval >= MAX_INTERVAL
    {
        return err(format!("stepcompress {}: invalid sequence", describe()));
    }
    let mut interval = candidate.interval;
    let mut p: i64 = 0;
    for i in 0..candidate.count {
        let point = window.minmax_point(window.pos + i);
        p += interval;
        if p < point.minp || p > point.maxp {
            return err(format!(
                "stepcompress {}: point {}: {} not in {}:{}",
                describe(),
                i + 1,
                p,
                point.minp,
                point.maxp
            ));
        }
        if interval >= MAX_INTERVAL {
            return err(format!(
                "stepcompress {}: point {}: interval overflow {}",
                describe(),
                i + 1,
                interval
            ));
        }
        interval += candidate.add;
    }
    Ok(())
}

fn validate_monotonic(steps: &[u64], last_step_clock: u64) -> Result<(), CompressError> {
    let mut prev = last_step_clock;
    for (i, &clock) in steps.iter().enumerate() {
        if clock <= prev {
            return err(format!(
                "stepcompress: step {i} clock {clock} not after previous clock {prev}"
            ));
        }
        prev = clock;
    }
    Ok(())
}

pub fn compress(
    steps: &[u64],
    last_step_clock: u64,
) -> Result<(Vec<StepMove>, usize), CompressError> {
    compress_with_max_error(steps, last_step_clock, DEFAULT_MAX_ERROR_TICKS)
}

pub fn compress_with_max_error(
    steps: &[u64],
    last_step_clock: u64,
    max_error: u32,
) -> Result<(Vec<StepMove>, usize), CompressError> {
    validate_monotonic(steps, last_step_clock)?;
    let mut moves = Vec::new();
    let mut pos = 0usize;
    let mut lsc = last_step_clock;
    let mut window_end = 0usize;
    while pos < steps.len() {
        let cap = (pos + MAX_MOVE_STEPS).min(steps.len());
        if window_end < pos {
            window_end = pos;
        }
        while window_end < cap && steps[window_end] - lsc < CLOCK_DIFF_MAX {
            window_end += 1;
        }
        if window_end == pos {
            if pos == 0 {
                return err(format!(
                    "stepcompress: first step clock {} is {} ticks after {}, not representable",
                    steps[0],
                    steps[0] - lsc,
                    lsc
                ));
            }
            break;
        }
        let window = Window {
            steps,
            pos,
            last_step_clock: lsc,
            max_error: i64::from(max_error),
        };
        let candidate = compress_bisect_add(&window, window_end);
        if candidate.count > MAX_MOVE_STEPS
            || candidate.interval < 0
            || candidate.interval >= MAX_INTERVAL
            || candidate.add < i64::from(i16::MIN)
            || candidate.add > i64::from(i16::MAX)
        {
            return err(format!(
                "stepcompress: unrepresentable move interval={} count={} add={}",
                candidate.interval, candidate.count, candidate.add
            ));
        }
        let emitted = StepMove {
            interval: candidate.interval as u32,
            count: candidate.count as u16,
            add: candidate.add as i16,
        };
        check_line(&window, &candidate, &emitted)?;
        lsc = emitted.last_clock(lsc);
        pos += candidate.count;
        moves.push(emitted);
    }
    Ok((moves, pos))
}

#[cfg(test)]
#[path = "compress_tests.rs"]
mod tests;
