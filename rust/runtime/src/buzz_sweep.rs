use crate::buzz_gen::{
    ToneCrossing, ToneCursor, ToneError, ToneParams, cycle_at, envelope, next_crossing,
};

const TWO_PI: f32 = 2.0 * core::f32::consts::PI;

#[inline]
#[must_use]
fn start_hz(p: &ToneParams) -> f32 {
    p.omega / TWO_PI
}

#[inline]
#[must_use]
fn hz_per_sec(p: &ToneParams) -> f32 {
    p.mu / TWO_PI
}

#[inline]
#[must_use]
fn end_hz(p: &ToneParams) -> f32 {
    (p.omega + p.mu * p.total_seconds) / TWO_PI
}

#[inline]
#[must_use]
fn step_freq(p: &ToneParams, f: f32) -> f32 {
    let half_period = 0.5 / f;
    let next = f + hz_per_sec(p) * half_period;
    let end = end_hz(p);
    if hz_per_sec(p) >= 0.0 {
        next.min(end)
    } else {
        next.max(end)
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SweepCursor {
    seg_f: f32,
    seg_t0: f32,
    flip: bool,
    inner: ToneCursor,
    done: bool,
}

impl SweepCursor {
    #[must_use]
    pub fn start(p: &ToneParams) -> Self {
        Self {
            seg_f: start_hz(p),
            seg_t0: 0.0,
            flip: false,
            inner: ToneCursor::start(),
            done: false,
        }
    }
}

#[must_use]
fn lobe_params(p: &ToneParams, cursor: &SweepCursor) -> ToneParams {
    let f = cursor.seg_f;
    let omega = TWO_PI * f;
    let half_period = 0.5 / f;
    let env = envelope(
        cursor.seg_t0 + 0.5 * half_period,
        p.total_seconds,
        p.ramp_seconds,
    );
    let amplitude_mm = p.amplitude_mm * (start_hz(p) / f) * env;
    let sign = if cursor.flip { -p.sign } else { p.sign };
    ToneParams {
        omega,
        mu: 0.0,
        amplitude_mm,
        sign,
        base_mm: p.base_mm,
        microstep_distance: p.microstep_distance,
        anchor_cycle: cycle_at(p, cursor.seg_t0),
        cycles_per_second: p.cycles_per_second,
        total_seconds: half_period,
        ramp_seconds: f32::MIN_POSITIVE,
    }
}

#[must_use]
fn next_lobe(p: &ToneParams, cursor: SweepCursor) -> SweepCursor {
    let half_period = 0.5 / cursor.seg_f;
    let seg_t0 = cursor.seg_t0 + half_period;
    if seg_t0 >= p.total_seconds {
        return SweepCursor {
            done: true,
            ..cursor
        };
    }
    SweepCursor {
        seg_f: step_freq(p, cursor.seg_f),
        seg_t0,
        flip: !cursor.flip,
        inner: ToneCursor::start(),
        done: false,
    }
}

pub fn next_crossing_sweep(
    p: &ToneParams,
    mut cursor: SweepCursor,
) -> Result<(ToneCrossing, SweepCursor), ToneError> {
    if cursor.done {
        return Err(ToneError::Done);
    }
    loop {
        let tp = lobe_params(p, &cursor);
        if tp.amplitude_mm < 0.5 * p.microstep_distance {
            let advanced = next_lobe(p, cursor);
            if advanced.done {
                return Err(ToneError::Done);
            }
            cursor = advanced;
            continue;
        }
        match next_crossing(&tp, cursor.inner) {
            Ok(c) => {
                let next = SweepCursor {
                    inner: ToneCursor {
                        level: c.level,
                        t_cursor: c.t,
                    },
                    ..cursor
                };
                let crossing = ToneCrossing {
                    cycle_abs: c.cycle_abs,
                    dir: c.dir,
                    t: cursor.seg_t0 + c.t,
                    level: c.level,
                };
                return Ok((crossing, next));
            }
            Err(ToneError::Done) => {
                let advanced = next_lobe(p, cursor);
                if advanced.done {
                    return Err(ToneError::Done);
                }
                cursor = advanced;
            }
            Err(e) => return Err(e),
        }
    }
}

#[cfg(test)]
mod tests;
