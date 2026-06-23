use crate::buzz_gen::{ToneError, ToneParams, amp_eff, cycle_at, omega_inst, position_rel};
use core::f32::consts::PI;

const TWO_PI: f32 = 2.0 * PI;

pub const DEFAULT_XDIRECT_UPDATE_HZ: f32 = 10_000.0;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct XdirectConfig {
    pub lut_step_mm: f32,
    pub target_rate_hz: f32,
}

impl XdirectConfig {
    #[must_use]
    pub fn new(lut_step_mm: f32, target_rate_hz: f32) -> Self {
        Self {
            lut_step_mm,
            target_rate_hz,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct XdirectCursor {
    pub t: f32,
    seg_phi0: f32,
    seg_end: f32,
    seg_n: i32,
    k: i32,
    pub last_offset: i32,
}

impl XdirectCursor {
    #[must_use]
    pub fn start(p: &ToneParams, cfg: &XdirectConfig) -> Self {
        Self {
            t: 0.0,
            seg_phi0: 0.0,
            seg_end: 0.5 * PI,
            seg_n: choose_n(p, cfg, 0.0),
            k: 0,
            last_offset: 0,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct XdirectUpdate {
    pub cycle_abs: u32,
    pub offset_steps: i32,
    pub t: f32,
}

#[inline]
#[must_use]
fn time_at_phase(p: &ToneParams, phi: f32) -> f32 {
    let disc = (p.omega * p.omega + 2.0 * p.mu * phi).max(0.0);
    let root = libm::sqrtf(disc);
    2.0 * phi / (p.omega + root).max(f32::MIN_POSITIVE)
}

#[inline]
#[must_use]
#[allow(clippy::cast_possible_truncation)]
fn choose_n(p: &ToneParams, cfg: &XdirectConfig, t: f32) -> i32 {
    let f_inst = (omega_inst(p, t).abs() / TWO_PI).max(f32::MIN_POSITIVE);
    let n_rate = libm::roundf(cfg.target_rate_hz.max(1.0) / f_inst) as i32;
    let levels =
        (libm::roundf(amp_eff(p, t).abs() / cfg.lut_step_mm.max(f32::MIN_POSITIVE)) as i32).max(1);
    let n = n_rate.min(4 * levels).max(4);
    n + (4 - n % 4) % 4
}

#[inline]
#[must_use]
#[allow(clippy::cast_possible_truncation)]
fn offset_at(p: &ToneParams, cfg: &XdirectConfig, t: f32) -> i32 {
    libm::roundf(position_rel(p, t) / cfg.lut_step_mm) as i32
}

#[inline]
#[must_use]
#[allow(clippy::cast_possible_truncation)]
fn seg_steps(seg_phi0: f32, seg_end: f32, seg_n: i32) -> i32 {
    libm::roundf((seg_end - seg_phi0) * seg_n as f32 / TWO_PI) as i32
}

pub fn next_update(
    p: &ToneParams,
    cfg: &XdirectConfig,
    cursor: XdirectCursor,
) -> Result<(XdirectUpdate, XdirectCursor), ToneError> {
    let total = p.total_seconds;
    if cursor.t >= total {
        return Err(ToneError::Done);
    }

    let steps = seg_steps(cursor.seg_phi0, cursor.seg_end, cursor.seg_n);
    let k1 = cursor.k + 1;
    let at_turn = k1 >= steps;
    #[allow(clippy::cast_precision_loss)]
    let phi = if at_turn {
        cursor.seg_end
    } else {
        cursor.seg_phi0 + k1 as f32 * (TWO_PI / cursor.seg_n as f32)
    };
    let t = time_at_phase(p, phi);

    if t >= total {
        let next = XdirectCursor {
            t: total,
            last_offset: 0,
            k: k1,
            ..cursor
        };
        return Ok((
            XdirectUpdate {
                cycle_abs: cycle_at(p, total),
                offset_steps: 0,
                t: total,
            },
            next,
        ));
    }
    if t <= cursor.t {
        return Err(ToneError::NonMonotonic);
    }

    let offset = offset_at(p, cfg, t);
    let next = if at_turn {
        XdirectCursor {
            t,
            seg_phi0: phi,
            seg_end: phi + PI,
            seg_n: choose_n(p, cfg, t),
            k: 0,
            last_offset: offset,
        }
    } else {
        XdirectCursor {
            t,
            k: k1,
            last_offset: offset,
            ..cursor
        }
    };
    Ok((
        XdirectUpdate {
            cycle_abs: cycle_at(p, t),
            offset_steps: offset,
            t,
        },
        next,
    ))
}

#[cfg(test)]
mod tests;
