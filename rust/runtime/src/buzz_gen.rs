use crate::error::FaultCode;

const TWO_PI: f32 = 2.0 * core::f32::consts::PI;

const MAX_REFINE_ITERS: u32 = 100;
const REFINE_TIME_TOL: f32 = 1.0e-6;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ToneParams {
    pub omega: f32,
    pub mu: f32,
    pub amplitude_mm: f32,
    pub sign: f32,
    pub base_mm: f32,
    pub microstep_distance: f32,
    pub anchor_cycle: u32,
    pub cycles_per_second: f64,
    pub total_seconds: f32,
    pub ramp_seconds: f32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ToneCursor {
    pub level: i32,
    pub t_cursor: f32,
}

impl ToneCursor {
    #[must_use]
    pub const fn start() -> Self {
        Self {
            level: 0,
            t_cursor: 0.0,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ToneCrossing {
    pub cycle_abs: u32,
    pub dir: i8,
    pub t: f32,
    pub level: i32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ToneError {
    Done,
    RefineDiverged,
    NonMonotonic,
    /// A forward scan exceeded its iteration budget without resolving the next
    /// crossing — an f32 boundary coincidence pinned it. Fault rather than loop so
    /// the refill latches a clean fault instead of starving the IWDG feeder.
    ScanStalled,
}

impl ToneError {
    pub fn fault_code(self) -> Option<FaultCode> {
        match self {
            ToneError::Done => None,
            ToneError::RefineDiverged | ToneError::NonMonotonic | ToneError::ScanStalled => {
                Some(FaultCode::InternalInvariant)
            }
        }
    }
}

#[must_use]
pub fn envelope(t: f32, total: f32, ramp: f32) -> f32 {
    if total <= 0.0 || t <= 0.0 || t >= total {
        return 0.0;
    }
    let ramp = ramp.max(f32::MIN_POSITIVE);
    let up = (t / ramp).min(1.0);
    let down = ((total - t) / ramp).min(1.0);
    up.min(down).max(0.0)
}

#[inline]
#[must_use]
fn phase(p: &ToneParams, t: f32) -> f32 {
    (p.omega + 0.5 * p.mu * t) * t
}

#[inline]
#[must_use]
pub(crate) fn omega_inst(p: &ToneParams, t: f32) -> f32 {
    p.omega + p.mu * t
}

#[inline]
#[must_use]
pub(crate) fn amp_eff(p: &ToneParams, t: f32) -> f32 {
    if p.mu == 0.0 {
        return p.amplitude_mm;
    }
    let w = omega_inst(p, t);
    if w.abs() <= f32::MIN_POSITIVE {
        p.amplitude_mm
    } else {
        p.amplitude_mm * p.omega / w
    }
}

#[inline]
#[must_use]
fn amp_eff_rate(p: &ToneParams, t: f32) -> f32 {
    if p.mu == 0.0 {
        return 0.0;
    }
    let w = omega_inst(p, t);
    if w.abs() <= f32::MIN_POSITIVE {
        0.0
    } else {
        -p.amplitude_mm * p.omega * p.mu / (w * w)
    }
}

#[inline]
#[must_use]
fn amp_eff_accel(p: &ToneParams, t: f32) -> f32 {
    if p.mu == 0.0 {
        return 0.0;
    }
    let w = omega_inst(p, t);
    if w.abs() <= f32::MIN_POSITIVE {
        0.0
    } else {
        2.0 * p.amplitude_mm * p.omega * p.mu * p.mu / (w * w * w)
    }
}

#[inline]
#[must_use]
pub(crate) fn position_rel(p: &ToneParams, t: f32) -> f32 {
    p.sign * envelope(t, p.total_seconds, p.ramp_seconds) * amp_eff(p, t) * libm::sinf(phase(p, t))
}

#[inline]
#[must_use]
fn velocity_rel(p: &ToneParams, t: f32) -> f32 {
    let total = p.total_seconds;
    let ramp = p.ramp_seconds.max(f32::MIN_POSITIVE);
    let env = envelope(t, total, ramp);
    let denv = if t <= 0.0 || t >= total {
        0.0
    } else if t < ramp {
        1.0 / ramp
    } else if t > total - ramp {
        -1.0 / ramp
    } else {
        0.0
    };
    let a = amp_eff(p, t);
    let da = amp_eff_rate(p, t);
    let phi = phase(p, t);
    let s = libm::sinf(phi);
    let c = libm::cosf(phi);
    p.sign * (denv * a * s + env * da * s + env * a * omega_inst(p, t) * c)
}

#[inline]
#[must_use]
fn accel_rel(p: &ToneParams, t: f32) -> f32 {
    let total = p.total_seconds;
    let ramp = p.ramp_seconds.max(f32::MIN_POSITIVE);
    let env = envelope(t, total, ramp);
    let denv = if t <= 0.0 || t >= total {
        0.0
    } else if t < ramp {
        1.0 / ramp
    } else if t > total - ramp {
        -1.0 / ramp
    } else {
        0.0
    };
    let a = amp_eff(p, t);
    let da = amp_eff_rate(p, t);
    let dda = amp_eff_accel(p, t);
    let w = omega_inst(p, t);
    let phi = phase(p, t);
    let s = libm::sinf(phi);
    let c = libm::cosf(phi);
    let s_coeff = 2.0 * denv * da + env * dda - env * a * w * w;
    let c_coeff = 2.0 * denv * a * w + 2.0 * env * da * w + env * a * p.mu;
    p.sign * (s_coeff * s + c_coeff * c)
}

#[must_use]
pub fn sample_rel(p: &ToneParams, t: f32) -> (f32, f32, f32) {
    (position_rel(p, t), velocity_rel(p, t), accel_rel(p, t))
}

#[inline]
#[must_use]
fn flat_top_end(p: &ToneParams) -> f32 {
    (p.total_seconds - p.ramp_seconds).max(p.ramp_seconds)
}

const U32_MODULUS: f64 = 4_294_967_296.0;

#[inline]
#[must_use]
pub(crate) fn cycle_at(p: &ToneParams, t: f32) -> u32 {
    let offset = libm::fmod(f64::from(t) * p.cycles_per_second, U32_MODULUS);
    #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
    let cycles = offset as u32;
    p.anchor_cycle.wrapping_add(cycles)
}

const APEX_TANGENT_GUARD: f32 = 1e-4;

#[must_use]
fn flat_top_root_after(
    p: &ToneParams,
    target_norm: f32,
    after: f32,
    want_rising: bool,
) -> Option<f32> {
    if p.mu != 0.0 {
        return None;
    }
    if !(-1.0..=1.0).contains(&target_norm) {
        return None;
    }
    if target_norm.abs() >= 1.0 - APEX_TANGENT_GUARD {
        return None;
    }
    let base = libm::asinf(target_norm.clamp(-1.0, 1.0));
    let rising_displacement_branch = want_rising == (p.sign > 0.0);
    let principal = if rising_displacement_branch {
        base
    } else {
        core::f32::consts::PI - base
    };
    let phase_after = p.omega * after;
    let n = libm::ceilf((phase_after - principal) / TWO_PI);
    for k in [n, n + 1.0] {
        let theta = principal + TWO_PI * k;
        if theta > phase_after {
            let t = theta / p.omega;
            if t > after {
                return Some(t);
            }
        }
    }
    None
}

fn refine_crossing(p: &ToneParams, g: f32, lo: f32, hi: f32) -> Result<f32, ToneError> {
    let f = |t: f32| position_rel(p, t) - g;
    let (mut a, mut b) = (lo, hi);
    let (mut fa, mut fb) = (f(a), f(b));
    if !fa.is_finite() || !fb.is_finite() || fa * fb > 0.0 {
        return Err(ToneError::RefineDiverged);
    }
    let tol = (hi - lo).abs() * 1e-5 + REFINE_TIME_TOL;
    let mut t = 0.5 * (a + b);
    for _ in 0..MAX_REFINE_ITERS {
        let ft = f(t);
        if ft == 0.0 || (b - a).abs() <= tol {
            return Ok(t);
        }
        if fa * ft < 0.0 {
            b = t;
            fb = ft;
        } else {
            a = t;
            fa = ft;
        }
        let _ = fb;
        let dv = velocity_rel(p, t);
        let newton = if dv.abs() > f32::MIN_POSITIVE {
            t - ft / dv
        } else {
            f32::NAN
        };
        t = if newton.is_finite() && newton > a && newton < b {
            newton
        } else {
            0.5 * (a + b)
        };
    }
    Err(ToneError::RefineDiverged)
}

#[inline]
#[must_use]
fn microstep_level(p: &ToneParams, t: f32) -> i32 {
    #[allow(clippy::cast_possible_truncation)]
    let level = libm::roundf(position_rel(p, t) / p.microstep_distance) as i32;
    level
}

#[must_use]
fn flat_top_next_change(p: &ToneParams, level: i32, after: f32) -> Option<(f32, i32)> {
    let m = p.microstep_distance;
    let amp = p.sign * p.amplitude_mm;
    let flat_end = flat_top_end(p);
    #[allow(clippy::cast_precision_loss)]
    let g_up = (level as f32 + 0.5) * m;
    #[allow(clippy::cast_precision_loss)]
    let g_down = (level as f32 - 0.5) * m;
    let t_up = flat_top_root_after(p, g_up / amp, after, true).filter(|t| *t < flat_end);
    let t_down = flat_top_root_after(p, g_down / amp, after, false).filter(|t| *t < flat_end);
    match (t_up, t_down) {
        (None, None) => None,
        (Some(tu), None) => Some((tu, level + 1)),
        (None, Some(td)) => Some((td, level - 1)),
        (Some(tu), Some(td)) => {
            if tu <= td {
                Some((tu, level + 1))
            } else {
                Some((td, level - 1))
            }
        }
    }
}

fn refine_extremum(p: &ToneParams, lo: f32, hi: f32) -> Result<f32, ToneError> {
    let (mut a, mut b) = (lo, hi);
    let (mut va, vb) = (velocity_rel(p, a), velocity_rel(p, b));
    if !va.is_finite() || !vb.is_finite() || va * vb > 0.0 {
        return Err(ToneError::RefineDiverged);
    }
    let tol = (hi - lo).abs() * 1e-5 + REFINE_TIME_TOL;
    for _ in 0..MAX_REFINE_ITERS {
        let mid = 0.5 * (a + b);
        if (b - a).abs() <= tol {
            return Ok(mid);
        }
        let vm = velocity_rel(p, mid);
        if vm == 0.0 {
            return Ok(mid);
        }
        if va * vm < 0.0 {
            b = mid;
        } else {
            a = mid;
            va = vm;
        }
    }
    Err(ToneError::RefineDiverged)
}

#[must_use]
fn emit_level_change(
    p: &ToneParams,
    level: i32,
    after: f32,
    prev: f32,
    sample: f32,
    sample_level: i32,
) -> Option<(f32, i32)> {
    let new_level = if sample_level > level {
        level + 1
    } else {
        level - 1
    };
    #[allow(clippy::cast_precision_loss)]
    let step_dir = (new_level - level) as f32;
    #[allow(clippy::cast_precision_loss)]
    let g = (level as f32 + 0.5 * step_dir) * p.microstep_distance;
    let root = refine_crossing(p, g, prev, sample).ok()?;
    if root > after {
        Some((root, new_level))
    } else {
        None
    }
}

fn scan_next_change(
    p: &ToneParams,
    level: i32,
    after: f32,
) -> Result<Option<(f32, i32)>, ToneError> {
    let omega_hi = omega_inst(p, 0.0)
        .abs()
        .max(omega_inst(p, p.total_seconds).abs())
        .max(f32::MIN_POSITIVE);
    let period = TWO_PI / omega_hi;
    let remaining = (p.total_seconds - after).max(0.0);
    let v_peak = p.amplitude_mm * p.omega;
    let traverse_cap = if v_peak > 0.0 {
        0.4 * p.microstep_distance / v_peak
    } else {
        f32::INFINITY
    };
    let dt = (period / 32.0)
        .min(remaining / 32.0)
        .min(p.total_seconds / 4.0)
        .min(traverse_cap)
        .max(REFINE_TIME_TOL);

    // Forward progress is structural (`t_prev` advances by `dt` or to an interior
    // extremum every iteration); this budget is defense in depth so an f32 boundary
    // coincidence faults loud rather than starving the IWDG feeder.
    let grid_steps = (remaining / dt) + 1.0;
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let scan_budget = (grid_steps as u64).saturating_mul(64).saturating_add(1_000);

    let mut t_prev = after;
    let mut level_prev = level;
    let mut v_prev = velocity_rel(p, after);
    let mut t = after + dt;
    let mut iters: u64 = 0;
    loop {
        iters += 1;
        if iters > scan_budget {
            return Err(ToneError::ScanStalled);
        }
        let sample = t.min(p.total_seconds);
        let v_sample = velocity_rel(p, sample);

        if v_prev != 0.0 && v_sample != 0.0 && v_prev * v_sample < 0.0 {
            // The split point must lie STRICTLY inside (t_prev, sample) to make
            // progress; an f32 refine pinned to a bracket end falls through to the
            // grid step instead of re-splitting in place.
            let Ok(ext) = refine_extremum(p, t_prev, sample) else {
                return Ok(None);
            };
            if ext > t_prev && ext < sample {
                let ext_level = microstep_level(p, ext);
                if ext_level != level_prev {
                    if let Some(found) =
                        emit_level_change(p, level_prev, after, t_prev, ext, ext_level)
                    {
                        return Ok(Some(found));
                    }
                    level_prev += (ext_level - level_prev).signum();
                }
                t_prev = ext;
                v_prev = velocity_rel(p, ext);
                continue;
            }
        }

        let cur_level = microstep_level(p, sample);
        if cur_level != level_prev {
            if let Some(found) = emit_level_change(p, level_prev, after, t_prev, sample, cur_level)
            {
                return Ok(Some(found));
            }
            level_prev += (cur_level - level_prev).signum();
            t_prev = sample;
            v_prev = v_sample;
            continue;
        }

        if sample >= p.total_seconds {
            return Ok(None);
        }
        t_prev = sample;
        v_prev = v_sample;
        t += dt;
    }
}

pub fn next_crossing(p: &ToneParams, cursor: ToneCursor) -> Result<ToneCrossing, ToneError> {
    let after = cursor.t_cursor;
    let flat_start = p.ramp_seconds;
    let flat_end = flat_top_end(p);

    let inside_flat_top = p.mu == 0.0 && after >= flat_start && after < flat_end;
    let candidate = if inside_flat_top {
        flat_top_next_change(p, cursor.level, after)
    } else {
        None
    };
    let resolved = match candidate {
        Some(v) => Some(v),
        None => scan_next_change(p, cursor.level, after)?,
    };
    let (t, next_level) = match resolved {
        Some(v) => v,
        None => return Err(ToneError::Done),
    };

    if t > p.total_seconds {
        return Err(ToneError::Done);
    }
    if t <= after {
        return Err(ToneError::NonMonotonic);
    }

    let dir: i8 = if next_level > cursor.level { 1 } else { -1 };
    let v = velocity_rel(p, t);
    if v != 0.0 {
        let v_sign: i8 = if v > 0.0 { 1 } else { -1 };
        if v_sign != dir {
            return Err(ToneError::NonMonotonic);
        }
    }

    Ok(ToneCrossing {
        cycle_abs: cycle_at(p, t),
        dir,
        t,
        level: next_level,
    })
}

#[cfg(test)]
mod buzz_gen_tests;
