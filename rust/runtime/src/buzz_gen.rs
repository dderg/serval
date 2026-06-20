// Exact-crossing step generator for a tone or a linear-chirp excitation.
//
// The position of an excited motor relative to its parked base is
//
//     q(t) = sign * env(t) * A_eff(t) * sin(phi(t))
//
// with absolute seconds `t` measured from the excitation anchor. For a tone the
// chirp rate `mu` is zero, giving `phi(t) = omega * t` and `A_eff(t) = A`. For a
// linear chirp the instantaneous angular frequency slews linearly,
//
//     omega_inst(t) = omega + mu * t,   phi(t) = omega * t + 0.5 * mu * t^2,
//
// and the displacement amplitude tapers as `A_eff(t) = A * omega / omega_inst(t)`
// so the peak carrier velocity `A * omega` stays constant across the sweep (the
// constant-`accel_per_hz` regime mirrored from `buzz::sample`).
//
// A microstep edge fires whenever `round(q / m)` changes, i.e. whenever `q`
// crosses a half-microstep gridline at `(k + 0.5) * m` (going up) or
// `(k - 0.5) * m` (going down), where `k` is the current microstep level. This
// solver returns the exact time of the next such crossing analytically,
// decoupled from the motion ISR tick rate: `cycle_abs` depends only on the
// curve, the anchor, and `cycles_per_second`.
//
// Flat-top (env == 1) tone crossings are closed-form (asin plus the periodic
// branches). Whenever the amplitude or frequency is time-varying — both ramps,
// and the whole of a chirp — the crossing is found with a bracketed
// bisection/Newton hybrid over an adaptive forward scan, carrying a hard
// iteration cap and a strict-monotonic guard. Both fail loud rather than
// silently clamping. The bisection fallback also covers the chirp's carrier
// turning points, where `dq/dt -> 0` and a bare Newton step would stall.

use crate::error::FaultCode;

const TWO_PI: f32 = 2.0 * core::f32::consts::PI;

/// Hard cap on root-refinement iterations. A converging bisection on a bracket
/// shrinks by 2x per step, so ~30 iterations resolves any f32 bracket to ULP;
/// exceeding this means the bracket was malformed (no sign change / NaN) and we
/// fault rather than spin or accept garbage.
const MAX_REFINE_ITERS: u32 = 100;

/// Absolute convergence floor for the crossing/extremum brackets. f32 carries
/// ~7 significant digits, so over a multi-second buzz a crossing time near the
/// window end resolves to ~0.5 us; a ~1e-6 s floor (well above f32 ULP at those
/// magnitudes) converges without chasing noise, and a bracket that cannot reach
/// it within `MAX_REFINE_ITERS` is genuinely malformed and faults loud.
const REFINE_TIME_TOL: f32 = 1.0e-6;

/// Latched, immutable parameters for one tone. All distances in mm, all times
/// in absolute seconds from the anchor. `sign` folds the per-axis sign of the
/// excitation; `base` is the parked axis position (the curve oscillates about
/// it). `cycles_per_second` is the MCU cycle-counter rate used only to map a
/// crossing time to an absolute cycle — it never influences *which* crossings
/// exist.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ToneParams {
    pub omega: f32,
    /// Linear chirp rate (rad/s^2): `omega_inst(t) = omega + mu * t`. Zero for a
    /// fixed-frequency tone. `mu = 2*pi*(f_end - f_start)/total_seconds`.
    pub mu: f32,
    pub amplitude_mm: f32,
    pub sign: f32,
    pub base_mm: f32,
    pub microstep_distance: f32,
    pub anchor_cycle: u32,
    /// MCU cycle-counter rate, used only by `cycle_at` to map a crossing time to
    /// an absolute cycle. Kept in f64: `t * cps` reaches ~1.5e11 over a long buzz,
    /// past the f32 mantissa, so the one cycle conversion per crossing promotes.
    pub cycles_per_second: f64,
    pub total_seconds: f32,
    pub ramp_seconds: f32,
}

/// Resumable position in the crossing stream: the microstep level already
/// emitted (`level`) and the time of the last emitted crossing (`t_cursor`).
/// The solver returns the next crossing strictly after `t_cursor`.
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

/// One emitted microstep edge.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ToneCrossing {
    pub cycle_abs: u32,
    pub dir: i8,
    /// Exact crossing time (absolute seconds from anchor); carried so the caller
    /// can resume and so tests can compare against the brute-force oracle.
    pub t: f32,
    /// Microstep level after this edge.
    pub level: i32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ToneError {
    /// No further crossing before `total_seconds`: the stream is exhausted.
    Done,
    /// Root refinement failed to converge within the iteration cap, or a bracket
    /// was malformed. Fail loud — the caller raises a fault.
    RefineDiverged,
    /// Time did not advance strictly (next crossing <= cursor). A monotonicity
    /// violation; never clamp, always fault.
    NonMonotonic,
}

impl ToneError {
    pub fn fault_code(self) -> Option<FaultCode> {
        match self {
            ToneError::Done => None,
            ToneError::RefineDiverged | ToneError::NonMonotonic => {
                Some(FaultCode::InternalInvariant)
            }
        }
    }
}

/// Continuous-time trapezoidal envelope in [0, 1], parameterized in seconds.
/// Zero at `t == 0` and `t == total`, unity across the flat top. Linear ramps
/// of length `ramp` at each end. This is the continuous analogue of
/// `buzz::envelope`; it hits exactly 0 at `t == total` so the final edge lands
/// on base (net-zero excitation).
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

/// Carrier phase `phi(t) = omega*t + 0.5*mu*t^2`. Reduces to `omega*t` for a tone.
#[inline]
#[must_use]
fn phase(p: &ToneParams, t: f32) -> f32 {
    (p.omega + 0.5 * p.mu * t) * t
}

/// Instantaneous angular frequency `omega + mu*t`. Constant for a tone.
#[inline]
#[must_use]
fn omega_inst(p: &ToneParams, t: f32) -> f32 {
    p.omega + p.mu * t
}

/// Displacement amplitude after the constant-peak-velocity taper:
/// `A * omega / omega_inst(t)`. Equal to `A` for a tone (and clamped to `A` if
/// the instantaneous frequency would ever dip to/through zero, which the host's
/// positive-frequency validation forbids — guarded here against div-by-zero).
#[inline]
#[must_use]
fn amp_eff(p: &ToneParams, t: f32) -> f32 {
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

/// d/dt of `amp_eff`: `-A * omega * mu / omega_inst(t)^2`. Zero for a tone.
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
fn position_rel(p: &ToneParams, t: f32) -> f32 {
    p.sign * envelope(t, p.total_seconds, p.ramp_seconds) * amp_eff(p, t) * libm::sinf(phase(p, t))
}

/// d/dt of `position_rel`, used to pick the crossing direction at the root and
/// to drive the Newton step where the closed form does not apply. Folds the
/// envelope slope, the chirp amplitude taper, and the carrier term; the carrier
/// term `env * A_eff * omega_inst * cos == env * A * omega * cos` has constant
/// peak across a chirp by construction.
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
fn flat_top_end(p: &ToneParams) -> f32 {
    (p.total_seconds - p.ramp_seconds).max(p.ramp_seconds)
}

const U32_MODULUS: f64 = 4_294_967_296.0;

/// Map an absolute crossing time to an absolute MCU cycle. Depends only on the
/// anchor and `cycles_per_second`, never the motion sample rate.
///
/// The MCU cycle counter is a 32-bit wrapping counter; at H7-class rates it
/// wraps every ~8 s, well inside a long chirp's `total_seconds`. `as u32` on an
/// out-of-range `f64` SATURATES (it does not wrap), so the cycle offset is
/// reduced modulo 2^32 first, keeping the value in `[0, 2^32)` where the
/// subsequent `as u32` is exact. `t` is non-negative and finite (the solver only
/// emits crossings with `0 < t <= total_seconds`), so `libm::fmod` (the no_std
/// f64 modulo; `f64::rem_euclid` is std-only) equals the Euclidean remainder.
///
/// This is the one f64 op in the solver (PRECISION EXCEPTION): `t` (f32 seconds)
/// times `cps` (~5e8) reaches ~1.5e11 over a long buzz, past the f32 mantissa, so
/// the time->cycle product and its modulo promote to f64. It runs once per
/// CROSSING, not per scan iteration; the per-iteration scan math stays f32.
#[inline]
#[must_use]
fn cycle_at(p: &ToneParams, t: f32) -> u32 {
    let offset = libm::fmod(f64::from(t) * p.cycles_per_second, U32_MODULUS);
    #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
    let cycles = offset as u32;
    p.anchor_cycle.wrapping_add(cycles)
}

/// Solve `sin(omega * t) == target_norm` on the flat top for the smallest root
/// strictly greater than `after` whose crossing direction matches `want_rising`
/// (the displacement must be moving toward the requested adjacent level). The
/// direction filter — not a time margin — is what rejects the re-detection of
/// the gridline the cursor just stepped across: that line is being left in the
/// opposite sense, so its pinned root at `after` has the wrong crossing
/// direction and is skipped, while a genuine grazing re-crossing microseconds
/// later (which approaches from the correct side) is kept. Returns `None` when
/// the gridline is unreachable (`|target_norm| > 1`).
#[must_use]
fn flat_top_root_after(
    p: &ToneParams,
    target_norm: f32,
    after: f32,
    want_rising: bool,
) -> Option<f32> {
    // The asin / periodic-branch closed form assumes a constant carrier
    // frequency and constant amplitude. A chirp has neither; its crossings are
    // found by the adaptive scan instead.
    if p.mu != 0.0 {
        return None;
    }
    if !(-1.0..=1.0).contains(&target_norm) {
        return None;
    }
    // A gridline within f64-noise of the carrier apex (`|target_norm| -> 1`) is a
    // tangent: the two asin branches collapse to a single double root, so the
    // rising/falling closed form cannot separate the up- and down-crossings that
    // straddle the apex. Defer to the adaptive scan, which resolves grazing
    // touches via its carrier-extremum checkpoint. The f32 noise floor near the
    // apex is far wider than f64's, so the tangent guard widens to ~1e-4.
    if target_norm.abs() >= 1.0 - 1e-4 {
        return None;
    }
    let base = libm::asinf(target_norm.clamp(-1.0, 1.0));
    // sin(theta) == target has two solutions per period: the rising branch
    // theta == base + 2*pi*n (cos > 0) and the falling branch theta == pi - base
    // + 2*pi*n (cos < 0). `position_rel` rises when sign * cos(theta) > 0, so the
    // branch we want depends on `want_rising` xor (sign < 0).
    let rising_displacement_branch = want_rising == (p.sign > 0.0);
    let principal = if rising_displacement_branch {
        base
    } else {
        core::f32::consts::PI - base
    };
    // A strict `theta > omega*after` excludes the root pinned at `after`; the
    // direction filter above already removed the wrong-sense re-detection, so the
    // smallest later same-branch root is the next true crossing.
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

/// Refine a crossing of `position_rel(t) == g` inside `[lo, hi]`, which must
/// bracket exactly one sign change of `f(t) = position_rel(t) - g`. Bisection
/// with a Newton acceleration when the derivative is well-behaved. Fails loud on
/// a malformed bracket or non-convergence.
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
        // Newton candidate; accept only if it stays inside the bracket.
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
    // round(q/m); q is bounded by the validated amplitude so the cast cannot
    // wrap a realistic level count.
    #[allow(clippy::cast_possible_truncation)]
    let level = libm::roundf(position_rel(p, t) / p.microstep_distance) as i32;
    level
}

/// Closed-form next adjacent-gridline crossing when `after` lies strictly inside
/// the flat top (env == 1). Returns the sooner of the next crossing of
/// `(level+0.5)` and `(level-0.5)`, with the resulting level, provided it stays
/// inside the flat top. `None` if no such crossing precedes the flat-top end.
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

/// Refine the carrier extremum (velocity zero) inside `[lo, hi]`, which must
/// bracket exactly one sign change of `velocity_rel`. Bisection only — the
/// second derivative is not maintained and Newton on a velocity root near a
/// peak is ill-conditioned. Fails loud on a malformed bracket.
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

/// Bracket and emit the first level change in the half-open `(prev, sample]`,
/// where `level` is the level at `prev`. The level at `sample` differs by ±1
/// (the scan grid is fine enough that no two adjacent gridlines are skipped),
/// so the crossed gridline is determined and refined. `None` only on a root
/// pinned at or before `after` (a re-detection of the boundary just crossed).
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

/// Forward-scan the first microstep-level change strictly after `after`.
///
/// Tracks `round(q/m)` on an adaptive grid (fine relative to the fastest carrier
/// half-period and the remaining window). Two samples reading the same level can
/// still hide a carrier excursion whose apex grazes an adjacent gridline and
/// returns — a tangent peak with arbitrarily small overshoot that no grid
/// spacing rules out. So whenever `velocity_rel` changes sign across a grid step
/// (a carrier extremum lies inside), the extremum time is located and inserted
/// as an extra checkpoint: the apex is the only place such a hidden touch can
/// occur, and checking its level catches it. On the first level departure the
/// exact crossing of the specific half-gridline passed is refined. Returns
/// `(t, new_level)`, or `None` once the curve closes on base for the remainder.
#[must_use]
fn scan_next_change(p: &ToneParams, level: i32, after: f32) -> Option<(f32, i32)> {
    // The period grid uses the FASTEST instantaneous frequency over the window
    // so the sampling stays fine at a chirp's high-frequency end. The traverse
    // cap keeps the per-sample carrier motion small (peak carrier velocity is the
    // constant A * omega; the chirp amplitude taper holds A_eff * omega_inst ==
    // A * omega), which together with the extremum insertion below makes the scan
    // robust to grazing excursions.
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
    let mut t_prev = after;
    let mut level_prev = level;
    let mut v_prev = velocity_rel(p, after);
    let mut t = after + dt;
    loop {
        let sample = t.min(p.total_seconds);
        let v_sample = velocity_rel(p, sample);

        // A carrier extremum inside (t_prev, sample): check its level first, so a
        // grazing peak that crosses a gridline and returns is not skipped.
        if v_prev != 0.0 && v_sample != 0.0 && v_prev * v_sample < 0.0 {
            let ext = refine_extremum(p, t_prev, sample).ok()?;
            let ext_level = microstep_level(p, ext);
            if ext_level != level_prev {
                if let Some(found) = emit_level_change(p, level_prev, after, t_prev, ext, ext_level)
                {
                    return Some(found);
                }
                level_prev = if ext_level > level_prev {
                    level_prev + 1
                } else {
                    level_prev - 1
                };
                t_prev = ext;
                v_prev = velocity_rel(p, ext);
                continue;
            }
        }

        let cur_level = microstep_level(p, sample);
        if cur_level != level_prev {
            if let Some(found) = emit_level_change(p, level_prev, after, t_prev, sample, cur_level)
            {
                return Some(found);
            }
            // A root pinned to `after` means we re-detected the boundary just
            // crossed; step the tracked level one toward `cur_level` and keep
            // scanning from this sample.
            level_prev = if cur_level > level_prev {
                level_prev + 1
            } else {
                level_prev - 1
            };
            t_prev = sample;
            v_prev = v_sample;
            continue;
        }

        if sample >= p.total_seconds {
            return None;
        }
        t_prev = sample;
        v_prev = v_sample;
        t += dt;
    }
}

/// Advance the stream: return the next microstep edge strictly after the cursor.
///
/// The next edge is the first time `round(q/m)` changes from `cursor.level`.
/// Inside a tone's flat top the next adjacent-gridline crossing is closed-form;
/// on the ramps, across region boundaries, and everywhere in a chirp it is found
/// by an adaptive forward scan plus a bracketed refine. The edge direction
/// follows the level change.
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
    let (t, next_level) = match candidate.or_else(|| scan_next_change(p, cursor.level, after)) {
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
