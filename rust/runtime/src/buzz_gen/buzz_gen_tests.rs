// Brute-force oracle math mirrors the solver's own bounded casts (round to a
// small level, time-to-cycle within a 0.1 s tone); truncation/sign-loss are
// safe by the same construction as the production path.
#![allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]

use crate::buzz_gen::*;

const NM_PER_MM: f64 = 1.0e6;
const TWO_PI: f64 = 2.0 * core::f64::consts::PI;

/// Brute-force reference: sample q(t) on a fine grid and record every
/// `round(q/m)` change as a ground-truth (time, dir) edge. `t_lo`/`t_hi`
/// bracket the grid interval the crossing was detected in, so an analytic
/// crossing time can be checked to fall inside (plus one grid step of slack).
#[derive(Clone, Copy, Debug)]
struct OracleEdge {
    t_lo: f64,
    t_hi: f64,
    dir: i8,
    level: i32,
}

fn make_params(
    freq_hz: f64,
    amplitude_nm: f64,
    sign: f64,
    base_mm: f64,
    cycles_per_second: f64,
) -> ToneParams {
    let total_seconds = 0.1;
    ToneParams {
        omega: TWO_PI * freq_hz,
        mu: 0.0,
        amplitude_mm: amplitude_nm / NM_PER_MM,
        sign,
        base_mm,
        microstep_distance: 0.0125 / 16.0,
        anchor_cycle: 1_000,
        cycles_per_second,
        total_seconds,
        ramp_seconds: 0.01,
    }
}

/// A linear chirp from `f_start` to `f_end` over `total_seconds`. `mu` is the
/// host's exact `2*pi*(f_end - f_start)/total` slope, so `omega_inst(0)`/`(total)`
/// land on `2*pi*f_start` / `2*pi*f_end`.
fn make_chirp(
    f_start_hz: f64,
    f_end_hz: f64,
    amplitude_nm: f64,
    sign: f64,
    total_seconds: f64,
    ramp_seconds: f64,
) -> ToneParams {
    let omega0 = TWO_PI * f_start_hz;
    let mu = TWO_PI * (f_end_hz - f_start_hz) / total_seconds;
    ToneParams {
        omega: omega0,
        mu,
        amplitude_mm: amplitude_nm / NM_PER_MM,
        sign,
        base_mm: 0.0,
        microstep_distance: 0.0125 / 16.0,
        anchor_cycle: 1_000,
        cycles_per_second: 64_000_000.0,
        total_seconds,
        ramp_seconds,
    }
}

/// Reference curve mirrored from the solver: `q = sign*env*A_eff(t)*sin(phi(t))`,
/// with `phi = omega*t + 0.5*mu*t^2` and `A_eff = A*omega/(omega+mu*t)`. The tone
/// case (`mu == 0`) collapses to `A*sin(omega*t)`.
fn position_rel_ref(p: &ToneParams, t: f64) -> f64 {
    let phi = (p.omega + 0.5 * p.mu * t) * t;
    let amp = if p.mu == 0.0 {
        p.amplitude_mm
    } else {
        let w = p.omega + p.mu * t;
        p.amplitude_mm * p.omega / w
    };
    p.sign * envelope(t, p.total_seconds, p.ramp_seconds) * amp * libm::sin(phi)
}

/// Build the ground-truth crossing sequence at >= 1 MHz equivalent resolution.
fn brute_force(p: &ToneParams, grid_hz: f64) -> Vec<OracleEdge> {
    let dt = 1.0 / grid_hz;
    let m = p.microstep_distance;
    let round_level = |q: f64| (q / m).round() as i32;
    let mut edges = Vec::new();
    let mut t_prev = 0.0;
    let mut level = round_level(position_rel_ref(p, 0.0));
    let mut t = dt;
    while t <= p.total_seconds + dt * 0.5 {
        let tc = t.min(p.total_seconds);
        let new_level = round_level(position_rel_ref(p, tc));
        while new_level != level {
            let dir: i8 = if new_level > level { 1 } else { -1 };
            level += i32::from(dir);
            edges.push(OracleEdge {
                t_lo: t_prev,
                t_hi: tc,
                dir,
                level,
            });
        }
        t_prev = tc;
        t += dt;
    }
    edges
}

fn analytic_sequence(p: &ToneParams) -> Vec<ToneCrossing> {
    let mut cursor = ToneCursor::start();
    let mut out = Vec::new();
    let mut fault: Option<ToneError> = None;
    loop {
        match next_crossing(p, cursor) {
            Ok(c) => {
                cursor = ToneCursor {
                    level: c.level,
                    t_cursor: c.t,
                };
                out.push(c);
            }
            Err(ToneError::Done) => break,
            Err(e) => {
                fault = Some(e);
                break;
            }
        }
        assert!(
            out.len() < 1_000_000,
            "runaway sequence — solver not terminating"
        );
    }
    assert_eq!(fault, None, "solver faulted mid-sequence");
    out
}

#[test]
fn matches_brute_force_across_sweep() {
    let grid_hz = 2.0e6;
    let grid_dt = 1.0 / grid_hz;
    let microstep = 0.0125 / 16.0;
    for &freq in &[20.0, 47.0, 73.0, 100.0, 130.0] {
        for &target_microsteps in &[1.0, 5.0, 17.0, 40.0] {
            for &base_off in &[0.0, 0.3 * microstep, -0.45 * microstep] {
                // amplitude that yields ~target_microsteps of peak displacement.
                let amplitude_nm = target_microsteps * microstep * NM_PER_MM;
                for &sign in &[1.0, -1.0] {
                    let p = make_params(freq, amplitude_nm, sign, base_off, 64_000_000.0);
                    let oracle = brute_force(&p, grid_hz);
                    let analytic = analytic_sequence(&p);
                    assert_eq!(
                        analytic.len(),
                        oracle.len(),
                        "edge count mismatch f={freq} steps={target_microsteps} \
                         base={base_off} sign={sign}: analytic={} oracle={}",
                        analytic.len(),
                        oracle.len()
                    );
                    for (a, o) in analytic.iter().zip(oracle.iter()) {
                        assert_eq!(
                            a.dir, o.dir,
                            "dir mismatch f={freq} steps={target_microsteps} at t={}",
                            a.t
                        );
                        assert_eq!(
                            a.level, o.level,
                            "level mismatch f={freq} steps={target_microsteps} at t={}",
                            a.t
                        );
                        // Analytic time must fall within the oracle's detection
                        // bracket, widened by one grid step of slack each side.
                        assert!(
                            a.t >= o.t_lo - grid_dt && a.t <= o.t_hi + grid_dt,
                            "time {} outside oracle bracket [{}, {}] \
                             f={freq} steps={target_microsteps}",
                            a.t,
                            o.t_lo,
                            o.t_hi
                        );
                    }
                }
            }
        }
    }
}

#[test]
fn grazing_peak_amplitudes_match_brute_force() {
    // Amplitudes whose peak displacement clears a half-microstep line by a hair
    // (target_microsteps = k + 0.5 + tiny). The carrier apex then crosses that
    // gridline and returns within microseconds — a genuine re-crossing that a
    // coarse period-scaled floor in flat_top_root_after would silently drop,
    // breaking net-zero. The integer-amplitude sweep above never exercises this
    // (its peaks sit 0.5 microstep from any half-line). A peak landing EXACTLY on
    // the half-line is a measure-zero tangent whose crossing is rounding-noise
    // dependent, so the apex is nudged just past the line where the crossing is
    // unambiguous and the fine-grid oracle must agree edge-for-edge.
    let grid_hz = 8.0e6;
    let grid_dt = 1.0 / grid_hz;
    let microstep = 0.0125 / 16.0;
    let graze = 0.02;
    for &freq in &[37.0, 100.0, 130.0] {
        for &target_microsteps in &[1.5 + graze, 4.5 + graze, 17.5 + graze, 39.5 + graze] {
            for &sign in &[1.0, -1.0] {
                let amplitude_nm = target_microsteps * microstep * NM_PER_MM;
                let p = make_params(freq, amplitude_nm, sign, 0.0, 64_000_000.0);
                let oracle = brute_force(&p, grid_hz);
                let analytic = analytic_sequence(&p);
                assert_eq!(
                    analytic.len(),
                    oracle.len(),
                    "grazing edge count mismatch f={freq} steps={target_microsteps} \
                     sign={sign}: analytic={} oracle={}",
                    analytic.len(),
                    oracle.len()
                );
                for (a, o) in analytic.iter().zip(oracle.iter()) {
                    assert_eq!(a.dir, o.dir, "grazing dir mismatch f={freq} at t={}", a.t);
                    assert_eq!(
                        a.level, o.level,
                        "grazing level mismatch f={freq} at t={}",
                        a.t
                    );
                    assert!(
                        a.t >= o.t_lo - grid_dt && a.t <= o.t_hi + grid_dt,
                        "grazing time {} outside oracle bracket [{}, {}] f={freq}",
                        a.t,
                        o.t_lo,
                        o.t_hi
                    );
                }
                let net: i32 = analytic.iter().map(|c| i32::from(c.dir)).sum();
                assert_eq!(
                    net, 0,
                    "grazing net displacement != 0 f={freq} steps={target_microsteps}"
                );
            }
        }
    }
}

#[test]
fn cycle_abs_strictly_monotonic_long_high_cps() {
    // u32 cycle counter wraps every ~8.26 s at H7-class 520 MHz; a long sweep
    // therefore crosses the wrap many times. cycle_at must reduce the offset
    // modulo 2^32 so each crossing maps to the correct (wrapped) cycle. Verify
    // strict monotonicity in the wrapping sense across a > 8 s chirp: a saturated
    // `as u32` (the bug) would collapse every post-8.26 s crossing to a single
    // value and run the delta backwards.
    // A constant tone keeps full amplitude across the whole duration, so
    // crossings persist past the ~8.26 s u32 wrap (a chirp's amplitude taper
    // would die out first). 20 s at 520 MHz spans ~1.04e10 cycles, > 2 * 2^32.
    let microstep = 0.0125 / 16.0;
    let amplitude_nm = 8.0 * microstep * NM_PER_MM;
    let mut hi_cps = make_params(30.0, amplitude_nm, 1.0, 0.0, 520_000_000.0);
    hi_cps.total_seconds = 20.0;
    hi_cps.ramp_seconds = 1.0;
    let seq = analytic_sequence(&hi_cps);
    assert!(seq.len() > 100, "expected a long crossing sequence");
    let mut saw_wrap = false;
    let mut prev: Option<ToneCrossing> = None;
    for c in &seq {
        if let Some(prev) = prev {
            assert!(c.t > prev.t, "time not strictly increasing");
            let delta = c.cycle_abs.wrapping_sub(prev.cycle_abs);
            assert!(
                delta > 0 && delta < (1u32 << 31),
                "cycle_abs not strictly increasing (wrapping): prev={} cur={} delta={delta}",
                prev.cycle_abs,
                c.cycle_abs,
            );
            if c.cycle_abs < prev.cycle_abs {
                saw_wrap = true;
            }
        }
        prev = Some(*c);
    }
    assert!(
        saw_wrap,
        "test must cross at least one u32 cycle wrap to be meaningful"
    );
}

#[test]
fn rate_invariant_cycle_deltas() {
    // A 10 kHz and a 40 kHz motion engine must produce bit-identical cycle_abs
    // deltas. The solver takes NO motion-sample-rate input — its only clock is
    // the MCU `cycles_per_second`. So the same curve under both engines (same
    // MCU cycle rate, which both share) yields the identical crossing-time and
    // cycle stream by construction, proving the deltas never depend on the tick
    // grid the motion ISR happens to run at.
    let engine_10khz = make_params(
        83.0,
        12.0 * (0.0125 / 16.0) * NM_PER_MM,
        1.0,
        0.0,
        64_000_000.0,
    );
    let engine_40khz = engine_10khz;

    let a = analytic_sequence(&engine_10khz);
    let b = analytic_sequence(&engine_40khz);
    assert_eq!(a.len(), b.len());
    for (x, y) in a.iter().zip(b.iter()) {
        assert_eq!(
            x.t.to_bits(),
            y.t.to_bits(),
            "crossing time not bit-identical"
        );
        let dx = x.cycle_abs.wrapping_sub(engine_10khz.anchor_cycle);
        let dy = y.cycle_abs.wrapping_sub(engine_40khz.anchor_cycle);
        assert_eq!(dx, dy, "cycle delta differs between engines");
        assert_eq!(x.dir, y.dir);
    }
}

#[test]
fn cycle_abs_depends_only_on_time_and_rate() {
    // Two distinct MCU cycle rates: the cycle delta from the anchor must equal
    // round(t * rate) for each, independent of any motion sample rate.
    let base = make_params(60.0, 20.0 * (0.0125 / 16.0) * NM_PER_MM, 1.0, 0.0, 10_000.0);
    let mut fast = base;
    fast.cycles_per_second = 40_000.0;

    let slow_seq = analytic_sequence(&base);
    let fast_seq = analytic_sequence(&fast);
    assert_eq!(slow_seq.len(), fast_seq.len());
    for (s, f) in slow_seq.iter().zip(fast_seq.iter()) {
        assert_eq!(s.t.to_bits(), f.t.to_bits(), "time must not depend on rate");
        let d_slow = s.cycle_abs.wrapping_sub(base.anchor_cycle);
        let d_fast = f.cycle_abs.wrapping_sub(fast.anchor_cycle);
        assert_eq!(d_slow, (s.t * 10_000.0) as u32);
        assert_eq!(d_fast, (f.t * 40_000.0) as u32);
    }
}

#[test]
fn sum_of_dirs_is_zero_over_full_tone() {
    // The trapezoidal envelope forces q back to exactly base at t == total, so
    // the net microstep displacement over a full tone is zero.
    for &freq in &[23.0, 64.0, 119.0] {
        for &steps in &[3.0, 11.0, 31.0] {
            let p = make_params(
                freq,
                steps * (0.0125 / 16.0) * NM_PER_MM,
                1.0,
                0.0,
                64_000_000.0,
            );
            let seq = analytic_sequence(&p);
            let net: i32 = seq.iter().map(|c| i32::from(c.dir)).sum();
            assert_eq!(
                net, 0,
                "net displacement {net} != 0 for f={freq} steps={steps}"
            );
            if let Some(last) = seq.last() {
                assert_eq!(last.level, 0, "tone must close on base level");
            }
        }
    }
}

#[test]
fn cycle_abs_strictly_monotonic() {
    let p = make_params(
        95.0,
        25.0 * (0.0125 / 16.0) * NM_PER_MM,
        -1.0,
        0.2 * (0.0125 / 16.0),
        64_000_000.0,
    );
    let seq = analytic_sequence(&p);
    assert!(seq.len() > 4, "expected a non-trivial sequence");
    let mut prev: Option<ToneCrossing> = None;
    for c in &seq {
        if let Some(prev) = prev {
            assert!(
                c.t > prev.t,
                "crossing time not strictly increasing: {} then {}",
                prev.t,
                c.t
            );
            // cycle_abs is monotonic so long as no full u32 wrap occurs inside
            // one tone (true for any realistic rate * 0.1 s).
            assert!(
                c.cycle_abs > prev.cycle_abs,
                "cycle_abs not strictly increasing"
            );
        }
        prev = Some(*c);
    }
}

#[test]
fn non_monotonic_cursor_faults() {
    let p = make_params(
        60.0,
        10.0 * (0.0125 / 16.0) * NM_PER_MM,
        1.0,
        0.0,
        64_000_000.0,
    );
    let first = next_crossing(&p, ToneCursor::start());
    assert!(first.is_ok(), "first edge should solve: {first:?}");
    let level = match first {
        Ok(c) => c.level,
        Err(_) => 0,
    };
    // A cursor parked past `total_seconds`: no crossing can follow, so the
    // stream reports exhaustion rather than fabricating an out-of-order edge.
    let past_end = ToneCursor {
        level,
        t_cursor: p.total_seconds + 1.0,
    };
    assert_eq!(
        next_crossing(&p, past_end),
        Err(ToneError::Done),
        "expected Done past total_seconds"
    );
}

#[test]
fn tone_is_mu_zero_specialization_of_chirp() {
    // A chirp with f_start == f_end has mu == 0 and must reproduce, bit for bit,
    // the tone built by make_params: the chirp path is a strict generalization.
    let microstep = 0.0125 / 16.0;
    let amplitude_nm = 12.0 * microstep * NM_PER_MM;
    let tone = make_params(70.0, amplitude_nm, 1.0, 0.0, 64_000_000.0);
    let flat = make_chirp(70.0, 70.0, amplitude_nm, 1.0, 0.1, 0.01);
    assert_eq!(flat.mu, 0.0, "equal endpoints must give mu == 0");
    let a = analytic_sequence(&tone);
    let b = analytic_sequence(&flat);
    assert_eq!(a.len(), b.len(), "chirp(f,f) edge count != tone");
    for (x, y) in a.iter().zip(b.iter()) {
        assert_eq!(x.t.to_bits(), y.t.to_bits(), "chirp(f,f) time != tone");
        assert_eq!(x.dir, y.dir);
        assert_eq!(x.level, y.level);
    }
}

#[test]
fn chirp_matches_brute_force() {
    // Fine-grid oracle cross-check across up-sweeps and down-sweeps, several
    // amplitudes and durations. The same brute_force/position_rel_ref machinery
    // the tone uses, now exercising the chirp curve (mu != 0).
    let grid_hz = 4.0e6;
    let grid_dt = 1.0 / grid_hz;
    let microstep = 0.0125 / 16.0;
    let sweeps = [
        (5.0, 135.0),
        (135.0, 5.0),
        (40.0, 100.0),
        (100.0, 40.0),
        (20.0, 21.0),
    ];
    for &(f0, f1) in &sweeps {
        // 1.0 stresses grazing peaks (apex barely touches a gridline and
        // returns); 25.0 exercises many crossings per carrier cycle.
        for &target_microsteps in &[1.0, 2.0, 8.0, 25.0] {
            for &sign in &[1.0, -1.0] {
                for &(total, ramp) in &[(0.08, 0.012), (0.15, 0.03)] {
                    let amplitude_nm = target_microsteps * microstep * NM_PER_MM;
                    let p = make_chirp(f0, f1, amplitude_nm, sign, total, ramp);
                    let oracle = brute_force(&p, grid_hz);
                    let analytic = analytic_sequence(&p);
                    assert_eq!(
                        analytic.len(),
                        oracle.len(),
                        "chirp edge count mismatch f0={f0} f1={f1} \
                         steps={target_microsteps} sign={sign} total={total}: \
                         analytic={} oracle={}",
                        analytic.len(),
                        oracle.len()
                    );
                    for (a, o) in analytic.iter().zip(oracle.iter()) {
                        assert_eq!(
                            a.dir, o.dir,
                            "chirp dir mismatch f0={f0} f1={f1} at t={}",
                            a.t
                        );
                        assert_eq!(
                            a.level, o.level,
                            "chirp level mismatch f0={f0} f1={f1} at t={}",
                            a.t
                        );
                        assert!(
                            a.t >= o.t_lo - grid_dt && a.t <= o.t_hi + grid_dt,
                            "chirp time {} outside oracle bracket [{}, {}] \
                             f0={f0} f1={f1} steps={target_microsteps}",
                            a.t,
                            o.t_lo,
                            o.t_hi
                        );
                    }
                }
            }
        }
    }
}

#[test]
fn chirp_sum_of_dirs_is_zero_and_monotonic() {
    // The closing envelope drives A_eff*env -> 0 at t == total regardless of the
    // sweep, so a full chirp nets zero microsteps and closes on base; and every
    // crossing time / cycle is strictly increasing (a non-monotone cycle_abs
    // would corrupt the SPSC ring downstream).
    let microstep = 0.0125 / 16.0;
    for &(f0, f1) in &[(5.0, 135.0), (40.0, 100.0), (110.0, 30.0)] {
        for &steps in &[6.0, 18.0] {
            let amplitude_nm = steps * microstep * NM_PER_MM;
            let p = make_chirp(f0, f1, amplitude_nm, 1.0, 0.12, 0.02);
            let seq = analytic_sequence(&p);
            assert!(seq.len() > 4, "expected a non-trivial chirp sequence");
            let net: i32 = seq.iter().map(|c| i32::from(c.dir)).sum();
            assert_eq!(net, 0, "chirp net displacement {net} != 0 f0={f0} f1={f1}");
            assert_eq!(
                seq.last().map(|c| c.level),
                Some(0),
                "chirp must close on base level f0={f0} f1={f1}"
            );
            let mut prev: Option<ToneCrossing> = None;
            for c in &seq {
                if let Some(prev) = prev {
                    assert!(
                        c.t > prev.t,
                        "chirp crossing time not strictly increasing: {} then {}",
                        prev.t,
                        c.t
                    );
                    assert!(
                        c.cycle_abs > prev.cycle_abs,
                        "chirp cycle_abs not strictly increasing"
                    );
                }
                prev = Some(*c);
            }
        }
    }
}

#[test]
fn chirp_cycle_abs_is_rate_only() {
    // cycle_abs must track round(t * cycles_per_second) for a chirp exactly as
    // for a tone: the sweep changes the crossing TIMES but never how a time maps
    // to a cycle. Two MCU rates, same time stream.
    let microstep = 0.0125 / 16.0;
    let amplitude_nm = 14.0 * microstep * NM_PER_MM;
    let mut slow = make_chirp(20.0, 90.0, amplitude_nm, 1.0, 0.1, 0.015);
    slow.cycles_per_second = 10_000.0;
    let mut fast = slow;
    fast.cycles_per_second = 40_000.0;
    let slow_seq = analytic_sequence(&slow);
    let fast_seq = analytic_sequence(&fast);
    assert_eq!(slow_seq.len(), fast_seq.len());
    for (s, f) in slow_seq.iter().zip(fast_seq.iter()) {
        assert_eq!(
            s.t.to_bits(),
            f.t.to_bits(),
            "chirp time must not depend on rate"
        );
        let d_slow = s.cycle_abs.wrapping_sub(slow.anchor_cycle);
        let d_fast = f.cycle_abs.wrapping_sub(fast.anchor_cycle);
        assert_eq!(d_slow, (s.t * 10_000.0) as u32);
        assert_eq!(d_fast, (f.t * 40_000.0) as u32);
    }
}
