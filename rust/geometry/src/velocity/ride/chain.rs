use super::super::profile::StraightPhase;
use super::POS_EPS_MM;
use super::state::{State, advance, time_to_cross};

pub(super) fn phase_end_s(p: &StraightPhase) -> f64 {
    p.s0 + p.v0 * p.dt + 0.5 * p.a0 * p.dt * p.dt + p.j * p.dt * p.dt * p.dt / 6.0
}

/// Whether every joint in the chain is state-continuous — each phase's end
/// state is the next phase's start state. A chain with a kicked joint (a
/// landing snap, a rail clamp, a splice contact) still serves as sampling
/// truth, but must not lower to exact cubics: the kick would dispatch as a
/// genuine trajectory discontinuity. Under a jerk limit acceleration is part
/// of the continuous state; with jerk unlimited the profile steps its
/// acceleration at phase joints by design, so only position and velocity
/// gate (`require_accel = false`).
pub(in crate::velocity) fn chain_is_continuous(
    chain: &[StraightPhase],
    require_accel: bool,
) -> bool {
    chain.windows(2).all(|w| {
        let (p, q) = (&w[0], &w[1]);
        let e = advance(
            State {
                t: 0.0,
                s: p.s0,
                v: p.v0,
                a: p.a0,
            },
            p.j,
            p.dt,
        );
        (e.s - q.s0).abs() <= 1e-9 * (1.0 + q.s0.abs())
            && (e.v - q.v0).abs() <= 1e-8 * (1.0 + q.v0)
            && (!require_accel || (e.a - q.a0).abs() <= 1e-4 * (1.0 + q.a0.abs()))
    })
}

/// The chain's phases clipped to `[s_lo, s_hi]`, rebased to span-local time
/// and arc-length (`t0 = 0`, `s0 = 0` at the span start). Natural phase
/// boundaries chain bit-exactly; a genuine clip solves the interior time —
/// once per boundary, so abutting spans read the same float and their
/// durations tile the chain's total.
pub(in crate::velocity) fn clip_phases(
    chain: &[StraightPhase],
    s_lo: f64,
    s_hi: f64,
) -> Vec<StraightPhase> {
    let mut out = Vec::new();
    let mut t_base: Option<f64> = None;
    for p in chain {
        let p_end = phase_end_s(p);
        if p_end <= s_lo + POS_EPS_MM || p.s0 >= s_hi - POS_EPS_MM {
            continue;
        }
        let st = State {
            t: 0.0,
            s: p.s0,
            v: p.v0,
            a: p.a0,
        };
        let entry = if s_lo <= p.s0 + POS_EPS_MM {
            0.0
        } else {
            time_to_cross(st, p.j, s_lo - p.s0).unwrap_or(0.0)
        };
        let exit = if s_hi >= p_end - POS_EPS_MM {
            p.dt
        } else {
            time_to_cross(st, p.j, s_hi - p.s0).unwrap_or(p.dt)
        };
        if exit <= entry + 1e-9 {
            continue;
        }
        let at = advance(st, p.j, entry);
        let base = *t_base.get_or_insert(p.t0 + entry);
        out.push(StraightPhase {
            t0: (p.t0 + entry) - base,
            dt: exit - entry,
            s0: at.s - s_lo,
            v0: at.v,
            a0: at.a,
            j: p.j,
        });
    }
    out
}

/// Exact `(v, a)` at each grid arc-length from the phase chain, so a
/// completed run's samples come from the same closed-form phases the lowering
/// executes — never from ride chords, whose half-cell smear the sampled
/// profile would otherwise carry.
pub(in crate::velocity) fn chain_states(chain: &[StraightPhase], s: &[f64]) -> Vec<(f64, f64)> {
    let mut out = Vec::with_capacity(s.len());
    let mut idx = 0usize;
    for &x in s {
        while idx + 1 < chain.len() && phase_end_s(&chain[idx]) < x - POS_EPS_MM {
            idx += 1;
        }
        let p = &chain[idx];
        let st = State {
            t: 0.0,
            s: p.s0,
            v: p.v0,
            a: p.a0,
        };
        let end = phase_end_s(p);
        let state = if x <= p.s0 + POS_EPS_MM {
            (p.v0, p.a0)
        } else if x >= end - POS_EPS_MM {
            let e = advance(st, p.j, p.dt);
            (e.v, e.a)
        } else {
            match time_to_cross(st, p.j, x - p.s0) {
                Some(tau) => {
                    let e = advance(st, p.j, tau);
                    (e.v, e.a)
                }
                None => (p.v0, p.a0),
            }
        };
        out.push(state);
    }
    out
}

/// Map the backward pass's phase chain into the forward frame: time reverses,
/// so each phase enters at its backward end state with negated acceleration
/// and the same jerk, and the chain order flips.
pub(in crate::velocity) fn reverse_chain(
    rev: &[StraightPhase],
    total_len: f64,
) -> Vec<StraightPhase> {
    let mut out: Vec<StraightPhase> = rev
        .iter()
        .rev()
        .map(|p| {
            let end = advance(
                State {
                    t: 0.0,
                    s: p.s0,
                    v: p.v0,
                    a: p.a0,
                },
                p.j,
                p.dt,
            );
            StraightPhase {
                t0: 0.0,
                dt: p.dt,
                s0: total_len - end.s,
                v0: end.v,
                a0: -end.a,
                j: p.j,
            }
        })
        .collect();
    let mut t = 0.0;
    for p in &mut out {
        p.t0 = t;
        t += p.dt;
    }
    out
}
