//! Event-driven, time-domain reconstruction of the jerk-limited velocity
//! profile along a run.
//!
//! One pass integrates the fastest jerk- and disk-feasible profile under a
//! sampled speed cap, as a sequence of constant-jerk motions: jerk toward the
//! acceleration rail, hold the rail, jerk down to land *tangent* on the cap
//! (matching both its value and its slope), ride the cap, repeat. The control
//! is chosen by a committed-peel feasibility lookahead — "would jerking the
//! acceleration down starting now still land tangentially without crossing
//! the cap?" — so peel points anticipate the cap's shape at the landing
//! point, not its local slope, and a cap kink (a corner-speed notch, a brake
//! envelope crossing a cruise ceiling) is approached by dipping below it
//! tangentially instead of inheriting the kink as an acceleration step.
//!
//! The same pass runs backward (on reversed arrays) to build the brake
//! envelope, and forward against `min(vlc, brake)`. Integration is in time,
//! so rest is an ordinary state, not the arc-length singularity the previous
//! grid marcher wobbled on. Within straight cells every advance and event is
//! closed-form, so an all-straight run reproduces the analytic 7-segment
//! profile and its phase log lowers to exact cubics; curved cells fall back
//! to short time substeps that bill the normal jerk share of the rotating
//! acceleration vector before steering with what remains.

use super::profile::StraightPhase;

mod chain;
mod grid;
mod phase_log;
mod state;

use grid::Grid;
use phase_log::{LogMark, Mode, PhaseLog};
use state::{State, advance, march_step, step_within};

pub(super) use chain::{chain_is_continuous, chain_states, clip_phases, reverse_chain};

const SUBSTEP_TIME_FRACTION: f64 = 1.0 / 16.0;
/// Ceiling on curved-cell substep resolution. The substep timescale is the
/// jerk swing time `a/j`, but with an effectively-unlimited jerk that
/// collapses to nanoseconds while the disk rail still only varies over the
/// cell's curvature ramp — finer resolution buys nothing and starves
/// `CELL_GUARD_STEPS`, whose stall fallback then pins the velocity.
const SUBSTEPS_PER_CELL_MAX: f64 = 256.0;
const EVENT_BISECT_ITERS: u32 = 48;
/// Iterations for the peel-trigger and ride-departure bisections, where every
/// probe is a full feasibility march. Trigger placement feeds the downstream
/// piece tiling: 20 iterations demonstrably shifts seams past the lowering's
/// contiguity slop (seam_accel_feedforward), 24 does not; 32 keeps four
/// doublings of margin while still cutting a third of the probes from the
/// crossing solver's 48.
const TRIGGER_BISECT_ITERS: u32 = 32;
/// Near-contact snap band, relative: where the peel arc and the cap coincide
/// (a triangle apex), the gap closes only asymptotically — a slope-matched
/// state this close to the cap is on it.
const CONTACT_SNAP_REL: f64 = 1e-5;
const FEASIBILITY_GUARD_STEPS: u32 = 200_000;
const CELL_GUARD_STEPS: u32 = 100_000;
/// In Ride and Flight the trajectory is oracle-independent — `peel_feasible`
/// only decides when the mode ends — so the pass verifies it once per stride
/// and, on a flip or any unverified mode change, rolls back to the last
/// verified checkpoint and re-marches the window with the per-step oracle.
/// The verdict is monotone along a stretch (the in-step trigger bisection
/// already relies on that), so the re-march finds the exact trigger the
/// unstrided pass would and the stride is a pure cost cut.
const ORACLE_STRIDE: u32 = 32;
const POS_EPS_MM: f64 = 1e-12;
/// Speeds at or below this are rest. A stall short of a node commanding rest
/// pins to that node; a stall under a real cap means the pass overbraked —
/// it becomes an explicit rest event (chain goes opaque, pass resumes from
/// `v = 0, a = 0`) instead of integrating through the position fold.
const STALL_CAP_FLOOR: f64 = 1e-9;

fn rel_eps(x: f64) -> f64 {
    1e-9 * (1.0 + x.abs())
}

/// The sampled constraint data the pass integrates against. `cap_a` is the
/// cap's slope-accel (`dv/dt` along the cap) per *cell*, evaluated where the
/// cell is (its right-open span), so a slope kink at a node is visible to the
/// cell entering it.
pub(super) struct Track<'a> {
    pub s: &'a [f64],
    pub cap_v: &'a [f64],
    /// Per-cell cap slope-accel: `cap_a[i]` covers `[s[i], s[i+1])`; the last
    /// entry duplicates the final cell.
    pub cap_a: &'a [f64],
    pub accel: &'a [f64],
    pub kappa: &'a [f64],
    pub j_max: f64,
}

/// The brake envelope's analytic phase chain (forward frame), for splicing
/// into the forward pass's log at the contact point. `binding[i]` marks nodes
/// where the envelope, not the velocity-limit curve, is the cap.
pub(super) struct BrakeChain<'a> {
    pub phases: &'a [StraightPhase],
    pub binding: &'a [bool],
}

pub(super) struct Pass {
    pub v: Vec<f64>,
    pub a: Vec<f64>,
    /// Per node: the emitted state respects the disk and jerk dynamics. False
    /// where the pass rode a cap chord braking harder than the disk rail —
    /// the backward pass does this descending a raw-vlc wall it has no brake
    /// envelope to be caught by, and such nodes must not count as `binding`.
    pub feasible: Vec<bool>,
    /// Constant-jerk phase chain in run frame, tiling the whole pass when
    /// `complete`. Curved-cell substeps and cap-chord rides are phases too —
    /// each is a constant-jerk cubic — so mixed runs chain as well as straight
    /// ones; only a stall or a rejected splice leaves the chain incomplete.
    pub phases: Vec<StraightPhase>,
    pub complete: bool,
}

/// Whether a committed jerk-down from `st` lands tangent on the cap (value
/// and slope both met) without ever crossing it. Marches the analytic arc,
/// clamping the tangency target to the disk rail so an unreachable plunge in
/// the cap reads as "brake at the rail", which the cap (via the brake
/// envelope half of `min`) can always catch.
fn peel_feasible(g: &Grid, st: State) -> bool {
    let j = g.t.j_max;
    // Quick accept: already tangent-or-under, diverging below.
    let slope0 = g.slope_at(st.s);
    if st.v <= g.cap_at(st.s) + rel_eps(st.v) && st.a <= slope0 + rel_eps(st.a.abs()) {
        return true;
    }
    // Quick accept: the whole swing to full brake stays under every cap ahead.
    {
        let rail = g.rail_at(st.s, st.v);
        let v_peak = st.v + st.a.max(0.0) * st.a.max(0.0) / (2.0 * j);
        let swing_t = (st.a + rail).max(0.0) / j;
        let reach = v_peak * swing_t;
        if v_peak < g.cap_min_over(st.s, reach) {
            return true;
        }
    }
    let mut st = st;
    let mut cell = g.cell(st.s);
    let mut guard = 0;
    loop {
        guard += 1;
        if guard > FEASIBILITY_GUARD_STEPS {
            return false;
        }
        let cap = g.cap_in(cell, st.s);
        if st.v > cap + rel_eps(cap) {
            return false;
        }
        let rail = g.rail_in(cell, st.s, st.v);
        let slope = g.t.cap_a[cell].max(-rail);
        if st.a <= slope + rel_eps(st.a.abs().max(slope.abs())) {
            return true;
        }
        if st.s >= g.end_s() - POS_EPS_MM {
            return true;
        }
        let to_span_end = g.span_end_s(cell) - st.s;
        let dt_tan = if st.a > slope {
            (st.a - slope) / j
        } else {
            substep_budget(g, st, j)
        };
        let dt = march_step(st, -j, dt_tan, to_span_end).max(1e-12);
        // The arc's velocity peak is its critical point against the span's
        // monotone cap chord; check it exactly, plus the midpoint.
        if st.a > 0.0 {
            let t_peak = st.a / j;
            if t_peak < dt {
                let peak = advance(st, -j, t_peak);
                let c = g.cell_ahead(cell, peak.s);
                if peak.v > g.cap_in(c, peak.s) + rel_eps(peak.v) {
                    return false;
                }
            }
        }
        let mid = advance(st, -j, 0.5 * dt);
        let c_mid = g.cell_ahead(cell, mid.s);
        if mid.v > g.cap_in(c_mid, mid.s) + rel_eps(mid.v) {
            return false;
        }
        st = advance(st, -j, dt);
        // Came to rest under the cap: a stalled arc can never cross the
        // non-negative cap ahead.
        if st.v <= STALL_CAP_FLOOR {
            return true;
        }
        cell = g.cell_ahead(cell, st.s);
    }
}

/// Integrate the fastest feasible profile under the track's cap from
/// `(start_v, start_a)`, logging the constant-jerk phase chain as it goes;
/// `brake` supplies the backward envelope's chain for splicing at contact.
pub(super) fn reach_pass(
    track: &Track,
    start_v: f64,
    start_a: f64,
    brake: Option<&BrakeChain>,
) -> Pass {
    let g = Grid::new(track);
    let n = g.n();
    let mut v_out = vec![0.0_f64; n];
    let mut a_out = vec![0.0_f64; n];
    let mut feasible = vec![true; n];

    let mut st = State {
        t: 0.0,
        s: track.s[0],
        v: start_v.min(track.cap_v[0]),
        a: 0.0,
    };
    let rail0 = g.rail_at(st.s, st.v);
    st.a = start_a.clamp(-rail0, rail0);
    v_out[0] = st.v;
    a_out[0] = st.a;

    let mut log = PhaseLog::new();
    let mut mode = initial_mode(&g, st);

    let fast_forward = |st: &mut State,
                        mode: &mut Mode,
                        i: usize,
                        k: usize,
                        end: State,
                        v_out: &mut [f64],
                        a_out: &mut [f64]| {
        for m in i..=k {
            let v = track.cap_v[m];
            let rail = g.rail_at(track.s[m], v);
            v_out[m] = v;
            a_out[m] = track.cap_a[m - 1].clamp(-rail, rail);
        }
        *st = end;
        *mode = initial_mode(&g, *st);
    };

    let mut i = 1;
    if mode == Mode::Ride {
        if let Some((k, end)) = try_splice(st, brake, &g, &mut log, i) {
            fast_forward(&mut st, &mut mode, i, k, end, &mut v_out, &mut a_out);
            i = k + 1;
        }
    }

    struct StrideCkpt {
        st: State,
        i: usize,
        mode: Mode,
        mark: LogMark,
    }
    let mut ckpt: Option<StrideCkpt> = None;
    let mut skip_left: u32 = 0;
    let mut fine_left: u32 = 0;

    'nodes: while i < n {
        let mut guard = 0;
        while st.s < track.s[i] - POS_EPS_MM {
            guard += 1;
            if guard > CELL_GUARD_STEPS {
                log.opaque(st);
                // Cannot make progress (fully stalled under a zero cap):
                // hold position on the grid so downstream stays defined.
                st.s = track.s[i];
                st.v = st.v.min(track.cap_v[i]);
                break;
            }
            // Rewind to the stride checkpoint and re-march the window with
            // the per-step oracle; everything a skipped step may have touched
            // (position in the node loop, phase log, per-node feasibility) is
            // restored or rewritten by the re-march.
            macro_rules! rollback {
                ($c:expr) => {{
                    let c = $c;
                    for k in feasible.iter_mut().take(i.min(n - 1) + 1).skip(c.i) {
                        *k = true;
                    }
                    st = c.st;
                    i = c.i;
                    mode = c.mode;
                    log.rewind(c.mark);
                    skip_left = 0;
                    fine_left = 2 * ORACLE_STRIDE;
                    continue 'nodes;
                }};
            }
            let was = mode;
            let strided = matches!(mode, Mode::Ride | Mode::Flight);
            if strided && fine_left == 0 && skip_left == 0 {
                if peel_feasible(&g, st) {
                    ckpt = Some(StrideCkpt {
                        st,
                        i,
                        mode,
                        mark: log.mark(),
                    });
                    skip_left = ORACLE_STRIDE;
                } else if let Some(c) = ckpt.take() {
                    rollback!(c);
                }
                // Oracle already false with nothing to rewind to: the checked
                // step below finds the trigger itself.
            }
            let assume = strided && skip_left > 0;
            if assume {
                skip_left -= 1;
            } else {
                fine_left = fine_left.saturating_sub(1);
            }
            let s_before = st.s;
            match mode {
                Mode::Ride => {
                    ride_step(&g, &mut st, &mut log, &mut mode, i, &mut feasible, assume);
                }
                Mode::Flight => flight_step(&g, &mut st, &mut log, &mut mode, i, assume),
                Mode::Peel => peel_step(&g, &mut st, &mut log, &mut mode, i),
            }
            debug_assert!(
                st.s >= s_before - rel_eps(s_before),
                "ride pass moved backwards: s {} -> {} (v={}, a={}, mode={mode:?})",
                s_before,
                st.s,
                st.v,
                st.a
            );
            let stalled = st.v <= STALL_CAP_FLOOR && st.a < 0.0 && st.s <= s_before + POS_EPS_MM;
            if stalled {
                if assume {
                    let c = ckpt.take().expect("strided step without a checkpoint");
                    rollback!(c);
                }
                log.opaque(st);
                if track.cap_v[i] <= STALL_CAP_FLOOR {
                    st.s = track.s[i];
                    st.v = 0.0;
                    break;
                }
                st.v = 0.0;
                st.a = 0.0;
                mode = initial_mode(&g, st);
                ckpt = None;
                skip_left = 0;
                fine_left = 0;
                continue;
            }
            if mode != was {
                if assume {
                    // Unverified transition inside the stride window: rewind
                    // and let the checked march decide.
                    let c = ckpt.take().expect("strided step without a checkpoint");
                    rollback!(c);
                }
                ckpt = None;
                skip_left = 0;
                fine_left = 0;
            }
            if mode == Mode::Ride && was != Mode::Ride {
                if let Some((k, end)) = try_splice(st, brake, &g, &mut log, i) {
                    fast_forward(&mut st, &mut mode, i, k, end, &mut v_out, &mut a_out);
                    i = k + 1;
                    continue 'nodes;
                }
            }
        }
        st.s = track.s[i];
        let rail = g.rail_at(track.s[i], st.v);
        if mode == Mode::Ride {
            st.v = track.cap_v[i];
            st.a = track.cap_a[i - 1].clamp(-rail, rail);
        }
        v_out[i] = st.v;
        a_out[i] = st.a.clamp(-rail, rail);
        i += 1;
    }
    log.close(st);

    Pass {
        v: v_out,
        a: a_out,
        feasible,
        complete: log.complete,
        phases: log.phases,
    }
}

fn initial_mode(g: &Grid, st: State) -> Mode {
    let on_cap = st.v >= g.cap_at(st.s) - rel_eps(st.v);
    if on_cap && st.a <= g.slope_at(st.s) + rel_eps(st.a.abs()) {
        Mode::Ride
    } else if peel_feasible(g, st) {
        Mode::Flight
    } else {
        Mode::Peel
    }
}

/// Splice the brake chain over the contiguous binding stretch starting at the
/// contact: the envelope *is* the profile there, and the backward pass already
/// integrated it exactly, so adopting its phases replaces the cell-quantized
/// ride (whose per-cell landings would kick the chain off C1). Returns the
/// stretch's last node and the resume state there; `None` when the contact
/// node is not binding, the stretch does not reach the target node, or the
/// chain misses the pass's state at the joint.
fn try_splice(
    st: State,
    brake: Option<&BrakeChain>,
    g: &Grid,
    log: &mut PhaseLog,
    i: usize,
) -> Option<(usize, State)> {
    let brake = brake?;
    if brake.phases.is_empty() || !brake.binding[i - 1] {
        return None;
    }
    let n = g.n();
    let mut k = i - 1;
    while k + 1 < n && brake.binding[k + 1] {
        k += 1;
    }
    if k < i {
        return None;
    }
    let end = log.splice(st, brake.phases, g.t.s[k], splice_joint_v_tol(g, st, i))?;
    Some((k, end))
}

/// Joint tolerance for adopting the brake chain: the landing snapped onto the
/// sampled cap's linear chord, but the chain is the exact constant-accel
/// cubic under it, so the two disagree by up to the chord's sag —
/// `|v''|·ds²/8` with `v'' = -a²/v³` along a constant-accel arc — over the
/// contact cell. Doubled for the jerk swing the bound ignores.
fn splice_joint_v_tol(g: &Grid, st: State, i: usize) -> f64 {
    let ds_cell = g.t.s[i] - g.t.s[i - 1];
    let v = st.v.max(1e-3);
    let sag = st.a * st.a * ds_cell * ds_cell / (4.0 * v * v * v);
    1e-5 * (1.0 + st.v) + sag
}

/// One cell of riding the cap. Returns `false` when the ride cannot continue
/// (mode changed); the caller re-dispatches.
fn ride_step(
    g: &Grid,
    st: &mut State,
    log: &mut PhaseLog,
    mode: &mut Mode,
    i: usize,
    feasible: &mut [bool],
    assume_feasible: bool,
) -> bool {
    let track = g.t;
    let cell = i - 1;
    let ds = track.s[i] - st.s;
    let v1 = track.cap_v[i];
    let dt = 2.0 * ds / (st.v + v1).max(1e-9);
    let rail1 = g.rail_at(track.s[i], v1);
    // Arrival state at the node: the slope of the cell just traversed. The
    // next cell's slope is lookahead only — assigning it to the state would
    // leak a kink's chord into the profile as an instantaneous accel step.
    let next_state = State {
        t: st.t + dt,
        s: track.s[i],
        v: v1,
        a: track.cap_a[cell].clamp(-rail1, rail1),
    };
    // Detach: the cap accelerates away faster than the rail allows.
    let rail = g.rail_at(st.s, st.v);
    if track.cap_a[cell] > rail + rel_eps(rail) {
        *mode = Mode::Flight;
        return false;
    }
    if track.cap_a[cell] < -(rail + rel_eps(rail)) {
        feasible[i] = false;
    }
    // Kink lookahead: leaving from the next node must still be feasible.
    if !assume_feasible && !peel_feasible(g, next_state) {
        // Departure point within this cell: latest cap state that can still
        // peel tangentially under the kink.
        let (mut lo, mut hi) = (st.s, track.s[i]);
        for _ in 0..TRIGGER_BISECT_ITERS {
            let mid = 0.5 * (lo + hi);
            if peel_feasible(g, cap_state(g, *st, mid, cell)) {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        *st = cap_state(g, *st, lo, cell);
        *mode = Mode::Peel;
        log.set_jerk(*st, -track.j_max);
        return false;
    }
    // Riding the cell's cap chord is constant-accel motion (v² linear in s),
    // so the ride logs as a j = 0 phase at the chord's slope — the state's own
    // `a` may carry the rail clamp, which the phase must not.
    log.set_jerk(
        State {
            a: track.cap_a[cell],
            ..*st
        },
        0.0,
    );
    *st = next_state;
    true
}

/// The on-cap state at arc `s` within `cell`, timed from `from`.
fn cap_state(g: &Grid, from: State, s: f64, cell: usize) -> State {
    let v = g.cap_at(s);
    let dt = 2.0 * (s - from.s).max(0.0) / (from.v + v).max(1e-9);
    let rail = g.rail_at(s, v);
    State {
        t: from.t + dt,
        s,
        v,
        a: g.t.cap_a[cell].clamp(-rail, rail),
    }
}

fn substep_budget(g: &Grid, st: State, j_nominal: f64) -> f64 {
    let a_scale = g.lerp_node(g.t.accel, st.s);
    let jerk_dt = (a_scale / j_nominal.max(1e-9)) * SUBSTEP_TIME_FRACTION;
    let c = g.cell(st.s);
    let span = g.t.s[c + 1] - g.t.s[c];
    let cell_dt = span / st.v.max(1e-9) / SUBSTEPS_PER_CELL_MAX;
    jerk_dt.max(cell_dt)
}

/// Tangential jerk available after billing the normal share the rotating
/// acceleration vector demands over `dt` (the increment disk).
fn effective_jerk(g: &Grid, st: State, dt: f64) -> f64 {
    if !g.curved_near(st.s) {
        return g.t.j_max;
    }
    let ds = st.v * dt + 0.5 * st.a * dt * dt;
    let k0 = g.kappa_at(st.s);
    let k1 = g.kappa_at(st.s + ds.max(0.0));
    let a_n0 = k0 * st.v * st.v;
    let v1 = st.v + st.a * dt;
    let a_n1 = k1 * v1 * v1;
    let dtheta = k1 * ds;
    let j_norm = (a_n1 - a_n0) + st.a * dtheta;
    let budget = (g.t.j_max * dt) * (g.t.j_max * dt) - j_norm * j_norm;
    budget.max(0.0).sqrt() / dt.max(1e-12)
}

/// One flight step: accelerate toward the rail, watching for the peel
/// trigger. The trigger is bisected inside the step so the emitted jerk stays
/// bang-bang; on straight cells the step spans the whole cell and every
/// event is exact.
fn flight_step(
    g: &Grid,
    st: &mut State,
    log: &mut PhaseLog,
    mode: &mut Mode,
    i: usize,
    assume_feasible: bool,
) {
    let track = g.t;
    let curved = g.curved_near(st.s);
    let rem = track.s[i] - st.s;
    let rail = g.rail_at(st.s, st.v);
    let dt_budget = if curved {
        substep_budget(g, *st, track.j_max)
    } else {
        f64::INFINITY
    };
    let j_up = if curved {
        effective_jerk(g, *st, dt_budget.max(1e-9))
    } else {
        track.j_max
    };
    // A jerk swing smaller than one floor-duration step cannot be integrated:
    // with an effectively-unlimited jerk the rail's hold band is far narrower
    // than one step's `j·dt`, so the correction overshoots into a
    // zero-progress limit cycle ping-ponging across the rail until the cell
    // guard gives up. At this resolution the swing is instantaneous — snap.
    if (st.a - rail).abs() <= j_up * 2e-12 {
        st.a = rail;
    }
    let (j_cmd, dt_event) = if st.a > rail + rel_eps(rail) {
        // Above the rail (a curved cell shrank it, or a cap slope leaked into
        // the state): jerk back down to it.
        (-j_up, (st.a - rail) / j_up.max(1e-9))
    } else if st.a >= rail - rel_eps(rail) {
        // Hold the rail. On straight cells the rail is a constant, so the
        // hold is exact; on curved cells the substep re-clamps below.
        (0.0, f64::INFINITY)
    } else {
        (j_up, (rail - st.a) / j_up.max(1e-9))
    };
    let dt = step_within(*st, j_cmd, dt_event.min(dt_budget), rem).max(1e-12);
    let mut next = advance(*st, j_cmd, dt);
    if curved {
        let r = g.rail_at(next.s, next.v);
        next.a = next.a.clamp(-r, r);
    }
    if assume_feasible || peel_feasible(g, next) {
        log.set_jerk(*st, j_cmd);
        // Landed on the cap from below (tangential arrival)?
        if next.v >= g.cap_at(next.s) - rel_eps(next.v) {
            next.v = g.cap_at(next.s);
            next.a = g.slope_at(next.s);
            log.close(next);
            *mode = Mode::Ride;
        }
        *st = next;
        return;
    }
    // The peel trigger fires inside this step: bisect it.
    let (mut lo, mut hi) = (0.0, dt);
    for _ in 0..TRIGGER_BISECT_ITERS {
        let mid = 0.5 * (lo + hi);
        if peel_feasible(g, advance(*st, j_cmd, mid)) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    if lo > 0.0 {
        log.set_jerk(*st, j_cmd);
    }
    *st = advance(*st, j_cmd, lo);
    *mode = Mode::Peel;
    log.set_jerk(*st, -track.j_max);
}

/// One committed-peel step: jerk the acceleration down until the profile
/// lands tangent on the cap (ride) or bottoms out safely under it (flight).
fn peel_step(g: &Grid, st: &mut State, log: &mut PhaseLog, mode: &mut Mode, i: usize) {
    let track = g.t;
    let curved = g.curved_near(st.s);
    let dt_budget = if curved {
        substep_budget(g, *st, track.j_max)
    } else {
        f64::INFINITY
    };
    let j_dn = if curved {
        effective_jerk(g, *st, dt_budget.max(1e-9))
    } else {
        track.j_max
    };
    let j_cmd = -j_dn;
    log.set_jerk(*st, j_cmd);
    let rail = g.rail_at(st.s, st.v);
    let slope = g.slope_at(st.s).max(-rail);
    let rem = track.s[i] - st.s;
    // Aim for the cap's slope; already at or below it (still committed
    // because free flight is not yet feasible — a kink ahead), keep braking
    // in real substeps so the loop always makes progress.
    let dt_aim = if st.a > slope {
        (st.a - slope) / j_dn.max(1e-9)
    } else {
        substep_budget(g, *st, track.j_max)
    };
    let dt = step_within(*st, j_cmd, dt_aim.min(dt_budget), rem).max(1e-12);
    let next = advance(*st, j_cmd, dt);
    let cap_next = g.cap_at(next.s);
    if next.v >= cap_next - rel_eps(next.v) {
        // Contact. Bisect the touch point if we overshot past the cap.
        let mut touch = dt;
        if next.v > cap_next + rel_eps(next.v) {
            let (mut lo, mut hi) = (0.0, dt);
            for _ in 0..EVENT_BISECT_ITERS {
                let mid = 0.5 * (lo + hi);
                let m = advance(*st, j_cmd, mid);
                if m.v < g.cap_at(m.s) {
                    lo = mid;
                } else {
                    hi = mid;
                }
            }
            touch = 0.5 * (lo + hi);
        }
        // Land at the moment the arc's acceleration meets the landing slope,
        // not at the v-crossing: contact fires inside the epsilon band, where
        // the arc still has `sqrt(2·j·eps)` of acceleration to shed (or has
        // already shed past the slope) — snapping `a` there would kick the
        // phase chain off C1. At the tangency time the acceleration snap is
        // zero and the velocity snap is epsilon-sized. The tangency may fall
        // slightly past the contact step (the slope is re-read at the probe,
        // not at the step start); the apex-band check is what keeps the
        // landing on the cap, so tolerate that drift rather than gate on it.
        {
            let probe = advance(*st, j_cmd, touch);
            let rail_probe = g.rail_at(probe.s, probe.v);
            let slope_probe = g.slope_at(probe.s).clamp(-rail_probe, rail_probe);
            if st.a > slope_probe {
                let t_tan = (st.a - slope_probe) / j_dn.max(1e-9);
                let cand = advance(*st, j_cmd, t_tan);
                let apex_band = 1e-8 * (1.0 + cand.v);
                if t_tan <= dt * (1.0 + 1e-3) && cand.v <= g.cap_at(cand.s) + apex_band {
                    touch = t_tan;
                }
            }
        }
        let mut landed = advance(*st, j_cmd, touch);
        landed.v = g.cap_at(landed.s);
        let rail_land = g.rail_at(landed.s, landed.v);
        landed.a = g.slope_at(landed.s).clamp(-rail_land, rail_land);
        *st = landed;
        *mode = Mode::Ride;
        return;
    }
    // Slope matched with only an asymptotic gap left: that is the tangency,
    // snap onto the cap.
    let rail_next = g.rail_at(next.s, next.v);
    let slope_next = g.slope_at(next.s).clamp(-rail_next, rail_next);
    if next.a <= slope_next + rel_eps(slope_next.abs())
        && cap_next - next.v <= CONTACT_SNAP_REL * (1.0 + cap_next)
    {
        let mut landed = next;
        landed.v = cap_next;
        landed.a = slope_next;
        *st = landed;
        *mode = Mode::Ride;
        return;
    }
    // Bottomed under the cap with the slope matched and free flight feasible
    // again: resume accelerating.
    if next.a <= slope_next + rel_eps(slope_next.abs())
        && next.v < cap_next - rel_eps(next.v)
        && peel_feasible(g, next)
    {
        *mode = Mode::Flight;
    }
    *st = next;
}

#[cfg(test)]
mod tests;
