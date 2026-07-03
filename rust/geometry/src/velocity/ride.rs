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

use super::disk::disk_rail_accel;
use super::profile::StraightPhase;

const SUBSTEP_TIME_FRACTION: f64 = 1.0 / 16.0;
const EVENT_BISECT_ITERS: u32 = 48;
/// Near-contact snap band, relative: where the peel arc and the cap coincide
/// (a triangle apex), the gap closes only asymptotically — a slope-matched
/// state this close to the cap is on it.
const CONTACT_SNAP_REL: f64 = 1e-5;
const FEASIBILITY_GUARD_STEPS: u32 = 200_000;
const CELL_GUARD_STEPS: u32 = 100_000;
const RANGE_MIN_BLOCK: usize = 64;
const POS_EPS_MM: f64 = 1e-12;
const KAPPA_EPS: f64 = 1e-12;

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
    /// Constant-jerk phase chain in run frame; complete only when `analytic`.
    pub phases: Vec<StraightPhase>,
    pub analytic: bool,
}

#[derive(Clone, Copy, Debug)]
struct State {
    t: f64,
    s: f64,
    v: f64,
    a: f64,
}

fn advance(st: State, j: f64, dt: f64) -> State {
    State {
        t: st.t + dt,
        s: st.s + st.v * dt + 0.5 * st.a * dt * dt + j * dt * dt * dt / 6.0,
        v: (st.v + st.a * dt + 0.5 * j * dt * dt).max(0.0),
        a: st.a + j * dt,
    }
}

/// Time for the constant-jerk motion from `st` to advance `ds`, or `None` if
/// it stalls (speed reaches zero) first. The bisection bracket is capped at
/// the stall, where the position curve folds — a bracket extending past it
/// would converge onto the fold instead of the crossing.
fn time_to_cross(st: State, j: f64, ds: f64) -> Option<f64> {
    if ds <= 0.0 {
        return Some(0.0);
    }
    let pos = |dt: f64| st.v * dt + 0.5 * st.a * dt * dt + j * dt * dt * dt / 6.0;
    let stall = next_stall(st, j);
    let mut hi = {
        let by_v = ds / st.v.max(1e-9);
        let by_a = (2.0 * ds / st.a.abs().max(1e-9)).sqrt();
        let by_j = (6.0 * ds / j.abs().max(1e-9)).cbrt();
        by_v.min(by_a).min(by_j).max(1e-12)
    };
    let mut guard = 0;
    loop {
        if let Some(ts) = stall {
            if hi >= ts {
                if pos(ts) < ds {
                    return None;
                }
                hi = ts;
                break;
            }
        }
        if pos(hi) >= ds {
            break;
        }
        hi *= 2.0;
        guard += 1;
        if guard > 200 {
            return None;
        }
    }
    let mut lo = 0.0;
    for _ in 0..EVENT_BISECT_ITERS {
        let mid = 0.5 * (lo + hi);
        if pos(mid) < ds {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    Some(0.5 * (lo + hi))
}

/// Smallest non-negative time at which the speed reaches zero from above, in
/// closed form. A root the motion departs from positively (rest with
/// positive jerk or acceleration behind it) is not a stall.
fn next_stall(st: State, j: f64) -> Option<f64> {
    let arriving = |t: f64| st.a + j * t < 0.0 || (st.a + j * t == 0.0 && j <= 0.0 && st.v <= 0.0);
    if j.abs() < 1e-300 {
        if st.a >= 0.0 {
            return (st.v <= 0.0 && st.a == 0.0).then_some(0.0);
        }
        return Some((st.v / -st.a).max(0.0));
    }
    let disc = st.a * st.a - 2.0 * j * st.v;
    if disc < 0.0 {
        return None;
    }
    let sq = disc.sqrt();
    let (r1, r2) = ((-st.a - sq) / j, (-st.a + sq) / j);
    [r1.min(r2), r1.max(r2)]
        .into_iter()
        .find(|&t| t >= 0.0 && arriving(t))
}

struct Grid<'a> {
    t: &'a Track<'a>,
    cap_range_min: Vec<f64>,
}

impl<'a> Grid<'a> {
    fn new(t: &'a Track<'a>) -> Self {
        let cap_range_min = t
            .cap_v
            .chunks(RANGE_MIN_BLOCK)
            .map(|c| c.iter().fold(f64::INFINITY, |m, &x| m.min(x)))
            .collect();
        Self { t, cap_range_min }
    }

    fn n(&self) -> usize {
        self.t.s.len()
    }

    /// Cell whose span contains `s` (clamped).
    fn cell(&self, s: f64) -> usize {
        let n = self.n();
        let i = self.t.s.partition_point(|&x| x <= s);
        i.clamp(1, n - 1) - 1
    }

    fn lerp_node(&self, arr: &[f64], s: f64) -> f64 {
        let c = self.cell(s);
        let span = self.t.s[c + 1] - self.t.s[c];
        let f = if span > POS_EPS_MM {
            ((s - self.t.s[c]) / span).clamp(0.0, 1.0)
        } else {
            0.0
        };
        arr[c] + f * (arr[c + 1] - arr[c])
    }

    /// Cap speed at `s`, linear in `v²` between nodes: a constant-decel brake
    /// segment of the envelope is linear in `v²`, so this represents it
    /// exactly where a linear-in-`v` chord would sag below it.
    fn cap_at(&self, s: f64) -> f64 {
        let c = self.cell(s);
        let span = self.t.s[c + 1] - self.t.s[c];
        let f = if span > POS_EPS_MM {
            ((s - self.t.s[c]) / span).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let (w0, w1) = (
            self.t.cap_v[c] * self.t.cap_v[c],
            self.t.cap_v[c + 1] * self.t.cap_v[c + 1],
        );
        (w0 + f * (w1 - w0)).max(0.0).sqrt()
    }

    fn slope_at(&self, s: f64) -> f64 {
        self.t.cap_a[self.cell(s)]
    }

    fn rail_at(&self, s: f64, v: f64) -> f64 {
        disk_rail_accel(
            self.lerp_node(self.t.accel, s),
            self.lerp_node(self.t.kappa, s),
            v,
        )
    }

    fn kappa_at(&self, s: f64) -> f64 {
        self.lerp_node(self.t.kappa, s)
    }

    fn curved_near(&self, s: f64) -> bool {
        let c = self.cell(s);
        self.t.kappa[c].abs() > KAPPA_EPS || self.t.kappa[c + 1].abs() > KAPPA_EPS
    }

    /// Lower bound of the cap over `[s, s + dist]`.
    fn cap_min_over(&self, s: f64, dist: f64) -> f64 {
        let lo = self.cell(s);
        let hi = self.cell(s + dist) + 1;
        let (b_lo, b_hi) = (lo / RANGE_MIN_BLOCK, hi / RANGE_MIN_BLOCK);
        let mut m = f64::INFINITY;
        if b_lo == b_hi {
            for &x in &self.t.cap_v[lo..=hi] {
                m = m.min(x);
            }
            return m;
        }
        for &x in &self.t.cap_v[lo..(b_lo + 1) * RANGE_MIN_BLOCK] {
            m = m.min(x);
        }
        for &x in &self.cap_range_min[(b_lo + 1)..b_hi] {
            m = m.min(x);
        }
        for &x in &self.t.cap_v[b_hi * RANGE_MIN_BLOCK..=hi] {
            m = m.min(x);
        }
        m
    }

    fn end_s(&self) -> f64 {
        self.t.s[self.n() - 1]
    }
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
    let mut guard = 0;
    loop {
        guard += 1;
        if guard > FEASIBILITY_GUARD_STEPS {
            return false;
        }
        let cap = g.cap_at(st.s);
        if st.v > cap + rel_eps(cap) {
            return false;
        }
        let rail = g.rail_at(st.s, st.v);
        let slope = g.slope_at(st.s).max(-rail);
        if st.a <= slope + rel_eps(st.a.abs().max(slope.abs())) {
            return true;
        }
        if st.s >= g.end_s() - POS_EPS_MM {
            return true;
        }
        let cell = g.cell(st.s);
        let to_cell_end = g.t.s[cell + 1] - st.s;
        let dt_tan = if st.a > slope {
            (st.a - slope) / j
        } else {
            substep_budget(g, st, j)
        };
        let dt = match time_to_cross(st, -j, to_cell_end) {
            Some(t) => t.min(dt_tan).max(1e-12),
            None => dt_tan.max(1e-12),
        };
        // Midpoint violation check within the step.
        let mid = advance(st, -j, 0.5 * dt);
        if mid.v > g.cap_at(mid.s) + rel_eps(mid.v) {
            return false;
        }
        st = advance(st, -j, dt);
    }
}

#[derive(PartialEq, Clone, Copy, Debug)]
enum Mode {
    Flight,
    Peel,
    Ride,
}

struct PhaseLog {
    phases: Vec<StraightPhase>,
    active: bool,
    analytic: bool,
    open: Option<(State, f64)>,
}

impl PhaseLog {
    fn new(active: bool) -> Self {
        Self {
            phases: Vec::new(),
            active,
            analytic: active,
            open: None,
        }
    }

    fn set_jerk(&mut self, st: State, j: f64) {
        if !self.active {
            return;
        }
        if let Some((_, j0)) = self.open {
            if j0 == j {
                return;
            }
        }
        self.close(st);
        self.open = Some((st, j));
    }

    fn close(&mut self, st: State) {
        if let Some((p0, j)) = self.open.take() {
            let dt = st.t - p0.t;
            if dt > 0.0 {
                self.phases.push(StraightPhase {
                    t0: p0.t,
                    dt,
                    s0: p0.s,
                    v0: p0.v,
                    a0: p0.a,
                    j,
                });
            }
        }
    }

    fn opaque(&mut self, st: State) {
        self.close(st);
        self.analytic = false;
        self.active = false;
        self.phases.clear();
    }

    /// Replace everything from `st` on with the brake chain's tail.
    fn splice(&mut self, st: State, chain: &[StraightPhase]) {
        if !self.active {
            return;
        }
        self.close(st);
        let Some(tail) = clip_chain_from(chain, st.s) else {
            self.opaque(st);
            return;
        };
        // The chain is exact; the landing state carries up to a cell of chord
        // smear, so only the velocity is checked — the samples are re-derived
        // from the chain afterwards, which absorbs the accel mismatch.
        let head = &tail[0];
        if (head.v0 - st.v).abs() > 1e-5 * (1.0 + st.v) {
            self.opaque(st);
            return;
        }
        let mut t = st.t;
        for p in tail {
            self.phases.push(StraightPhase { t0: t, ..p });
            t += p.dt;
        }
        self.active = false;
    }
}

/// The chain's suffix starting at run-arc `s_from`: the containing phase is
/// entered mid-flight (state advanced to `s_from`), the rest follow whole.
fn clip_chain_from(chain: &[StraightPhase], s_from: f64) -> Option<Vec<StraightPhase>> {
    let idx = chain
        .iter()
        .position(|p| phase_end_s(p) > s_from + POS_EPS_MM)?;
    let p = &chain[idx];
    let mut out = Vec::with_capacity(chain.len() - idx);
    if s_from > p.s0 + POS_EPS_MM {
        let st = State {
            t: 0.0,
            s: p.s0,
            v: p.v0,
            a: p.a0,
        };
        let tau = time_to_cross(st, p.j, s_from - p.s0)?;
        let at = advance(st, p.j, tau);
        out.push(StraightPhase {
            t0: 0.0,
            dt: p.dt - tau,
            s0: at.s,
            v0: at.v,
            a0: at.a,
            j: p.j,
        });
    } else {
        out.push(*p);
    }
    out.extend_from_slice(&chain[idx + 1..]);
    Some(out)
}

fn phase_end_s(p: &StraightPhase) -> f64 {
    p.s0 + p.v0 * p.dt + 0.5 * p.a0 * p.dt * p.dt + p.j * p.dt * p.dt * p.dt / 6.0
}

/// The chain's phases clipped to `[s_lo, s_hi]`, rebased to span-local time
/// and arc-length (`t0 = 0`, `s0 = 0` at the span start). Natural phase
/// boundaries chain bit-exactly; a genuine clip solves the interior time —
/// once per boundary, so abutting spans read the same float and their
/// durations tile the chain's total.
pub(super) fn clip_phases(chain: &[StraightPhase], s_lo: f64, s_hi: f64) -> Vec<StraightPhase> {
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
        if exit <= entry + 1e-15 {
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

/// Exact `(v, a)` at each grid arc-length from the phase chain, so an
/// analytic run's samples come from the same closed-form phases the lowering
/// executes — never from ride chords, whose half-cell smear the sampled
/// profile would otherwise carry.
pub(super) fn chain_states(chain: &[StraightPhase], s: &[f64]) -> Vec<(f64, f64)> {
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
pub(super) fn reverse_chain(rev: &[StraightPhase], total_len: f64) -> Vec<StraightPhase> {
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

/// Integrate the fastest feasible profile under the track's cap from
/// `(start_v, start_a)`. `log_phases` enables the analytic phase log (callers
/// pass it for all-straight runs); `brake` supplies the backward envelope's
/// chain for splicing at contact.
pub(super) fn reach_pass(
    track: &Track,
    start_v: f64,
    start_a: f64,
    log_phases: bool,
    brake: Option<&BrakeChain>,
) -> Pass {
    let g = Grid::new(track);
    let n = g.n();
    let mut v_out = vec![0.0_f64; n];
    let mut a_out = vec![0.0_f64; n];

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

    let mut log = PhaseLog::new(log_phases);
    let mut mode = initial_mode(&g, st);
    let mut spliced = false;
    if mode == Mode::Ride {
        try_splice(st, brake, &mut log, &mut spliced, 1);
    }

    for i in 1..n {
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
            let was = mode;
            match mode {
                Mode::Ride => {
                    if spliced {
                        ride_copy(track, &mut st, i);
                    } else if !ride_step(&g, &mut st, &mut log, &mut mode, i) {
                        continue;
                    }
                }
                Mode::Flight => flight_step(&g, &mut st, &mut log, &mut mode, i),
                Mode::Peel => peel_step(&g, &mut st, &mut log, &mut mode, i),
            }
            if mode == Mode::Ride && was != Mode::Ride {
                try_splice(st, brake, &mut log, &mut spliced, i);
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
    }
    log.close(st);

    Pass {
        v: v_out,
        a: a_out,
        analytic: log.analytic,
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

fn try_splice(
    st: State,
    brake: Option<&BrakeChain>,
    log: &mut PhaseLog,
    spliced: &mut bool,
    i: usize,
) {
    if !log.active {
        return;
    }
    let Some(brake) = brake else {
        return;
    };
    let from = i.saturating_sub(1);
    if brake.binding[from..].iter().all(|&b| b) {
        log.splice(st, brake.phases);
        *spliced = true;
    }
}

/// After a splice the samples simply copy the cap (the brake envelope binds
/// through to the end and the emitted profile *is* the chain).
fn ride_copy(track: &Track, st: &mut State, i: usize) {
    let ds = track.s[i] - st.s;
    let v1 = track.cap_v[i];
    let dt = 2.0 * ds / (st.v + v1).max(1e-9);
    st.t += dt;
    st.s = track.s[i];
    st.v = v1;
    st.a = track.cap_a[i - 1];
}

/// One cell of riding the cap. Returns `false` when the ride cannot continue
/// (mode changed); the caller re-dispatches.
fn ride_step(g: &Grid, st: &mut State, log: &mut PhaseLog, mode: &mut Mode, i: usize) -> bool {
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
    // Kink lookahead: leaving from the next node must still be feasible.
    if !peel_feasible(g, next_state) {
        // Departure point within this cell: latest cap state that can still
        // peel tangentially under the kink.
        let (mut lo, mut hi) = (st.s, track.s[i]);
        for _ in 0..EVENT_BISECT_ITERS {
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
    let flat = track.cap_a[cell] == 0.0 && (v1 - st.v).abs() <= rel_eps(st.v);
    let straight_ride = !g.curved_near(st.s) && flat;
    if straight_ride {
        log.set_jerk(*st, 0.0);
    } else if log.active {
        // A curved or sloped cap ride is not a constant-jerk cubic.
        log.opaque(*st);
    }
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
    (a_scale / j_nominal.max(1e-9)) * SUBSTEP_TIME_FRACTION
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
fn flight_step(g: &Grid, st: &mut State, log: &mut PhaseLog, mode: &mut Mode, i: usize) {
    let track = g.t;
    let curved = g.curved_near(st.s);
    if curved && log.active {
        log.opaque(*st);
    }
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
    let dt_cross = time_to_cross(*st, j_cmd, rem).unwrap_or(f64::INFINITY);
    let dt = dt_cross.min(dt_event).min(dt_budget).max(1e-12);
    let mut next = advance(*st, j_cmd, dt);
    if curved {
        let r = g.rail_at(next.s, next.v);
        next.a = next.a.clamp(-r, r);
    }
    if peel_feasible(g, next) {
        if !curved {
            log.set_jerk(*st, j_cmd);
        }
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
    for _ in 0..EVENT_BISECT_ITERS {
        let mid = 0.5 * (lo + hi);
        if peel_feasible(g, advance(*st, j_cmd, mid)) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    if lo > 0.0 && !curved {
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
    if curved && log.active {
        log.opaque(*st);
    }
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
    if !curved {
        log.set_jerk(*st, j_cmd);
    }
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
    let dt_cross = time_to_cross(*st, j_cmd, rem).unwrap_or(f64::INFINITY);
    let dt = dt_aim.min(dt_cross).min(dt_budget).max(1e-12);
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
