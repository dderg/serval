use super::super::profile::StraightPhase;
use super::chain::phase_end_s;
use super::state::{State, advance, time_to_cross};
use super::{POS_EPS_MM, rel_eps};

#[derive(PartialEq, Clone, Copy, Debug)]
pub(super) enum Mode {
    Flight,
    Peel,
    Ride,
}

pub(super) struct PhaseLog {
    pub(super) phases: Vec<StraightPhase>,
    active: bool,
    pub(super) complete: bool,
    open: Option<(State, f64)>,
}

/// Whether `st` lies on the constant-jerk cubic from `p0` — i.e. the open
/// phase still describes the motion. A rail clamp or a node snap mutates the
/// state off the cubic; the phase must close there so the chain stays exact.
fn on_cubic(p0: State, j: f64, st: State) -> bool {
    let e = advance(p0, j, st.t - p0.t);
    (e.s - st.s).abs() <= rel_eps(st.s)
        && (e.v - st.v).abs() <= rel_eps(st.v)
        && (e.a - st.a).abs() <= rel_eps(st.a)
}

/// The log state a stride checkpoint captures; rewinding to it discards
/// everything a rolled-back window recorded.
#[derive(Clone, Copy)]
pub(super) struct LogMark {
    phases_len: usize,
    open: Option<(State, f64)>,
}

impl PhaseLog {
    pub(super) fn new() -> Self {
        Self {
            phases: Vec::new(),
            active: true,
            complete: true,
            open: None,
        }
    }

    pub(super) fn mark(&self) -> LogMark {
        LogMark {
            phases_len: self.phases.len(),
            open: self.open,
        }
    }

    pub(super) fn rewind(&mut self, mark: LogMark) {
        self.phases.truncate(mark.phases_len);
        self.open = mark.open;
    }

    pub(super) fn set_jerk(&mut self, st: State, j: f64) {
        if !self.active {
            return;
        }
        if let Some((p0, j0)) = self.open {
            if j0 == j && on_cubic(p0, j0, st) {
                return;
            }
        }
        self.close(st);
        self.open = Some((st, j));
    }

    pub(super) fn close(&mut self, st: State) {
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

    pub(super) fn opaque(&mut self, st: State) {
        self.close(st);
        self.complete = false;
        self.active = false;
        self.phases.clear();
    }

    /// Adopt the brake chain over `[st.s, s_hi]`: append its clipped phases in
    /// place of integrating the stretch, returning the state at `s_hi` for the
    /// pass to resume from. `None` (no log mutation) if the chain does not
    /// meet the pass's state at `st` — the pass then integrates the stretch
    /// itself. Only the velocity is checked at the joint: the landing state
    /// carries up to a cell of chord smear in `a`, which the chain absorbs.
    pub(super) fn splice(
        &mut self,
        st: State,
        chain: &[StraightPhase],
        s_hi: f64,
    ) -> Option<State> {
        if !self.active {
            return None;
        }
        let tail = clip_chain_from(chain, st.s)?;
        let head = &tail[0];
        if (head.v0 - st.v).abs() > 1e-5 * (1.0 + st.v) {
            return None;
        }
        self.close(st);
        let mut end = st;
        for p in &tail {
            if p.s0 >= s_hi - POS_EPS_MM {
                break;
            }
            let p_state = State {
                t: 0.0,
                s: p.s0,
                v: p.v0,
                a: p.a0,
            };
            let dt = if phase_end_s(p) <= s_hi + POS_EPS_MM {
                p.dt
            } else {
                match time_to_cross(p_state, p.j, s_hi - p.s0) {
                    Some(tau) => tau,
                    None => break,
                }
            };
            if dt <= 0.0 {
                break;
            }
            self.phases.push(StraightPhase {
                t0: end.t,
                dt,
                ..*p
            });
            let e = advance(p_state, p.j, dt);
            end = State {
                t: end.t + dt,
                s: e.s,
                v: e.v,
                a: e.a,
            };
        }
        end.s = s_hi;
        Some(end)
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
