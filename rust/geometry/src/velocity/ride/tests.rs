use super::*;

#[test]
fn time_to_cross_from_rest_is_the_jerk_cubic() {
    let st = State {
        t: 0.0,
        s: 0.0,
        v: 0.0,
        a: 0.0,
    };
    let (j, ds) = (1e5, 0.01);
    let t = time_to_cross(st, j, ds).unwrap();
    let expected = (6.0 * ds / j).cbrt();
    assert!((t - expected).abs() < 1e-12 * expected, "t={t}");
}

#[test]
fn time_to_cross_reports_a_stall() {
    // Braking to rest before covering the span.
    let st = State {
        t: 0.0,
        s: 0.0,
        v: 1.0,
        a: -100.0,
    };
    assert!(time_to_cross(st, 0.0, 1.0).is_none());
}

#[test]
fn advance_is_the_constant_jerk_cubic() {
    let st = State {
        t: 1.0,
        s: 2.0,
        v: 10.0,
        a: 100.0,
    };
    let (j, dt) = (1e4, 0.02);
    let n = advance(st, j, dt);
    assert!((n.s - (2.0 + 10.0 * dt + 50.0 * dt * dt + j * dt * dt * dt / 6.0)).abs() < 1e-12);
    assert!((n.v - (10.0 + 100.0 * dt + 0.5 * j * dt * dt)).abs() < 1e-12);
    assert!((n.a - (100.0 + j * dt)).abs() < 1e-12);
    assert!((n.t - 1.02).abs() < 1e-15);
}

fn eval_chain(chain: &[StraightPhase], t: f64) -> (f64, f64, f64) {
    for p in chain {
        if t <= p.t0 + p.dt + 1e-15 {
            let tau = (t - p.t0).max(0.0);
            return (
                p.s0 + p.v0 * tau + 0.5 * p.a0 * tau * tau + p.j * tau * tau * tau / 6.0,
                p.v0 + p.a0 * tau + 0.5 * p.j * tau * tau,
                p.a0 + p.j * tau,
            );
        }
    }
    let p = chain.last().unwrap();
    eval_chain(std::slice::from_ref(p), p.t0 + p.dt)
}

fn two_phase_chain() -> Vec<StraightPhase> {
    // Jerk up from rest, then hold the accel.
    let p0 = StraightPhase {
        t0: 0.0,
        dt: 0.01,
        s0: 0.0,
        v0: 0.0,
        a0: 0.0,
        j: 1e5,
    };
    let st = advance(
        State {
            t: 0.0,
            s: 0.0,
            v: 0.0,
            a: 0.0,
        },
        p0.j,
        p0.dt,
    );
    let p1 = StraightPhase {
        t0: p0.dt,
        dt: 0.05,
        s0: st.s,
        v0: st.v,
        a0: st.a,
        j: 0.0,
    };
    vec![p0, p1]
}

#[test]
fn clip_phases_tiles_the_chain() {
    let chain = two_phase_chain();
    let end = phase_end_s(&chain[1]);
    let cut = 0.6 * end;
    let head = clip_phases(&chain, 0.0, cut);
    let tail = clip_phases(&chain, cut, end);
    let total: f64 = chain.iter().map(|p| p.dt).sum();
    let split: f64 = head.iter().chain(&tail).map(|p| p.dt).sum();
    assert!((total - split).abs() < 1e-12, "durations must tile");
    let head_len = phase_end_s(head.last().unwrap());
    assert!((head_len - cut).abs() < 1e-9, "head spans to the cut");
    let (s0, v0, a0) = eval_chain(&tail, 0.0);
    assert!(s0.abs() < 1e-12, "tail rebased to s=0");
    let t_cut: f64 = head.iter().map(|p| p.dt).sum();
    let (_, vh, ah) = eval_chain(&chain, t_cut);
    assert!(
        (v0 - vh).abs() < 1e-9 && (a0 - ah).abs() < 1e-6,
        "C1 at cut"
    );
}

#[test]
fn reverse_chain_maps_states_into_the_forward_frame() {
    let chain = two_phase_chain();
    let total_len = phase_end_s(&chain[1]);
    let fwd = reverse_chain(&chain, total_len);
    assert_eq!(fwd.len(), 2);
    // Backward start (rest at the exit) becomes the forward end.
    let last = &fwd[1];
    let end = advance(
        State {
            t: 0.0,
            s: last.s0,
            v: last.v0,
            a: last.a0,
        },
        last.j,
        last.dt,
    );
    assert!((end.s - total_len).abs() < 1e-9);
    assert!(end.v.abs() < 1e-12, "forward profile ends at rest");
    assert!(end.a.abs() < 1e-9, "forward profile ends at zero accel");
    // Chain is time-contiguous from zero.
    assert!(fwd[0].t0.abs() < 1e-15);
    assert!((fwd[1].t0 - fwd[0].dt).abs() < 1e-12);
}
