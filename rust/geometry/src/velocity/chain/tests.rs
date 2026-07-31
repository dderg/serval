use super::*;

#[test]
fn time_to_cross_from_rest_is_the_jerk_cubic() {
    let st = State {
        s: 0.0,
        v: 0.0,
        a: 0.0,
    };
    let (j, ds) = (1e5, 0.01);
    let t = time_to_cross(st, j, ds).unwrap();
    let expected = libm::cbrt(6.0 * ds / j);
    assert!((t - expected).abs() < 1e-12 * expected, "t={t}");
}

#[test]
fn time_to_cross_reports_a_stall() {
    let braking_to_rest_before_the_span = State {
        s: 0.0,
        v: 1.0,
        a: -100.0,
    };
    assert!(time_to_cross(braking_to_rest_before_the_span, 0.0, 1.0).is_none());
}

#[test]
fn advance_is_the_constant_jerk_cubic() {
    let st = State {
        s: 2.0,
        v: 10.0,
        a: 100.0,
    };
    let (j, dt) = (1e4, 0.02);
    let n = advance(st, j, dt);
    assert!((n.s - (2.0 + 10.0 * dt + 50.0 * dt * dt + j * dt * dt * dt / 6.0)).abs() < 1e-12);
    assert!((n.v - (10.0 + 100.0 * dt + 0.5 * j * dt * dt)).abs() < 1e-12);
    assert!((n.a - (100.0 + j * dt)).abs() < 1e-12);
}

fn two_phase_chain() -> Vec<StraightPhase> {
    let (v0, a0, j, dt) = (10.0, 0.0, 1e5, 0.01);
    let p0 = StraightPhase {
        t0: 0.0,
        dt,
        s0: 0.0,
        v0,
        a0,
        j,
    };
    let e = advance(start_state(&p0), j, dt);
    vec![
        p0,
        StraightPhase {
            t0: dt,
            dt,
            s0: e.s,
            v0: e.v,
            a0: e.a,
            j: -j,
        },
    ]
}

#[test]
fn chain_states_reads_the_phase_the_arc_falls_in() {
    let chain = two_phase_chain();
    let joint = chain[1].s0;
    let end = phase_end_s(&chain[1]);
    let arcs = [0.0, 0.5 * joint, joint, 0.5 * (joint + end), end];
    let states = chain_states(&chain, &arcs);
    assert_eq!(states[0], (chain[0].v0, chain[0].a0));
    assert_eq!(states[2], (chain[1].v0, chain[1].a0));
    let last = advance(start_state(&chain[1]), chain[1].j, chain[1].dt);
    assert!((states[4].0 - last.v).abs() < 1e-9);
    assert!((states[4].1 - last.a).abs() < 1e-9);
    assert!(states[1].0 > chain[0].v0 && states[1].0 < chain[1].v0);
}

#[test]
fn chain_is_continuous_rejects_an_acceleration_kick() {
    let mut chain = two_phase_chain();
    assert!(chain_is_continuous(&chain, true));
    chain[1].a0 += 1.0;
    assert!(!chain_is_continuous(&chain, true));
    assert!(chain_is_continuous(&chain, false));
}

#[test]
fn chain_is_continuous_rejects_a_velocity_kick() {
    let mut chain = two_phase_chain();
    chain[1].v0 += 1.0;
    assert!(!chain_is_continuous(&chain, false));
}
