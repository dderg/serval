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

#[test]
fn clip_phases_slices_sum_back_to_the_composite() {
    let chain = two_phase_chain();
    let total = phase_end_s(&chain[1]);
    let cuts = [0.0, 0.3 * total, total];
    let slices = clip_phases(&chain, &cuts);
    let sliced_time: f64 = slices.iter().flatten().map(|p| p.dt).sum();
    let whole_time: f64 = chain.iter().map(|p| p.dt).sum();
    assert!(
        (sliced_time - whole_time).abs() < 1e-15 * whole_time,
        "sliced {sliced_time} whole {whole_time}"
    );
}

#[test]
fn clip_phases_rebases_every_slice_onto_its_own_origin() {
    let chain = two_phase_chain();
    let total = phase_end_s(&chain[1]);
    let cuts = [0.0, 0.3 * total, 0.8 * total, total];
    let slices = clip_phases(&chain, &cuts);
    assert_eq!(slices.len(), 3);
    for (slice, w) in slices.iter().zip(cuts.windows(2)) {
        let first = slice.first().expect("a slice of positive length");
        assert_eq!(first.s0, 0.0);
        assert_eq!(first.t0, 0.0);
        let end = phase_end_s(slice.last().expect("a slice of positive length"));
        assert!((end - (w[1] - w[0])).abs() < 1e-9, "slice spans {end}");
    }
}

#[test]
fn clip_phases_slices_hand_over_one_continuous_state() {
    let chain = two_phase_chain();
    let total = phase_end_s(&chain[1]);
    let slices = clip_phases(&chain, &[0.0, 0.45 * total, total]);
    for slice in &slices {
        assert!(chain_is_continuous(slice, true));
    }
    let handover = advance(
        start_state(slices[0].last().expect("a slice of positive length")),
        slices[0].last().expect("a slice").j,
        slices[0].last().expect("a slice").dt,
    );
    let resumes = &slices[1][0];
    assert!((handover.v - resumes.v0).abs() < 1e-9);
    assert!((handover.a - resumes.a0).abs() < 1e-6);
}

#[test]
fn clip_phases_takes_the_final_cut_at_the_chain_end_not_by_arc_length() {
    let stopping = vec![StraightPhase {
        t0: 0.0,
        dt: 0.02,
        s0: 0.0,
        v0: 0.0,
        a0: 0.0,
        j: 1e5,
    }];
    let total = phase_end_s(&stopping[0]);
    let slices = clip_phases(&stopping, &[0.0, 0.5 * total, total]);
    let sliced_time: f64 = slices.iter().flatten().map(|p| p.dt).sum();
    assert!((sliced_time - 0.02).abs() < 1e-15, "sliced {sliced_time}");
}
