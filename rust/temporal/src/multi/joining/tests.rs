use super::*;

fn make_state(v_start: f64, v_end: f64) -> ChainState {
    ChainState {
        v_start,
        v_end,
        a_start: None,
        profile: None,
        dirty: false,
    }
}

#[test]
fn bidirectional_sweep_uses_lower_achieved_side() {
    let mut states = vec![make_state(0.0, 120.0), make_state(80.0, 0.0)];
    let corner_caps = vec![150.0];

    let dirty = bidirectional_junction_sweep(&mut states, &corner_caps);

    assert_eq!(dirty, 1);
    assert!((states[0].v_end - 80.0).abs() < 1e-6);
    assert!((states[1].v_start - 80.0).abs() < 1e-6);
    assert!(states[0].dirty);
    assert!(!states[1].dirty);
}
