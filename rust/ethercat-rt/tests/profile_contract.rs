use ethercat_rt::dynamics::DynamicsModel;
use servo_ident::model::PhysicalParams;
use servo_ident::profile_out::{render_profile, PairSplit};

#[test]
fn fitter_rendered_profile_loads_and_evaluates() {
    let p = PhysicalParams {
        mass: vec![0.030, 0.020],
        viscous: vec![0.004, 0.005],
        coulomb: vec![1.0, 0.9],
    };
    let frame = vec![vec![1.0, 0.0], vec![0.0, 1.0]];
    let text = render_profile(&p, &["a", "b"], &["x", "y"], &frame, &[0.5, 0.6], &[]);
    let m = DynamicsModel::from_toml_str(&text).expect("fitter output must load");
    assert_eq!(m.n_slots, 2);
    assert_eq!(m.n_modes, 2);
    assert_eq!(m.axes, ["a", "b"]);
    assert_eq!(m.modes, ["x", "y"]);
    let fwd = m.torque_ff(0, &[1000.0, 0.0], &[100.0, 0.0], &[0.0, 0.0]);
    let expect_fwd = 0.030 * 1000.0 + 0.004 * 100.0 + 1.0;
    assert!((fwd - expect_fwd).abs() < 1e-3, "{fwd} vs {expect_fwd}");
    let rev = m.torque_ff(1, &[0.0, -500.0], &[0.0, -20.0], &[0.0, 0.0]);
    let expect_rev = 0.020 * -500.0 + 0.005 * -20.0 - 0.9;
    assert!((rev - expect_rev).abs() < 1e-3, "{rev} vs {expect_rev}");
}

#[test]
fn integer_valued_fit_results_still_load() {
    let p = PhysicalParams {
        mass: vec![2.0],
        viscous: vec![0.0],
        coulomb: vec![1.0],
    };
    let frame = vec![vec![1.0]];
    let text = render_profile(&p, &["x"], &["x"], &frame, &[1.0], &[]);
    let m = DynamicsModel::from_toml_str(&text).expect("integer-valued floats must load");
    assert_eq!(m.n_slots, 1);
    assert_eq!(m.n_modes, 1);
}

#[test]
fn fitter_rendered_pair_loads_and_applies_a_differential() {
    let p = PhysicalParams {
        mass: vec![0.030, 0.060],
        viscous: vec![0.002, 0.002],
        coulomb: vec![1.0, 1.0],
    };
    let frame = vec![
        vec![0.25, -0.25, -0.25, -0.25],
        vec![0.25, -0.25, 0.25, 0.25],
    ];
    let pairs = [PairSplit {
        first: 0,
        second: 1,
        w: [0.1, 0.01, 0.0, 0.0, 0.0, 0.0],
    }];
    let text = render_profile(
        &p,
        &["a", "a1", "b", "b1"],
        &["x", "y"],
        &frame,
        &[0.5, 0.5, 0.5, 0.5],
        &pairs,
    );
    let mut m = DynamicsModel::from_toml_str(&text).expect("fitter pair output must load");
    assert_eq!(m.n_slots, 4);
    m.bind_drive_signs(&[1.0, -1.0, -1.0, -1.0]);
    let acc = [4.0, 0.0, 0.0, 0.0];
    let vel = [0.0, 0.0, 0.0, 0.0];
    let pos = [10.0, 10.0, 0.0, 0.0];
    let base0 = 0.25 * 0.030 + 0.25 * 0.060;
    let tau0 = m.torque_ff(0, &acc, &vel, &pos);
    assert!((tau0 - (base0 + 0.0045)).abs() < 1e-5, "{tau0}");
}
