use ethercat_rt::dynamics::DynamicsModel;
use servo_ident::model::PhysicalParams;
use servo_ident::profile_out::render_profile;

#[test]
fn fitter_rendered_profile_loads_and_evaluates() {
    let p = PhysicalParams {
        mass: vec![0.030, 0.020],
        viscous: vec![0.004, 0.005],
        coulomb: vec![1.0, 0.9],
    };
    let frame = vec![vec![1.0, 0.0], vec![0.0, 1.0]];
    let text = render_profile(&p, &["a", "b"], &["x", "y"], &frame, &[0.5, 0.6]);
    let m = DynamicsModel::from_toml_str(&text).expect("fitter output must load");
    assert_eq!(m.n_slots, 2);
    assert_eq!(m.n_modes, 2);
    assert_eq!(m.axes, ["a", "b"]);
    assert_eq!(m.modes, ["x", "y"]);
    let fwd = m.torque_ff(0, &[1000.0, 0.0], &[100.0, 0.0]);
    let expect_fwd = 0.030 * 1000.0 + 0.004 * 100.0 + 1.0;
    assert!((fwd - expect_fwd).abs() < 1e-3, "{fwd} vs {expect_fwd}");
    let rev = m.torque_ff(1, &[0.0, -500.0], &[0.0, -20.0]);
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
    let text = render_profile(&p, &["x"], &["x"], &frame, &[1.0]);
    let m = DynamicsModel::from_toml_str(&text).expect("integer-valued floats must load");
    assert_eq!(m.n_slots, 1);
    assert_eq!(m.n_modes, 1);
}
