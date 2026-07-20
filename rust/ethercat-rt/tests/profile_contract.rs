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
    let fwd = m.torque_ff(0, &[1000.0, 0.0], &[100.0, 0.0]);
    let expect_fwd = 0.030 * 1000.0 + 0.004 * 100.0 + 1.0;
    assert!((fwd - expect_fwd).abs() < 1e-3, "{fwd} vs {expect_fwd}");
    let rev = m.torque_ff(1, &[0.0, -500.0], &[0.0, -20.0]);
    let expect_rev = 0.020 * -500.0 + 0.005 * -20.0 - 0.9;
    assert!((rev - expect_rev).abs() < 1e-3, "{rev} vs {expect_rev}");
}

#[test]
fn coulomb_ramps_through_the_fit_deadband_instead_of_stepping() {
    let p = PhysicalParams {
        mass: vec![0.01],
        viscous: vec![0.0],
        coulomb: vec![10.0],
    };
    let text = render_profile(&p, &["x"], &["x"], &[vec![1.0]], &[0.0], &[]);
    let m = DynamicsModel::from_toml_str(&text).expect("profile must load");
    let deadband = ethercat_rt::dynamics::COULOMB_DEADBAND_MM_S;
    assert_eq!(
        deadband as f64,
        servo_ident::model::COULOMB_DEADBAND_MM_S,
        "runtime ramp deadband must match the fit's coulomb_sign exclusion"
    );
    assert_eq!(m.torque_ff(0, &[0.0], &[0.0]), 0.0);
    let half = m.torque_ff(0, &[0.0], &[deadband / 2.0]);
    assert!(
        (half - 5.0).abs() < 1e-4,
        "mid-deadband must be c/2: {half}"
    );
    let edge = m.torque_ff(0, &[0.0], &[deadband]);
    assert!(
        (edge - 10.0).abs() < 1e-4,
        "deadband edge must be full c: {edge}"
    );
    let above = m.torque_ff(0, &[0.0], &[deadband * 4.0]);
    assert!(
        (above - 10.0).abs() < 1e-4,
        "above deadband stays full c: {above}"
    );
    let rev = m.torque_ff(0, &[0.0], &[-deadband / 4.0]);
    assert!(
        (rev + 2.5).abs() < 1e-4,
        "ramp must be odd-symmetric: {rev}"
    );
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
fn paired_fitter_profile_preserves_pair_force_and_loads_provenance() {
    let p = PhysicalParams {
        mass: vec![0.040, 0.080],
        viscous: vec![0.0, 0.0],
        coulomb: vec![0.0, 0.0],
    };
    let frame = vec![vec![0.25, 0.25, 0.5], vec![0.25, 0.25, -0.5]];
    let text = render_profile(
        &p,
        &["a", "a1", "b"],
        &["x", "y"],
        &frame,
        &[0.5, 0.6],
        &[PairSplit {
            first: 0,
            second: 1,
            direction_split: 0.2,
        }],
    )
    .replacen(
        "\n[[pair]]",
        "\nrefined_source = \"baseline.toml\"\nrefined_term = \"direction_split\"\nrefined_delta_direction_split_a = 0.01\n\n[[pair]]",
        1,
    );
    let model = DynamicsModel::from_toml_str(&text).expect("paired fitter output must load");
    let acc = [1000.0, 1000.0, 0.0];
    let vel = [0.0; 3];
    let first = model.torque_ff(0, &acc, &vel);
    let second = model.torque_ff(1, &acc, &vel);
    assert!((first - 18.0).abs() < 1e-6, "first torque {first}");
    assert!((second - 12.0).abs() < 1e-6, "second torque {second}");
    assert!((first - 15.0 - 0.2 * (2.0 * 15.0) / 2.0).abs() < 1e-6);
    assert!((0.25 * (first + second) - 7.5).abs() < 1e-6);
}

#[test]
fn profile_with_ff_lead_lands_in_the_endpoints_per_slot_lead() {
    let text = "\
version = 6
axes = [\"a\", \"b\"]
modes = [\"x\", \"y\"]
frame = [[1.0, 0.0], [0.0, 1.0]]
mass = [0.05, 0.06]
viscous = [0.01, 0.02]
coulomb = [1.0, 1.5]
ff_lead_us = 300.0
";
    let m = DynamicsModel::from_toml_str(text).expect("profile with ff_lead_us must load");
    assert_eq!(m.ff_lead_ns(), vec![300_000, 300_000]);
}

#[test]
fn timing_only_profile_with_zero_viscous_and_coulomb_loads() {
    let text = "\
version = 6
axes = [\"x\"]
modes = [\"x\"]
frame = [[1.0]]
mass = [0.001]
viscous = [0.0]
coulomb = [0.0]
ff_lead_us = 120.0
";
    let m = DynamicsModel::from_toml_str(text).expect("timing-only profile must load");
    assert_eq!(m.ff_lead_ns(), vec![120_000]);
    let tau = m.torque_ff(0, &[1000.0], &[0.0]);
    assert!((tau - 1.0).abs() < 1e-6, "{tau}");
}
