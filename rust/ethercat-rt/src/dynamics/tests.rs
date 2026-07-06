use super::*;

const SCALAR: &str = r#"
version = 1
axes = ["x"]
mass = [[0.0123]]
viscous = [0.0045]
coulomb_fwd = [1.2]
coulomb_rev = [-1.1]
coulomb_deadband_mm_s = 0.5
fit_rms_residual = [0.8]
"#;

const SCALAR_Y: &str = r#"
version = 1
axes = ["y"]
mass = [[0.050]]
viscous = [0.006]
coulomb_fwd = [2.0]
coulomb_rev = [-1.9]
coulomb_deadband_mm_s = 0.5
fit_rms_residual = [0.3]
"#;

const COREXY: &str = r#"
version = 1
axes = ["a", "b"]
mass = [[0.030, -0.010], [-0.010, 0.030]]
viscous = [0.004, 0.004]
coulomb_fwd = [1.0, 1.0]
coulomb_rev = [-1.0, -1.0]
coulomb_deadband_mm_s = 0.5
fit_rms_residual = [0.5, 0.5]
"#;

#[test]
fn parses_scalar_profile() {
    let m = DynamicsModel::from_toml_str(SCALAR).unwrap();
    assert_eq!(m.n, 1);
    assert_eq!(m.axes, ["x"]);
}

#[test]
fn torque_ff_scalar() {
    let m = DynamicsModel::from_toml_str(SCALAR).unwrap();
    let tau = m.torque_ff(0, &[1000.0], &[100.0]);
    let expect = 0.0123 * 1000.0 + 0.0045 * 100.0 + 1.2;
    assert!((tau - expect).abs() < 1e-4, "{tau} vs {expect}");
}

#[test]
fn torque_ff_reverse_coulomb_and_deadband() {
    let m = DynamicsModel::from_toml_str(SCALAR).unwrap();
    let rev = m.torque_ff(0, &[0.0], &[-100.0]);
    assert!((rev - (0.0045 * -100.0 + -1.1)).abs() < 1e-4);
    let dead = m.torque_ff(0, &[0.0], &[0.1]);
    assert!(
        (dead - 0.0045 * 0.1).abs() < 1e-4,
        "no coulomb inside deadband"
    );
}

#[test]
fn corexy_effective_inertia_is_direction_dependent() {
    let m = DynamicsModel::from_toml_str(COREXY).unwrap();
    let x_move = m.torque_ff(0, &[1000.0, 1000.0], &[0.0, 0.0]);
    let y_move = m.torque_ff(0, &[1000.0, -1000.0], &[0.0, 0.0]);
    assert!((x_move - 20.0).abs() < 1e-3);
    assert!((y_move - 40.0).abs() < 1e-3);
}

#[test]
fn block_diagonal_stacks_independent_axes() {
    let x = DynamicsModel::from_toml_str(SCALAR).unwrap();
    let y = DynamicsModel::from_toml_str(SCALAR_Y).unwrap();
    let m = DynamicsModel::block_diagonal(vec![x, y]).unwrap();
    assert_eq!(m.n, 2);
    assert_eq!(m.axes, ["x", "y"]);
    let t0 = m.torque_ff(0, &[1000.0, 7777.0], &[100.0, 0.0]);
    let expect0 = 0.0123 * 1000.0 + 0.0045 * 100.0 + 1.2;
    assert!((t0 - expect0).abs() < 1e-3, "axis 0 ignores axis 1 accel");
    let t1 = m.torque_ff(1, &[9999.0, 1000.0], &[0.0, 100.0]);
    let expect1 = 0.050 * 1000.0 + 0.006 * 100.0 + 2.0;
    assert!((t1 - expect1).abs() < 1e-2, "axis 1 ignores axis 0 accel");
}

#[test]
fn block_diagonal_rejects_deadband_mismatch() {
    let x = DynamicsModel::from_toml_str(SCALAR).unwrap();
    let y = DynamicsModel::from_toml_str(
        &SCALAR_Y.replace("coulomb_deadband_mm_s = 0.5", "coulomb_deadband_mm_s = 0.9"),
    )
    .unwrap();
    assert!(matches!(
        DynamicsModel::block_diagonal(vec![x, y]),
        Err(ProfileError::Dim(_))
    ));
}

#[test]
fn block_diagonal_rejects_empty() {
    assert!(matches!(
        DynamicsModel::block_diagonal(vec![]),
        Err(ProfileError::Dim(_))
    ));
}

#[test]
fn rejects_each_invariant_violation() {
    let bad_version = SCALAR.replace("version = 1", "version = 2");
    assert!(matches!(
        DynamicsModel::from_toml_str(&bad_version),
        Err(ProfileError::Version(2))
    ));
    let bad_dim = SCALAR.replace("viscous = [0.0045]", "viscous = [0.0045, 1.0]");
    assert!(matches!(
        DynamicsModel::from_toml_str(&bad_dim),
        Err(ProfileError::Dim(_))
    ));
    let asym = COREXY.replace("[-0.010, 0.030]", "[-0.011, 0.030]");
    assert!(matches!(
        DynamicsModel::from_toml_str(&asym),
        Err(ProfileError::NotSymmetric)
    ));
    let not_pd = SCALAR.replace("mass = [[0.0123]]", "mass = [[-0.0123]]");
    assert!(matches!(
        DynamicsModel::from_toml_str(&not_pd),
        Err(ProfileError::NotPositiveDefinite)
    ));
    let nan = SCALAR.replace("viscous = [0.0045]", "viscous = [nan]");
    assert!(matches!(
        DynamicsModel::from_toml_str(&nan),
        Err(ProfileError::NotFinite(_))
    ));
    let nan_deadband = SCALAR.replace("coulomb_deadband_mm_s = 0.5", "coulomb_deadband_mm_s = nan");
    assert!(matches!(
        DynamicsModel::from_toml_str(&nan_deadband),
        Err(ProfileError::NotFinite(_))
    ));
    assert!(matches!(
        DynamicsModel::from_toml_str("not toml ["),
        Err(ProfileError::Parse(_))
    ));
}

#[test]
#[should_panic(expected = "non-finite torque FF")]
fn clamp_panics_on_nan() {
    let mut sat = 0u32;
    let _ = clamp_torque(f32::NAN, 300, &mut sat);
}

#[test]
#[should_panic(expected = "torque clamp limit must be positive")]
fn clamp_panics_on_nonpositive_limit() {
    let mut sat = 0u32;
    let _ = clamp_torque(0.0, 0, &mut sat);
}

#[test]
fn clamp_counts_saturation() {
    let mut sat = 0u32;
    assert_eq!(clamp_torque(50.0, 300, &mut sat), 50);
    assert_eq!(sat, 0);
    assert_eq!(clamp_torque(450.7, 300, &mut sat), 300);
    assert_eq!(clamp_torque(-450.7, 300, &mut sat), -300);
    assert_eq!(sat, 2);
}
