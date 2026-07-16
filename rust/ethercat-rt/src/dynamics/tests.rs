use super::*;

const SCALAR: &str = r#"
version = 4
axes = ["x"]
modes = ["x"]
frame = [[1.0]]
mass = [0.0123]
viscous = [0.0045]
coulomb = [1.2]
fit_rms_residual = [0.8]
"#;

const SCALAR_Y: &str = r#"
version = 4
axes = ["y"]
modes = ["y"]
frame = [[1.0]]
mass = [0.050]
viscous = [0.006]
coulomb = [2.0]
fit_rms_residual = [0.3]
"#;

const COREXY: &str = r#"
version = 4
axes = ["a", "b"]
modes = ["x", "y"]
frame = [[0.5, 0.5], [0.5, -0.5]]
mass = [0.040, 0.080]
viscous = [0.004, 0.004]
coulomb = [1.0, 1.0]
fit_rms_residual = [0.5, 0.5]
"#;

#[test]
fn parses_scalar_profile() {
    let m = DynamicsModel::from_toml_str(SCALAR).unwrap();
    assert_eq!(m.n_slots, 1);
    assert_eq!(m.n_modes, 1);
    assert_eq!(m.axes, ["x"]);
    assert_eq!(m.modes, ["x"]);
}

#[test]
fn torque_ff_scalar() {
    let m = DynamicsModel::from_toml_str(SCALAR).unwrap();
    let tau = m.torque_ff(0, &[1000.0], &[100.0], &[0.0]);
    let expect = 0.0123 * 1000.0 + 0.0045 * 100.0 + 1.2;
    assert!((tau - expect).abs() < 1e-4, "{tau} vs {expect}");
}

#[test]
fn torque_ff_reverse_coulomb_is_symmetric_and_zero_at_rest() {
    let m = DynamicsModel::from_toml_str(SCALAR).unwrap();
    let rev = m.torque_ff(0, &[0.0], &[-100.0], &[0.0]);
    assert!((rev - (0.0045 * -100.0 - 1.2)).abs() < 1e-4, "{rev}");
    let at_rest = m.torque_ff(0, &[0.0], &[0.0], &[0.0]);
    assert!(at_rest.abs() < 1e-6, "no coulomb at exactly zero velocity");
}

#[test]
fn corexy_effective_inertia_is_direction_dependent() {
    let m = DynamicsModel::from_toml_str(COREXY).unwrap();
    let x_move = m.torque_ff(0, &[1000.0, 1000.0], &[0.0, 0.0], &[0.0, 0.0]);
    let y_move = m.torque_ff(0, &[1000.0, -1000.0], &[0.0, 0.0], &[0.0, 0.0]);
    assert!((x_move - 20.0).abs() < 1e-3, "{x_move}");
    assert!((y_move - 40.0).abs() < 1e-3, "{y_move}");
}

#[test]
fn block_diagonal_stacks_independent_axes() {
    let x = DynamicsModel::from_toml_str(SCALAR).unwrap();
    let y = DynamicsModel::from_toml_str(SCALAR_Y).unwrap();
    let m = DynamicsModel::block_diagonal(vec![x, y]).unwrap();
    assert_eq!(m.n_slots, 2);
    assert_eq!(m.n_modes, 2);
    assert_eq!(m.axes, ["x", "y"]);
    assert_eq!(m.modes, ["x", "y"]);
    let t0 = m.torque_ff(0, &[1000.0, 7777.0], &[100.0, 0.0], &[0.0, 0.0]);
    let expect0 = 0.0123 * 1000.0 + 0.0045 * 100.0 + 1.2;
    assert!((t0 - expect0).abs() < 1e-3, "axis 0 ignores axis 1 accel");
    let t1 = m.torque_ff(1, &[9999.0, 1000.0], &[0.0, 100.0], &[0.0, 0.0]);
    let expect1 = 0.050 * 1000.0 + 0.006 * 100.0 + 2.0;
    assert!((t1 - expect1).abs() < 1e-2, "axis 1 ignores axis 0 accel");
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
    let bad_version = SCALAR.replace("version = 4", "version = 2");
    assert!(matches!(
        DynamicsModel::from_toml_str(&bad_version),
        Err(ProfileError::Version(2))
    ));
    let bad_width = COREXY.replace("[0.5, 0.5]", "[0.5, 0.5, 0.5]");
    assert!(matches!(
        DynamicsModel::from_toml_str(&bad_width),
        Err(ProfileError::Dim(_))
    ));
    let too_many_modes = SCALAR
        .replace("modes = [\"x\"]", "modes = [\"x\", \"y\"]")
        .replace("frame = [[1.0]]", "frame = [[1.0], [1.0]]")
        .replace("mass = [0.0123]", "mass = [0.0123, 0.0123]")
        .replace("viscous = [0.0045]", "viscous = [0.0045, 0.0045]")
        .replace("coulomb = [1.2]", "coulomb = [1.2, 1.2]");
    assert!(matches!(
        DynamicsModel::from_toml_str(&too_many_modes),
        Err(ProfileError::Dim(_))
    ));
    let zero_row = COREXY.replace("[0.5, 0.5]", "[0.0, 0.0]");
    assert!(matches!(
        DynamicsModel::from_toml_str(&zero_row),
        Err(ProfileError::ZeroFrameRow(0))
    ));
    let rank_deficient = COREXY.replace("[0.5, -0.5]", "[0.5, 0.5]");
    assert!(matches!(
        DynamicsModel::from_toml_str(&rank_deficient),
        Err(ProfileError::FrameRankDeficient)
    ));
    let bad_mass = SCALAR.replace("mass = [0.0123]", "mass = [0.0]");
    assert!(matches!(
        DynamicsModel::from_toml_str(&bad_mass),
        Err(ProfileError::NonPositive(_))
    ));
    let neg_viscous = SCALAR.replace("viscous = [0.0045]", "viscous = [-0.0045]");
    assert!(matches!(
        DynamicsModel::from_toml_str(&neg_viscous),
        Err(ProfileError::NonPositive(_))
    ));
    let neg_coulomb = SCALAR.replace("coulomb = [1.2]", "coulomb = [-1.2]");
    assert!(matches!(
        DynamicsModel::from_toml_str(&neg_coulomb),
        Err(ProfileError::NonPositive(_))
    ));
    let nan = SCALAR.replace("viscous = [0.0045]", "viscous = [nan]");
    assert!(matches!(
        DynamicsModel::from_toml_str(&nan),
        Err(ProfileError::NotFinite(_))
    ));
    assert!(matches!(
        DynamicsModel::from_toml_str("not toml ["),
        Err(ProfileError::Parse(_))
    ));
}

#[test]
fn from_parts_agrees_with_toml_parse() {
    let toml = DynamicsModel::from_toml_str(COREXY).unwrap();
    let parts = DynamicsModel::from_parts(
        2,
        2,
        &[0.5, 0.5, 0.5, -0.5],
        &[0.040, 0.080],
        &[0.004, 0.004],
        &[1.0, 1.0],
        &[],
    )
    .unwrap();
    assert_eq!(parts.n_slots, 2);
    assert_eq!(parts.n_modes, 2);
    assert_eq!(parts.axes, ["slot0", "slot1"]);
    assert_eq!(parts.modes, ["mode0", "mode1"]);
    let acc = [1000.0, -400.0];
    let vel = [100.0, -30.0];
    let pos = [0.0, 0.0];
    for slot in 0..2 {
        let a = toml.torque_ff(slot, &acc, &vel, &pos);
        let b = parts.torque_ff(slot, &acc, &vel, &pos);
        assert!((a - b).abs() < 1e-6, "slot {slot}: {a} vs {b}");
    }
}

#[test]
fn from_parts_rejects_each_invariant_violation() {
    let frame = [0.5, 0.5, 0.5, -0.5];
    let mode2 = [0.004, 0.004];
    assert!(matches!(
        DynamicsModel::from_parts(0, 0, &[], &[], &[], &[], &[]),
        Err(ProfileError::Dim(_))
    ));
    assert!(matches!(
        DynamicsModel::from_parts(2, 2, &frame[..3], &[0.04, 0.08], &mode2, &mode2, &[]),
        Err(ProfileError::Dim(_))
    ));
    assert!(matches!(
        DynamicsModel::from_parts(2, 2, &frame, &[0.04], &mode2, &mode2, &[]),
        Err(ProfileError::Dim(_))
    ));
    assert!(matches!(
        DynamicsModel::from_parts(
            2,
            2,
            &[0.5, 0.5, 0.5, 0.5],
            &[0.04, 0.08],
            &mode2,
            &mode2,
            &[]
        ),
        Err(ProfileError::FrameRankDeficient)
    ));
    assert!(matches!(
        DynamicsModel::from_parts(2, 2, &frame, &[0.0, 0.08], &mode2, &mode2, &[]),
        Err(ProfileError::NonPositive(_))
    ));
    assert!(matches!(
        DynamicsModel::from_parts(2, 2, &frame, &[0.04, 0.08], &[f32::NAN, 0.004], &mode2, &[]),
        Err(ProfileError::NotFinite(_))
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

const COREXY_AWD: &str = r#"
version = 4
axes = ["a", "a1", "b", "b1"]
modes = ["x", "y"]
frame = [[0.25, 0.25, 0.25, 0.25], [0.25, 0.25, -0.25, -0.25]]
mass = [0.030, 0.060]
viscous = [0.002, 0.002]
coulomb = [1.0, 1.0]
fit_rms_residual = [0.5, 0.5, 0.5, 0.5]
"#;

#[test]
fn corexy_awd_pair_slots_share_the_load() {
    let m = DynamicsModel::from_toml_str(COREXY_AWD).unwrap();
    assert_eq!(m.n_slots, 4);
    assert_eq!(m.n_modes, 2);
    let acc = [1000.0, 1000.0, 1000.0, 1000.0];
    let vel = [0.0, 0.0, 0.0, 0.0];
    let pos = [0.0, 0.0, 0.0, 0.0];
    let tau0 = m.torque_ff(0, &acc, &vel, &pos);
    let tau1 = m.torque_ff(1, &acc, &vel, &pos);
    assert!((tau0 - tau1).abs() < 1e-6, "pair drives share the load");
}

#[test]
fn torque_ff_without_coulomb_keeps_the_linear_terms() {
    let m = DynamicsModel::from_toml_str(SCALAR).unwrap();
    let linear = m.torque_ff_without_coulomb(0, &[1000.0], &[100.0], &[0.0]);
    let expect = 0.0123 * 1000.0 + 0.0045 * 100.0;
    assert!((linear - expect).abs() < 1e-4, "{linear} vs {expect}");
    let full = m.torque_ff(0, &[1000.0], &[100.0], &[0.0]);
    assert!((full - (expect + 1.2)).abs() < 1e-4, "{full}");
}

const AWD_PAIR: &str = r#"
version = 4
axes = ["a", "a1", "b", "b1"]
modes = ["x", "y"]
frame = [[0.25, -0.25, -0.25, -0.25], [0.25, -0.25, 0.25, 0.25]]
mass = [0.030, 0.060]
viscous = [0.002, 0.002]
coulomb = [1.0, 1.0]
fit_rms_residual = [0.5, 0.5, 0.5, 0.5]

[[pair]]
slots = ["a", "a1"]
belt_position_split = [0.1, 0.01]
"#;

fn awd_pair_model() -> DynamicsModel {
    let mut m = DynamicsModel::from_toml_str(AWD_PAIR).unwrap();
    m.bind_drive_signs(&[1.0, -1.0, -1.0, -1.0]);
    m
}

#[test]
fn pair_lambda_is_derived_from_the_frame() {
    let m = DynamicsModel::from_toml_str(AWD_PAIR).unwrap();
    assert_eq!(m.pairs.len(), 1);
    assert_eq!(m.pairs[0].first, 0);
    assert_eq!(m.pairs[0].second, 1);
    assert_eq!(m.pairs[0].lambda, -1.0);
}

// frame columns 0 and 1 are anti-parallel (λ = −1); signs s = [1, −1, −1, −1].
// acc = [4,0,0,0] gives a_mode = [1, 1]; vel = 0 kills viscous and coulomb.
// belt inertial share g^I = 0.25·(m0+m1) = 0.0225; belt_sign = s0 + λ·s1 = 2;
// F_belt^I = 0.045; p_belt = s0·pos0 = 10; coeff = 0.1 + 0.01·10 = 0.2;
// D̂ = 0.009; both slots receive +D̂/2 = +0.0045.
#[test]
fn pair_differential_is_split_antisymmetrically() {
    let m = awd_pair_model();
    let acc = [4.0, 0.0, 0.0, 0.0];
    let vel = [0.0, 0.0, 0.0, 0.0];
    let pos = [10.0, 10.0, 0.0, 0.0];

    let base0 = 0.25 * 0.030 + 0.25 * 0.060;
    let base1 = -0.25 * 0.030 - 0.25 * 0.060;
    let delta = 0.0045f32;

    let tau0 = m.torque_ff(0, &acc, &vel, &pos);
    let tau1 = m.torque_ff(1, &acc, &vel, &pos);
    assert!((tau0 - (base0 + delta)).abs() < 1e-6, "tau0 {tau0}");
    assert!((tau1 - (base1 + delta)).abs() < 1e-6, "tau1 {tau1}");

    // Belt-frame pair sum (s_i·τ_i) is unchanged by the differential.
    let belt_sum = 1.0 * tau0 + (-1.0) * tau1;
    let base_belt_sum = 1.0 * base0 + (-1.0) * base1;
    assert!((belt_sum - base_belt_sum).abs() < 1e-6, "{belt_sum}");
}

// A coulomb-free stroke isolates the differential: the full eval and the
// no-coulomb eval differ only by the differential (coulomb is zero at v = 0),
// so zero split weights must make the two evals identical for every slot.
#[test]
fn zero_weight_pair_differential_vanishes() {
    let zeroed = AWD_PAIR.replace(
        "belt_position_split = [0.1, 0.01]",
        "belt_position_split = [0.0, 0.0]",
    );
    let mut paired = DynamicsModel::from_toml_str(&zeroed).unwrap();
    paired.bind_drive_signs(&[1.0, -1.0, -1.0, -1.0]);
    let acc = [4.0, 0.0, 0.0, 0.0];
    let vel = [0.0, 0.0, 0.0, 0.0];
    let pos = [10.0, 10.0, 0.0, 0.0];
    for slot in 0..4 {
        let full = paired.torque_ff(slot, &acc, &vel, &pos);
        let without = paired.torque_ff_without_coulomb(slot, &acc, &vel, &pos);
        assert!(
            (full - without).abs() < 1e-6,
            "zero-weight differential must vanish, slot {slot}"
        );
    }
}

#[test]
fn no_coulomb_variant_drops_the_whole_differential() {
    let m = awd_pair_model();
    let acc = [4.0, 0.0, 0.0, 0.0];
    let vel = [0.0, 0.0, 0.0, 0.0];
    let pos = [10.0, 10.0, 0.0, 0.0];
    let base0 = 0.25 * 0.030 + 0.25 * 0.060;
    let without = m.torque_ff_without_coulomb(0, &acc, &vel, &pos);
    assert!(
        (without - base0).abs() < 1e-6,
        "buzz variant must omit the pair differential: {without}"
    );
}

#[test]
#[should_panic(expected = "drive signs were bound")]
fn pair_eval_without_bound_signs_panics() {
    let m = DynamicsModel::from_toml_str(AWD_PAIR).unwrap();
    let _ = m.torque_ff(0, &[4.0, 0.0, 0.0, 0.0], &[0.0; 4], &[0.0; 4]);
}

#[test]
fn pair_validation_rejections() {
    let unknown = AWD_PAIR.replace("slots = [\"a\", \"a1\"]", "slots = [\"a\", \"zz\"]");
    assert!(matches!(
        DynamicsModel::from_toml_str(&unknown),
        Err(ProfileError::PairSlot(_))
    ));
    let same = AWD_PAIR.replace("slots = [\"a\", \"a1\"]", "slots = [\"a\", \"a\"]");
    assert!(matches!(
        DynamicsModel::from_toml_str(&same),
        Err(ProfileError::PairSlot(_))
    ));
    // slots "a" and "b" are not parallel columns.
    let not_parallel = AWD_PAIR.replace("slots = [\"a\", \"a1\"]", "slots = [\"a\", \"b\"]");
    assert!(matches!(
        DynamicsModel::from_toml_str(&not_parallel),
        Err(ProfileError::PairNotParallel(0))
    ));
    let nan_w = AWD_PAIR.replace(
        "belt_position_split = [0.1, 0.01]",
        "belt_position_split = [nan, 0.01]",
    );
    assert!(matches!(
        DynamicsModel::from_toml_str(&nan_w),
        Err(ProfileError::NotFinite(_))
    ));
    let reused = r#"
version = 4
axes = ["a", "a1", "b", "b1"]
modes = ["x", "y"]
frame = [[0.25, -0.25, -0.25, -0.25], [0.25, -0.25, 0.25, 0.25]]
mass = [0.030, 0.060]
viscous = [0.002, 0.002]
coulomb = [1.0, 1.0]

[[pair]]
slots = ["a", "a1"]
belt_position_split = [0.1, 0.0]

[[pair]]
slots = ["a1", "b"]
belt_position_split = [0.1, 0.0]
"#;
    assert!(matches!(
        DynamicsModel::from_toml_str(reused),
        Err(ProfileError::PairSlot(_))
    ));
}

#[test]
fn from_parts_carries_pairs_and_derives_lambda() {
    let frame = [0.25, -0.25, -0.25, -0.25, 0.25, -0.25, 0.25, 0.25];
    let pair = PairSpec {
        first: 0,
        second: 1,
        w: [0.1, 0.01],
    };
    let mut m = DynamicsModel::from_parts(
        4,
        2,
        &frame,
        &[0.030, 0.060],
        &[0.002, 0.002],
        &[1.0, 1.0],
        &[pair],
    )
    .unwrap();
    m.bind_drive_signs(&[1.0, -1.0, -1.0, -1.0]);
    let tau0 = m.torque_ff(0, &[4.0, 0.0, 0.0, 0.0], &[0.0; 4], &[10.0, 10.0, 0.0, 0.0]);
    let base0 = 0.25 * 0.030 + 0.25 * 0.060;
    assert!((tau0 - (base0 + 0.0045)).abs() < 1e-6, "{tau0}");
}
