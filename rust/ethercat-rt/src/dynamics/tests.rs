use super::*;

const SCALAR: &str = r#"
version = 6
axes = ["x"]
modes = ["x"]
frame = [[1.0]]
mass = [0.0123]
viscous = [0.0045]
coulomb = [1.2]
fit_rms_residual = [0.8]
"#;

const SCALAR_Y: &str = r#"
version = 6
axes = ["y"]
modes = ["y"]
frame = [[1.0]]
mass = [0.050]
viscous = [0.006]
coulomb = [2.0]
fit_rms_residual = [0.3]
"#;

const COREXY: &str = r#"
version = 6
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
    let tau = m.torque_ff(0, &[1000.0], &[100.0]);
    let expect = 0.0123 * 1000.0 + 0.0045 * 100.0 + 1.2;
    assert!((tau - expect).abs() < 1e-4, "{tau} vs {expect}");
}

#[test]
fn ff_lead_us_absent_defaults_to_zero() {
    let m = DynamicsModel::from_toml_str(SCALAR).unwrap();
    assert_eq!(m.ff_lead_ns(), vec![0]);
}

#[test]
fn ff_lead_us_present_broadcasts_to_every_slot() {
    let with_lead = format!("{COREXY}\nff_lead_us = 500.0\n");
    let m = DynamicsModel::from_toml_str(&with_lead).unwrap();
    assert_eq!(m.ff_lead_ns(), vec![500_000, 500_000]);
}

#[test]
fn ff_lead_us_accepts_fractional_microseconds() {
    let with_lead = format!("{SCALAR}\nff_lead_us = 100.25\n");
    let m = DynamicsModel::from_toml_str(&with_lead).unwrap();
    assert_eq!(m.ff_lead_ns(), vec![100_250]);
}

#[test]
fn ff_lead_us_rejects_out_of_range_values() {
    let negative = format!("{SCALAR}\nff_lead_us = -1.0\n");
    assert!(matches!(
        DynamicsModel::from_toml_str(&negative),
        Err(ProfileError::FfLeadOutOfRange(v)) if v == -1.0
    ));
    let too_high = format!("{SCALAR}\nff_lead_us = 10001.0\n");
    assert!(matches!(
        DynamicsModel::from_toml_str(&too_high),
        Err(ProfileError::FfLeadOutOfRange(v)) if v == 10001.0
    ));
}

#[test]
fn torque_ff_reverse_coulomb_is_symmetric_and_zero_at_rest() {
    let m = DynamicsModel::from_toml_str(SCALAR).unwrap();
    let rev = m.torque_ff(0, &[0.0], &[-100.0]);
    assert!((rev - (0.0045 * -100.0 - 1.2)).abs() < 1e-4, "{rev}");
    let at_rest = m.torque_ff(0, &[0.0], &[0.0]);
    assert!(at_rest.abs() < 1e-6, "no coulomb at exactly zero velocity");
}

#[test]
fn corexy_effective_inertia_is_direction_dependent() {
    let m = DynamicsModel::from_toml_str(COREXY).unwrap();
    let x_move = m.torque_ff(0, &[1000.0, 1000.0], &[0.0, 0.0]);
    let y_move = m.torque_ff(0, &[1000.0, -1000.0], &[0.0, 0.0]);
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
    let t0 = m.torque_ff(0, &[1000.0, 7777.0], &[100.0, 0.0]);
    let expect0 = 0.0123 * 1000.0 + 0.0045 * 100.0 + 1.2;
    assert!((t0 - expect0).abs() < 1e-3, "axis 0 ignores axis 1 accel");
    let t1 = m.torque_ff(1, &[9999.0, 1000.0], &[0.0, 100.0]);
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
    let bad_version = SCALAR.replace("version = 6", "version = 2");
    assert!(matches!(
        DynamicsModel::from_toml_str(&bad_version),
        Err(ProfileError::Version(2))
    ));
    let pre_split_removal = SCALAR.replace("version = 6", "version = 5");
    assert!(matches!(
        DynamicsModel::from_toml_str(&pre_split_removal),
        Err(ProfileError::Version(5))
    ));
    let with_pair_table =
        format!("{SCALAR}\n[[pair]]\nslots = [\"x\", \"x1\"]\ndirection_split = 0.1\n");
    assert!(matches!(
        DynamicsModel::from_toml_str(&with_pair_table),
        Err(ProfileError::PairSlot(_))
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
        &[],
        &[],
        0.0,
        &[],
    )
    .unwrap();
    assert_eq!(parts.n_slots, 2);
    assert_eq!(parts.n_modes, 2);
    assert_eq!(parts.axes, ["slot0", "slot1"]);
    assert_eq!(parts.modes, ["mode0", "mode1"]);
    let acc = [1000.0, -400.0];
    let vel = [100.0, -30.0];
    for slot in 0..2 {
        let a = toml.torque_ff(slot, &acc, &vel);
        let b = parts.torque_ff(slot, &acc, &vel);
        assert!((a - b).abs() < 1e-6, "slot {slot}: {a} vs {b}");
    }
}

#[test]
fn from_parts_rejects_each_invariant_violation() {
    let frame = [0.5, 0.5, 0.5, -0.5];
    let mode2 = [0.004, 0.004];
    assert!(matches!(
        DynamicsModel::from_parts(0, 0, &[], &[], &[], &[], &[], &[], &[], 0.0, &[]),
        Err(ProfileError::Dim(_))
    ));
    assert!(matches!(
        DynamicsModel::from_parts(
            2,
            2,
            &frame[..3],
            &[0.04, 0.08],
            &mode2,
            &mode2,
            &[],
            &[],
            &[],
            0.0,
            &[]
        ),
        Err(ProfileError::Dim(_))
    ));
    assert!(matches!(
        DynamicsModel::from_parts(
            2,
            2,
            &frame,
            &[0.04],
            &mode2,
            &mode2,
            &[],
            &[],
            &[],
            0.0,
            &[]
        ),
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
            &[],
            &[],
            &[],
            0.0,
            &[]
        ),
        Err(ProfileError::FrameRankDeficient)
    ));
    assert!(matches!(
        DynamicsModel::from_parts(
            2,
            2,
            &frame,
            &[0.0, 0.08],
            &mode2,
            &mode2,
            &[],
            &[],
            &[],
            0.0,
            &[]
        ),
        Err(ProfileError::NonPositive(_))
    ));
    assert!(matches!(
        DynamicsModel::from_parts(
            2,
            2,
            &frame,
            &[0.04, 0.08],
            &[f32::NAN, 0.004],
            &mode2,
            &[],
            &[],
            &[],
            0.0,
            &[]
        ),
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
version = 6
axes = ["a", "a1", "b", "b1"]
modes = ["x", "y"]
frame = [[0.25, 0.25, 0.25, 0.25], [0.25, 0.25, -0.25, -0.25]]
mass = [0.030, 0.060]
viscous = [0.002, 0.002]
coulomb = [1.0, 1.0]
fit_rms_residual = [0.5, 0.5, 0.5, 0.5]

[[pair]]
slots = ["a", "a1"]
direction_split = 0.2
"#;

#[test]
fn profile_pair_applies_the_total_belt_force_factor() {
    let m = DynamicsModel::from_toml_str(COREXY_AWD).unwrap();
    assert_eq!(m.n_slots, 4);
    assert_eq!(m.n_modes, 2);
    let acc = [1000.0, 1000.0, 1000.0, 1000.0];
    let vel = [0.0, 0.0, 0.0, 0.0];
    let tau0 = m.torque_ff(0, &acc, &vel);
    let tau1 = m.torque_ff(1, &acc, &vel);
    assert!((tau0 - 9.0).abs() < 1e-6, "first torque {tau0}");
    assert!((tau1 - 6.0).abs() < 1e-6, "second torque {tau1}");
    assert!((tau0 + tau1 - 15.0).abs() < 1e-6, "pair sum changed");
}

#[test]
fn opposite_pair_columns_preserve_generalized_force() {
    let pair = PairSpec {
        first: 0,
        second: 1,
        direction_split: 0.2,
    };
    let model = DynamicsModel::from_parts(
        2,
        1,
        &[0.5, -0.5],
        &[0.04],
        &[0.0],
        &[0.0],
        &[],
        &[],
        &[],
        0.0,
        &[pair],
    )
    .unwrap();
    let acc = [1000.0, -1000.0];
    let first = model.torque_ff(0, &acc, &[0.0, 0.0]);
    let second = model.torque_ff(1, &acc, &[0.0, 0.0]);
    assert!((first - 24.0).abs() < 1e-6, "first torque {first}");
    assert!((second + 16.0).abs() < 1e-6, "second torque {second}");
    assert!((0.5 * first - 0.5 * second - 20.0).abs() < 1e-6);
}

#[test]
fn pair_validation_rejects_invalid_contracts() {
    let frame = [0.5, 0.5, 0.5, 0.5];
    let vectors = [0.04, 0.08];
    let pair = |first, second, direction_split| PairSpec {
        first,
        second,
        direction_split,
    };
    for spec in [
        pair(0, 0, 0.1),
        pair(0, 2, 0.1),
        pair(0, 1, 0.5),
        pair(0, 1, -0.5),
        pair(0, 1, f32::NAN),
    ] {
        assert!(DynamicsModel::from_parts(
            2,
            1,
            &frame[..2],
            &vectors[..1],
            &[0.0],
            &[0.0],
            &[],
            &[],
            &[],
            0.0,
            &[spec]
        )
        .is_err());
    }
    assert!(matches!(
        DynamicsModel::from_parts(
            4,
            1,
            &[0.25; 4],
            &vectors[..1],
            &[0.0],
            &[0.0],
            &[],
            &[],
            &[],
            0.0,
            &[pair(0, 1, 0.1), pair(1, 2, 0.1)]
        ),
        Err(ProfileError::PairSlot(_))
    ));
    assert!(matches!(
        DynamicsModel::from_parts(
            2,
            1,
            &[0.5, 0.500_001],
            &vectors[..1],
            &[0.0],
            &[0.0],
            &[],
            &[],
            &[],
            0.0,
            &[pair(0, 1, 0.1)]
        ),
        Err(ProfileError::PairNotParallel(0))
    ));
}

#[test]
fn profile_rejects_pair_orientation_and_global_split() {
    let orientation = COREXY_AWD.replace(
        "direction_split = 0.2",
        "direction_split = 0.2\norientation = 1",
    );
    assert!(matches!(
        DynamicsModel::from_toml_str(&orientation),
        Err(ProfileError::Parse(_))
    ));
    let global = SCALAR.replace("mass =", "direction_split = 0.1\nmass =");
    assert!(matches!(
        DynamicsModel::from_toml_str(&global),
        Err(ProfileError::ForbiddenField("direction_split"))
    ));
    let global_orientation = SCALAR.replace("mass =", "orientation = 1\nmass =");
    assert!(matches!(
        DynamicsModel::from_toml_str(&global_orientation),
        Err(ProfileError::ForbiddenField("orientation"))
    ));
}

#[test]
fn profile_accepts_refinement_provenance() {
    let refined = SCALAR.replace(
        "mass =",
        "refined_source = \"baseline.toml\"\nrefined_term = \"mass\"\nrefined_delta_mass = 0.001\nrefined_delta_direction_split_a = -0.02\nmass =",
    );
    DynamicsModel::from_toml_str(&refined).expect("refinement provenance must be ignored");
}

#[test]
fn profile_rejects_duplicate_axes_before_pair_resolution() {
    let duplicate = COREXY_AWD.replace(
        "axes = [\"a\", \"a1\", \"b\", \"b1\"]",
        "axes = [\"a\", \"a\", \"b\", \"b1\"]",
    );
    assert!(matches!(
        DynamicsModel::from_toml_str(&duplicate),
        Err(ProfileError::DuplicateAxis(axis)) if axis == "a"
    ));
}

#[test]
fn pair_rejects_zero_first_frame_column() {
    assert!(matches!(
        DynamicsModel::from_parts(
            3,
            1,
            &[0.0, 0.0, 1.0],
            &[0.04],
            &[0.0],
            &[0.0],
            &[],
            &[],
            &[],
            0.0,
            &[PairSpec {
                first: 0,
                second: 1,
                direction_split: 0.1,
            }]
        ),
        Err(ProfileError::PairFirstColumnZero(0))
    ));
}

#[test]
fn buzz_path_suppresses_pair_correction() {
    let paired = DynamicsModel::from_toml_str(COREXY_AWD).unwrap();
    let unpaired = DynamicsModel::from_toml_str(&COREXY_AWD.replace(
        "[[pair]]\nslots = [\"a\", \"a1\"]\ndirection_split = 0.2",
        "",
    ))
    .unwrap();
    let acc = [1000.0; 4];
    let vel = [0.0; 4];
    assert_eq!(
        paired.torque_ff_without_coulomb(0, &acc, &vel),
        unpaired.torque_ff_without_coulomb(0, &acc, &vel)
    );
}

#[test]
fn torque_ff_without_coulomb_keeps_the_linear_terms() {
    let m = DynamicsModel::from_toml_str(SCALAR).unwrap();
    let linear = m.torque_ff_without_coulomb(0, &[1000.0], &[100.0]);
    let expect = 0.0123 * 1000.0 + 0.0045 * 100.0;
    assert!((linear - expect).abs() < 1e-4, "{linear} vs {expect}");
    let full = m.torque_ff(0, &[1000.0], &[100.0]);
    assert!((full - (expect + 1.2)).abs() < 1e-4, "{full}");
}

const COREXY_V7: &str = r#"
version = 7
axes = ["a", "b"]
modes = ["x", "y"]
frame = [[0.5, 0.5], [0.5, -0.5]]
mass = [0.040, 0.080]
viscous = [0.004, 0.004]
coulomb = [1.0, 1.0]
compliance = [7.0e-6, 1.76e-5]
fit_rms_residual = [0.5, 0.5]
"#;

#[test]
fn v7_compliance_is_retained_per_mode() {
    let m = DynamicsModel::from_toml_str(COREXY_V7).unwrap();
    assert!((f64::from(m.compliance(0)) - 7.0e-6).abs() < 1e-12);
    assert!((f64::from(m.compliance(1)) - 1.76e-5).abs() < 1e-12);
}

#[test]
fn v6_defaults_every_mode_compliance_to_zero() {
    let m = DynamicsModel::from_toml_str(COREXY).unwrap();
    assert_eq!(m.compliance(0), 0.0);
    assert_eq!(m.compliance(1), 0.0);
}

#[test]
fn v7_all_zero_compliance_is_accepted() {
    let zeroed = COREXY_V7.replace("compliance = [7.0e-6, 1.76e-5]", "compliance = [0.0, 0.0]");
    let m = DynamicsModel::from_toml_str(&zeroed).unwrap();
    assert_eq!(m.compliance(0), 0.0);
    assert_eq!(m.compliance(1), 0.0);
}

#[test]
fn compliance_on_v6_profile_is_rejected() {
    let v6 = COREXY_V7.replace("version = 7", "version = 6");
    assert!(matches!(
        DynamicsModel::from_toml_str(&v6),
        Err(ProfileError::ForbiddenField(_))
    ));
}

#[test]
fn compliance_validation_rejects_bad_values() {
    for bad in [
        "[7.0e-6]",
        "[-1.0e-6, 1.0e-6]",
        "[nan, 1.0e-6]",
        "[1.0e-2, 1.0e-6]",
        // 1e-4 s² is ω_b/2π ≈ 15.9 Hz — softer than the documented 20 Hz
        // floor, so the ceiling must reject it.
        "[1.0e-4, 1.0e-6]",
    ] {
        let s = COREXY_V7.replace("[7.0e-6, 1.76e-5]", bad);
        let r = DynamicsModel::from_toml_str(&s);
        assert!(r.is_err(), "compliance {bad} must be rejected");
    }
}

#[test]
fn compliance_ceiling_admits_a_stiff_belt_just_inside_20_hz() {
    // 6.0e-5 s² is ≈ 20.5 Hz, just stiffer than the floor: accepted.
    let s = COREXY_V7.replace("[7.0e-6, 1.76e-5]", "[6.0e-5, 1.0e-6]");
    assert!(DynamicsModel::from_toml_str(&s).is_ok());
}

#[test]
fn block_diagonal_concatenates_compliance() {
    let x = format!(
        "{}\ncompliance = [1.0e-5]\n",
        SCALAR.replace("version = 6", "version = 7")
    );
    let y = DynamicsModel::from_toml_str(SCALAR_Y).unwrap();
    let x = DynamicsModel::from_toml_str(&x).unwrap();
    let node = DynamicsModel::block_diagonal(vec![x, y]).unwrap();
    assert!((f64::from(node.compliance(0)) - 1.0e-5).abs() < 1e-12);
    assert_eq!(node.compliance(1), 0.0, "y profile has no compliance term");
}

const COREXY_V8: &str = r#"
version = 8
axes = ["a", "b"]
modes = ["x", "y"]
frame = [[0.5, 0.5], [0.5, -0.5]]
mass = [0.040, 0.080]
viscous = [0.004, 0.004]
coulomb = [1.0, 1.0]
compliance = [7.0e-6, 1.76e-5]
pin_mass = [0.012, 0.0]
pin_zeta = [0.05, 0.0]
pin_lead_us = 300.0
fit_rms_residual = [0.5, 0.5]
"#;

#[test]
fn v8_pin_fields_parse_and_expose_per_mode_state() {
    let m = DynamicsModel::from_toml_str(COREXY_V8).unwrap();
    assert!((f64::from(m.compliance(0)) - 7.0e-6).abs() < 1e-12);
    assert_eq!(m.pin_mass, [0.012, 0.0]);
    assert_eq!(m.pin_zeta, [0.05, 0.0]);
    assert!(m.pin_active(0), "mode 0 has nonzero pin mass");
    assert!(!m.pin_active(1), "mode 1 pin mass is zero");
    assert!(!m.pin_active(9), "out-of-range mode is inactive");
    assert_eq!(m.pin_lead_ns(), vec![300_000, 300_000]);
}

#[test]
fn v8_pin_fields_default_to_disabled_when_absent() {
    let bare = COREXY_V8
        .lines()
        .filter(|l| !l.starts_with("pin_"))
        .collect::<Vec<_>>()
        .join("\n");
    let m = DynamicsModel::from_toml_str(&bare).unwrap();
    assert_eq!(m.pin_mass, [0.0, 0.0]);
    assert_eq!(m.pin_zeta, [0.0, 0.0]);
    assert!(!m.pin_active(0));
    assert_eq!(m.pin_lead_ns(), vec![0, 0]);
}

#[test]
fn pin_mass_without_compliance_is_rejected() {
    let s = COREXY_V8.replace(
        "compliance = [7.0e-6, 1.76e-5]",
        "compliance = [0.0, 1.76e-5]",
    );
    assert!(matches!(
        DynamicsModel::from_toml_str(&s),
        Err(ProfileError::PinNeedsCompliance(0))
    ));
}

#[test]
fn pin_fields_on_version_7_are_rejected() {
    let s = COREXY_V8.replace("version = 8", "version = 7");
    assert!(matches!(
        DynamicsModel::from_toml_str(&s),
        Err(ProfileError::ForbiddenField(_))
    ));
}

#[test]
fn pin_mass_present_without_pin_zeta_is_dim_error() {
    let s = COREXY_V8
        .lines()
        .filter(|l| !l.starts_with("pin_zeta"))
        .collect::<Vec<_>>()
        .join("\n");
    assert!(matches!(
        DynamicsModel::from_toml_str(&s),
        Err(ProfileError::Dim(_))
    ));
}

#[test]
fn pin_validation_rejects_out_of_range_values() {
    // No upper zeta cap: any finite zeta >= 0 is legal (the endpoint
    // evaluates under-, critically-, and overdamped regimes). The hard
    // invariants are sign and finiteness only.
    for ok in ["0.8", "1.0", "1.4", "10.0"] {
        let toml = COREXY_V8.replace("pin_zeta = [0.05, 0.0]", &format!("pin_zeta = [{ok}, 0.0]"));
        assert!(
            DynamicsModel::from_toml_str(&toml).is_ok(),
            "zeta {ok} must be accepted"
        );
    }
    let zeta_neg = COREXY_V8.replace("pin_zeta = [0.05, 0.0]", "pin_zeta = [-0.1, 0.0]");
    assert!(matches!(
        DynamicsModel::from_toml_str(&zeta_neg),
        Err(ProfileError::PinZetaOutOfRange(_))
    ));
    let zeta_nan = COREXY_V8.replace("pin_zeta = [0.05, 0.0]", "pin_zeta = [nan, 0.0]");
    assert!(matches!(
        DynamicsModel::from_toml_str(&zeta_nan),
        Err(ProfileError::PinZetaOutOfRange(_)) | Err(ProfileError::Parse(_))
    ));
    let mass_neg = COREXY_V8.replace("pin_mass = [0.012, 0.0]", "pin_mass = [-0.1, 0.0]");
    assert!(matches!(
        DynamicsModel::from_toml_str(&mass_neg),
        Err(ProfileError::NonPositive(_))
    ));
    let lead_hi = COREXY_V8.replace("pin_lead_us = 300.0", "pin_lead_us = 20000.0");
    assert!(matches!(
        DynamicsModel::from_toml_str(&lead_hi),
        Err(ProfileError::PinLeadOutOfRange(_))
    ));
}

#[test]
fn block_diagonal_concatenates_pin_fields() {
    let x = format!(
        "{}\ncompliance = [1.0e-5]\npin_mass = [0.02]\npin_zeta = [0.04]\npin_lead_us = 120.0\n",
        SCALAR.replace("version = 6", "version = 8")
    );
    let x = DynamicsModel::from_toml_str(&x).unwrap();
    let y = DynamicsModel::from_toml_str(SCALAR_Y).unwrap();
    let node = DynamicsModel::block_diagonal(vec![x, y]).unwrap();
    assert_eq!(node.pin_mass, [0.02, 0.0]);
    assert_eq!(node.pin_zeta, [0.04, 0.0]);
    assert!(node.pin_active(0));
    assert!(!node.pin_active(1));
    assert_eq!(node.pin_lead_ns(), vec![120_000, 0]);
}

#[test]
fn v6_and_v7_fixtures_still_report_no_pin() {
    let v6 = DynamicsModel::from_toml_str(COREXY).unwrap();
    assert!(!v6.pin_active(0));
    assert_eq!(v6.pin_lead_ns(), vec![0, 0]);
    let v7 = DynamicsModel::from_toml_str(COREXY_V7).unwrap();
    assert!(!v7.pin_active(0));
    assert_eq!(v7.pin_mass, [0.0, 0.0]);
}

#[test]
fn pinned_mode_retains_its_raw_compliance() {
    // The pin oscillator reads ω_b = 1/√compliance for the mode it holds, so
    // pinning must not consume or zero the per-mode compliance it was
    // derived from.
    let pinned = DynamicsModel::from_toml_str(COREXY_V8).unwrap();
    assert!(pinned.pin_active(0));
    assert!((f64::from(pinned.compliance(0)) - 7.0e-6).abs() < 1e-10);
    assert!((f64::from(pinned.compliance(1)) - 1.76e-5).abs() < 1e-10);
    assert_eq!(pinned.frame_row(0), &[0.5, 0.5]);
}
