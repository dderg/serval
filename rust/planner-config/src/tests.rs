use super::*;

#[test]
fn cartesian_validate_accepts_zero_and_positive_corner_deviation() {
    let mut c = CartesianLimits::default();
    c.corner_deviation = 0.0;
    assert!(c.validate().is_ok());
    c.corner_deviation = 8.0;
    assert!(c.validate().is_ok());
}

#[test]
fn cartesian_validate_rejects_negative_or_nan_corner_deviation() {
    let mut c = CartesianLimits::default();
    c.corner_deviation = -1.0;
    assert!(c.validate().is_err());
    c.corner_deviation = f64::NAN;
    assert!(c.validate().is_err());
}

#[test]
fn effective_limits_without_overrides_are_the_config_base() {
    let cfg = PlannerConfig::default();
    let (v, a, corner_deviation) = cfg.effective_limits();
    assert_eq!(v, cfg.cartesian.max_velocity);
    assert_eq!(a, cfg.cartesian.max_accel);
    assert_eq!(corner_deviation, cfg.cartesian.corner_deviation);
}

#[test]
fn effective_limits_runtime_caps_clamp_but_never_raise() {
    let mut cfg = PlannerConfig::default();
    cfg.runtime_caps = RuntimeCaps {
        velocity: Some(50.0),
        accel: Some(10_000_000.0),
    };
    let (v, a, _) = cfg.effective_limits();
    assert_eq!(v, 50.0);
    assert_eq!(a, cfg.cartesian.max_accel);
}

#[test]
fn effective_limits_runtime_corner_deviation_replaces_the_base() {
    let mut cfg = PlannerConfig::default();
    cfg.runtime_corner_deviation = Some(0.01);
    assert_eq!(cfg.effective_limits().2, 0.01);
    cfg.runtime_corner_deviation = None;
    assert_eq!(cfg.effective_limits().2, cfg.cartesian.corner_deviation);
}

#[test]
fn default_config_chains_are_passthrough() {
    let c = PlannerConfig::default();
    let chains = c.post_processors.compile(&c.axis_registry).unwrap();
    assert_eq!(chains.n_axes(), 3);
    assert!(chains.chains.iter().all(|ch| ch.stages.is_empty()));
    assert!(chains.followers.is_empty());
}

#[test]
fn linear_pressure_advance_rejects_negative_or_non_finite_k() {
    let reg = AxisRegistry::default();
    for bad in [-0.01, f64::NAN, f64::INFINITY] {
        let decls = vec![PostProcessorDecl {
            name: "pa".into(),
            ty: "linear_pressure_advance".into(),
            params: vec![("k".into(), bad)],
        }];
        assert!(
            PostProcessorSet::try_new(&reg, &decls).is_err(),
            "k={bad} should be rejected at config build"
        );
    }
    let ok = vec![PostProcessorDecl {
        name: "pa".into(),
        ty: "linear_pressure_advance".into(),
        params: vec![("k".into(), 0.0)],
    }];
    assert!(
        PostProcessorSet::try_new(&reg, &ok).is_ok(),
        "k=0 is a valid no-op gain"
    );
}

fn decl(name: &str, follows: &[&str]) -> AxisDecl {
    AxisDecl {
        name: name.into(),
        follows: follows.iter().map(|s| s.to_string()).collect(),
        motors: vec![],
        post_processors: vec![],
    }
}

#[test]
fn registry_orders_spatial_then_followers() {
    let reg = AxisRegistry::try_new(vec![
        decl("e", &["x", "y", "z"]),
        decl("x", &[]),
        decl("y", &[]),
        decl("z", &[]),
    ])
    .unwrap();
    assert_eq!(reg.axis_index("x").unwrap(), 0);
    assert_eq!(reg.axis_index("e").unwrap(), 3);
    assert_eq!(
        reg.follower_words(),
        vec![geometry::FollowerWord {
            letter: b'E',
            axis_index: 3
        }]
    );
}

#[test]
fn registry_requires_spatial_axes() {
    let err = AxisRegistry::try_new(vec![decl("x", &[]), decl("y", &[])]).unwrap_err();
    assert!(matches!(err, AxisConfigError::MissingSpatialAxis { name } if name == "z"));
}

#[test]
fn registry_rejects_reserved_letters_and_long_names() {
    for bad in ["i", "j", "p", "q", "f", "g", "m", "n", "t", "ab"] {
        let mut decls = vec![decl("x", &[]), decl("y", &[]), decl("z", &[])];
        decls.push(decl(bad, &["x"]));
        assert!(
            AxisRegistry::try_new(decls).is_err(),
            "expected rejection: {bad}"
        );
    }
}

#[test]
fn follows_must_reference_declared_axes_and_spatial_cannot_follow() {
    let decls = vec![
        decl("x", &[]),
        decl("y", &[]),
        decl("z", &[]),
        decl("e", &["w"]),
    ];
    assert!(matches!(
        AxisRegistry::try_new(decls).unwrap_err(),
        AxisConfigError::UnknownFollowTarget { .. }
    ));
    let decls = vec![decl("x", &["y"]), decl("y", &[]), decl("z", &[])];
    assert!(matches!(
        AxisRegistry::try_new(decls).unwrap_err(),
        AxisConfigError::SpatialAxisCannotFollow { .. }
    ));
}

fn decl_with_motors(name: &str, follows: &[&str], motors: &[&str]) -> AxisDecl {
    AxisDecl {
        name: name.into(),
        follows: follows.iter().map(|s| s.to_string()).collect(),
        motors: motors.iter().map(|s| s.to_string()).collect(),
        post_processors: vec![],
    }
}

#[test]
fn axis_claimed_by_kinematics_and_motors_key_is_rejected() {
    let registry = AxisRegistry::try_new(vec![
        decl("x", &[]),
        decl("y", &[]),
        decl_with_motors("z", &[], &["m1"]),
    ])
    .unwrap();
    let kinematics_axes = ["x".to_string(), "y".to_string(), "z".to_string()];
    let err = registry
        .validate_motor_mapping(&kinematics_axes)
        .unwrap_err();
    assert!(matches!(err, AxisConfigError::MotorMappingDuplicate { axis } if axis == "z"));
}

#[test]
fn axis_with_neither_claim_nor_motors_is_rejected() {
    let registry = AxisRegistry::try_new(vec![
        decl("x", &[]),
        decl("y", &[]),
        decl("z", &[]),
        decl("e", &["x", "y", "z"]),
    ])
    .unwrap();
    let kinematics_axes = ["x".to_string(), "y".to_string(), "z".to_string()];
    let err = registry
        .validate_motor_mapping(&kinematics_axes)
        .unwrap_err();
    assert!(matches!(err, AxisConfigError::MotorMappingMissing { axis } if axis == "e"));
}

#[test]
fn kinematics_claim_of_undeclared_axis_is_rejected() {
    let registry =
        AxisRegistry::try_new(vec![decl("x", &[]), decl("y", &[]), decl("z", &[])]).unwrap();
    let kinematics_axes = [
        "x".to_string(),
        "y".to_string(),
        "z".to_string(),
        "w".to_string(),
    ];
    let err = registry
        .validate_motor_mapping(&kinematics_axes)
        .unwrap_err();
    assert!(matches!(err, AxisConfigError::UnknownClaimedAxis { axis } if axis == "w"));
}

#[test]
fn follower_with_own_motors_and_spatial_claims_pass() {
    let registry = AxisRegistry::try_new(vec![
        decl("x", &[]),
        decl("y", &[]),
        decl("z", &[]),
        decl_with_motors("e", &["x", "y", "z"], &["extruder_motor"]),
    ])
    .unwrap();
    let kinematics_axes = ["x".to_string(), "y".to_string(), "z".to_string()];
    assert!(registry.validate_motor_mapping(&kinematics_axes).is_ok());
}

fn pp(name: &str, ty: &str, params: &[(&str, f64)]) -> PostProcessorDecl {
    PostProcessorDecl {
        name: name.into(),
        ty: ty.into(),
        params: params.iter().map(|(k, v)| ((*k).to_string(), *v)).collect(),
    }
}

fn registry_with_e(e_post_processors: &[&str]) -> AxisRegistry {
    let mut decls: Vec<AxisDecl> = ["x", "y", "z"].iter().map(|n| decl(n, &[])).collect();
    let mut e = decl("e", &["x", "y", "z"]);
    e.post_processors = e_post_processors.iter().map(|s| (*s).to_string()).collect();
    decls.push(e);
    AxisRegistry::try_new(decls).unwrap()
}

#[test]
fn post_processor_unknown_type_rejected() {
    let registry = registry_with_e(&[]);
    let err = PostProcessorSet::try_new(
        &registry,
        &[pp("is", "zv_classic", &[("frequency_hz", 50.0)])],
    )
    .unwrap_err();
    assert!(err.to_string().contains("zv_classic"), "got: {err}");
}

#[test]
fn post_processor_duplicate_name_rejected() {
    let registry = registry_with_e(&[]);
    let err = PostProcessorSet::try_new(
        &registry,
        &[
            pp("is", "smooth_bell", &[("smooth_time", 0.01605)]),
            pp("is", "smooth_bell", &[("smooth_time", 0.02390625)]),
        ],
    )
    .unwrap_err();
    assert!(err.to_string().contains("duplicate"), "got: {err}");
}

#[test]
fn axis_referencing_undeclared_post_processor_rejected() {
    let registry = registry_with_e(&["ghost"]);
    let err = PostProcessorSet::try_new(&registry, &[]).unwrap_err();
    assert!(err.to_string().contains("ghost"), "got: {err}");
}

#[test]
fn two_kernels_on_one_axis_rejected_with_v1_message() {
    let registry = registry_with_e(&["is_a", "is_b"]);
    let err = PostProcessorSet::try_new(
        &registry,
        &[
            pp("is_a", "smooth_bell", &[("smooth_time", 0.01605)]),
            pp("is_b", "smooth_bell", &[("smooth_time", 0.02390625)]),
        ],
    )
    .unwrap_err();
    assert!(err.to_string().contains("v1"), "got: {err}");
}

#[test]
fn kernel_and_pa_on_follower_e_compiles() {
    let registry = registry_with_e(&["pa", "st"]);
    let set = PostProcessorSet::try_new(
        &registry,
        &[
            pp("pa", "linear_pressure_advance", &[("k", 0.04)]),
            pp("st", "smooth_bell", &[("smooth_time", 0.01605)]),
        ],
    )
    .unwrap();
    let chains = set.compile(&registry).unwrap();
    assert!(
        matches!(chains.chains[3].stages[0], trajectory::ChainStage::LinearPressureAdvance { k } if k == 0.04)
    );
    assert!(matches!(
        chains.chains[3].stages[1],
        trajectory::ChainStage::SmoothKernel(_)
    ));
    assert_eq!(chains.followers, vec![(3, vec![0, 1, 2])]);
}

#[test]
fn happy_path_compiles_pa_on_follower_e() {
    let registry = registry_with_e(&["pa"]);
    let set = PostProcessorSet::try_new(
        &registry,
        &[pp("pa", "linear_pressure_advance", &[("k", 0.04)])],
    )
    .unwrap();
    let chains = set.compile(&registry).unwrap();
    assert_eq!(chains.n_axes(), 4);
    assert!(
        matches!(chains.chains[3].stages[0], trajectory::ChainStage::LinearPressureAdvance { k } if k == 0.04)
    );
    assert_eq!(chains.followers, vec![(3, vec![0, 1, 2])]);
}

#[test]
fn set_param_updates_named_instance_and_recompile_reflects_it() {
    let registry = registry_with_e(&["pa"]);
    let mut set = PostProcessorSet::try_new(
        &registry,
        &[pp("pa", "linear_pressure_advance", &[("k", 0.04)])],
    )
    .unwrap();
    set.set_param("pa", "k", 0.07).unwrap();
    let chains = set.compile(&registry).unwrap();
    assert!(
        matches!(chains.chains[3].stages[0], trajectory::ChainStage::LinearPressureAdvance { k } if k == 0.07)
    );
    assert!(set.set_param("nope", "k", 1.0).is_err());
    assert!(set.set_param("pa", "frequency_hz", 1.0).is_err());
}

#[test]
fn post_processor_missing_required_param_rejected() {
    let registry = registry_with_e(&[]);
    let err = PostProcessorSet::try_new(&registry, &[pp("is", "smooth_bell", &[])]).unwrap_err();
    assert!(err.to_string().contains("smooth_time"), "got: {err}");
}
