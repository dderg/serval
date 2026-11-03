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
    cfg.runtime.set_velocity_cap(Some(50.0)).unwrap();
    cfg.runtime.set_accel_cap(Some(10_000_000.0)).unwrap();
    let (v, a, _) = cfg.effective_limits();
    assert_eq!(v, 50.0);
    assert_eq!(a, cfg.cartesian.max_accel);
}

#[test]
fn effective_limits_runtime_corner_deviation_replaces_the_base() {
    let mut cfg = PlannerConfig::default();
    cfg.runtime.set_corner_deviation(Some(0.01)).unwrap();
    assert_eq!(cfg.effective_limits().2, 0.01);
    cfg.runtime.set_corner_deviation(None).unwrap();
    assert_eq!(cfg.effective_limits().2, cfg.cartesian.corner_deviation);
}

#[test]
fn runtime_caps_refuse_non_positive_or_non_finite_values_and_keep_the_old_cap() {
    let mut runtime = RuntimeLimits::default();
    runtime.set_accel_cap(Some(500.0)).unwrap();
    for bad in [0.0, -1.0, f64::NAN, f64::INFINITY] {
        assert_eq!(
            runtime.set_accel_cap(Some(bad)),
            Err(RuntimeLimitError::NotPositive { name: "accel" })
        );
        assert_eq!(
            runtime.set_velocity_cap(Some(bad)),
            Err(RuntimeLimitError::NotPositive { name: "velocity" })
        );
    }
    assert_eq!(runtime.accel, Some(500.0));
    assert_eq!(
        runtime.set_corner_deviation(Some(-0.1)),
        Err(RuntimeLimitError::InvalidCornerDeviation)
    );
    runtime.set_corner_deviation(Some(0.0)).unwrap();
}

#[test]
fn default_config_chains_are_passthrough() {
    let c = PlannerConfig::default();
    let chains = c.post_processors.compile(&c.axis_registry).unwrap();
    assert_eq!(chains.n_axes(), 3);
    assert!(
        chains
            .chains
            .iter()
            .all(trajectory::CompiledChain::is_empty)
    );
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
        ratio_ramp_time: None,
        max_velocity: None,
        max_accel: None,
        instantaneous_corner_velocity: None,
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
        ratio_ramp_time: None,
        max_velocity: None,
        max_accel: None,
        instantaneous_corner_velocity: None,
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

fn limited_e() -> AxisDecl {
    AxisDecl {
        max_velocity: Some(80.0),
        max_accel: Some(10_000.0),
        ..decl("e", &["x", "y", "z"])
    }
}

fn registry_with_e(e_post_processors: &[&str]) -> AxisRegistry {
    let mut decls: Vec<AxisDecl> = ["x", "y", "z"].iter().map(|n| decl(n, &[])).collect();
    let mut e = limited_e();
    e.post_processors = e_post_processors.iter().map(|s| (*s).to_string()).collect();
    decls.push(e);
    AxisRegistry::try_new(decls).unwrap()
}

#[test]
fn a_follower_advance_needs_both_follower_limits() {
    let advance = [
        pp("pa", "linear_pressure_advance", &[("k", 0.04)]),
        e_smoothing(),
    ];
    for (max_velocity, max_accel) in [(None, None), (Some(80.0), None), (None, Some(10_000.0))] {
        let mut decls: Vec<AxisDecl> = ["x", "y", "z"].iter().map(|n| decl(n, &[])).collect();
        decls.push(AxisDecl {
            post_processors: vec!["pa".into(), "st".into()],
            max_velocity,
            max_accel,
            ..decl("e", &["x", "y", "z"])
        });
        let refused = PostProcessorSet::try_new(&AxisRegistry::try_new(decls).unwrap(), &advance);
        assert!(
            matches!(
                refused,
                Err(PostProcessorConfigError::FollowerAdvanceWithoutLimits { ref axis, ref name })
                    if axis == "e" && name == "pa"
            ),
            "limits {max_velocity:?}/{max_accel:?} must be refused, got {refused:?}"
        );
    }
    assert!(PostProcessorSet::try_new(&registry_with_e(&["pa", "st"]), &advance).is_ok());
    assert!(
        PostProcessorSet::try_new(&registry_with_e(&["st"]), &advance).is_ok(),
        "a follower without an advance needs no limits"
    );
}

#[test]
fn bypass_compiles_identity_chains_and_preserves_config() {
    let mut decls: Vec<AxisDecl> = ["x", "y", "z"].iter().map(|n| decl(n, &[])).collect();
    decls[0].post_processors = vec!["is".to_string()];
    let registry = AxisRegistry::try_new(decls).unwrap();
    let set = PostProcessorSet::try_new(
        &registry,
        &[pp("is", "smooth_bell", &[("smooth_time", 0.01605)])],
    )
    .unwrap();
    let mut cfg = PlannerConfig::default();
    cfg.axis_registry = registry;
    cfg.post_processors = set;
    let shaped = cfg.compile_active_chains().unwrap();
    assert!(shaped.chains[0].kernel_variance_s2() > 0.0);

    cfg.post_processor_bypass = true;
    let identity = cfg.compile_active_chains().unwrap();
    assert_eq!(identity.chains.len(), shaped.chains.len());
    assert!(
        identity
            .chains
            .iter()
            .all(|c| c.kernel_variance_s2() == 0.0),
        "bypass must yield identity chains"
    );

    cfg.post_processor_bypass = false;
    let restored = cfg.compile_active_chains().unwrap();
    assert!(
        restored.chains[0].kernel_variance_s2() > 0.0,
        "clearing the bypass must restore the configured chains"
    );
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
        matches!(chains.chains[3].leading_transform(), Some(trajectory::ChainStage::DerivativeGains { k1, k2: 0.0 }) if *k1 == 0.04)
    );
    assert!(chains.chains[3].kernel().is_some());
    assert_eq!(chains.followers, vec![(3, vec![0, 1, 2])]);
}

fn registry_with_x_pps_and_e(x_post_processors: &[&str]) -> AxisRegistry {
    let mut x = decl("x", &[]);
    x.post_processors = x_post_processors.iter().map(|s| (*s).to_string()).collect();
    AxisRegistry::try_new(vec![
        x,
        decl("y", &[]),
        decl("z", &[]),
        decl("e", &["x", "y", "z"]),
    ])
    .unwrap()
}

#[test]
fn gain_before_kernel_on_leader_axis_rejected() {
    let registry = registry_with_x_pps_and_e(&["pa", "is"]);
    let err = PostProcessorSet::try_new(
        &registry,
        &[
            pp("pa", "linear_pressure_advance", &[("k", 0.002)]),
            pp("is", "smooth_bell", &[("smooth_time", 0.01605)]),
        ],
    )
    .unwrap_err();
    let msg = err.to_string();
    assert!(
        msg.contains("axis 'x'") && msg.contains("'pa'"),
        "got: {msg}"
    );
}

#[test]
fn gain_after_kernel_on_leader_axis_compiles() {
    let registry = registry_with_x_pps_and_e(&["is", "pa"]);
    assert!(
        PostProcessorSet::try_new(
            &registry,
            &[
                pp("is", "smooth_bell", &[("smooth_time", 0.01605)]),
                pp("pa", "linear_pressure_advance", &[("k", 0.002)]),
            ],
        )
        .is_ok()
    );
}

#[test]
fn gain_before_disabled_kernel_on_leader_axis_rejected() {
    let registry = registry_with_x_pps_and_e(&["pa", "is"]);
    let err = PostProcessorSet::try_new(
        &registry,
        &[
            pp("pa", "linear_pressure_advance", &[("k", 0.002)]),
            pp("is", "smooth_bell", &[("smooth_time", 0.0)]),
        ],
    )
    .unwrap_err();
    assert!(err.to_string().contains("'pa'"), "got: {err}");
}

#[test]
fn gain_without_kernel_on_axis_leading_nobody_compiles() {
    let mut x = decl("x", &[]);
    x.post_processors = vec!["pa".to_string()];
    let registry = AxisRegistry::try_new(vec![x, decl("y", &[]), decl("z", &[])]).unwrap();
    assert!(
        PostProcessorSet::try_new(
            &registry,
            &[pp("pa", "linear_pressure_advance", &[("k", 0.002)])],
        )
        .is_ok()
    );
}

fn e_smoothing() -> PostProcessorDecl {
    pp("st", "smooth_triangle", &[("smooth_time", 0.01)])
}

#[test]
fn happy_path_compiles_pa_on_follower_e() {
    let registry = registry_with_e(&["pa", "st"]);
    let set = PostProcessorSet::try_new(
        &registry,
        &[
            pp("pa", "linear_pressure_advance", &[("k", 0.04)]),
            e_smoothing(),
        ],
    )
    .unwrap();
    let chains = set.compile(&registry).unwrap();
    assert_eq!(chains.n_axes(), 4);
    assert!(
        matches!(chains.chains[3].transform(), Some(trajectory::ChainStage::DerivativeGains { k1, k2: 0.0 }) if *k1 == 0.04)
    );
    assert_eq!(chains.followers, vec![(3, vec![0, 1, 2])]);
}

#[test]
fn set_param_updates_named_instance_and_recompile_reflects_it() {
    let registry = registry_with_e(&["pa", "st"]);
    let mut set = PostProcessorSet::try_new(
        &registry,
        &[
            pp("pa", "linear_pressure_advance", &[("k", 0.04)]),
            e_smoothing(),
        ],
    )
    .unwrap();
    set.set_param("pa", "k", 0.07).unwrap();
    let chains = set.compile(&registry).unwrap();
    assert!(
        matches!(chains.chains[3].transform(), Some(trajectory::ChainStage::DerivativeGains { k1, k2: 0.0 }) if *k1 == 0.07)
    );
    assert!(set.set_param("nope", "k", 1.0).is_err());
    assert!(set.set_param("pa", "frequency_hz", 1.0).is_err());
}

#[test]
fn mode_inverse_after_kernel_compiles_into_the_axis_chain() {
    let registry = registry_with_e(&["slew", "belt"]);
    let set = PostProcessorSet::try_new(
        &registry,
        &[
            pp("slew", "smooth_bell", &[("smooth_time", 0.0015)]),
            pp(
                "belt",
                "mode_inverse",
                &[("frequency_hz", 131.0), ("damping_ratio", 0.05)],
            ),
        ],
    )
    .unwrap();
    let chains = set.compile(&registry).unwrap();
    let omega = 2.0 * std::f64::consts::PI * 131.0;
    assert!(chains.chains[3].kernel().is_some());
    assert!(matches!(
        chains.chains[3].trailing_transform(),
        Some(trajectory::ChainStage::DerivativeGains { k1, k2 })
            if *k1 == 2.0 * 0.05 / omega && *k2 == 1.0 / (omega * omega)
    ));
}

#[test]
fn mode_inverse_without_a_kernel_rejected_at_config_compile() {
    let registry = registry_with_e(&["belt"]);
    let err = PostProcessorSet::try_new(
        &registry,
        &[pp(
            "belt",
            "mode_inverse",
            &[("frequency_hz", 131.0), ("damping_ratio", 0.05)],
        )],
    )
    .unwrap_err();
    assert!(err.to_string().contains("smoothing kernel"), "got: {err}");
}

#[test]
fn mode_inverse_missing_params_rejected_by_key_name() {
    let registry = registry_with_e(&[]);
    let err = PostProcessorSet::try_new(
        &registry,
        &[pp("belt", "mode_inverse", &[("damping_ratio", 0.05)])],
    )
    .unwrap_err();
    assert!(err.to_string().contains("frequency_hz"), "got: {err}");
    let err = PostProcessorSet::try_new(
        &registry,
        &[pp("belt", "mode_inverse", &[("frequency_hz", 131.0)])],
    )
    .unwrap_err();
    assert!(err.to_string().contains("damping_ratio"), "got: {err}");
}

#[test]
fn mode_inverse_overdamped_ratio_rejected() {
    let registry = registry_with_e(&[]);
    let err = PostProcessorSet::try_new(
        &registry,
        &[pp(
            "belt",
            "mode_inverse",
            &[("frequency_hz", 131.0), ("damping_ratio", 1.0)],
        )],
    )
    .unwrap_err();
    assert!(err.to_string().contains("damping_ratio"), "got: {err}");
}

#[test]
fn post_processor_missing_required_param_rejected() {
    let registry = registry_with_e(&[]);
    let err = PostProcessorSet::try_new(&registry, &[pp("is", "smooth_bell", &[])]).unwrap_err();
    assert!(err.to_string().contains("smooth_time"), "got: {err}");
}

#[test]
fn a_follower_advance_needs_a_smoothing_kernel_on_its_axis() {
    let advances = [
        pp("pa", "linear_pressure_advance", &[("k", 0.04)]),
        pp(
            "pa",
            "tanh_pressure_advance",
            &[
                ("linear_advance", 0.02),
                ("nonlinear_offset", 0.05),
                ("linearization_velocity", 20.0),
            ],
        ),
    ];
    for advance in advances {
        let rejected = PostProcessorSet::try_new(&registry_with_e(&["pa"]), &[advance.clone()]);
        assert!(
            matches!(
                rejected,
                Err(PostProcessorConfigError::FollowerAdvanceWithoutKernel { ref axis, ref name })
                    if axis == "e" && name == "pa"
            ),
            "{} with no kernel on a follower must be rejected, got {rejected:?}",
            advance.ty
        );
        for chain in [["pa", "st"], ["st", "pa"]] {
            assert!(
                PostProcessorSet::try_new(
                    &registry_with_e(&chain),
                    &[advance.clone(), e_smoothing()]
                )
                .is_ok(),
                "{} with a kernel ({chain:?}) is representable",
                advance.ty
            );
        }
    }
}

#[test]
fn a_disabled_follower_advance_needs_no_kernel_until_it_is_raised() {
    let registry = registry_with_e(&["pa"]);
    let mut set = PostProcessorSet::try_new(
        &registry,
        &[pp("pa", "linear_pressure_advance", &[("k", 0.0)])],
    )
    .expect("a zero advance makes no steps");
    set.set_param("pa", "k", 0.04).unwrap();
    assert!(matches!(
        set.compile(&registry),
        Err(PostProcessorConfigError::FollowerAdvanceWithoutKernel { .. })
    ));
}

#[test]
fn an_advance_on_an_axis_that_follows_nothing_is_not_a_follower_advance() {
    let mut decls: Vec<AxisDecl> = ["x", "y", "z"].iter().map(|n| decl(n, &[])).collect();
    let mut e = decl("e", &[]);
    e.post_processors = vec!["pa".to_string()];
    decls.push(e);
    let registry = AxisRegistry::try_new(decls).unwrap();
    assert!(
        PostProcessorSet::try_new(
            &registry,
            &[pp("pa", "linear_pressure_advance", &[("k", 0.04)])]
        )
        .is_ok()
    );
}

fn ramped_registry_with_e(e_post_processors: &[&str], ratio_ramp_time: f64) -> AxisRegistry {
    let mut decls: Vec<AxisDecl> = ["x", "y", "z"].iter().map(|n| decl(n, &[])).collect();
    let mut e = limited_e();
    e.post_processors = e_post_processors.iter().map(|s| (*s).to_string()).collect();
    e.ratio_ramp_time = Some(ratio_ramp_time);
    decls.push(e);
    AxisRegistry::try_new(decls).unwrap()
}

#[test]
fn a_ratio_ramp_compiles_onto_its_follower_chain_and_nowhere_else() {
    let registry = ramped_registry_with_e(&[], 0.015);
    let chains = PostProcessorSet::try_new(&registry, &[])
        .unwrap()
        .compile(&registry)
        .unwrap();
    assert_eq!(chains.chains[3].ratio_ramp_time(), Some(0.015));
    assert!(
        chains.chains[..3]
            .iter()
            .all(|chain| chain.ratio_ramp_time().is_none())
    );
    assert!(chains.has_ratio_ramped_follower());
    let plain = registry_with_e(&[]);
    let chains = PostProcessorSet::try_new(&plain, &[])
        .unwrap()
        .compile(&plain)
        .unwrap();
    assert!(!chains.has_ratio_ramped_follower());
}

#[test]
fn a_bypassed_chain_set_carries_no_ratio_ramp() {
    let registry = ramped_registry_with_e(&[], 0.015);
    let set = PostProcessorSet::try_new(&registry, &[]).unwrap();
    let bypassed = set.compile_active_chains(&registry, true).unwrap();
    assert!(!bypassed.has_ratio_ramped_follower());
}

#[test]
fn a_ratio_ramp_compiles_next_to_a_smoothing_kernel() {
    let registry = ramped_registry_with_e(&["st"], 0.015);
    let chains = PostProcessorSet::try_new(&registry, &[e_smoothing()])
        .unwrap()
        .compile(&registry)
        .unwrap();
    assert!(chains.chains[3].kernel().is_some());
    assert_eq!(chains.chains[3].ratio_ramp_time(), Some(0.015));
    assert!(chains.has_ratio_ramped_follower());
}

#[test]
fn a_ratio_ramp_needs_an_axis_that_follows_others() {
    let mut decls: Vec<AxisDecl> = ["x", "y", "z"].iter().map(|n| decl(n, &[])).collect();
    decls[0].ratio_ramp_time = Some(0.01);
    assert!(matches!(
        AxisRegistry::try_new(decls),
        Err(AxisConfigError::RatioRampWithoutLeaders { ref axis }) if axis == "x"
    ));
    let mut decls: Vec<AxisDecl> = ["x", "y", "z"].iter().map(|n| decl(n, &[])).collect();
    let mut e = decl("e", &[]);
    e.ratio_ramp_time = Some(0.01);
    decls.push(e);
    assert!(matches!(
        AxisRegistry::try_new(decls),
        Err(AxisConfigError::RatioRampWithoutLeaders { ref axis }) if axis == "e"
    ));
}

#[test]
fn a_ratio_ramp_time_must_be_finite_and_positive() {
    for value in [0.0, -0.01, f64::NAN, f64::INFINITY] {
        let mut decls: Vec<AxisDecl> = ["x", "y", "z"].iter().map(|n| decl(n, &[])).collect();
        let mut e = decl("e", &["x", "y", "z"]);
        e.ratio_ramp_time = Some(value);
        decls.push(e);
        assert!(
            matches!(
                AxisRegistry::try_new(decls),
                Err(AxisConfigError::BadRatioRampTime { .. })
            ),
            "ratio_ramp_time {value} must be rejected"
        );
    }
}

#[test]
fn follower_limits_belong_to_followers_and_must_be_positive() {
    let mut decls: Vec<AxisDecl> = ["x", "y", "z"].iter().map(|n| decl(n, &[])).collect();
    decls[0].max_velocity = Some(60.0);
    assert!(matches!(
        AxisRegistry::try_new(decls),
        Err(AxisConfigError::LimitWithoutLeaders { ref axis, option: "max_velocity" }) if axis == "x"
    ));
    for value in [0.0, -1.0, f64::NAN, f64::INFINITY] {
        let mut decls: Vec<AxisDecl> = ["x", "y", "z"].iter().map(|n| decl(n, &[])).collect();
        let mut e = decl("e", &["x", "y", "z"]);
        e.max_accel = Some(value);
        decls.push(e);
        assert!(
            matches!(
                AxisRegistry::try_new(decls),
                Err(AxisConfigError::BadLimit {
                    option: "max_accel",
                    ..
                })
            ),
            "max_accel {value} must be rejected"
        );
    }
}

#[test]
fn the_instantaneous_corner_velocity_belongs_to_followers_and_must_be_positive() {
    let mut decls: Vec<AxisDecl> = ["x", "y", "z"].iter().map(|n| decl(n, &[])).collect();
    decls[2].instantaneous_corner_velocity = Some(1.0);
    assert!(matches!(
        AxisRegistry::try_new(decls),
        Err(AxisConfigError::LimitWithoutLeaders {
            ref axis,
            option: "instantaneous_corner_velocity"
        }) if axis == "z"
    ));
    for value in [0.0, -1.0, f64::NAN, f64::INFINITY] {
        let mut decls: Vec<AxisDecl> = ["x", "y", "z"].iter().map(|n| decl(n, &[])).collect();
        let mut e = decl("e", &["x", "y", "z"]);
        e.instantaneous_corner_velocity = Some(value);
        decls.push(e);
        assert!(
            matches!(
                AxisRegistry::try_new(decls),
                Err(AxisConfigError::BadLimit {
                    option: "instantaneous_corner_velocity",
                    ..
                })
            ),
            "instantaneous_corner_velocity {value} must be rejected"
        );
    }
}

#[test]
fn a_ratio_ramp_requires_finite_extrude_only_caps() {
    let ramped = PlannerConfig {
        axis_registry: ramped_registry_with_e(&[], 0.015),
        ..PlannerConfig::default()
    };
    for (velocity, accel) in [
        (None, Some(2000.0)),
        (Some(40.0), None),
        (Some(f64::INFINITY), Some(2000.0)),
        (Some(40.0), Some(f64::INFINITY)),
    ] {
        let config = PlannerConfig {
            max_extrude_only_velocity: velocity,
            max_extrude_only_accel: accel,
            ..ramped.clone()
        };
        let refused = config.check_ratio_ramp_extrude_only_caps();
        assert!(
            refused
                .as_ref()
                .is_err_and(|message| message.contains("axis 'e' has a ratio_ramp_time")),
            "caps {velocity:?}/{accel:?} must be refused, got {refused:?}"
        );
    }
    let capped = PlannerConfig {
        max_extrude_only_velocity: Some(40.0),
        max_extrude_only_accel: Some(2000.0),
        ..ramped
    };
    assert!(capped.check_ratio_ramp_extrude_only_caps().is_ok());
    assert!(
        PlannerConfig::default()
            .check_ratio_ramp_extrude_only_caps()
            .is_ok(),
        "with no ratio ramp the caps stay optional"
    );
}

#[test]
fn a_follower_advance_with_a_ratio_ramp_needs_no_kernel() {
    let advances = [
        pp("pa", "linear_pressure_advance", &[("k", 0.02)]),
        pp(
            "pa",
            "tanh_pressure_advance",
            &[
                ("linear_advance", 0.02),
                ("nonlinear_offset", 0.05),
                ("linearization_velocity", 20.0),
            ],
        ),
    ];
    for advance in advances {
        let registry = ramped_registry_with_e(&["pa"], 0.015);
        let chains = PostProcessorSet::try_new(&registry, &[advance.clone()])
            .unwrap_or_else(|error| panic!("{} on a ratio ramp is refused: {error}", advance.ty))
            .compile(&registry)
            .unwrap();
        assert_eq!(chains.kernel_free_follower_transform(), None);
        assert!(chains.chains[3].kernel().is_none());
        let refused = PostProcessorSet::try_new(&registry_with_e(&["pa"]), &[advance.clone()]);
        assert!(
            refused
                .as_ref()
                .is_err_and(|error| error.to_string().contains("or set ratio_ramp_time")),
            "with neither a kernel nor a ramp the refusal names ratio_ramp_time, got {refused:?}"
        );
    }
}
