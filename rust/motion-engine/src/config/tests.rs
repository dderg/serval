use super::*;

#[test]
fn default_config_has_sensible_values() {
    let c = PlannerConfig::default();
    assert_eq!(c.window_capacity, 32);
    assert_eq!(c.beta_max_iters, 10);
}

#[test]
fn cartesian_default_square_corner_velocity_matches_const() {
    let c = CartesianLimits::default();
    assert_eq!(
        c.square_corner_velocity,
        DEFAULT_SQUARE_CORNER_VELOCITY_MM_S
    );
}

#[test]
fn cartesian_validate_accepts_zero_and_positive_scv() {
    let mut c = CartesianLimits::default();
    c.square_corner_velocity = 0.0;
    assert!(c.validate().is_ok());
    c.square_corner_velocity = 8.0;
    assert!(c.validate().is_ok());
}

#[test]
fn cartesian_validate_rejects_negative_or_nan_scv() {
    let mut c = CartesianLimits::default();
    c.square_corner_velocity = -1.0;
    assert!(c.validate().is_err());
    c.square_corner_velocity = f64::NAN;
    assert!(c.validate().is_err());
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
            .all(|ch| ch.kernel.is_none() && ch.gain == 0.0)
    );
    assert!(chains.followers.is_empty());
}

#[test]
fn sections_convert_to_temporal_sets() {
    let cfg = PlannerConfig::default();
    let lims = cfg.to_temporal_limits().unwrap();
    assert_eq!(lims.sets().len(), 2);
    assert_eq!(lims.sets()[0].v_max, 300.0);
    assert_eq!(lims.sets()[1].a_max, 100.0);
}

#[test]
fn jerk_defaults_to_twice_accel_per_section() {
    let cfg = PlannerConfig::default();
    let lims = cfg.to_temporal_limits().unwrap();
    assert_eq!(lims.sets()[0].j_max, 6000.0);
}

#[test]
fn missing_axis_coverage_is_an_error() {
    let mut cfg = PlannerConfig::default();
    cfg.limit_sections.retain(|s| s.name != "z");
    assert!(cfg.to_temporal_limits().is_err());
}

#[test]
fn runtime_caps_append_an_all_axis_overlay() {
    let mut cfg = PlannerConfig::default();
    cfg.runtime_caps = RuntimeCaps {
        velocity: Some(100.0),
        accel: Some(1000.0),
    };
    let lims = cfg.to_temporal_limits().unwrap();
    let overlay = lims.sets().last().unwrap();
    assert_eq!(overlay.v_max, 100.0);
    assert_eq!(overlay.a_max, 1000.0);
    assert_eq!(overlay.axes, temporal::AxisSet::spatial());
}

#[test]
fn section_with_no_caps_is_an_error() {
    let mut cfg = PlannerConfig::default();
    cfg.limit_sections.push(LimitSection {
        name: "empty".into(),
        axes: vec![0],
        max_velocity: None,
        max_accel: None,
        max_jerk: None,
    });
    assert!(cfg.to_temporal_limits().is_err());
}

#[test]
fn overlay_above_config_cannot_raise_limits() {
    let mut cfg = PlannerConfig::default();
    cfg.runtime_caps = RuntimeCaps {
        velocity: Some(10_000.0),
        accel: Some(1_000_000.0),
    };
    let lims = cfg.to_temporal_limits().unwrap();
    let x_tangent = [1.0, 0.0, 0.0];
    assert_eq!(lims.mvc_b(&x_tangent, 1e-12), 300.0 * 300.0);
    assert_eq!(lims.a_tan_cap(&x_tangent, 1e-12), 3000.0);
}

#[test]
fn overlay_below_config_tightens() {
    let mut cfg = PlannerConfig::default();
    cfg.runtime_caps = RuntimeCaps {
        velocity: Some(50.0),
        accel: Some(500.0),
    };
    let lims = cfg.to_temporal_limits().unwrap();
    let x_tangent = [1.0, 0.0, 0.0];
    assert_eq!(lims.mvc_b(&x_tangent, 1e-12), 50.0 * 50.0);
    assert_eq!(lims.a_tan_cap(&x_tangent, 1e-12), 500.0);
}

#[test]
fn clearing_runtime_caps_restores_config_limits() {
    let mut cfg = PlannerConfig::default();
    cfg.runtime_caps = RuntimeCaps {
        velocity: Some(50.0),
        accel: Some(500.0),
    };
    cfg.runtime_caps = RuntimeCaps::default();
    let lims = cfg.to_temporal_limits().unwrap();
    assert_eq!(lims.sets().len(), 2);
    assert_eq!(lims.mvc_b(&[1.0, 0.0, 0.0], 1e-12), 300.0 * 300.0);
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

#[test]
fn limit_sections_partition_spatial_follower_mixed() {
    let reg = AxisRegistry::try_new(vec![
        decl("x", &[]),
        decl("y", &[]),
        decl("z", &[]),
        decl("e", &["x", "y", "z"]),
    ])
    .unwrap();
    let mut cfg = PlannerConfig::default();
    cfg.axis_registry = reg;
    cfg.limit_sections.push(LimitSection {
        name: "extruder".into(),
        axes: vec![3],
        max_velocity: Some(75.0),
        max_accel: Some(1500.0),
        max_jerk: None,
    });
    cfg.to_temporal_limits().unwrap();
    cfg.limit_sections.push(LimitSection {
        name: "mixed".into(),
        axes: vec![0, 3],
        max_velocity: Some(10.0),
        max_accel: None,
        max_jerk: None,
    });
    assert!(matches!(
        cfg.to_temporal_limits().unwrap_err(),
        LimitConfigError::MixedSpatialFollower { .. }
    ));
}

#[test]
fn follower_axis_without_limit_coverage_is_an_error() {
    let reg = AxisRegistry::try_new(vec![
        decl("x", &[]),
        decl("y", &[]),
        decl("z", &[]),
        decl("e", &["x", "y", "z"]),
    ])
    .unwrap();
    let mut cfg = PlannerConfig::default();
    cfg.axis_registry = reg;
    assert!(matches!(
        cfg.to_temporal_limits().unwrap_err(),
        LimitConfigError::NoFollowerCoverage { .. }
    ));
}

#[test]
fn follower_sections_become_temporal_sets() {
    let reg = AxisRegistry::try_new(vec![
        decl("x", &[]),
        decl("y", &[]),
        decl("z", &[]),
        decl("e", &["x", "y", "z"]),
    ])
    .unwrap();
    let mut cfg = PlannerConfig::default();
    cfg.axis_registry = reg;
    cfg.limit_sections.push(LimitSection {
        name: "extruder".into(),
        axes: vec![3],
        max_velocity: Some(75.0),
        max_accel: Some(1500.0),
        max_jerk: None,
    });
    let lims = cfg.to_temporal_limits().unwrap();
    assert_eq!(lims.n_axes(), 4);
    let followers: Vec<_> = lims.follower_sets().collect();
    assert_eq!(followers.len(), 1);
    let (_, set) = followers[0];
    assert!(set.axes.contains(3));
    assert_eq!(set.v_max, 75.0);
    assert_eq!(set.a_max, 1500.0);
    assert_eq!(set.j_max, 3000.0);
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
            pp("is", "smooth_zv", &[("frequency_hz", 50.0)]),
            pp("is", "smooth_mzv", &[("frequency_hz", 40.0)]),
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
            pp("is_a", "smooth_zv", &[("frequency_hz", 50.0)]),
            pp("is_b", "smooth_mzv", &[("frequency_hz", 40.0)]),
        ],
    )
    .unwrap_err();
    assert!(err.to_string().contains("v1"), "got: {err}");
}

#[test]
fn happy_path_compiles_kernel_and_pa_on_e() {
    let registry = registry_with_e(&["is", "pa"]);
    let set = PostProcessorSet::try_new(
        &registry,
        &[
            pp("is", "smooth_zv", &[("frequency_hz", 50.0)]),
            pp("pa", "linear_pressure_advance", &[("k", 0.04)]),
        ],
    )
    .unwrap();
    let chains = set.compile(&registry).unwrap();
    assert_eq!(chains.n_axes(), 4);
    assert!(chains.chains[3].kernel.is_some());
    assert_eq!(chains.chains[3].gain, 0.04);
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
    assert_eq!(chains.chains[3].gain, 0.07);
    assert!(set.set_param("nope", "k", 1.0).is_err());
    assert!(set.set_param("pa", "frequency_hz", 1.0).is_err());
}

#[test]
fn post_processor_missing_required_param_rejected() {
    let registry = registry_with_e(&[]);
    let err = PostProcessorSet::try_new(&registry, &[pp("is", "smooth_zv", &[])]).unwrap_err();
    assert!(err.to_string().contains("frequency_hz"), "got: {err}");
}

#[test]
fn limit_set_names_follow_section_order() {
    let reg = AxisRegistry::try_new(vec![
        decl("x", &[]),
        decl("y", &[]),
        decl("z", &[]),
        decl("e", &["x", "y", "z"]),
    ])
    .unwrap();
    let post_processors = PostProcessorSet::try_new(&reg, &[]).unwrap();
    let cfg = PlannerConfig {
        axis_registry: reg,
        limit_sections: vec![
            LimitSection {
                name: "gantry".into(),
                axes: vec![0, 1],
                max_velocity: Some(300.0),
                max_accel: Some(3000.0),
                max_jerk: None,
            },
            LimitSection {
                name: "extruder".into(),
                axes: vec![3],
                max_velocity: Some(75.0),
                max_accel: Some(1500.0),
                max_jerk: None,
            },
        ],
        cartesian: CartesianLimits::default(),
        runtime_caps: RuntimeCaps::default(),
        post_processors,
        window_capacity: 32,
        beta_max_iters: 10,
        beta_convergence_ratio: 0.05,
        fit_tolerance_mm: 0.005,
        worker_threads: 3,
    };
    assert_eq!(
        cfg.limit_set_names(),
        vec!["gantry".to_string(), "extruder".to_string()]
    );
}
