use super::*;

#[test]
fn default_config_has_sensible_values() {
    let c = PlannerConfig::default();
    assert_eq!(c.window_capacity, 32);
    assert_eq!(c.beta_max_iters, 10);
}

#[test]
fn default_config_shaper_is_passthrough() {
    let c = PlannerConfig::default();
    assert!(matches!(c.shaper.x, AxisShaper::Passthrough));
    assert!(matches!(c.shaper.y, AxisShaper::Passthrough));
    assert!(matches!(c.shaper.z, AxisShaper::Passthrough));
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
    assert_eq!(overlay.axes, temporal::AxisSet::all());
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
fn parse_shaper_types() {
    assert!(matches!(
        parse_axis_shaper("smooth_mzv", 50.0),
        Ok(AxisShaper::SmoothMzv { frequency_hz }) if (frequency_hz - 50.0).abs() < 1e-9
    ));
    assert!(parse_axis_shaper("smooth_zv", 50.0).is_ok());
    assert!(parse_axis_shaper("ei", 50.0).is_err());

    // freq ≤ 0 or non-finite → Passthrough, not an error
    assert!(matches!(
        parse_axis_shaper("smooth_zv", 0.0),
        Ok(AxisShaper::Passthrough)
    ));
    assert!(matches!(
        parse_axis_shaper("smooth_mzv", -1.0),
        Ok(AxisShaper::Passthrough)
    ));
    assert!(matches!(
        parse_axis_shaper("smooth_zv", f64::NAN),
        Ok(AxisShaper::Passthrough)
    ));
    assert!(matches!(
        parse_axis_shaper("smooth_zv", f64::INFINITY),
        Ok(AxisShaper::Passthrough)
    ));
}

#[test]
fn parse_explicit_passthrough_names() {
    assert!(matches!(
        parse_axis_shaper("", 0.0),
        Ok(AxisShaper::Passthrough)
    ));
    assert!(matches!(
        parse_axis_shaper("none", 50.0),
        Ok(AxisShaper::Passthrough)
    ));
    assert!(matches!(
        parse_axis_shaper("passthrough", 50.0),
        Ok(AxisShaper::Passthrough)
    ));
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
