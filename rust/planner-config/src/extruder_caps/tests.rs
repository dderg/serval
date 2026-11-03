use super::*;
use crate::{AxisDecl, PostProcessorDecl, PostProcessorSet};

fn decl(name: &str, follows: &[&str], post_processors: &[&str]) -> AxisDecl {
    AxisDecl {
        name: name.into(),
        follows: follows.iter().map(|s| (*s).to_string()).collect(),
        motors: vec![],
        post_processors: post_processors.iter().map(|s| (*s).to_string()).collect(),
        ratio_ramp_time: None,
        max_velocity: None,
        max_accel: None,
        instantaneous_corner_velocity: None,
    }
}

fn pp(name: &str, ty: &str, params: &[(&str, f64)]) -> PostProcessorDecl {
    PostProcessorDecl {
        name: name.into(),
        ty: ty.into(),
        params: params.iter().map(|(k, v)| ((*k).into(), *v)).collect(),
    }
}

fn caps_of(e: AxisDecl, decls: &[PostProcessorDecl]) -> Result<Vec<ExtruderCaps>, String> {
    let registry = AxisRegistry::try_new(vec![
        decl("x", &[], &["shaper_x"]),
        decl("y", &[], &["shaper_y"]),
        decl("z", &[], &[]),
        e,
    ])
    .unwrap();
    let mut all = vec![
        pp("shaper_x", "smooth_mzv", &[("frequency_hz", 75.0)]),
        pp("shaper_y", "smooth_mzv", &[("frequency_hz", 62.0)]),
    ];
    all.extend_from_slice(decls);
    let chains = PostProcessorSet::try_new(&registry, &all)
        .unwrap()
        .compile(&registry)
        .unwrap();
    extruder_caps(&registry, &chains)
}

fn limited(post_processors: &[&str]) -> AxisDecl {
    AxisDecl {
        max_velocity: Some(100.0),
        max_accel: Some(10_000.0),
        ratio_ramp_time: Some(0.015),
        ..decl("e", &["x", "y", "z"], post_processors)
    }
}

fn mzv_peak(frequency_hz: f64) -> f64 {
    trajectory::build_smooth_mzv_kernel(frequency_hz).peak_abs()
}

#[test]
fn a_ramped_linear_advance_takes_k_and_its_leaders_kernels() {
    let caps = caps_of(
        limited(&["pa"]),
        &[pp("pa", "linear_pressure_advance", &[("k", 0.03)])],
    )
    .unwrap();
    assert_eq!(caps.len(), 1);
    let caps = caps[0];
    assert_eq!(caps.axis_index, 3);
    assert_eq!(
        (caps.max_velocity_mm_s, caps.max_accel_mm_s2),
        (100.0, 10_000.0)
    );
    assert_eq!(
        (caps.advance_slope_s, caps.advance_curvature_s2_per_mm),
        (0.03, 0.0)
    );
    assert_eq!(
        caps.path_jerk_per_accel_hz,
        [2.0 * mzv_peak(75.0), 2.0 * mzv_peak(62.0), f64::INFINITY]
    );
    assert!(caps.own_jerk_per_accel_hz.is_infinite());
}

#[test]
fn the_reach_takes_the_ramp_time_and_the_widest_leader_kernel() {
    let caps = caps_of(
        limited(&["pa"]),
        &[pp("pa", "linear_pressure_advance", &[("k", 0.03)])],
    )
    .unwrap()[0];
    assert_eq!(caps.ratio_ramp_time_s, 0.015);
    let (lo, hi) = trajectory::build_smooth_mzv_kernel(62.0).support();
    assert_eq!(caps.leader_kernel_reach_s, hi.max(-lo));
    let unramped = caps_of(
        AxisDecl {
            ratio_ramp_time: None,
            ..limited(&["pa", "st"])
        },
        &[
            pp("pa", "linear_pressure_advance", &[("k", 0.03)]),
            pp("st", "smooth_triangle", &[("smooth_time", 0.01)]),
        ],
    )
    .unwrap()[0];
    assert_eq!(unramped.ratio_ramp_time_s, 0.0);
}

#[test]
fn a_kernel_on_the_follower_bounds_its_own_jerk() {
    let caps = caps_of(
        limited(&["pa", "st"]),
        &[
            pp("pa", "linear_pressure_advance", &[("k", 0.03)]),
            pp("st", "smooth_triangle", &[("smooth_time", 0.01)]),
        ],
    )
    .unwrap();
    assert!((caps[0].own_jerk_per_accel_hz - 2.0 * 2.0 / 0.01).abs() < 1e-9);
}

#[test]
fn a_nonlinear_advance_takes_its_largest_slope_and_curvature() {
    let caps = caps_of(
        limited(&["pa"]),
        &[pp(
            "pa",
            "tanh_pressure_advance",
            &[
                ("linear_advance", 0.01),
                ("nonlinear_offset", 0.05),
                ("linearization_velocity", 2.0),
            ],
        )],
    )
    .unwrap();
    assert!((caps[0].advance_slope_s - 0.035).abs() < 1e-15);
    assert!(caps[0].advance_curvature_s2_per_mm > 0.0);
}

#[test]
fn a_follower_without_limits_has_no_caps_and_one_limit_leaves_the_other_open() {
    let unlimited = decl("e", &["x", "y", "z"], &[]);
    assert!(caps_of(unlimited, &[]).unwrap().is_empty());
    let velocity_only = AxisDecl {
        max_velocity: Some(50.0),
        ..decl("e", &["x", "y", "z"], &[])
    };
    let caps = caps_of(velocity_only, &[]).unwrap();
    assert_eq!(caps[0].max_velocity_mm_s, 50.0);
    assert!(caps[0].max_accel_mm_s2.is_infinite());
    assert_eq!(caps[0].advance_slope_s, 0.0);
}

#[test]
fn the_instantaneous_corner_velocity_defaults_like_upstream_klippy() {
    let caps = caps_of(
        limited(&["pa"]),
        &[pp("pa", "linear_pressure_advance", &[("k", 0.03)])],
    )
    .unwrap();
    assert_eq!(
        caps[0].instantaneous_corner_velocity_mm_s,
        trajectory::DEFAULT_INSTANTANEOUS_CORNER_VELOCITY_MM_S
    );
    let set = AxisDecl {
        instantaneous_corner_velocity: Some(2.5),
        ..limited(&["pa"])
    };
    let caps = caps_of(set, &[pp("pa", "linear_pressure_advance", &[("k", 0.03)])]).unwrap();
    assert_eq!(caps[0].instantaneous_corner_velocity_mm_s, 2.5);
}
