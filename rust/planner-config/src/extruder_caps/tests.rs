use super::*;
use crate::{AxisDecl, PostProcessorDecl, PostProcessorSet};

fn decl(name: &str, follows: &[&str], post_processors: &[&str]) -> AxisDecl {
    AxisDecl {
        name: name.into(),
        follows: follows.iter().map(|s| (*s).to_string()).collect(),
        motors: vec![],
        post_processors: post_processors.iter().map(|s| (*s).to_string()).collect(),
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
    extruder_caps(&registry, &chains, 0.005)
}

fn limited(post_processors: &[&str]) -> AxisDecl {
    AxisDecl {
        max_velocity: Some(100.0),
        max_accel: Some(10_000.0),
        ..decl("e", &["x", "y", "z"], post_processors)
    }
}

fn mzv_variation(frequencies_hz: &[f64]) -> f64 {
    let kernels: Vec<_> = frequencies_hz
        .iter()
        .map(|&f| trajectory::build_smooth_mzv_kernel(f))
        .collect();
    PiecewisePolynomialKernel::directional_variation(&kernels.iter().collect::<Vec<_>>())
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
        [
            0.0,
            mzv_variation(&[75.0]),
            mzv_variation(&[62.0]),
            mzv_variation(&[75.0, 62.0]),
            0.0,
            mzv_variation(&[75.0]),
            mzv_variation(&[62.0]),
            mzv_variation(&[75.0, 62.0]),
        ]
    );
    assert_eq!(caps.unshaped_leaders, [false, false, true]);
    assert!(caps.own_jerk_per_accel_hz.is_infinite());
}

#[test]
fn the_bimodal_mzv_kernel_gains_more_than_twice_its_peak_and_the_plane_more_than_either_axis() {
    let caps = caps_of(
        limited(&["pa"]),
        &[pp("pa", "linear_pressure_advance", &[("k", 0.03)])],
    )
    .unwrap()[0];
    let x = caps.path_jerk_gain([true, false, false]);
    let samples = 100_000;
    let kernel = trajectory::build_smooth_mzv_kernel(75.0);
    let (lo, hi) = kernel.support();
    let peak = (0..=samples)
        .map(|i| kernel.pieces[0].evaluate(lo + (hi - lo) * f64::from(i) / f64::from(samples)))
        .fold(0.0, f64::max);
    assert!(x > 2.1 * peak, "{x} vs twice the peak {}", 2.0 * peak);
    let plane = caps.path_jerk_gain([true, true, false]);
    assert!(plane > x && plane > caps.path_jerk_gain([false, true, false]));
}

#[test]
fn the_switch_gain_is_twice_the_tallest_kernel_peak_of_the_set() {
    let caps = caps_of(
        limited(&["pa"]),
        &[pp("pa", "linear_pressure_advance", &[("k", 0.03)])],
    )
    .unwrap()[0];
    let switch = |f: f64| {
        2.0 * PiecewisePolynomialKernel::directional_peak(&[&trajectory::build_smooth_mzv_kernel(
            f,
        )])
    };
    let (x, y) = (switch(75.0), switch(62.0));
    assert_eq!(
        caps.path_jerk_per_accel_switch_hz,
        [0.0, x, y, x.max(y), 0.0, x, y, x.max(y)]
    );
    assert!(x < caps.path_jerk_gain([true, false, false]));
    assert_eq!(
        caps.planned_path_jerk_gain([true, true, false]),
        x.max(y),
        "a chain without a follower kernel plans at the switch gain"
    );
}

#[test]
fn the_switch_gain_never_exceeds_the_variation() {
    let caps = caps_of(
        limited(&["pa"]),
        &[pp("pa", "linear_pressure_advance", &[("k", 0.03)])],
    )
    .unwrap()[0];
    let variation = caps.path_jerk_per_accel_hz;
    for (set, switch) in caps.path_jerk_per_accel_switch_hz.iter().enumerate() {
        assert!(
            *switch <= variation[set],
            "set {set}: {switch} over {}",
            variation[set]
        );
    }
}

#[test]
fn the_reach_takes_the_widest_leader_kernel() {
    let caps = caps_of(
        limited(&["pa"]),
        &[pp("pa", "linear_pressure_advance", &[("k", 0.03)])],
    )
    .unwrap()[0];
    let (lo, hi) = trajectory::build_smooth_mzv_kernel(62.0).support();
    assert_eq!(caps.leader_kernel_reach_s, hi.max(-lo));
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
    let plane = [true, true, false];
    assert_eq!(
        caps[0].planned_path_jerk_gain(plane),
        caps[0].path_jerk_gain(plane),
        "a kernel follower plans at the total variation"
    );
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
