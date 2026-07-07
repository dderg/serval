use super::*;

const TENSION: f64 = 0.2;

fn wavy_grid() -> MeshGrid {
    let (nx, ny) = (5, 4);
    let z = (0..ny)
        .flat_map(|j| {
            (0..nx).map(move |i| {
                0.08 * libm::sin(0.9 * i as f64) + 0.05 * libm::cos(1.3 * j as f64)
                    - 0.02 * i as f64
            })
        })
        .collect();
    MeshGrid::new(10.0, 20.0, 25.0, 30.0, nx, ny, z, TENSION).unwrap()
}

fn transform(fade: Fade) -> SurfaceTransform {
    SurfaceTransform::new(wavy_grid(), fade)
}

#[test]
fn interpolates_probed_points_exactly_at_nodes() {
    let g = wavy_grid();
    for j in 0..4 {
        for i in 0..5 {
            let x = 10.0 + 25.0 * i as f64;
            let y = 20.0 + 30.0 * j as f64;
            let expected = 0.08 * libm::sin(0.9 * i as f64) + 0.05 * libm::cos(1.3 * j as f64)
                - 0.02 * i as f64;
            assert!((g.sample(x, y).z - expected).abs() < 1e-12);
        }
    }
}

/// Mirrors mainline's `_cardinal_spline`: z = p1·(2t³−3t²+1) + p2·(−2t³+3t²)
/// plus m1·(t³−2t²+t) + m2·(t³−t²), with m1 = c·(p2−p0), m2 = c·(p3−p1). A
/// 4x1-wide strip with constant rows reduces the tensor product to that 1-D
/// formula.
#[test]
fn matches_mainline_cardinal_spline_along_one_axis() {
    let row = [0.10, -0.05, 0.20, 0.08];
    let z: Vec<f64> = row.iter().chain(row.iter()).copied().collect();
    let g = MeshGrid::new(0.0, 0.0, 10.0, 10.0, 4, 2, z, TENSION).unwrap();
    for t in [0.0, 0.25, 0.5, 0.75, 1.0] {
        let m1 = TENSION * (row[2] - row[0]);
        let m2 = TENSION * (row[3] - row[1]);
        let t2 = t * t;
        let t3 = t2 * t;
        let expected = row[1] * (2.0 * t3 - 3.0 * t2 + 1.0)
            + row[2] * (-2.0 * t3 + 3.0 * t2)
            + m1 * (t3 - 2.0 * t2 + t)
            + m2 * (t3 - t2);
        let got = g.sample(10.0 + 10.0 * t, 5.0).z;
        assert!(
            (got - expected).abs() < 1e-12,
            "t={t}: got {got}, expected {expected}"
        );
    }
}

#[test]
fn derivatives_match_finite_differences() {
    let g = wavy_grid();
    let h = 1e-5;
    for &(x, y) in &[(31.0, 47.0), (12.5, 21.0), (77.7, 95.0), (95.0, 105.0)] {
        let s = g.sample(x, y);
        let fd_zx = (g.sample(x + h, y).z - g.sample(x - h, y).z) / (2.0 * h);
        let fd_zy = (g.sample(x, y + h).z - g.sample(x, y - h).z) / (2.0 * h);
        let fd_zxx = (g.sample(x + h, y).zx - g.sample(x - h, y).zx) / (2.0 * h);
        let fd_zyy = (g.sample(x, y + h).zy - g.sample(x, y - h).zy) / (2.0 * h);
        let fd_zxy = (g.sample(x, y + h).zx - g.sample(x, y - h).zx) / (2.0 * h);
        assert!((s.zx - fd_zx).abs() < 1e-7, "zx at ({x},{y})");
        assert!((s.zy - fd_zy).abs() < 1e-7, "zy at ({x},{y})");
        assert!((s.zxx - fd_zxx).abs() < 1e-6, "zxx at ({x},{y})");
        assert!((s.zyy - fd_zyy).abs() < 1e-6, "zyy at ({x},{y})");
        assert!((s.zxy - fd_zxy).abs() < 1e-6, "zxy at ({x},{y})");
    }
}

#[test]
fn clamps_outside_the_grid_with_zero_gradient() {
    let g = wavy_grid();
    let (x0, x1) = g.x_range();
    let (y0, y1) = g.y_range();
    let corner = g.sample(x1, y1);
    let outside = g.sample(x1 + 50.0, y1 + 5.0);
    assert!((outside.z - corner.z).abs() < 1e-12);
    assert_eq!(outside.zx, 0.0);
    assert_eq!(outside.zy, 0.0);
    assert_eq!(outside.zxx, 0.0);
    assert_eq!(outside.zxy, 0.0);
    let below = g.sample(x0 - 1.0, (y0 + y1) / 2.0);
    assert_eq!(below.zx, 0.0);
    assert_ne!(below.zy, 0.0);
}

#[test]
fn zero_at_pins_the_reference_point() {
    let mut g = wavy_grid();
    let before = g.sample(40.0, 55.0).z;
    assert!(before.abs() > 1e-6);
    g.zero_at(40.0, 55.0);
    assert!(g.sample(40.0, 55.0).z.abs() < 1e-12);
    assert!((g.sample(10.0, 20.0).z - (wavy_grid().sample(10.0, 20.0).z - before)).abs() < 1e-12);
}

#[test]
fn fade_factor_ramps_and_disables() {
    let fade = Fade::new(1.0, 10.0, 0.15).unwrap();
    assert_eq!(fade.factor(0.2), 1.0);
    assert_eq!(fade.factor(1.0), 1.0);
    assert!((fade.factor(5.5) - 0.5).abs() < 1e-12);
    assert_eq!(fade.factor(10.0), 0.0);
    assert_eq!(fade.factor(50.0), 0.0);
    assert_eq!(fade.dfactor(0.5), 0.0);
    assert!((fade.dfactor(5.5) + 1.0 / 9.0).abs() < 1e-12);
    assert_eq!(fade.dfactor(10.0), 0.0);

    let off = Fade::disabled();
    assert!(off.is_disabled());
    assert_eq!(off.factor(0.0), 1.0);
    assert_eq!(off.factor(1e6), 1.0);
    assert_eq!(off.dfactor(1e6), 0.0);

    assert!(matches!(
        Fade::new(5.0, 5.0, 0.0),
        Err(SurfaceError::FadeBandInverted { .. })
    ));
}

#[test]
fn warp_matches_mainline_composition() {
    let t = transform(Fade::new(1.0, 10.0, 0.15).unwrap());
    let (x, y) = (31.0, 47.0);
    let mesh_z = t.mesh().sample(x, y).z;
    assert!((t.correction_at(x, y, 0.5) - mesh_z).abs() < 1e-12);
    let mid = t.correction_at(x, y, 5.5);
    assert!((mid - (0.5 * (mesh_z - 0.15) + 0.15)).abs() < 1e-12);
    assert!((t.correction_at(x, y, 12.0) - 0.15).abs() < 1e-12);
}

#[test]
fn warp_partials_match_finite_differences() {
    let t = transform(Fade::new(1.0, 10.0, 0.15).unwrap());
    let h = 1e-5;
    for &(x, y, z) in &[(31.0, 47.0, 0.4), (55.0, 62.0, 5.5), (77.7, 95.0, 3.3)] {
        let w = t.warp(x, y, z);
        let fd = |f: &dyn Fn(f64) -> f64| (f(h) - f(-h)) / (2.0 * h);
        assert!((w.wx - fd(&|e| t.warp(x + e, y, z).w)).abs() < 1e-7);
        assert!((w.wy - fd(&|e| t.warp(x, y + e, z).w)).abs() < 1e-7);
        assert!((w.wz - fd(&|e| t.warp(x, y, z + e).w)).abs() < 1e-7);
        assert!((w.wxx - fd(&|e| t.warp(x + e, y, z).wx)).abs() < 1e-6);
        assert!((w.wyy - fd(&|e| t.warp(x, y + e, z).wy)).abs() < 1e-6);
        assert!((w.wxy - fd(&|e| t.warp(x, y + e, z).wx)).abs() < 1e-6);
        assert!((w.wxz - fd(&|e| t.warp(x, y, z + e).wx)).abs() < 1e-6);
        assert!((w.wyz - fd(&|e| t.warp(x, y, z + e).wy)).abs() < 1e-6);
    }
}

#[test]
fn bounds_cover_dense_sampling() {
    let g = wavy_grid();
    let b = g.bounds();
    let (x0, x1) = g.x_range();
    let (y0, y1) = g.y_range();
    let n = 173;
    for j in 0..=n {
        for i in 0..=n {
            let x = x0 + (x1 - x0) * i as f64 / n as f64;
            let y = y0 + (y1 - y0) * j as f64 / n as f64;
            let s = g.sample(x, y);
            assert!(libm::hypot(s.zx, s.zy) <= b.max_gradient);
            assert!(s.zxx.abs() + s.zxy.abs() <= b.max_curvature);
            assert!(s.zyy.abs() + s.zxy.abs() <= b.max_curvature);
            assert!(s.z >= b.z_min && s.z <= b.z_max);
        }
    }
}

#[test]
fn z_spread_bounds_the_true_spread() {
    let g = wavy_grid();
    let max_gradient = g.bounds().max_gradient;
    let (x0, x1, y0, y1) = (15.0, 90.0, 25.0, 100.0);
    let spread = g.z_spread_over(x0, x1, y0, y1, max_gradient);
    let n = 200;
    let mut z_min = f64::INFINITY;
    let mut z_max = f64::NEG_INFINITY;
    for j in 0..=n {
        for i in 0..=n {
            let z = g
                .sample(
                    x0 + (x1 - x0) * i as f64 / n as f64,
                    y0 + (y1 - y0) * j as f64 / n as f64,
                )
                .z;
            z_min = z_min.min(z);
            z_max = z_max.max(z);
        }
    }
    assert!(spread >= z_max - z_min);
    assert!(
        spread <= (z_max - z_min) * 2.0 + 1e-9,
        "bound uselessly loose"
    );
}

#[test]
fn inverse_round_trips_across_all_fade_branches() {
    for fade in [Fade::new(1.0, 10.0, 0.15).unwrap(), Fade::disabled()] {
        let t = transform(fade);
        for &(x, y) in &[(31.0, 47.0), (12.5, 21.0), (77.7, 95.0)] {
            for z_g in [0.0, 0.3, 1.0, 2.5, 5.5, 9.99, 10.0, 25.0] {
                let z_m = z_g + t.correction_at(x, y, z_g);
                let back = t.gcode_z(x, y, z_m);
                assert!(
                    (back - z_g).abs() < 1e-9,
                    "round trip at ({x},{y}) z_g={z_g}: got {back}"
                );
            }
        }
    }
}

#[test]
fn correction_spread_bounds_true_correction_variation() {
    let t = transform(Fade::new(1.0, 10.0, 0.15).unwrap());
    let (x0, x1, y0, y1, z0, z1) = (15.0, 60.0, 25.0, 70.0, 0.5, 4.0);
    let bound = t.correction_spread_over(x0, x1, y0, y1, z0, z1);
    let n = 40;
    let mut w_min = f64::INFINITY;
    let mut w_max = f64::NEG_INFINITY;
    for k in 0..=n {
        for j in 0..=n {
            for i in 0..=n {
                let w = t.correction_at(
                    x0 + (x1 - x0) * i as f64 / n as f64,
                    y0 + (y1 - y0) * j as f64 / n as f64,
                    z0 + (z1 - z0) * k as f64 / n as f64,
                );
                w_min = w_min.min(w);
                w_max = w_max.max(w);
            }
        }
    }
    assert!(bound >= w_max - w_min);
    assert_eq!(
        t.correction_spread_over(x0, x1, y0, y1, 12.0, 15.0),
        0.0,
        "fully faded box has constant correction"
    );
}

#[test]
fn constructor_rejects_bad_grids() {
    assert!(matches!(
        MeshGrid::new(0.0, 0.0, 1.0, 1.0, 1, 3, vec![0.0; 3], TENSION),
        Err(SurfaceError::GridTooSmall { .. })
    ));
    assert!(matches!(
        MeshGrid::new(0.0, 0.0, 1.0, 1.0, 2, 2, vec![0.0; 3], TENSION),
        Err(SurfaceError::PointCountMismatch { .. })
    ));
    assert!(matches!(
        MeshGrid::new(0.0, 0.0, 0.0, 1.0, 2, 2, vec![0.0; 4], TENSION),
        Err(SurfaceError::NonPositiveSpacing { .. })
    ));
    assert!(matches!(
        MeshGrid::new(
            0.0,
            0.0,
            1.0,
            1.0,
            2,
            2,
            vec![0.0, 1.0, f64::NAN, 0.0],
            TENSION
        ),
        Err(SurfaceError::NonFinitePoint { index: 2 })
    ));
}
