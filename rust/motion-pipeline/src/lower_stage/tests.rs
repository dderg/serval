use super::rest_z_warp;
use geometry::{Fade, MeshGrid, MeshGridSpec, SurfaceTransform};

fn flat_mesh() -> SurfaceTransform {
    let mut mesh = MeshGrid::new(
        MeshGridSpec {
            x_min: 0.0,
            y_min: 0.0,
            dx: 100.0,
            dy: 100.0,
            nx: 3,
            ny: 3,
            tension: 0.2,
        },
        vec![0.1; 9],
    )
    .unwrap();
    mesh.zero_at(0.0, 0.0);
    SurfaceTransform::new(mesh, Fade::new(1.0, 10.0, 0.0).unwrap())
}

#[test]
fn without_a_mesh_a_rest_hold_has_no_warp_whatever_the_odometer() {
    assert_eq!(rest_z_warp(None, &[]), 0.0);
}

#[test]
fn a_mesh_warps_a_rest_hold_from_the_x_y_z_odometer() {
    let warp = rest_z_warp(Some(&flat_mesh()), &[50.0, 50.0, 0.0, 12.0]);
    assert!(warp.is_finite());
}

#[test]
#[should_panic(expected = "a mesh needs the x, y and z odometer")]
fn a_mesh_with_a_short_odometer_is_refused_not_read_as_zero_positions() {
    let _ = rest_z_warp(Some(&flat_mesh()), &[50.0, 50.0]);
}
