use crate::space::{GcodePos, MachinePos};
use crate::surface::{Fade, MeshGrid, SurfaceTransform};

fn tilted_mesh() -> SurfaceTransform {
    let points: Vec<f64> = (0..9)
        .map(|i| 0.05 * (i % 3) as f64 + 0.02 * (i / 3) as f64)
        .collect();
    let mesh = MeshGrid::new(0.0, 0.0, 50.0, 50.0, 3, 3, points, 0.5).unwrap();
    SurfaceTransform::new(mesh, Fade::new(1.0, 10.0, 0.0).unwrap())
}

#[test]
fn round_trip_is_identity_without_mesh() {
    let g = GcodePos([12.0, -3.0, 4.5]);
    let m = g.to_machine(None);
    assert_eq!(m.0, g.0);
    assert_eq!(m.to_gcode(None).0, g.0);
}

#[test]
fn round_trip_through_mesh_interior_boundary_and_beyond() {
    let t = tilted_mesh();
    for &(x, y) in &[
        (50.0, 50.0),   // interior
        (0.0, 0.0),     // corner
        (100.0, 50.0),  // edge
        (150.0, 150.0), // clamped, outside the grid
        (-40.0, 20.0),  // clamped, outside the grid
    ] {
        for &z in &[0.0, 0.5, 1.0, 3.7, 9.99, 10.0, 25.0] {
            let g = GcodePos([x, y, z]);
            let back = g.to_machine(Some(&t)).to_gcode(Some(&t));
            assert!(
                (back.z() - z).abs() < 1e-12,
                "round trip drifted at ({x}, {y}, {z}): got {}",
                back.z()
            );
            assert_eq!((back.x(), back.y()), (x, y));
        }
    }
}

#[test]
fn contact_touch_cycle_is_a_fixed_point() {
    // The PRINT_START failure mode: each contact touch measures a machine-space
    // trigger, converts it to gcode space, set_position renames the frame, and
    // the counters are re-seeded from the gcode position. If any leg of that
    // loop skips the warp, the frame ratchets by correction_at(x, y) per touch
    // and the sample spread grows until the step dispatcher faults. The full
    // loop must be a fixed point at a probe XY with a NONZERO correction.
    let t = tilted_mesh();
    let probe_xy = (150.0, 150.0);
    let correction = t.correction_at(probe_xy.0, probe_xy.1, 0.0);
    assert!(
        correction.abs() > 1e-3,
        "test mesh must have a nonzero correction at the probe point"
    );
    let physical_trigger_z = 0.012;
    let mut seeded_machine = MachinePos([probe_xy.0, probe_xy.1, physical_trigger_z]);
    for touch in 0..10 {
        let gcode = seeded_machine.to_gcode(Some(&t));
        seeded_machine = gcode.to_machine(Some(&t));
        assert!(
            (seeded_machine.z() - physical_trigger_z).abs() < 1e-12,
            "touch {touch}: seeded machine z drifted to {} (physical {physical_trigger_z})",
            seeded_machine.z()
        );
    }
}

#[test]
fn unwarp_z_state_inverts_the_lowerer_chain_rule() {
    let t = tilted_mesh();
    let (x, y) = (60.0, 40.0);
    let (vx, vy, ax, ay) = (30.0, -12.0, 500.0, -200.0);
    for &(z_g, vz_g, az_g) in &[
        (0.4, -5.0, 0.0),
        (2.5, -8.0, 300.0),
        (9.5, 1.0, -50.0),
        (0.0, 0.0, 0.0),
    ] {
        let w = t.warp(x, y, z_g);
        let z_m = z_g + w.w;
        let vz_m = vz_g * (1.0 + w.wz) + w.wx * vx + w.wy * vy;
        let az_m = az_g * (1.0 + w.wz)
            + 2.0 * (w.wxz * vx + w.wyz * vy) * vz_g
            + w.wxx * vx * vx
            + 2.0 * w.wxy * vx * vy
            + w.wyy * vy * vy
            + w.wx * ax
            + w.wy * ay;
        let (rz, rv, ra) = t.unwarp_z_state([x, y], [vx, vy], [ax, ay], (z_m, vz_m, az_m));
        assert!((rz - z_g).abs() < 1e-11, "pos: {rz} vs {z_g}");
        assert!((rv - vz_g).abs() < 1e-9, "vel: {rv} vs {vz_g}");
        assert!((ra - az_g).abs() < 1e-7, "accel: {ra} vs {az_g}");
    }
}
