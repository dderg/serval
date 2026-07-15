use servo_ident::model::{coulomb_sign, PhysicalParams, Structure, COULOMB_DEADBAND_MM_S};

fn physical_torque(
    frame: &[Vec<f64>],
    p: &PhysicalParams,
    motor: usize,
    acc_mode: &[f64],
    vel_mode: &[f64],
) -> f64 {
    let mut tau = 0.0;
    for k in 0..frame.len() {
        let f = frame[k][motor];
        let coulomb = p.coulomb[k] * coulomb_sign(vel_mode[k]);
        tau += f * (p.mass[k] * acc_mode[k] + p.viscous[k] * vel_mode[k] + coulomb);
    }
    tau
}

#[test]
fn row_dot_theta_matches_unpacked_physics() {
    let cases: &[(Vec<Vec<f64>>, Vec<f64>)] = &[
        (vec![vec![1.0]], vec![0.0123, 0.09, 1.2]),
        (
            vec![vec![0.25, -0.25], vec![0.25, 0.25]],
            vec![0.030, 0.004, 1.0, 0.020, 0.005, 0.9],
        ),
    ];
    for (frame, theta) in cases {
        let s = Structure::new(frame.clone());
        let p = s.unpack(theta);
        let n_modes = s.mode_count();
        let n_slots = s.axis_count();
        #[allow(clippy::cast_precision_loss)]
        let probes: &[(Vec<f64>, Vec<f64>)] = &[
            (vec![1000.0; n_modes], vec![100.0; n_modes]),
            (vec![-500.0; n_modes], vec![-30.0; n_modes]),
            (
                (0..n_modes).map(|i| 800.0 - 1600.0 * i as f64).collect(),
                (0..n_modes).map(|i| 40.0 - 90.0 * i as f64).collect(),
            ),
        ];
        for (acc_mode, vel_mode) in probes {
            let cs: Vec<f64> = vel_mode.iter().map(|&v| coulomb_sign(v)).collect();
            for motor in 0..n_slots {
                let via_row: f64 = s
                    .row(motor, acc_mode, vel_mode, &cs)
                    .iter()
                    .zip(theta)
                    .map(|(r, t)| r * t)
                    .sum();
                let via_physics = physical_torque(frame, &p, motor, acc_mode, vel_mode);
                assert!(
                    (via_row - via_physics).abs() < 1e-12,
                    "frame {frame:?} motor {motor}: row {via_row} vs physics {via_physics}"
                );
            }
        }
    }
}

#[test]
fn identity_frame_row_layout() {
    let s = Structure::new(vec![vec![1.0]]);
    assert_eq!(s.axis_count(), 1);
    assert_eq!(s.mode_count(), 1);
    assert_eq!(s.param_count(), 3);
    let row = s.row(0, &[1000.0], &[100.0], &[coulomb_sign(100.0)]);
    assert_eq!(row, vec![1000.0, 100.0, 1.0]);
    let row_rev = s.row(0, &[1000.0], &[-100.0], &[coulomb_sign(-100.0)]);
    assert_eq!(row_rev, vec![1000.0, -100.0, -1.0]);
    let dead = s.row(
        0,
        &[1000.0],
        &[COULOMB_DEADBAND_MM_S / 2.0],
        &[coulomb_sign(COULOMB_DEADBAND_MM_S / 2.0)],
    );
    assert_eq!(dead[2], 0.0);
}

#[test]
fn corexy_frame_row_projects_with_frame_weights() {
    let s = Structure::new(vec![vec![0.5, -0.5], vec![0.5, 0.5]]);
    let row0 = s.row(0, &[100.0, 60.0], &[10.0, -10.0], &[1.0, -1.0]);
    assert_eq!(row0, vec![50.0, 5.0, 0.5, 30.0, -5.0, -0.5]);
    let row1 = s.row(1, &[100.0, 60.0], &[10.0, -10.0], &[1.0, -1.0]);
    assert_eq!(row1, vec![-50.0, -5.0, -0.5, 30.0, -5.0, -0.5]);
}

#[test]
fn awd_frame_pairs_parallel_columns() {
    let s = Structure::new(vec![
        vec![0.25, -0.25, -0.25, -0.25],
        vec![0.25, -0.25, 0.25, 0.25],
    ]);
    assert_eq!(s.pairs(), vec![(0, 1, -1.0), (2, 3, 1.0)]);
}

#[test]
fn corexy_frame_has_no_pairs() {
    let s = Structure::new(vec![vec![0.5, -0.5], vec![0.5, 0.5]]);
    assert!(s.pairs().is_empty(), "corexy columns are not parallel");
    let s2 = Structure::new(vec![vec![0.5, 0.5], vec![0.5, -0.5]]);
    assert!(s2.pairs().is_empty());
}

#[test]
fn identity_frame_has_no_pairs() {
    let s = Structure::new(vec![vec![1.0]]);
    assert!(s.pairs().is_empty());
}

#[test]
#[should_panic(expected = "|λ|")]
fn parallel_column_with_non_unit_scale_panics() {
    let s = Structure::new(vec![vec![0.25, 0.5], vec![0.25, 0.5]]);
    let _ = s.pairs();
}

#[test]
fn param_layout_is_grouped_per_mode() {
    let s = Structure::new(vec![vec![0.25, -0.25], vec![0.25, 0.25]]);
    assert_eq!(s.param_count(), 6);
    let theta = vec![0.030, 0.004, 1.0, 0.020, 0.005, 0.9];
    let p = s.unpack(&theta);
    assert_eq!(p.mass, vec![0.030, 0.020]);
    assert_eq!(p.viscous, vec![0.004, 0.005]);
    assert_eq!(p.coulomb, vec![1.0, 0.9]);
    assert_eq!(s.pack(&p), theta);
}
