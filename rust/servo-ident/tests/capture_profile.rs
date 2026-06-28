use servo_ident::capture::{parse_capture_csv, restrict_to_steady_accel, Capture, PlateauOptions};
use servo_ident::model::PhysicalParams;
use servo_ident::profile_out::{c0006_recommendation, render_profile};

#[test]
fn parses_commanded_kinematics_columns() {
    let mut csv = String::from("t,accel_x,vel_x,torque_x\n");
    for k in 0..100 {
        let t = k as f64 * 0.001;
        csv.push_str(&format!("{t},{},{},{}\n", 1000.0, 50.0 * t, 12.0));
    }
    let cap = parse_capture_csv(&csv, &["x"]).unwrap();
    assert_eq!(cap.torque[0][50], 12.0);
    assert_eq!(cap.acc[0][50], 1000.0);
    assert!((cap.vel[0][50] - 50.0 * 0.050).abs() < 1e-9);
    assert_eq!(cap.acc[0].len(), 100);
}

#[test]
fn rejects_missing_column() {
    // accel, vel, and torque columns are all required now.
    assert!(parse_capture_csv("t,accel_x,vel_x\n0,0,0\n", &["x"]).is_err());
    assert!(parse_capture_csv("t,target_x,torque_x\n0,0,0\n", &["x"]).is_err());
}

#[test]
fn steady_accel_mask_keeps_plateau_drops_ramp() {
    // 1 kHz: 30 cycles of accel ramping 0→1000, then 60 cycles flat at 1000.
    let mut t = Vec::new();
    let mut acc = Vec::new();
    for k in 0..90 {
        t.push(k as f64 * 0.001);
        acc.push(if k < 30 {
            1000.0 * k as f64 / 30.0
        } else {
            1000.0
        });
    }
    let n = t.len();
    let cap = Capture {
        t,
        acc: vec![acc],
        vel: vec![vec![0.0; n]],
        torque: vec![vec![5.0; n]],
    };
    let masked = restrict_to_steady_accel(&cap, &PlateauOptions::default());
    assert!(!masked.t.is_empty());
    // every kept cycle is on the flat plateau, none from the ramp.
    assert!(masked.acc[0].iter().all(|&a| (a - 1000.0).abs() < 1.0));
    // 60-wide plateau minus the 12-cycle settle window ≈ 48 kept.
    assert!(
        masked.acc[0].len() >= 40 && masked.acc[0].len() <= 60,
        "kept {}",
        masked.acc[0].len()
    );
}

#[test]
fn renders_loadable_profile() {
    let p = PhysicalParams {
        mass: vec![vec![0.0123]],
        viscous: vec![0.0045],
        coulomb_fwd: vec![1.2],
        coulomb_rev: vec![-1.1],
    };
    let toml_text = render_profile(&p, &["x"], &[0.8]);
    assert!(toml_text.contains("version = 1"));
    assert!(toml_text.contains("coulomb_deadband_mm_s = 0.5"));
    assert!(toml_text.contains("mass = [[0.0123]]"));
}

#[test]
fn c0006_matches_hand_calculation() {
    let j_total = 0.0123 * (1.27 / 1000.0) * 40.0 / (2.0 * std::f64::consts::PI);
    let rotor = 0.269e-4;
    let expect = (j_total - rotor) / rotor * 100.0;
    let got = c0006_recommendation(0.0123, 1.27, 40.0, rotor);
    assert!((got - expect).abs() < 1e-9, "{got} vs {expect}");
    assert!((got - 269.69).abs() < 0.01, "independent pin: {got}");
}

#[test]
fn renders_integer_valued_floats_as_toml_floats() {
    let p = PhysicalParams {
        mass: vec![vec![2.0]],
        viscous: vec![0.0],
        coulomb_fwd: vec![1.0],
        coulomb_rev: vec![-1.0],
    };
    let toml_text = render_profile(&p, &["x"], &[1.0]);
    assert!(toml_text.contains("mass = [[2.0]]"), "{toml_text}");
    assert!(toml_text.contains("viscous = [0.0]"), "{toml_text}");
    assert!(
        toml_text.contains("fit_rms_residual = [1.0]"),
        "{toml_text}"
    );
}
