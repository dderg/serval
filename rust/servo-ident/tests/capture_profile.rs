use servo_ident::capture::{
    parse_capture_csv, steady_accel_keep, tracking_keep, Capture, PlateauOptions, TrackingOptions,
};
use servo_ident::model::PhysicalParams;
use servo_ident::profile_out::{c0006_recommendation, render_profile};

#[test]
fn parses_commanded_kinematics_columns() {
    let mut csv = String::from("t,accel_x,vel_x,vel_act_x,torque_x\n");
    for k in 0..100 {
        let t = k as f64 * 0.001;
        csv.push_str(&format!(
            "{t},{},{},{},{}\n",
            1000.0,
            50.0 * t,
            50.0 * t - 0.5,
            12.0
        ));
    }
    let cap = parse_capture_csv(&csv, &["x"]).unwrap();
    assert_eq!(cap.torque[0][50], 12.0);
    assert_eq!(cap.acc[0][50], 1000.0);
    assert!((cap.vel[0][50] - 50.0 * 0.050).abs() < 1e-9);
    assert!((cap.vel_act[0][50] - (50.0 * 0.050 - 0.5)).abs() < 1e-9);
    assert_eq!(cap.acc[0].len(), 100);
}

#[test]
fn rejects_missing_column() {
    // accel, vel, vel_act, and torque columns are all required.
    assert!(parse_capture_csv("t,accel_x,vel_x,vel_act_x\n0,0,0,0\n", &["x"]).is_err());
    assert!(parse_capture_csv("t,accel_x,vel_x,torque_x\n0,0,0,0\n", &["x"]).is_err());
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
        vel_act: vec![vec![0.0; n]],
        torque: vec![vec![5.0; n]],
        pos: Vec::new(),
    };
    let keep_mask = steady_accel_keep(&cap.t, &cap.acc, &PlateauOptions::default());
    let kept: Vec<f64> = (0..cap.t.len())
        .filter(|&k| keep_mask[k])
        .map(|k| cap.acc[0][k])
        .collect();
    assert!(!kept.is_empty());
    // every kept cycle is on the flat plateau, none from the ramp.
    assert!(kept.iter().all(|&a| (a - 1000.0).abs() < 1.0));
    // 60-wide plateau minus the 12-cycle settle window ≈ 48 kept.
    assert!(kept.len() >= 40 && kept.len() <= 60, "kept {}", kept.len());
}

#[test]
fn tracking_mask_drops_stiction_and_overshoot() {
    // 1 kHz stroke launch: commanded velocity ramps 0→300; the motor sticks
    // at zero for the first 40 cycles, overshoots to ~1.6x for the next 30,
    // then tracks within a small lag.
    let n = 200;
    let t: Vec<f64> = (0..n).map(|k| k as f64 * 0.001).collect();
    let vel: Vec<f64> = (0..n).map(|k| (k as f64 * 3.0).min(300.0)).collect();
    let vel_act: Vec<f64> = (0..n)
        .map(|k| match k {
            0..=39 => 0.0,
            40..=69 => 480.0,
            _ => (k as f64 * 3.0).min(300.0) - 20.0,
        })
        .collect();
    let cap = Capture {
        t,
        acc: vec![vec![3000.0; n]],
        vel: vec![vel],
        vel_act: vec![vel_act],
        torque: vec![vec![100.0; n]],
        pos: Vec::new(),
    };
    // tol = 0.2 * 300 = 60 mm/s: the first cycles of the stuck phase pass
    // (commanded velocity still small), the rest of the stick and the whole
    // overshoot are dropped, the lagging-but-tracking tail is kept.
    let keep_mask = tracking_keep(&cap.vel, &cap.vel_act, &TrackingOptions::default());
    let kept: Vec<f64> = (0..cap.t.len())
        .filter(|&k| keep_mask[k])
        .map(|k| cap.t[k])
        .collect();
    assert_eq!(kept.len(), 21 + (n - 70));
    assert!(
        kept.iter().all(|&t| t <= 0.020 + 1e-9 || t >= 0.070 - 1e-9),
        "kept a stuck/overshoot sample"
    );
}

#[test]
fn renders_loadable_v3_profile_without_pairs() {
    let p = PhysicalParams {
        mass: vec![0.0123, 0.0119],
        viscous: vec![0.09, 0.11],
        coulomb: vec![160.0, 175.0],
    };
    let frame = vec![
        vec![0.25, -0.25, -0.25, -0.25],
        vec![0.25, -0.25, 0.25, 0.25],
    ];
    let toml_text = render_profile(
        &p,
        &["motor_a", "motor_a1", "motor_b", "motor_b1"],
        &["x", "y"],
        &frame,
        &[0.8, 0.7, 0.8, 0.9],
        &[],
    );
    assert!(toml_text.contains("version = 3"), "{toml_text}");
    assert!(
        toml_text.contains("axes = [\"motor_a\", \"motor_a1\", \"motor_b\", \"motor_b1\"]"),
        "{toml_text}"
    );
    assert!(toml_text.contains("modes = [\"x\", \"y\"]"), "{toml_text}");
    assert!(
        toml_text.contains("frame = [[0.25, -0.25, -0.25, -0.25], [0.25, -0.25, 0.25, 0.25]]"),
        "{toml_text}"
    );
    assert!(toml_text.contains("mass = [0.0123, 0.0119]"), "{toml_text}");
    assert!(toml_text.contains("viscous = [0.09, 0.11]"), "{toml_text}");
    assert!(
        toml_text.contains("coulomb = [160.0, 175.0]"),
        "{toml_text}"
    );
    assert!(
        toml_text.contains("fit_rms_residual = [0.8, 0.7, 0.8, 0.9]"),
        "{toml_text}"
    );
    assert!(!toml_text.contains("[[pair]]"), "{toml_text}");
    assert!(!toml_text.contains("coulomb_fwd"), "{toml_text}");
    assert!(!toml_text.contains("coulomb_deadband"), "{toml_text}");
}

#[test]
fn renders_v3_profile_with_pairs() {
    use servo_ident::profile_out::PairSplit;
    let p = PhysicalParams {
        mass: vec![0.012, 0.011],
        viscous: vec![0.09, 0.11],
        coulomb: vec![160.0, 175.0],
    };
    let frame = vec![
        vec![0.25, -0.25, -0.25, -0.25],
        vec![0.25, -0.25, 0.25, 0.25],
    ];
    let pairs = [
        PairSplit {
            first: 0,
            second: 1,
            w: [0.02, -0.0002, 0.05, 0.0, -0.01, 0.0001],
        },
        PairSplit {
            first: 2,
            second: 3,
            w: [0.03, 0.0004, 0.06, 0.0, -0.02, 0.0],
        },
    ];
    let toml_text = render_profile(
        &p,
        &["motor_a", "motor_a1", "motor_b", "motor_b1"],
        &["x", "y"],
        &frame,
        &[0.8, 0.7, 0.8, 0.9],
        &pairs,
    );
    assert!(toml_text.contains("version = 3"), "{toml_text}");
    assert_eq!(
        toml_text.matches("[[pair]]").count(),
        2,
        "two pair tables: {toml_text}"
    );
    assert!(
        toml_text.contains("slots = [\"motor_a\", \"motor_a1\"]"),
        "{toml_text}"
    );
    assert!(
        toml_text.contains("slots = [\"motor_b\", \"motor_b1\"]"),
        "{toml_text}"
    );
    assert!(
        toml_text.contains("split_inertial = [0.02, -0.0002]"),
        "{toml_text}"
    );
    assert!(
        toml_text.contains("split_viscous  = [0.05, 0.0]"),
        "{toml_text}"
    );
    assert!(
        toml_text.contains("split_coulomb  = [-0.01, 0.0001]"),
        "{toml_text}"
    );
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
        mass: vec![2.0],
        viscous: vec![0.0],
        coulomb: vec![1.0],
    };
    let toml_text = render_profile(&p, &["x"], &["x"], &[vec![1.0]], &[1.0], &[]);
    assert!(toml_text.contains("mass = [2.0]"), "{toml_text}");
    assert!(toml_text.contains("viscous = [0.0]"), "{toml_text}");
    assert!(toml_text.contains("frame = [[1.0]]"), "{toml_text}");
    assert!(toml_text.contains("coulomb = [1.0]"), "{toml_text}");
    assert!(
        toml_text.contains("fit_rms_residual = [1.0]"),
        "{toml_text}"
    );
}
