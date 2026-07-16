use servo_ident::metrics::{
    compute_metrics, motion_segments, target_motion_segments, torque_summary, DriveSeries,
};

fn series_from(ferr: Vec<i64>, target: Vec<i64>, torque: Vec<i64>, flags: Vec<i64>) -> DriveSeries {
    let n = ferr.len();
    let position_actual: Vec<i64> = (0..n).map(|k| target[k] - ferr[k]).collect();
    DriveSeries {
        following_error: ferr.iter().map(|&v| v as f64).collect(),
        following_error_i: ferr,
        target,
        position_actual,
        torque,
        flags,
        velocity_offset: None,
        torque_offset: None,
    }
}

#[test]
fn motion_segments_splits_on_flag_edges() {
    let flags = vec![0, 2, 2, 0, 0, 2, 2, 2];
    assert_eq!(motion_segments(&flags), vec![(1, 3), (5, 8)]);
}

#[test]
fn target_motion_segments_keeps_long_moves() {
    let fs = 50.0; // close = round(0.02*50) = 1
    let mut target = vec![0i64; 10];
    for k in 1..=6 {
        target.push(k);
    }
    target.extend(std::iter::repeat(6).take(10));
    let segs = target_motion_segments(&target, fs);
    assert_eq!(segs, vec![(10, 16)]);
}

#[test]
fn torque_summary_flags_the_rail() {
    let n = 100;
    let mut torque = vec![100i64; n];
    for t in torque.iter_mut().take(60).skip(20) {
        *t = 1500;
    }
    let flags = vec![2i64; n];
    let d = series_from(vec![0; n], vec![0; n], torque, flags);
    let s = torque_summary(&d, 1400, 1000.0);
    assert!(s.rail_detected);
    assert_eq!(s.rail_samples, 40);
    assert_eq!(s.peak, 1500);
    assert!((s.rail_pct_moving - 40.0).abs() < 1e-9);
    assert!((s.longest_burst_ms - 40.0).abs() < 1e-9);
}

#[test]
fn compute_metrics_settle_and_overshoot() {
    let fs = 1000.0; // hold = 50, close = 20
    let mut target = vec![0i64; 10];
    for k in 1..=100 {
        target.push(k);
    }
    target.extend(std::iter::repeat(100).take(200));
    let n = target.len();
    let mut ferr = vec![0i64; n];
    for f in ferr.iter_mut().take(110).skip(10) {
        *f = 200; // during the move
    }
    for f in ferr.iter_mut().take(140).skip(110) {
        *f = 100; // 30 samples out of the 50-count band
    }
    let flags = vec![2i64; n];
    let d = series_from(ferr, target, vec![0; n], flags);
    let m = compute_metrics(&d, 50, 1400, fs, 0).unwrap();
    assert_eq!(m.moves.len(), 1);
    let mv = &m.moves[0];
    assert_eq!(mv.ferr_peak, 200.0);
    assert_eq!(mv.overshoot, 100.0);
    assert_eq!(mv.settle_ms, Some(30.0));
    assert!(!mv.settle_window_truncated);
    assert_eq!(m.ferr_crosscheck_max, 0);
    assert_eq!(m.torque_saturation_pct, 0.0);
}

#[test]
fn ff_lead_extends_the_error_window_before_the_move() {
    let fs = 1000.0;
    let mut target = vec![0i64; 10];
    for k in 1..=100 {
        target.push(k);
    }
    target.extend(std::iter::repeat(100).take(200));
    let n = target.len();
    let mut ferr = vec![0i64; n];
    ferr[8] = 300; // FF applied 2 cycles ahead of the position command
    for f in ferr.iter_mut().take(110).skip(10) {
        *f = 200;
    }
    let flags = vec![2i64; n];
    let d = series_from(ferr, target, vec![0; n], flags);
    let without_lead = compute_metrics(&d, 50, 1400, fs, 0).unwrap();
    assert_eq!(without_lead.moves[0].ferr_peak, 200.0);
    let with_lead = compute_metrics(&d, 50, 1400, fs, 2).unwrap();
    assert_eq!(with_lead.moves[0].ferr_peak, 300.0);
}

#[test]
fn move_direction_and_signed_mean_use_only_the_moving_window() {
    let fs = 1000.0;
    let n = 220;
    let mut target = vec![0i64; n];
    for k in 1..n {
        let step = if (20..60).contains(&k) {
            1
        } else if (100..140).contains(&k) {
            -1
        } else {
            0
        };
        target[k] = target[k - 1] + step;
    }
    let mut ferr = vec![0i64; n];
    let forward_lead = 18..20;
    let forward_move = 20..60;
    let forward_settle = 60..100;
    let reverse_lead = 98..100;
    let reverse_move = 100..140;
    let reverse_settle = 140..150;
    ferr[forward_lead].fill(1000);
    ferr[forward_move].fill(12);
    ferr[forward_settle].fill(600);
    ferr[reverse_lead].fill(-1000);
    ferr[reverse_move].fill(-8);
    ferr[reverse_settle].fill(-700);
    let d = series_from(ferr, target, vec![0; n], vec![2; n]);

    let metrics = compute_metrics(&d, 50, 1400, fs, 2).unwrap();
    assert_eq!(metrics.moves.len(), 2);
    assert_eq!(metrics.moves[0].direction, 1);
    assert_eq!(metrics.moves[0].ferr_mean_moving, 12.0);
    assert_eq!(metrics.moves[1].direction, -1);
    assert_eq!(metrics.moves[1].ferr_mean_moving, -8.0);
    assert_eq!(metrics.moves[0].ferr_peak, 1000.0);
    assert_eq!(metrics.moves[1].ferr_peak, 1000.0);
}

#[test]
fn merged_zero_net_move_has_zero_direction() {
    let fs = 1000.0;
    let n = 200;
    let mut target = vec![0i64; n];
    for k in 1..n {
        let step = if (20..60).contains(&k) {
            1
        } else if (70..110).contains(&k) {
            -1
        } else {
            0
        };
        target[k] = target[k - 1] + step;
    }
    let d = series_from(vec![3; n], target, vec![0; n], vec![2; n]);

    let metrics = compute_metrics(&d, 50, 1400, fs, 0).unwrap();
    assert_eq!(metrics.moves.len(), 1);
    assert_eq!(metrics.moves[0].direction, 0);
    assert_eq!(metrics.moves[0].ferr_mean_moving, 3.0);
}
