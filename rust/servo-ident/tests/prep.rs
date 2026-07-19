use servo_ident::capture::{
    steady_accel_keep, tracking_keep, Capture, PlateauOptions, TrackingOptions,
};
use servo_ident::fit::{fit, residual_by_motor, FitInput, FitOptions};
use servo_ident::model::Structure;
use servo_ident::prep::{
    band_limited_rms, biquad_in_place, median_dt, modal_biquad, prep, segments, sinc_kernel,
    ModalMode, PrepOptions,
};

const DT: f64 = 0.00025;

fn identity() -> Structure {
    Structure::new(vec![vec![1.0]])
}

fn white(k: usize) -> f64 {
    let h = (k as u32).wrapping_mul(2654435761);
    f64::from(h % 1000) / 1000.0 - 0.5
}

/// Trapezoid strokes with dwell gaps (only motion samples, like the ident
/// CSV): accel to cruise, cruise, decel to zero, alternating direction.
fn strokes(accel: f64, vmax: f64, cruise_s: f64, reps: usize) -> (Vec<f64>, Vec<f64>, Vec<f64>) {
    let mut t = Vec::new();
    let mut acc = Vec::new();
    let mut vel = Vec::new();
    let mut now = 0.0;
    for rep in 0..reps {
        let speed = vmax * (0.4 + 0.15 * (rep % 5) as f64);
        let ramp = (speed / accel / DT) as usize;
        let cruise = (cruise_s / DT) as usize;
        let dir = if rep % 2 == 0 { 1.0 } else { -1.0 };
        let mut v = 0.0;
        for phase in 0..(2 * ramp + cruise) {
            let a = if phase < ramp {
                dir * accel
            } else if phase < ramp + cruise {
                0.0
            } else {
                -dir * accel
            };
            v += a * DT;
            t.push(now);
            acc.push(a);
            vel.push(v);
            now += DT;
        }
        now += 0.1;
    }
    (t, acc, vel)
}

fn make_capture(delay_samples: usize, ripple_period_mm: f64) -> (Capture, f64, f64, f64) {
    let (m, b, coul) = (0.008, 0.09, 205.0);
    let (t, acc, vel) = strokes(10000.0, 500.0, 0.4, 8);
    let n = t.len();
    let clean: Vec<f64> = (0..n)
        .map(|k| {
            let c = if vel[k] > 0.5 {
                coul
            } else if vel[k] < -0.5 {
                -coul
            } else {
                0.0
            };
            m * acc[k] + b * vel[k] + c
        })
        .collect();
    let mut ring = vec![0.0; n];
    for k in 1..n {
        let jerk = acc[k] - acc[k - 1];
        if jerk.abs() > 1.0 {
            let sign = jerk.signum();
            let onset = k + delay_samples;
            for j in onset..n.min(onset + 400) {
                let tau = (j - onset) as f64 * DT;
                ring[j] += sign
                    * 60.0
                    * libm::exp(-tau / 0.02)
                    * libm::cos(2.0 * std::f64::consts::PI * 55.0 * tau);
            }
        }
    }
    let mut pos = 0.0;
    let mut torque = vec![0.0; n];
    for k in 0..n {
        pos += vel[k] * DT;
        let src = k.saturating_sub(delay_samples);
        let ripple = 30.0 * libm::sin(2.0 * std::f64::consts::PI * pos / ripple_period_mm);
        torque[k] = (clean[src] + ripple + ring[k] + 2.0 * white(k)).round();
    }
    (
        Capture {
            t,
            acc: vec![acc],
            vel: vec![vel.clone()],
            vel_act: vec![vel],
            torque: vec![torque],
        },
        m,
        b,
        coul,
    )
}

fn run_fit(cap: &Capture, opts: &PrepOptions) -> (servo_ident::fit::FitResult, f64) {
    run_fit_with(cap, opts, &FitOptions::default())
}

fn run_fit_with(
    cap: &Capture,
    opts: &PrepOptions,
    fit_opts: &FitOptions,
) -> (servo_ident::fit::FitResult, f64) {
    let structure = identity();
    let pp = prep(cap, &structure, opts);
    let track = tracking_keep(&cap.vel, &cap.vel_act, &TrackingOptions::default());
    let plateau = steady_accel_keep(&cap.t, &cap.acc, &PlateauOptions::default());
    let keep: Vec<usize> = (0..cap.t.len())
        .filter(|&k| pp.valid[k] && track[k] && plateau[k])
        .collect();
    assert!(keep.len() > 1000, "mask kept only {} samples", keep.len());
    let pick = |cols: &[Vec<f64>]| -> Vec<Vec<f64>> {
        cols.iter()
            .map(|c| keep.iter().map(|&k| c[k]).collect())
            .collect()
    };
    let input = FitInput {
        structure,
        acc_mode: pick(&pp.acc_mode),
        vel_mode: pick(&pp.vel_mode),
        cs_mode: pick(&pp.cs_mode),
        snap_mode: vec![],
        torque: pick(&pp.torque),
        extra: pp.extra.iter().map(|cols| pick(cols)).collect(),
    };
    (fit(&input, fit_opts).unwrap(), pp.delay_s)
}

#[test]
fn kernel_preserves_dc_and_kills_out_of_band() {
    let k = sinc_kernel(30.0, DT);
    assert_eq!(k.len() % 2, 1);
    let dc: f64 = k.iter().sum();
    assert!((dc - 1.0).abs() < 1e-9);
    let n = 8000;
    let sine: Vec<f64> = (0..n)
        .map(|i| libm::sin(2.0 * std::f64::consts::PI * 300.0 * i as f64 * DT))
        .collect();
    let filtered: Vec<f64> = (k.len()..n - k.len())
        .map(|i| {
            let half = k.len() / 2;
            k.iter()
                .enumerate()
                .map(|(j, &kv)| kv * sine[i + j - half])
                .sum::<f64>()
        })
        .collect();
    let amp = filtered.iter().fold(0.0_f64, |m, &v| m.max(v.abs()));
    assert!(amp < 0.02, "300 Hz leaked through 30 Hz cutoff: {amp}");
}

#[test]
fn segments_split_on_time_gaps() {
    let (cap, _, _, _) = make_capture(0, 40.0);
    let dt = median_dt(&cap.t);
    let segs = segments(&cap.t, dt);
    assert_eq!(segs.len(), 8);
    let total: usize = segs.iter().map(|s| s.len()).sum();
    assert_eq!(total, cap.t.len());
}

#[test]
fn recovers_injected_delay() {
    let (cap, _, _, _) = make_capture(4, 40.0);
    let pp = prep(&cap, &identity(), &PrepOptions::default());
    assert!(
        (pp.delay_s - 4.0 * DT).abs() < DT / 2.0,
        "delay {} vs injected {}",
        pp.delay_s,
        4.0 * DT
    );
}

#[test]
fn reversal_neighborhoods_are_blanked() {
    let (cap, _, _, _) = make_capture(0, 40.0);
    let pp = prep(&cap, &identity(), &PrepOptions::default());
    let blank = (PrepOptions::default().blank_reversal_s / DT) as usize;
    for k in 0..cap.t.len() {
        if cap.vel[0][k].abs() <= 0.5 {
            for j in k.saturating_sub(blank / 2)..(k + blank / 2).min(cap.t.len()) {
                if (cap.t[j] - cap.t[k]).abs() < 0.05 {
                    assert!(!pp.valid[j], "sample {j} near deadband {k} not blanked");
                }
            }
        }
    }
}

#[test]
fn prepped_fit_recovers_truth_through_pollution() {
    let (cap, m, b, coul) = make_capture(4, 40.0);
    let opts = PrepOptions {
        ripple_period_mm: Some(40.0),
        ..PrepOptions::default()
    };
    let (r, delay) = run_fit(&cap, &opts);
    assert!(delay > 0.0);
    let p = &r.params;
    assert!(
        (p.mass[0] - m).abs() < 0.02 * m,
        "mass {} vs {m}",
        p.mass[0]
    );
    assert!(
        (p.viscous[0] - b).abs() < 0.05 * b,
        "viscous {} vs {b}",
        p.viscous[0]
    );
    assert!(
        (p.coulomb[0] - coul).abs() < 0.02 * coul,
        "coulomb {} vs {coul}",
        p.coulomb[0]
    );
    assert!(
        r.rms_residual < 8.0,
        "residual {} with ripple columns and band-limiting",
        r.rms_residual
    );
    let amp = (r.extra_params[0][0].powi(2) + r.extra_params[0][1].powi(2)).sqrt();
    assert!(
        (amp - 30.0).abs() < 3.0,
        "ripple amplitude {amp} vs injected 30"
    );
    assert!(r.param_stderr.iter().all(|se| se.is_finite() && *se > 0.0));
}

#[test]
fn ripple_columns_debias_the_mass() {
    let (cap, m, _, _) = make_capture(4, 40.0);
    let with_cols = PrepOptions {
        ripple_period_mm: Some(40.0),
        ..PrepOptions::default()
    };
    let (r_with, _) = run_fit(&cap, &with_cols);
    let (r_without, _) = run_fit(&cap, &PrepOptions::default());
    let err_with = (r_with.params.mass[0] - m).abs() / m;
    let err_without = (r_without.params.mass[0] - m).abs() / m;
    assert!(
        err_without > 0.05,
        "stroke-locked ripple should bias the plain fit; got {err_without}"
    );
    assert!(err_with < 0.02, "ripple-column fit error {err_with}");
}

#[test]
fn in_band_residual_excludes_out_of_band_pollution() {
    let (cap, _, _, _) = make_capture(4, 40.0);
    let opts = PrepOptions {
        ripple_period_mm: Some(40.0),
        ..PrepOptions::default()
    };
    let pp = prep(&cap, &identity(), &opts);
    let (r, _) = run_fit(&cap, &opts);
    let full = FitInput {
        structure: identity(),
        acc_mode: pp.acc_mode.clone(),
        vel_mode: pp.vel_mode.clone(),
        cs_mode: pp.cs_mode.clone(),
        snap_mode: vec![],
        torque: pp.torque.clone(),
        extra: pp.extra.clone(),
    };
    let res = residual_by_motor(&full, &r.params, &r.snap_params, &r.extra_params);
    let track = tracking_keep(&cap.vel, &cap.vel_act, &TrackingOptions::default());
    let plateau = steady_accel_keep(&cap.t, &cap.acc, &PlateauOptions::default());
    let keep: Vec<bool> = (0..cap.t.len())
        .map(|k| pp.valid[k] && track[k] && plateau[k])
        .collect();
    let inband = band_limited_rms(&res, &pp.t, &keep, 30.0);
    assert!(
        inband < r.rms_residual,
        "in-band {inband} vs raw {}",
        r.rms_residual
    );
    assert!(inband < 5.0, "in-band residual {inband}");
}

fn corexy_capture(opposing_second_segment: bool) -> Capture {
    let (t1, acc1, vel1) = strokes(10000.0, 500.0, 0.4, 2);
    let n1 = t1.len();
    let gap = t1[n1 - 1] + 0.1;
    let (t2, acc2, vel2) = strokes(10000.0, 500.0, 0.4, 2);
    let t: Vec<f64> = t1
        .iter()
        .chain(t2.iter().map(|v| *v + gap).collect::<Vec<_>>().iter())
        .copied()
        .collect();
    let second_b: Vec<f64> = if opposing_second_segment {
        vel2.iter().map(|v| -v).collect()
    } else {
        vel2.clone()
    };
    let second_b_acc: Vec<f64> = if opposing_second_segment {
        acc2.iter().map(|a| -a).collect()
    } else {
        acc2.clone()
    };
    let vel_a: Vec<f64> = vel1.iter().chain(vel2.iter()).copied().collect();
    let vel_b: Vec<f64> = vel1.iter().chain(second_b.iter()).copied().collect();
    let acc_a: Vec<f64> = acc1.iter().chain(acc2.iter()).copied().collect();
    let acc_b: Vec<f64> = acc1.iter().chain(second_b_acc.iter()).copied().collect();
    let n = t.len();
    Capture {
        t,
        acc: vec![acc_a, acc_b],
        vel: vec![vel_a.clone(), vel_b.clone()],
        vel_act: vec![vel_a, vel_b],
        torque: vec![vec![0.0; n], vec![0.0; n]],
    }
}

#[test]
fn axis_aligned_strokes_survive_the_idle_modes_blanking() {
    let cap = corexy_capture(false);
    let structure = Structure::new(vec![vec![0.5, 0.5], vec![0.5, -0.5]]);
    let pp = prep(&cap, &structure, &PrepOptions::default());
    let kept = pp.valid.iter().filter(|v| **v).count();
    assert!(
        kept * 2 > cap.t.len(),
        "idle y mode blanked axis-aligned strokes: kept {kept}/{}",
        cap.t.len()
    );
}

#[test]
fn a_mode_active_in_the_segment_still_blanks_its_deadband() {
    let n = 8000;
    let v = 300.0;
    let t: Vec<f64> = (0..n).map(|k| k as f64 * DT).collect();
    let vel_a = vec![v; n];
    let vel_b: Vec<f64> = (0..n)
        .map(|k| -3.0 * v + 6.0 * v * k as f64 / (n - 1) as f64)
        .collect();
    let acc_b = vec![6.0 * v / ((n - 1) as f64 * DT); n];
    let cap = Capture {
        t,
        acc: vec![vec![0.0; n], acc_b],
        vel: vec![vel_a.clone(), vel_b.clone()],
        vel_act: vec![vel_a.clone(), vel_b.clone()],
        torque: vec![vec![0.0; n], vec![0.0; n]],
    };
    let structure = Structure::new(vec![vec![0.5, 0.5], vec![0.5, -0.5]]);
    let pp = prep(&cap, &structure, &PrepOptions::default());
    let mut crossing_blanked = 0;
    let mut crossing_total = 0;
    for k in 0..n {
        let vx = 0.5 * (vel_a[k] + vel_b[k]);
        let vy = 0.5 * (vel_a[k] - vel_b[k]);
        if vx.abs() <= 0.5 || vy.abs() <= 0.5 {
            crossing_total += 1;
            if !pp.valid[k] {
                crossing_blanked += 1;
            }
        }
    }
    assert!(crossing_total > 0, "test capture has no mode crossings");
    assert_eq!(
        crossing_blanked, crossing_total,
        "deadband samples of active modes must be blanked"
    );
    assert!(pp.valid[n / 2], "sample far from any crossing was blanked");
}

#[test]
fn modal_biquad_has_unit_dc_gain_and_resonant_peak() {
    let (fz, zeta) = (100.0, 0.04);
    let coef = modal_biquad(fz, zeta, DT);
    let mut step = vec![1.0; 8000];
    biquad_in_place(&mut step, coef);
    assert!((step[7999] - 1.0).abs() < 1e-6, "DC gain must be 1");
    let n = 80000;
    let mut sine: Vec<f64> = (0..n)
        .map(|k| libm::sin(2.0 * std::f64::consts::PI * fz * k as f64 * DT))
        .collect();
    biquad_in_place(&mut sine, coef);
    let peak = sine[n / 2..].iter().fold(0.0_f64, |m, &v| m.max(v.abs()));
    let q = 1.0 / (2.0 * zeta);
    assert!(
        (peak - q).abs() / q < 0.05,
        "gain at fz must be ~Q={q}, got {peak}"
    );
}

/// Torque carries a modal-filtered snap component (belt compliance ringing
/// through the measured resonance). The modal-shaped fit must recover the
/// rigid mass unbiased AND the compliance coefficient - the failure mode it
/// guards is exactly the accel-dependent mass drift.
#[test]
fn modal_snap_recovers_the_injected_compliance_term() {
    let (fz, zeta, c_snap) = (90.0, 0.05, 3.0e-8);
    let (m, b, coul) = (0.008, 0.09, 205.0);
    let (t, acc, vel) = strokes(10000.0, 500.0, 0.4, 8);
    let n = t.len();
    let mut snap = vec![0.0; n];
    for seg in segments(&t, DT) {
        for k in seg.start + 1..seg.end.saturating_sub(1) {
            snap[k] = (acc[k + 1] - 2.0 * acc[k] + acc[k - 1]) / (DT * DT);
        }
    }
    let mut modal = snap.clone();
    let coef = modal_biquad(fz, zeta, DT);
    for seg in segments(&t, DT) {
        biquad_in_place(&mut modal[seg], coef);
    }
    let torque: Vec<f64> = (0..n)
        .map(|k| {
            let cs = if vel[k] > 0.5 {
                coul
            } else if vel[k] < -0.5 {
                -coul
            } else {
                0.0
            };
            (m * acc[k] + b * vel[k] + cs + c_snap * modal[k] + 2.0 * white(k)).round()
        })
        .collect();
    let cap = Capture {
        t,
        acc: vec![acc],
        vel: vec![vel.clone()],
        vel_act: vec![vel],
        torque: vec![torque],
    };
    let structure = identity();
    let opts = PrepOptions {
        max_delay_s: 0.0,
        modal: vec![ModalMode {
            mode: 0,
            freq_hz: fz,
            zeta,
        }],
        ..PrepOptions::default()
    };
    let pp = prep(&cap, &structure, &opts);
    let track = tracking_keep(&cap.vel, &cap.vel_act, &TrackingOptions::default());
    let plateau = steady_accel_keep(&cap.t, &cap.acc, &PlateauOptions::default());
    let keep: Vec<usize> = (0..cap.t.len())
        .filter(|&k| pp.valid[k] && track[k] && plateau[k])
        .collect();
    assert!(keep.len() > 1000, "mask kept only {} samples", keep.len());
    let pick = |cols: &[Vec<f64>]| -> Vec<Vec<f64>> {
        cols.iter()
            .map(|c| keep.iter().map(|&k| c[k]).collect())
            .collect()
    };
    let input = FitInput {
        structure,
        acc_mode: pick(&pp.acc_mode),
        vel_mode: pick(&pp.vel_mode),
        cs_mode: pick(&pp.cs_mode),
        snap_mode: pick(&pp.snap_mode),
        torque: pick(&pp.torque),
        extra: pp.extra.iter().map(|cols| pick(cols)).collect(),
    };
    let r = fit(&input, &FitOptions::default()).unwrap();
    assert!(
        (r.params.mass[0] - m).abs() / m < 0.02,
        "mass {} vs {m} - modal term must keep mass unbiased",
        r.params.mass[0]
    );
    assert!(
        (r.snap_params[0] - c_snap).abs() / c_snap < 0.1,
        "snap coefficient {} vs injected {c_snap}",
        r.snap_params[0]
    );
}
