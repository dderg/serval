use servo_ident::capture::{
    steady_accel_keep, tracking_keep, Capture, PlateauOptions, TrackingOptions,
};
use servo_ident::fit::{fit, residual_by_motor, FitInput, FitOptions};
use servo_ident::model::Structure;
use servo_ident::prep::{
    band_limited_rms, biquad_in_place, direction_split, frame_pairs, median_dt, modal_biquad,
    onset_bias, prep, segments, sinc_kernel, transient_rms, ModalMode, PrepOptions, TransientKind,
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
            ferr: vec![vec![0.0; n]],
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
        ferr_mode: pick(&pp.ferr_mode),
        jerk_mode: pick(&pp.jerk_mode),
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
        ferr_mode: pp.ferr_mode.clone(),
        jerk_mode: pp.jerk_mode.clone(),
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
        ferr: vec![vec![0.0; n], vec![0.0; n]],
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
        ferr: vec![vec![0.0; n], vec![0.0; n]],
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
        ferr: vec![vec![0.0; n]],
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
        ferr_mode: pick(&pp.ferr_mode),
        jerk_mode: pick(&pp.jerk_mode),
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

/// At high accel the velocity steps over the coulomb deadband between
/// samples (12 mm/s per sample at 50k mm/s2, 4 kHz), so no sample lands
/// inside it - reversal blanking must key on the sign flip itself, or every
/// zigzag reversal's loop transient enters the fit (which biased mass ~35%
/// low on the bench).
#[test]
fn reversal_blanking_catches_deadband_skipping_flips() {
    let accel = 50_000.0;
    let n = 400;
    let mid = 200.5;
    let t: Vec<f64> = (0..n).map(|k| k as f64 * DT).collect();
    let acc = vec![vec![-accel; n]];
    let vel: Vec<f64> = (0..n).map(|k| (mid - k as f64) * DT * accel).collect();
    assert!(
        vel.iter().all(|v| v.abs() > 1.0),
        "the reversal must skip the deadband for this test to bite"
    );
    let cap = Capture {
        t,
        acc,
        vel: vec![vel.clone()],
        vel_act: vec![vel],
        torque: vec![vec![0.0; n]],
        ferr: vec![vec![0.0; n]],
    };
    let pp = prep(&cap, &identity(), &PrepOptions::default());
    let blank = (0.03 / DT) as usize;
    for k in 200 - blank + 1..200 + blank {
        assert!(
            !pp.valid[k],
            "sample {k} beside the sign flip must be blanked"
        );
    }
    assert!(pp.valid[60], "samples between warmup and blank stay valid");
    assert!(pp.valid[340], "samples after the blanked window stay valid");
}

/// Stepped accel: `TransientKind::Mass` opens one window per accel/decel
/// phase start — the same transitions `onset_bias` scores on a stepped
/// trace, minus re-triggers inside a phase.
#[test]
fn mass_transient_fires_once_per_accel_phase() {
    let dt = 0.001;
    let a = 10_000.0;
    let acc = vec![
        0.0, 0.0, a, a, a, 0.0, 0.0, -a, -a, -a, 0.0, 0.0, a, a, a, 0.0,
    ];
    let vel: Vec<f64> = (0..acc.len()).map(|k| k as f64).collect();
    let ferr: Vec<f64> = (0..acc.len()).map(|k| 0.1 * (k % 3) as f64 - 0.1).collect();
    let mass = transient_rms(TransientKind::Mass, &[acc], &[vel], &[ferr], dt, 0.002);
    assert_eq!(mass[0].windows, 3, "three accel phases, one window each");
}

/// Jerk-limited ramp: accel never steps by 0.25·amax between consecutive
/// samples, so the onset_bias step detector stays blind — but the mass
/// objective must still score the phase (the Trident Y mode on the real
/// tune pattern is exactly this shape: onset windows 0, yet accel peaks
/// at full amplitude).
#[test]
fn mass_transient_fires_on_jerk_limited_ramps() {
    let dt = 0.001;
    let a = 10_000.0;
    let n = 40;
    let ramp = 20;
    let acc: Vec<f64> = (0..n)
        .map(|k| {
            if k < ramp {
                a * k as f64 / ramp as f64
            } else {
                a * (n - 1 - k) as f64 / ramp as f64
            }
        })
        .collect();
    let vel: Vec<f64> = (0..n).map(|k| k as f64).collect();
    let ferr = vec![0.05; n];
    let (_, onset_windows) = onset_bias(&[acc.clone()], &[ferr.clone()], dt, 0.008);
    assert_eq!(onset_windows, 0, "the step detector must be blind here");
    let mass = transient_rms(TransientKind::Mass, &[acc], &[vel], &[ferr], dt, 0.008);
    assert_eq!(mass[0].windows, 1, "one activity onset for the ramp");
    assert!(mass[0].rms.is_some());
}

/// Ramp to cruise then decel to rest: exactly one accel→cruise handoff
/// while the mode is still moving.
#[test]
fn cruise_arrival_fires_once_per_trapezoid() {
    let dt = 0.001;
    let a = 10_000.0;
    let vmax = 100.0;
    let mut acc = Vec::new();
    let mut vel = Vec::new();
    let ramp = 10;
    for k in 0..ramp {
        acc.push(a);
        vel.push(vmax * (k as f64 + 1.0) / ramp as f64);
    }
    for _ in 0..20 {
        acc.push(0.0);
        vel.push(vmax);
    }
    for k in 0..ramp {
        acc.push(-a);
        vel.push(vmax * (1.0 - (k as f64 + 1.0) / ramp as f64));
    }
    let ferr = vec![0.02; acc.len()];
    let viscous = transient_rms(TransientKind::Viscous, &[acc], &[vel], &[ferr], dt, 0.008);
    assert_eq!(
        viscous[0].windows, 1,
        "one cruise arrival per single trapezoid stroke"
    );
}
/// The same trapezoid ends in a stop: exactly one lead window, opening
/// where |vel| falls through 5% of vmax on the final decel — the
/// corner-exit lobe location. The accel ramp start must NOT trigger.
#[test]
fn lead_stop_window_fires_once_at_the_stop() {
    let dt = 0.001;
    let a = 10_000.0;
    let vmax = 100.0;
    let mut acc = Vec::new();
    let mut vel = Vec::new();
    let ramp = 10;
    for k in 0..ramp {
        acc.push(a);
        vel.push(vmax * (k as f64 + 1.0) / ramp as f64);
    }
    for _ in 0..20 {
        acc.push(0.0);
        vel.push(vmax);
    }
    for k in 0..ramp {
        acc.push(-a);
        vel.push(vmax * (1.0 - (k as f64 + 1.0) / ramp as f64));
    }
    for _ in 0..15 {
        acc.push(0.0);
        vel.push(0.0);
    }
    let ferr = vec![0.02; acc.len()];
    let lead = transient_rms(TransientKind::Lead, &[acc], &[vel], &[ferr], dt, 0.008);
    assert_eq!(lead[0].windows, 1, "one decel-to-stop per stroke");
    assert!(lead[0].rms.is_some());
}

/// Commanded velocity that flips sign three times triggers three coulomb
/// windows, and the aggregate/sigma math over constant per-window ferr is
/// the textbook sample-std standard error.
#[test]
fn reversal_windows_and_sigma_math_on_known_values() {
    let dt = 0.001;
    let n = 40;
    let mut vel = vec![0.0; n];
    for (k, v) in vel.iter_mut().enumerate() {
        *v = match k / 10 {
            0 | 2 => 1.0,
            _ => -1.0,
        };
    }
    let acc = vec![0.0; n];
    let mut ferr = vec![0.0; n];
    for (start, val) in [(10, 3.0), (20, 4.0), (30, 5.0)] {
        for s in ferr.iter_mut().take(start + 3).skip(start) {
            *s = val;
        }
    }
    let coulomb = transient_rms(TransientKind::Coulomb, &[acc], &[vel], &[ferr], dt, 0.003);
    let t = &coulomb[0];
    assert_eq!(t.windows, 3, "three sign reversals");
    let rms = t.rms.expect("rms present with three windows");
    assert!(
        (rms - (150.0_f64 / 9.0).sqrt()).abs() < 1e-9,
        "sample-weighted rms over all window samples: got {rms}"
    );
    let sigma = t.sigma.expect("sigma present with three windows");
    assert!(
        (sigma - (1.0 / 3.0_f64).sqrt()).abs() < 1e-9,
        "sigma = std(3,4,5)/sqrt(3) = 1/sqrt(3): got {sigma}"
    );
}

/// No trigger at all: rms and sigma are both null.
#[test]
fn no_windows_yields_null_rms() {
    let n = 20;
    let vel = vec![1.0; n];
    let acc = vec![0.0; n];
    let ferr = vec![0.5; n];
    let coulomb = transient_rms(
        TransientKind::Coulomb,
        &[acc],
        &[vel],
        &[ferr],
        0.001,
        0.003,
    );
    assert_eq!(coulomb[0].windows, 0);
    assert_eq!(coulomb[0].rms, None);
    assert_eq!(coulomb[0].sigma, None);
}

/// A single window has a defined rms but no sample standard deviation, so
/// sigma is null.
#[test]
fn single_window_yields_null_sigma() {
    let n = 20;
    let mut vel = vec![1.0; n];
    for v in vel.iter_mut().skip(10) {
        *v = -1.0;
    }
    let acc = vec![0.0; n];
    let ferr = vec![0.7; n];
    let coulomb = transient_rms(
        TransientKind::Coulomb,
        &[acc],
        &[vel],
        &[ferr],
        0.001,
        0.003,
    );
    assert_eq!(coulomb[0].windows, 1, "one reversal");
    assert!(coulomb[0].rms.is_some(), "rms defined for one window");
    assert_eq!(coulomb[0].sigma, None, "sigma null below two windows");
}

#[test]
fn frame_pairs_reads_equal_and_opposite_columns() {
    let equal = frame_pairs(&[vec![0.5, 0.5], vec![0.5, 0.5]]);
    assert_eq!(equal, vec![([0, 1], 1.0)], "equal columns -> lambda +1");

    let opposite = frame_pairs(&[vec![0.5, -0.5], vec![0.5, -0.5]]);
    assert_eq!(
        opposite,
        vec![([0, 1], -1.0)],
        "sign-flipped columns -> lambda -1"
    );
}

#[test]
fn frame_pairs_omits_unpartnered_axes() {
    let pairs = frame_pairs(&[vec![1.0, 0.0, 0.5], vec![0.0, 1.0, 0.5]]);
    assert_eq!(
        pairs,
        Vec::<([usize; 2], f64)>::new(),
        "no two columns are equal or opposite"
    );
}

#[test]
fn frame_pairs_greedy_leaves_each_axis_in_at_most_one_pair() {
    let pairs = frame_pairs(&[vec![0.5, 0.5, 0.5], vec![0.5, 0.5, 0.5]]);
    assert_eq!(
        pairs,
        vec![([0, 1], 1.0)],
        "three equal columns pair the first two; the third is unpartnered"
    );
}

fn direction_ferr(bias: f64, reps: usize, len: usize) -> (Vec<f64>, Vec<f64>) {
    let mut vel = Vec::new();
    let mut ferr = Vec::new();
    for r in 0..reps {
        let sign = if r % 2 == 0 { 1.0 } else { -1.0 };
        for _ in 0..len {
            vel.push(sign);
            ferr.push(sign * bias);
        }
    }
    (vel, ferr)
}

#[test]
fn direction_split_q_is_positive_when_motor_i_lags_by_direction() {
    let frame = vec![vec![1.0, 1.0]];
    let (vel_i, ferr_i) = direction_ferr(0.01, 8, 40);
    let ferr_j = vec![0.0; ferr_i.len()];
    let vel_j = vec![0.0; vel_i.len()];
    let out = direction_split(&frame, &[ferr_i, ferr_j], &[vel_i, vel_j]);
    assert_eq!(out.len(), 1, "one pair");
    let d = &out[0];
    assert_eq!(d.pair, [0, 1]);
    assert_eq!(d.lambda, 1.0);
    assert!(
        d.q > 0.0,
        "positive bias in +dir yields positive q: {}",
        d.q
    );
    assert!((d.q - 0.005).abs() < 1e-9, "q = mean_+/2 = 0.005: {}", d.q);
    assert!((d.rms - d.q.abs()).abs() < 1e-12, "rms = |q|");
    assert_eq!(d.windows, 8, "eight direction runs scored");
    assert!(
        d.sigma.is_some(),
        "sigma defined with 4 windows per direction"
    );
}

#[test]
fn direction_split_respects_lambda_on_opposite_columns() {
    let frame = vec![vec![1.0, -1.0]];
    let (vel_i, ferr_i) = direction_ferr(0.01, 8, 40);
    let ferr_j: Vec<f64> = ferr_i.iter().map(|e| 0.6 * e).collect();
    let vel_j = vec![0.0; vel_i.len()];
    let out = direction_split(&frame, &[ferr_i, ferr_j], &[vel_i, vel_j]);
    let d = &out[0];
    assert_eq!(d.lambda, -1.0);
    assert!(
        (d.q - 0.008).abs() < 1e-9,
        "d = (ferr_i - lambda*ferr_j)/2 = (ferr_i + 0.6*ferr_i)/2 = 0.8*ferr_i, q=0.008: {}",
        d.q
    );
}

#[test]
fn direction_split_sigma_null_below_two_windows_per_direction() {
    let frame = vec![vec![1.0, 1.0]];
    let (vel_i, ferr_i) = direction_ferr(0.01, 3, 40);
    let ferr_j = vec![0.0; ferr_i.len()];
    let vel_j = vec![0.0; vel_i.len()];
    let out = direction_split(&frame, &[ferr_i, ferr_j], &[vel_i, vel_j]);
    let d = &out[0];
    assert_eq!(d.windows, 3, "two + runs, one - run");
    assert_eq!(
        d.sigma, None,
        "sigma null with a single window in one direction"
    );
}

#[test]
fn direction_split_drops_runs_below_minimum_window() {
    let frame = vec![vec![1.0, 1.0]];
    let (vel_i, ferr_i) = direction_ferr(0.01, 6, 10);
    let ferr_j = vec![0.0; ferr_i.len()];
    let vel_j = vec![0.0; vel_i.len()];
    let out = direction_split(&frame, &[ferr_i, ferr_j], &[vel_i, vel_j]);
    assert_eq!(
        out[0].windows, 0,
        "runs of 10 samples fall below the 20-sample floor"
    );
}

#[test]
fn direction_split_emits_no_entries_when_frame_has_no_pairs() {
    let frame = vec![vec![1.0, 0.0], vec![0.0, 1.0]];
    let ferr = vec![vec![0.0; 4], vec![0.0; 4]];
    let vel = vec![vec![1.0; 4], vec![1.0; 4]];
    let out = direction_split(&frame, &ferr, &vel);
    assert!(out.is_empty(), "no pairs -> no direction-split entries");
}
