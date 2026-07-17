use servo_ident::capture::Capture;
use servo_ident::model::{PairDiscoveryError, PhysicalParams, Structure};
use servo_ident::split::{
    fit_pair_splits, report_pair_splits, SplitCapture, DIRECTION_SPLIT_LIMIT,
};

struct SyntheticPair {
    structure: Structure,
    params: PhysicalParams,
    cap: Capture,
    residual: Vec<Vec<f64>>,
    keep: Vec<bool>,
}

fn synthetic_pair(lambda: f64, direction_split: f64, intercept: f64) -> SyntheticPair {
    let structure = Structure::new(vec![vec![0.5, 0.5 * lambda]]);
    let params = PhysicalParams {
        mass: vec![0.02],
        viscous: vec![0.15],
        coulomb: vec![4.0],
    };
    let samples = 240;
    let mut acc_mode = Vec::with_capacity(samples);
    let mut vel_mode = Vec::with_capacity(samples);
    let mut residual = vec![Vec::with_capacity(samples), Vec::with_capacity(samples)];
    for sample in 0..samples {
        let acc = ((sample * 17) % 31) as f64 * 90.0 - 1350.0;
        let vel = ((sample * 13) % 37) as f64 * 2.0 - 36.0;
        let cs = if vel > 0.5 {
            1.0
        } else if vel < -0.5 {
            -1.0
        } else {
            0.0
        };
        let mode_force = params.mass[0] * acc + params.viscous[0] * vel + params.coulomb[0] * cs;
        let base_first = 0.5 * mode_force;
        let differential = direction_split * 2.0 * base_first.abs() + intercept;
        residual[0].push(differential / 2.0);
        residual[1].push(-lambda * differential / 2.0);
        acc_mode.push(acc);
        vel_mode.push(vel);
    }
    let paired = |values: &[f64]| {
        vec![
            values.to_vec(),
            values.iter().map(|value| lambda * value).collect(),
        ]
    };
    let acc = paired(&acc_mode);
    let vel = paired(&vel_mode);
    SyntheticPair {
        structure,
        params,
        cap: Capture {
            t: (0..samples).map(|sample| sample as f64 * 0.001).collect(),
            acc,
            vel: vel.clone(),
            vel_act: vel,
            torque: vec![vec![0.0; samples], vec![0.0; samples]],
        },
        residual,
        keep: vec![true; samples],
    }
}

fn fit_split(synthetic: &SyntheticPair) -> servo_ident::split::PairReport {
    let capture = SplitCapture {
        cap: &synthetic.cap,
        residual_filt: &synthetic.residual,
        keep: &synthetic.keep,
    };
    fit_pair_splits(&synthetic.structure, &synthetic.params, 0.0, &capture)
        .expect("pair fit")
        .pop()
        .expect("one pair")
}

#[test]
fn recovers_signed_known_direction_split() {
    let report = fit_split(&synthetic_pair(-1.0, -0.18, 0.0));
    assert_eq!(report.lambda, -1.0);
    assert!((report.fitted_direction_split + 0.18).abs() < 1.0e-12);
    assert!(!report.rejected);
    assert!(report.rms_after < 1.0e-10);
}

#[test]
fn nuisance_intercept_keeps_direction_split_unbiased() {
    let report = fit_split(&synthetic_pair(1.0, 0.2, 37.5));
    assert!((report.fitted_direction_split - 0.2).abs() < 1.0e-12);
    assert!((report.intercept - 37.5).abs() < 1.0e-12);
    assert!(report.rms_after < 1.0e-10);
}

#[test]
fn pair_total_magnitude_is_twice_the_first_base_share() {
    let report = fit_split(&synthetic_pair(1.0, 0.2, 0.0));
    assert!((report.fitted_direction_split - 0.2).abs() < 1.0e-12);
    assert!((report.fitted_direction_split - 0.4).abs() > 0.1);
}

#[test]
fn cap_rejects_and_zeroes_at_the_limit() {
    let report = fit_split(&synthetic_pair(1.0, DIRECTION_SPLIT_LIMIT, 0.0));
    assert!(report.fitted_direction_split.abs() >= DIRECTION_SPLIT_LIMIT);
    assert!(report.rejected);
    assert_eq!(report.split.direction_split, 0.0);
    assert!(report_pair_splits(&[report], &["a", "b"]).is_empty());
}

#[test]
fn discovers_only_exact_equal_or_opposite_pairs() {
    let structure = Structure::new(vec![
        vec![1.0, -1.0, 0.0, 0.0, 1.0],
        vec![0.0, 0.0, 0.5, -0.5, 1.0],
    ]);
    let pairs = structure.pairs().expect("valid pairs");
    assert_eq!(pairs.len(), 2);
    assert_eq!(
        (pairs[0].first, pairs[0].second, pairs[0].lambda),
        (0, 1, -1.0)
    );
    assert_eq!(
        (pairs[1].first, pairs[1].second, pairs[1].lambda),
        (2, 3, -1.0)
    );
}

#[test]
fn parallel_unequal_singleton_does_not_block_exact_pair() {
    let pairs = Structure::new(vec![vec![1.0, -1.0, 2.0]])
        .pairs()
        .expect("exact pair plus unmatched column");
    assert_eq!(pairs.len(), 1);
    assert_eq!((pairs[0].first, pairs[0].second), (0, 1));
}

#[test]
fn approximate_equal_column_is_not_emitted() {
    let pairs = Structure::new(vec![vec![1.0, -1.0, 1.0 + 1.0e-12]])
        .pairs()
        .expect("exact pair remains valid");
    assert_eq!(pairs.len(), 1);
    assert_eq!((pairs[0].first, pairs[0].second), (0, 1));
}

#[test]
fn rejects_ambiguous_exact_group() {
    let error = Structure::new(vec![vec![1.0, -1.0, 1.0]])
        .pairs()
        .expect_err("three exact equal/opposite columns are ambiguous");
    assert_eq!(
        error,
        PairDiscoveryError::AmbiguousGroup {
            slots: vec![0, 1, 2]
        }
    );
}

#[test]
fn rejects_isolated_parallel_columns_with_unequal_magnitude() {
    let error = Structure::new(vec![vec![1.0, -2.0]])
        .pairs()
        .expect_err("isolated unequal pair must fail");
    assert_eq!(
        error,
        PairDiscoveryError::UnequalMagnitude {
            first: 0,
            second: 1
        }
    );
}
