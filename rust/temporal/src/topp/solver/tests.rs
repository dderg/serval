use super::*;
use crate::Limits;
use crate::topp::chain::ChainGrid;
use crate::topp::constraints::{BuildOutcome, EndpointConditions, build_chain};
use crate::topp::path::{ArclengthGrid, InterSample};

#[test]
fn axis_jerk_cut_row_norm_is_one() {
    let n_grid = 5_usize;
    let off_a = n_grid;
    let n_vars = 2 * n_grid;

    let h = 1e-3_f64;
    let b_val = 6.0_f64;
    let cp = 1.0_f64;
    let h_uniform = h;
    let w = crate::topp::stencil::b_dd_weights(h_uniform, h_uniform);
    let cut = AxisJerkCut {
        i: 2,
        axis: 0,
        idx: [1, 2, 3],
        w,
        b_bars: [b_val, b_val, b_val],
        a_bar_i: 0.0,
        cp,
        cpp: 0.0,
        cppp: 0.0,
        j_lim_inflated: 1_000.0,
    };

    let s = b_val.sqrt();
    let expected_scale = cp * s / (h * h);
    assert!(
        expected_scale > 1e5,
        "test is only meaningful with large unscaled coefficients; got {expected_scale}"
    );

    let mut rowval: Vec<Vec<usize>> = vec![Vec::new(); n_vars];
    let mut nzval: Vec<Vec<f64>> = vec![Vec::new(); n_vars];
    let mut b_rhs: Vec<f64> = Vec::new();
    let mut n_rows = 0_usize;

    let b_floor = 0.0_f64;
    append_axis_jerk_cut_to_clarabel(
        &cut,
        b_floor,
        &mut n_rows,
        &mut rowval,
        &mut nzval,
        &mut b_rhs,
        n_grid,
    );

    assert_eq!(n_rows, 2, "expected two rows (± pair)");
    assert_eq!(b_rhs.len(), 2);

    let max_coeff: f64 = nzval
        .iter()
        .flat_map(|col| col.iter().copied())
        .map(f64::abs)
        .fold(0.0_f64, f64::max);

    assert!(
        (max_coeff - 1.0).abs() < 1e-10,
        "∞-norm of emitted rows should be 1.0, got {max_coeff}"
    );

    let j = cut.j_lim_inflated;
    let expected_rhs = j / expected_scale;
    assert!(
        (b_rhs[0] - expected_rhs).abs() < 1e-10 * expected_rhs.abs(),
        "rhs[0] = {}, expected {expected_rhs}",
        b_rhs[0]
    );
    assert!(
        (b_rhs[1] - expected_rhs).abs() < 1e-10 * expected_rhs.abs(),
        "rhs[1] = {}, expected {expected_rhs}",
        b_rhs[1]
    );

    let coeff_pos: Vec<f64> = nzval
        .iter()
        .enumerate()
        .filter_map(|(col, entries)| {
            let idx = entries
                .iter()
                .zip(rowval[col].iter())
                .position(|(_, &r)| r == 0)?;
            Some(entries[idx])
        })
        .collect();
    let coeff_neg: Vec<f64> = nzval
        .iter()
        .enumerate()
        .filter_map(|(col, entries)| {
            let idx = entries
                .iter()
                .zip(rowval[col].iter())
                .position(|(_, &r)| r == 1)?;
            Some(entries[idx])
        })
        .collect();
    assert_eq!(
        coeff_pos.len(),
        coeff_neg.len(),
        "± rows must touch the same number of columns"
    );
    for (p, n) in coeff_pos.iter().zip(coeff_neg.iter()) {
        assert!(
            (p.abs() - n.abs()).abs() < 1e-14,
            "coefficient magnitudes must match between ± rows: {p} vs {n}"
        );
    }

    assert!(
        rowval[off_a + cut.i].is_empty(),
        "a_i column should be absent when cpp = 0"
    );
}

#[test]
fn find_jerk_violators_chain_ratio_has_no_spurious_h_factor() {
    let h = 0.5_f64;
    let j_path = 100.0_f64;
    let target_ratio = 1.10_f64;
    let b_mid = 400.0_f64;
    let b_dd = target_ratio * 2.0 * j_path / b_mid.sqrt();
    let b_side = b_mid + b_dd * (h * h) / 2.0;
    let b = vec![b_side, b_mid, b_side];
    let h_intervals = vec![h, h];
    let violators = find_jerk_violators_chain(&b, &h_intervals, &[j_path; 3]);
    assert_eq!(
        violators.len(),
        1,
        "middle point should be the lone violator"
    );
    let got_ratio = violators[0].ratio;
    assert!(
        (got_ratio - target_ratio).abs() < 1e-3,
        "ratio {got_ratio} should be ≈{target_ratio}; a spurious h² divisor would give {:.4} instead",
        got_ratio / (h * h),
    );
}

fn dummy_straight_grid(n: usize, length: f64) -> ArclengthGrid {
    let s: Vec<f64> = (0..n).map(|i| length * i as f64 / (n - 1) as f64).collect();
    let u = s.clone();
    let c = s.iter().map(|si| [*si, 0.0, 0.0]).collect();
    let c_prime = vec![[1.0, 0.0, 0.0]; n];
    let c_double_prime = vec![[0.0, 0.0, 0.0]; n];
    let c_triple_prime = vec![[0.0, 0.0, 0.0]; n];
    let inter_geom = vec![
        vec![
            InterSample::planar(0.25, 0.0),
            InterSample::planar(0.5, 0.0),
            InterSample::planar(0.75, 0.0)
        ];
        n.saturating_sub(1)
    ];
    ArclengthGrid {
        s,
        u,
        c,
        c_prime,
        c_double_prime,
        c_triple_prime,
        total_length: length,
        inter_geom,
    }
}

#[test]
fn straight_line_solves_to_nontrivial_profile() {
    let grid = dummy_straight_grid(50, 100.0);
    let limits = Limits::axis_boxes(
        [500.0, 500.0, 500.0],
        [5_000.0, 5_000.0, 5_000.0],
        [100_000.0, 100_000.0, 100_000.0],
    );
    let chain = ChainGrid::from_segment_grids(vec![grid], vec![limits]);
    let bundle = match build_chain(
        &chain,
        EndpointConditions {
            v_start: 0.0,
            v_end: 0.0,
            a_start: None,
        },
        &SolverScale::identity(),
    ) {
        BuildOutcome::Ok(b) => b,
        BuildOutcome::Boundary(b) => panic!("expected Ok, got Boundary({b:?})"),
    };
    let result = solve(&bundle).expect("solver setup");
    assert!(
        matches!(
            result.status,
            SolverStatus::Solved | SolverStatus::SolvedInexact { .. }
        ),
        "expected Solved or SolvedInexact, got {:?}",
        result.status
    );
    assert_eq!(result.b.len(), 50);

    assert!(
        result.b[0].abs() < 1e-6,
        "b[0] should be ~0, got {}",
        result.b[0]
    );
    assert!(
        result.b[49].abs() < 1e-6,
        "b[49] should be ~0, got {}",
        result.b[49]
    );

    let b_mid = result.b[25];
    assert!(
        b_mid > 1e4,
        "b[25] = {b_mid}, expected > 1e4 (substantially accelerating)"
    );
    assert!(
        b_mid <= 250_000.0 * 1.01,
        "b[25] = {b_mid}, expected ≤ v_max² + tolerance"
    );

    assert!(
        result.b[10] > result.b[1],
        "must accelerate from rest: b[1]={}, b[10]={}",
        result.b[1],
        result.b[10]
    );
    assert!(
        result.b[40] < result.b[25],
        "must decelerate toward end: b[25]={}, b[40]={}",
        result.b[25],
        result.b[40]
    );

    assert!(
        result.a[5] > 0.0,
        "a[5] = {} should be positive (accelerating)",
        result.a[5]
    );
    assert!(
        result.a[44] < 0.0,
        "a[44] = {} should be negative (decelerating)",
        result.a[44]
    );
}

#[test]
fn uniform_damp_achieves_target_on_manifold() {
    let n = 7_usize;
    let length = 6.0_f64;

    let b0 = 100.0_f64;
    let s_dot = b0.sqrt();
    let j_max = 1_000.0_f64;

    let target_initial_ratio = 1.8_f64;
    let a_val = 10.0_f64;
    let cpp_val = j_max * target_initial_ratio / (3.0 * s_dot * a_val);

    let grid = {
        let s: Vec<f64> = (0..n).map(|i| length * i as f64 / (n - 1) as f64).collect();
        let u = s.clone();
        let c = s.iter().map(|si| [*si, 0.0, 0.0]).collect();
        let c_prime = vec![[1.0, 0.0, 0.0]; n];
        let c_double_prime = vec![[0.0, cpp_val, 0.0]; n];
        let c_triple_prime = vec![[0.0, 0.0, 0.0]; n];
        let inter_geom = vec![
            vec![
                InterSample::planar(0.25, 0.0),
                InterSample::planar(0.5, 0.0),
                InterSample::planar(0.75, 0.0)
            ];
            n.saturating_sub(1)
        ];
        ArclengthGrid {
            s,
            u,
            c,
            c_prime,
            c_double_prime,
            c_triple_prime,
            total_length: length,
            inter_geom,
        }
    };
    let limits = Limits::axis_boxes(
        [500.0, 500.0, 500.0],
        [50_000.0, 50_000.0, 50_000.0],
        [j_max, j_max, j_max],
    );
    let chain = ChainGrid::from_segment_grids(vec![grid], vec![limits]);

    let b = vec![b0; n];
    let a: Vec<f64> = (0..n)
        .map(|i| if i == 0 || i == n - 1 { 0.0 } else { a_val })
        .collect();
    let result = SolverResult {
        b,
        a,
        status: SolverStatus::Solved,
    };

    let initial_ratio = max_axis_ratio_chain(&result, &chain, None);
    assert!(
        initial_ratio > SLP9_DAMP_TARGET_RATIO,
        "test requires initial_ratio > {SLP9_DAMP_TARGET_RATIO}, got {initial_ratio}",
    );

    let (damped, final_ratio) = uniform_damp_for_feasibility(&result, &chain, None, initial_ratio);

    assert!(
        final_ratio <= SLP9_DAMP_TARGET_RATIO,
        "uniform_damp_for_feasibility must bring ratio ≤ {SLP9_DAMP_TARGET_RATIO}; \
         ratio before={initial_ratio:.4}, after={final_ratio:.4}",
    );
    for i in 0..n {
        let lam2 = damped.b[i] / result.b[i].max(1e-30);
        if result.a[i].abs() > 1e-12 {
            let lam2_a = damped.a[i] / result.a[i];
            assert!(
                (lam2 - lam2_a).abs() < 1e-9,
                "uniform damp must scale b and a by the same λ² (on-manifold)",
            );
        }
    }
}

#[test]
fn build_axis_jerk_cuts_chain_places_maintenance_cuts() {
    let n = 5_usize;
    let h = 1.0_f64;
    let length = h * (n - 1) as f64;
    let j_max = 1_000.0_f64;

    let target_ratio = 1.20_f64;

    let b_val = 4.0_f64;
    let s_dot = b_val.sqrt();
    let s_dot3 = s_dot.powi(3);

    let intended_jerk_ratios = [0.0, 0.20, 1.30, 0.01, 0.0];
    let cppp_vals: Vec<f64> = intended_jerk_ratios
        .iter()
        .map(|&r| r * j_max / s_dot3)
        .collect();

    let grid = {
        let s: Vec<f64> = (0..n).map(|i| length * i as f64 / (n - 1) as f64).collect();
        let u = s.clone();
        let c = s.iter().map(|si| [*si, 0.0, 0.0]).collect();
        let c_prime = vec![[1.0, 0.0, 0.0]; n];
        let c_double_prime = vec![[0.0, 0.0, 0.0]; n];
        let c_triple_prime: Vec<[f64; 3]> = cppp_vals.iter().map(|&v| [v, 0.0, 0.0]).collect();
        let inter_geom = vec![
            vec![
                InterSample::planar(0.25, 0.0),
                InterSample::planar(0.5, 0.0),
                InterSample::planar(0.75, 0.0)
            ];
            n.saturating_sub(1)
        ];
        ArclengthGrid {
            s,
            u,
            c,
            c_prime,
            c_double_prime,
            c_triple_prime,
            total_length: length,
            inter_geom,
        }
    };
    let limits = Limits::axis_boxes(
        [500.0, 500.0, 500.0],
        [50_000.0, 50_000.0, 50_000.0],
        [j_max, j_max, j_max],
    );
    let chain = ChainGrid::from_segment_grids(vec![grid], vec![limits]);

    let b = vec![b_val; n];
    let a = vec![0.0_f64; n];
    let result = SolverResult {
        b,
        a,
        status: SolverStatus::Solved,
    };

    let cuts = build_axis_jerk_cuts_chain(&result, &chain, target_ratio);

    let axis_jerk_cuts: Vec<&AxisJerkCut> = cuts
        .iter()
        .filter_map(|c| {
            if let SlpCut::AxisJerk(aj) = c {
                Some(aj)
            } else {
                None
            }
        })
        .filter(|aj| aj.axis == 0)
        .collect();

    let cut_for = |grid_i: usize| -> Option<f64> {
        axis_jerk_cuts
            .iter()
            .find(|aj| aj.i == grid_i)
            .map(|aj| aj.j_lim_inflated)
    };

    assert!(
        cut_for(1).is_some(),
        "i=1 (ratio=0.20 > SLP9_EPS_FEAS=0.05) must produce a maintenance cut",
    );
    assert!(
        (cut_for(1).unwrap() - j_max).abs() < 1e-9,
        "i=1 maintenance cut must have j_lim = j_max={j_max}, got {:?}",
        cut_for(1),
    );

    let expected_tight = j_max * target_ratio;
    assert!(
        cut_for(2).is_some(),
        "i=2 (ratio=1.30 > tightening threshold) must produce a tightening cut",
    );
    assert!(
        (cut_for(2).unwrap() - expected_tight).abs() < 1e-9,
        "i=2 tightening cut must have j_lim = {expected_tight}, got {:?}",
        cut_for(2),
    );

    assert!(
        cut_for(3).is_none(),
        "i=3 (ratio=0.01 < SLP9_EPS_FEAS=0.05) must produce no cut, got {:?}",
        cut_for(3),
    );
}

#[test]
fn slp_solve_chain_zero_cuts_placeable_is_converged_not_max_iters() {
    let grid = dummy_straight_grid(20, 0.03_f64);
    let limits = Limits::axis_boxes(
        [300.0, 300.0, 15.0],
        [5_000.0, 5_000.0, 350.0],
        [1.0, 1.0, 1.0],
    );
    let chain = ChainGrid::from_segment_grids(vec![grid], vec![limits]);
    let scale = crate::topp::scaling::SolverScale::for_chain(&chain);
    let scaled = scale.scale_chain_grid(&chain);
    let bundle = match build_chain(
        &scaled,
        EndpointConditions {
            v_start: 0.0,
            v_end: scale.scale_velocity(4e-4_f64),
            a_start: None,
        },
        &scale,
    ) {
        BuildOutcome::Ok(b) => b,
        BuildOutcome::Boundary(b) => panic!("unexpected boundary infeasibility: {b:?}"),
    };

    let (_result, outcome) =
        slp_solve_chain(&bundle, 1e-8, &scale).expect("slp_solve_chain setup must succeed");

    assert!(
        !matches!(outcome, SlpOutcome::MaxIters { .. }),
        "slp_solve_chain with zero cuts placeable must not return MaxIters \
         (all b < SLP_B_CUT_FLOOR → converged-by-floor); got {outcome:?}",
    );
}
