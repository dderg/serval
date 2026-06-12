use super::*;
use crate::Limits;
use crate::topp::path::{ArclengthGrid, InterSample};
use crate::topp::solver::{SolverResult, SolverStatus};

fn limits_with_v_max(v_max: [f64; 3]) -> Limits {
    Limits::axis_boxes(v_max, [50_000.0; 3], [100_000.0; 3])
}

#[test]
fn for_limits_picks_max_axis_vmax_over_10() {
    let limits = limits_with_v_max([1000.0, 1000.0, 15.0]);
    let scale = SolverScale::for_limits(&limits);
    assert!(
        (scale.sigma() - 100.0).abs() < 1e-12,
        "expected sigma=100, got {}",
        scale.sigma()
    );
}

fn tiny_grid(s: f64, kappa: f64) -> ArclengthGrid {
    let n = 3;
    let s_vec: Vec<f64> = (0..n).map(|i| s * i as f64 / (n - 1) as f64).collect();
    let u = s_vec.clone();
    let c = s_vec.iter().map(|si| [*si, 0.0, 0.0]).collect();
    let c_prime = vec![[1.0, 0.0, 0.0]; n];
    let c_double_prime = vec![[0.001, 0.0, 0.0]; n];
    let c_triple_prime = vec![[0.00002, 0.0, 0.0]; n];
    ArclengthGrid {
        s: s_vec,
        u,
        c,
        c_prime,
        c_double_prime,
        c_triple_prime,
        total_length: s,
        inter_geom: vec![
            vec![
                InterSample::planar(0.25, kappa),
                InterSample::planar(0.5, kappa),
                InterSample::planar(0.75, kappa)
            ];
            n.saturating_sub(1)
        ],
    }
}

#[test]
fn grid_scaling_fields_have_correct_power_of_sigma() {
    let sigma = 100.0_f64;
    let scale = SolverScale { mm_per_unit: sigma };

    let raw_s = 600.0_f64;
    let raw_kappa = 0.05_f64;
    let grid = tiny_grid(raw_s, raw_kappa);
    let scaled = scale.scale_grid(&grid);

    // s ÷ σ
    for (raw, sc) in grid.s.iter().zip(scaled.s.iter()) {
        assert!(
            (sc - raw / sigma).abs() < 1e-12,
            "s field: expected {}, got {}",
            raw / sigma,
            sc
        );
    }

    // u unchanged
    for (raw, sc) in grid.u.iter().zip(scaled.u.iter()) {
        assert!((sc - raw).abs() < 1e-12, "u field should be unchanged");
    }

    // c ÷ σ
    for (raw, sc) in grid.c.iter().zip(scaled.c.iter()) {
        for ax in 0..3 {
            assert!(
                (sc[ax] - raw[ax] / sigma).abs() < 1e-12,
                "c field: expected {}, got {}",
                raw[ax] / sigma,
                sc[ax]
            );
        }
    }

    // c_prime unchanged (dimensionless dC/ds)
    for (raw, sc) in grid.c_prime.iter().zip(scaled.c_prime.iter()) {
        for ax in 0..3 {
            assert!(
                (sc[ax] - raw[ax]).abs() < 1e-12,
                "c_prime should be unchanged"
            );
        }
    }

    // c_double_prime ×σ
    for (raw, sc) in grid.c_double_prime.iter().zip(scaled.c_double_prime.iter()) {
        for ax in 0..3 {
            let expected = raw[ax] * sigma;
            assert!(
                (sc[ax] - expected).abs() < 1e-9,
                "c_double_prime: expected {}, got {}",
                expected,
                sc[ax]
            );
        }
    }

    // c_triple_prime ×σ²
    for (raw, sc) in grid.c_triple_prime.iter().zip(scaled.c_triple_prime.iter()) {
        for ax in 0..3 {
            let expected = raw[ax] * sigma * sigma;
            assert!(
                (sc[ax] - expected).abs() < 1e-6,
                "c_triple_prime: expected {}, got {}",
                expected,
                sc[ax]
            );
        }
    }

    // total_length ÷ σ
    assert!(
        (scaled.total_length - raw_s / sigma).abs() < 1e-12,
        "total_length: expected {}, got {}",
        raw_s / sigma,
        scaled.total_length
    );
}

#[test]
fn round_trip_b_scaling() {
    let scale = SolverScale { mm_per_unit: 100.0 };
    let x = 1_000_000.0_f64;
    let scaled = scale.to_scaled_b(x);
    let recovered = scale.unscale_b(scaled);
    assert!(
        (recovered - x).abs() <= x * f64::EPSILON * 4.0,
        "round-trip b: expected {x}, got {recovered}"
    );
}

#[test]
fn unscale_result_inverts_b_and_a() {
    let sigma = 100.0_f64;
    let scale = SolverScale { mm_per_unit: sigma };

    let b_scaled = vec![1.0, 2.5, 4.0];
    let a_scaled = vec![0.1, 0.2, 0.3];

    let mut result = SolverResult {
        b: b_scaled.clone(),
        a: a_scaled.clone(),
        status: SolverStatus::Solved,
    };
    scale.unscale_result(&mut result);

    for (orig, unscaled) in b_scaled.iter().zip(result.b.iter()) {
        let expected = orig * sigma * sigma;
        assert!(
            (unscaled - expected).abs() < 1e-9,
            "b unscale: expected {expected}, got {unscaled}"
        );
    }
    for (orig, unscaled) in a_scaled.iter().zip(result.a.iter()) {
        let expected = orig * sigma;
        assert!(
            (unscaled - expected).abs() < 1e-12,
            "a unscale: expected {expected}, got {unscaled}"
        );
    }
}

#[test]
fn limits_scaling_divides_all_four_families_by_sigma() {
    let sigma = 50.0_f64;
    let scale = SolverScale { mm_per_unit: sigma };
    let raw = Limits::axis_boxes(
        [1000.0, 800.0, 15.0],
        [50_000.0, 40_000.0, 100.0],
        [100_000.0, 100_000.0, 100_000.0],
    );
    let scaled = scale.scale_limits(&raw);

    for (rs, ss) in raw.sets().iter().zip(scaled.sets()) {
        assert_eq!(rs.axes, ss.axes);
        assert!((ss.v_max - rs.v_max / sigma).abs() < 1e-12);
        assert!((ss.a_max - rs.a_max / sigma).abs() < 1e-9);
        assert!((ss.j_max - rs.j_max / sigma).abs() < 1e-9);
    }
}

#[test]
fn chain_grid_scaling_matches_arclength_grid_scaling() {
    let c = crate::topp::chain::tests_support::line_50mm();
    let g = crate::topp::path::sample_arclength_grid(&c, 9).unwrap();
    let lims = crate::Limits::axis_boxes([1000.0; 3], [50_000.0; 3], [100_000.0; 3]);
    let chain = crate::topp::chain::ChainGrid::from_segment_grids(vec![g.clone()], vec![lims]);
    let scale = SolverScale::for_chain(&chain);
    let sg = scale.scale_grid(&g);
    let sc = scale.scale_chain_grid(&chain);
    assert_eq!(sc.s, sg.s);
    for i in 0..sc.n_points() {
        assert_eq!(sc.geom[i].c_double_prime, sg.c_double_prime[i]);
        assert_eq!(sc.geom[i].c_triple_prime, sg.c_triple_prime[i]);
    }
    assert!((sc.h_intervals[0] - (sg.s[1] - sg.s[0])).abs() < 1e-15);
}
