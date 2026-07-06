use proptest::prelude::*;

fn arb_degree() -> impl Strategy<Value = u8> {
    1u8..=4
}

fn arb_simple_polynomial_curve() -> impl Strategy<Value = nurbs::ScalarNurbs> {
    arb_degree().prop_flat_map(|p| {
        let n = p as usize + 1;
        let cps = prop::collection::vec(-5.0..5.0_f64, n);
        cps.prop_map(move |cps_vec| {
            let pad = p as usize + 1;
            let mut knots = vec![0.0; pad];
            knots.extend(vec![1.0; pad]);
            nurbs::ScalarNurbs::try_new(p, knots, cps_vec).unwrap()
        })
    })
}

fn arb_curve_with_existing_interior_multiplicity()
-> impl Strategy<Value = (nurbs::ScalarNurbs, f64, usize, usize)> {
    (3u8..=4, 0.1..0.45_f64, 0.55..0.9_f64, prop::bool::ANY).prop_flat_map(|(p, ka, kb, swap)| {
        let (u_knot, other_knot) = if swap { (ka, kb) } else { (kb, ka) };
        let existing_strategy: BoxedStrategy<usize> = (1usize..=(p as usize - 2)).boxed();
        existing_strategy.prop_flat_map(move |existing| {
            let n = p as usize + 2 + existing;
            let pad = p as usize + 1;
            prop::collection::vec(-3.0..3.0_f64, n).prop_map(move |cps| {
                let mut knots = vec![0.0; pad];
                let (lo_val, lo_mult, hi_val, hi_mult) = if u_knot < other_knot {
                    (u_knot, existing, other_knot, 1)
                } else {
                    (other_knot, 1, u_knot, existing)
                };
                for _ in 0..lo_mult {
                    knots.push(lo_val);
                }
                for _ in 0..hi_mult {
                    knots.push(hi_val);
                }
                knots.extend(vec![1.0; pad]);
                let curve = nurbs::ScalarNurbs::try_new(p, knots, cps).unwrap();
                (curve, u_knot, p as usize, existing)
            })
        })
    })
}

proptest! {
    #[test]
    fn insert_knot_preserves_evaluation(
        curve in arb_simple_polynomial_curve(),
        u in 0.01..0.99_f64,
    ) {
        let inserted = nurbs::knot::insert_knot(&curve, u, 1).unwrap();
        for sample_u in [0.0, 0.1, 0.3, 0.5, 0.7, 0.9, 1.0] {
            let before = nurbs::eval::eval(&curve.as_view(), sample_u);
            let after = nurbs::eval::eval(&inserted.as_view(), sample_u);
            prop_assert!(
                (before - after).abs() < 1e-9,
                "u={sample_u}: before={before}, after={after}"
            );
        }
    }

    #[test]
    fn insert_knot_multifold_preserves_evaluation(
        (curve, u, p, existing) in arb_curve_with_existing_interior_multiplicity(),
    ) {
        let r_max = p - existing;
        debug_assert!(r_max >= 2);
        for r in 1..=r_max {
            let inserted = nurbs::knot::insert_knot(&curve, u, r).unwrap();
            let mut samples = vec![0.0_f64, 0.1, 0.3, 0.5, 0.7, 0.9, 1.0, u];
            samples.sort_by(|a, b| a.partial_cmp(b).unwrap());
            for sample_u in samples {
                let before = nurbs::eval::eval(&curve.as_view(), sample_u);
                let after = nurbs::eval::eval(&inserted.as_view(), sample_u);
                prop_assert!(
                    (before - after).abs() < 1e-9,
                    "p={p}, existing={existing}, r={r}, u={sample_u}: before={before}, after={after}"
                );
            }
        }
    }
}
