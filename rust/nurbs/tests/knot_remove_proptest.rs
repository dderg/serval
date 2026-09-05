use nurbs::ScalarNurbs;
use nurbs::knot::{insert_knot, refined_to_full_multiplicity, remove_knot};
use proptest::prelude::*;
use proptest::test_runner::FileFailurePersistence;

fn distinct_knots(curve: &ScalarNurbs) -> Vec<f64> {
    let mut distinct: Vec<f64> = Vec::new();
    for k in curve.knots() {
        if distinct.last() != Some(k) {
            distinct.push(*k);
        }
    }
    distinct
}

fn control_point_scale(curve: &ScalarNurbs) -> f64 {
    curve
        .control_points()
        .iter()
        .fold(1.0_f64, |acc, c| acc.max(c.abs()))
}

fn multiplicity(curve: &ScalarNurbs, u: f64) -> usize {
    curve.knots().iter().filter(|k| **k == u).count()
}

/// Clamped curve on [0, 1] whose interior breakpoints are separated by at least
/// `0.4 / (interior_count + 1)`, each carrying a multiplicity in `1..=degree`.
fn arb_curve_with_interior_count(
    counts: std::ops::RangeInclusive<usize>,
) -> impl Strategy<Value = ScalarNurbs> {
    (1u8..=4, counts).prop_flat_map(|(p, interior_count)| {
        let offsets = prop::collection::vec(-0.3..0.3_f64, interior_count);
        let multiplicities = prop::collection::vec(1usize..=(p as usize), interior_count);
        (offsets, multiplicities).prop_flat_map(move |(offsets, multiplicities)| {
            let pad = p as usize + 1;
            let mut knots = vec![0.0; pad];
            for (i, (offset, count)) in offsets.iter().zip(multiplicities.iter()).enumerate() {
                let position = ((i + 1) as f64 + offset) / (interior_count + 1) as f64;
                for _ in 0..*count {
                    knots.push(position);
                }
            }
            knots.extend(std::iter::repeat_n(1.0, pad));
            let n = knots.len() - pad;
            prop::collection::vec(-10.0..10.0_f64, n)
                .prop_map(move |cps| ScalarNurbs::try_new(p, knots.clone(), cps).unwrap())
        })
    })
}

fn arb_curve() -> impl Strategy<Value = ScalarNurbs> {
    arb_curve_with_interior_count(0..=3)
}

/// A curve plus a parameter strictly inside one of its knot spans, kept clear of
/// the span ends so the Boehm/Tiller alpha weights stay bounded away from 0.
fn arb_curve_with_fresh_knot() -> impl Strategy<Value = (ScalarNurbs, f64)> {
    arb_curve().prop_flat_map(|curve| {
        let span_count = distinct_knots(&curve).len() - 1;
        (Just(curve), 0..span_count, 0.2..0.8_f64).prop_map(|(curve, span, fraction)| {
            let breaks = distinct_knots(&curve);
            let u = breaks[span] + fraction * (breaks[span + 1] - breaks[span]);
            (curve, u)
        })
    })
}

fn arb_curve_with_fresh_knot_and_multiplicity() -> impl Strategy<Value = (ScalarNurbs, f64, usize)>
{
    arb_curve_with_fresh_knot().prop_flat_map(|(curve, u)| {
        let p = curve.degree() as usize;
        (Just(curve), Just(u), 1usize..=p)
    })
}

fn arb_curve_with_interior_knot() -> impl Strategy<Value = (ScalarNurbs, f64)> {
    arb_curve_with_interior_count(1..=3).prop_flat_map(|curve| {
        let break_count = distinct_knots(&curve).len();
        (Just(curve), 1..break_count - 1).prop_map(|(curve, index)| {
            let u = distinct_knots(&curve)[index];
            (curve, u)
        })
    })
}

/// Degree-`p` curve with a single interior knot at full multiplicity `p`, so the
/// two Bezier halves meet with a tangent jump of exactly `-u * jump` in the
/// one-knot-removal residual `P[p] - (1-u) P[p-1] - u P[p+1]`.
fn arb_corner_curve() -> impl Strategy<Value = (ScalarNurbs, f64, f64)> {
    (1u8..=4, 0.25..0.75_f64).prop_flat_map(|(p, u)| {
        let pad = p as usize + 1;
        let n = 2 * p as usize + 1;
        let jump = prop_oneof![Just(0.0_f64), -2.0..-0.05_f64, 0.05..2.0_f64];
        (prop::collection::vec(-3.0..3.0_f64, n), jump).prop_map(move |(mut cps, jump)| {
            let corner = p as usize;
            let left_slope = (cps[corner] - cps[corner - 1]) / u;
            cps[corner + 1] = cps[corner] + left_slope * (1.0 - u) + jump;
            let mut knots = vec![0.0; pad];
            knots.extend(std::iter::repeat_n(u, p as usize));
            knots.extend(std::iter::repeat_n(1.0, pad));
            (ScalarNurbs::try_new(p, knots, cps).unwrap(), u, jump)
        })
    })
}

fn arb_unit_samples() -> impl Strategy<Value = Vec<f64>> {
    prop::collection::vec(0.0..=1.0_f64, 64)
}

proptest! {
    #![proptest_config(ProptestConfig {
        cases: 384,
        failure_persistence: Some(Box::new(FileFailurePersistence::Direct(
            "proptest-regressions/knot_remove.txt",
        ))),
        ..ProptestConfig::default()
    })]

    #[test]
    fn remove_knot_undoes_insert_knot(
        (curve, u) in arb_curve_with_fresh_knot(),
        samples in arb_unit_samples(),
    ) {
        let scale = control_point_scale(&curve);
        let inserted = insert_knot(&curve, u, 1).unwrap();
        let (recovered, removed) = remove_knot(&inserted, u, 1, 1e-9 * scale);

        prop_assert_eq!(removed, 1);
        prop_assert_eq!(recovered.knots(), curve.knots());
        prop_assert_eq!(
            recovered.control_points().len(),
            curve.control_points().len()
        );

        let budget = 1e-9 * scale;
        let mut probes = samples;
        probes.extend(distinct_knots(&curve));
        probes.push(u);
        for probe in probes {
            let before = nurbs::eval::eval(&curve.as_view(), probe);
            let after = nurbs::eval::eval(&recovered.as_view(), probe);
            prop_assert!(
                (before - after).abs() <= budget,
                "u={u} probe={probe}: {before} vs {after} (budget {budget}), knots {:?}",
                curve.knots()
            );
        }
    }

    #[test]
    fn refinement_to_full_multiplicity_preserves_evaluation(
        curve in arb_curve(),
        samples in arb_unit_samples(),
    ) {
        let refined = refined_to_full_multiplicity(&curve);
        let scale = control_point_scale(&curve);
        let budget = 1e-10 * scale;

        let mut probes = samples;
        probes.extend(distinct_knots(&curve));
        for probe in probes {
            let before = nurbs::eval::eval(&curve.as_view(), probe);
            let after = nurbs::eval::eval(&refined.as_view(), probe);
            prop_assert!(
                (before - after).abs() <= budget,
                "probe={probe}: {before} vs {after} (budget {budget}), knots {:?}",
                curve.knots()
            );
        }
    }

    #[test]
    fn refinement_to_full_multiplicity_saturates_interior_knots(curve in arb_curve()) {
        let refined = refined_to_full_multiplicity(&curve);
        let p = curve.degree() as usize;

        prop_assert_eq!(refined.degree(), curve.degree());
        prop_assert_eq!(distinct_knots(&refined), distinct_knots(&curve));
        prop_assert_eq!(
            refined.knots().len(),
            refined.control_points().len() + p + 1
        );

        let breaks = distinct_knots(&refined);
        for interior in &breaks[1..breaks.len() - 1] {
            prop_assert_eq!(
                multiplicity(&refined, *interior),
                p,
                "interior knot {} not saturated",
                interior
            );
        }
        prop_assert_eq!(multiplicity(&refined, breaks[0]), p + 1);
        prop_assert_eq!(multiplicity(&refined, breaks[breaks.len() - 1]), p + 1);
    }

    #[test]
    fn remove_knot_refuses_an_essential_knot(
        (curve, u, jump) in arb_corner_curve(),
        samples in arb_unit_samples(),
    ) {
        let scale = control_point_scale(&curve);
        let tol = 1e-9 * scale;
        let (result, removed) = remove_knot(&curve, u, 1, tol);

        if jump == 0.0 {
            prop_assert_eq!(
                removed, 1,
                "a C1 join at a full-multiplicity knot is removable"
            );
            for probe in samples {
                let before = nurbs::eval::eval(&curve.as_view(), probe);
                let after = nurbs::eval::eval(&result.as_view(), probe);
                prop_assert!(
                    (before - after).abs() <= 1e-9 * scale,
                    "probe={probe}: {before} vs {after}"
                );
            }
        } else {
            prop_assert_eq!(
                removed, 0,
                "residual {} exceeds tol {}",
                u * jump.abs(),
                tol
            );
            prop_assert_eq!(result.knots(), curve.knots());
            prop_assert_eq!(result.control_points(), curve.control_points());
        }
    }

    #[test]
    fn remove_knot_leaves_absent_knots_alone(
        (curve, u) in arb_curve_with_fresh_knot(),
        count in 1usize..=4,
    ) {
        let scale = control_point_scale(&curve);
        let (result, removed) = remove_knot(&curve, u, count, 1e6 * scale);
        prop_assert_eq!(removed, 0);
        prop_assert_eq!(result.knots(), curve.knots());
        prop_assert_eq!(result.control_points(), curve.control_points());
    }

    #[test]
    fn remove_knot_undoes_multifold_insert_knot(
        (curve, u, folds) in arb_curve_with_fresh_knot_and_multiplicity(),
        samples in arb_unit_samples(),
    ) {
        let scale = control_point_scale(&curve);
        let inserted = insert_knot(&curve, u, folds).unwrap();
        let (recovered, removed) = remove_knot(&inserted, u, folds, 1e-9 * scale);

        prop_assert_eq!(removed, folds);
        prop_assert_eq!(recovered.knots(), curve.knots());

        let budget = 1e-9 * scale;
        let mut probes = samples;
        probes.extend(distinct_knots(&curve));
        probes.push(u);
        for probe in probes {
            let before = nurbs::eval::eval(&curve.as_view(), probe);
            let after = nurbs::eval::eval(&recovered.as_view(), probe);
            prop_assert!(
                (before - after).abs() <= budget,
                "folds={folds} u={u} probe={probe}: {before} vs {after} (budget {budget})"
            );
        }
    }

    #[test]
    fn remove_knot_only_drops_copies_of_the_requested_knot(
        (curve, u) in arb_curve_with_interior_knot(),
        count in 1usize..=4,
    ) {
        let scale = control_point_scale(&curve);
        let (result, removed) = remove_knot(&curve, u, count, 1e6 * scale);

        prop_assert!(removed <= count.min(multiplicity(&curve, u)));
        prop_assert_eq!(
            result.control_points().len(),
            curve.control_points().len() - removed
        );

        let mut expected: Vec<f64> = curve.knots().to_vec();
        for _ in 0..removed {
            let at = expected.iter().position(|k| *k == u).unwrap();
            expected.remove(at);
        }
        prop_assert_eq!(result.knots(), &expected[..], "u={} removed={}", u, removed);
    }
}
