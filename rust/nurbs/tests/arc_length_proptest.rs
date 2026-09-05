use nurbs::VectorNurbs;
use nurbs::arc_length::path_arc_length;
use nurbs::eval::{vector_derivative, vector_eval};
use proptest::prelude::*;
use proptest::test_runner::FileFailurePersistence;

/// `path_arc_length` accepts a refinement level once two successive estimates
/// agree to this relative gate, so every length it returns carries at most this
/// much relative uncertainty.
const CONVERGENCE_GATE: f64 = 1e-9;

/// 5-point Gauss-Legendre on [-1, 1] from its closed form, so the reference
/// shares no decimal table with the implementation.
fn gauss_legendre_5() -> ([f64; 5], [f64; 5]) {
    let inner = (10.0_f64 / 7.0).sqrt();
    let near = (5.0 - 2.0 * inner).sqrt() / 3.0;
    let far = (5.0 + 2.0 * inner).sqrt() / 3.0;
    let root70 = 70.0_f64.sqrt();
    let w_near = (322.0 + 13.0 * root70) / 900.0;
    let w_far = (322.0 - 13.0 * root70) / 900.0;
    (
        [-far, -near, 0.0, near, far],
        [w_far, w_near, 128.0 / 225.0, w_near, w_far],
    )
}

const REFERENCE_PANELS_PER_SPAN: usize = 24;

fn reference_arc_length(curve: &VectorNurbs<3>) -> f64 {
    let (nodes, weights) = gauss_legendre_5();
    let deriv = vector_derivative(curve);
    let speed = |u: f64| {
        let d = vector_eval(&deriv, u);
        (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt()
    };
    let mut total = 0.0;
    for pair in curve.knots().windows(2) {
        if pair[1] <= pair[0] {
            continue;
        }
        let width = (pair[1] - pair[0]) / REFERENCE_PANELS_PER_SPAN as f64;
        for panel in 0..REFERENCE_PANELS_PER_SPAN {
            let a = pair[0] + width * panel as f64;
            let half = 0.5 * width;
            let mid = a + half;
            for node in 0..5 {
                total += weights[node] * speed(mid + half * nodes[node]) * half;
            }
        }
    }
    total
}

fn distance(a: [f64; 3], b: [f64; 3]) -> f64 {
    let d = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt()
}

fn chord(curve: &VectorNurbs<3>) -> f64 {
    let knots = curve.knots();
    distance(
        vector_eval(curve, knots[0]),
        vector_eval(curve, knots[knots.len() - 1]),
    )
}

/// A C0 chain of `breaks.len() - 1` Bezier segments: every interior knot sits at
/// full multiplicity, so the segments own disjoint control points and any
/// sub-chain is the exact restriction of the whole curve.
#[derive(Debug, Clone)]
struct BezierChain {
    degree: u8,
    breaks: Vec<f64>,
    cps: Vec<[f64; 3]>,
}

impl BezierChain {
    fn pieces(&self) -> usize {
        self.breaks.len() - 1
    }

    fn assemble(&self, from: usize, to: usize) -> VectorNurbs<3> {
        let p = self.degree as usize;
        let mut knots = vec![self.breaks[from]; p + 1];
        for interior in &self.breaks[(from + 1)..to] {
            knots.extend(std::iter::repeat_n(*interior, p));
        }
        knots.extend(std::iter::repeat_n(self.breaks[to], p + 1));
        let cps = self.cps[(from * p)..=(to * p)].to_vec();
        VectorNurbs::<3>::try_new(self.degree, knots, cps).unwrap()
    }
}

fn build_chain(degree: u8, widths: &[f64], steps: &[[f64; 3]]) -> BezierChain {
    let mut breaks = Vec::with_capacity(widths.len() + 1);
    breaks.push(0.0);
    for width in widths {
        breaks.push(breaks[breaks.len() - 1] + width);
    }
    let mut cps = Vec::with_capacity(steps.len() + 1);
    cps.push([0.0, 0.0, 0.0]);
    for step in steps {
        let prev = cps[cps.len() - 1];
        cps.push([prev[0] + step[0], prev[1] + step[1], prev[2] + step[2]]);
    }
    BezierChain {
        degree,
        breaks,
        cps,
    }
}

/// Control-point steps always advance in `x`, so the derivative curve's control
/// points all have a positive `x` and the parametric speed never reaches zero.
fn arb_chain() -> impl Strategy<Value = BezierChain> {
    (1u8..=4, 1usize..=4).prop_flat_map(|(degree, pieces)| {
        let widths = prop::collection::vec(0.3..1.0_f64, pieces);
        let steps = prop::collection::vec(
            [0.5..2.0_f64, -1.0..1.0_f64, -1.0..1.0_f64],
            pieces * degree as usize,
        );
        (widths, steps).prop_map(move |(widths, steps)| build_chain(degree, &widths, &steps))
    })
}

fn arb_straight_chain() -> impl Strategy<Value = BezierChain> {
    (1u8..=4, 1usize..=4).prop_flat_map(|(degree, pieces)| {
        let widths = prop::collection::vec(0.3..1.0_f64, pieces);
        let scales = prop::collection::vec(0.5..2.0_f64, pieces * degree as usize);
        let heading = [-1.0..1.0_f64, -1.0..1.0_f64];
        (widths, scales, heading).prop_map(move |(widths, scales, heading)| {
            let steps: Vec<[f64; 3]> = scales
                .iter()
                .map(|s| [*s, s * heading[0], s * heading[1]])
                .collect();
            build_chain(degree, &widths, &steps)
        })
    })
}

proptest! {
    #![proptest_config(ProptestConfig {
        cases: 256,
        failure_persistence: Some(Box::new(FileFailurePersistence::Direct(
            "proptest-regressions/arc_length.txt",
        ))),
        ..ProptestConfig::default()
    })]

    #[test]
    fn length_matches_a_knot_aligned_reference_quadrature(chain in arb_chain()) {
        let curve = chain.assemble(0, chain.pieces());
        let measured = path_arc_length(&curve);
        let reference = reference_arc_length(&curve);
        prop_assert!(
            (measured - reference).abs() <= CONVERGENCE_GATE * reference,
            "degree={} pieces={}: measured {measured}, reference {reference}",
            chain.degree,
            chain.pieces()
        );
    }

    #[test]
    fn length_grows_with_the_span_end(chain in arb_chain()) {
        let mut previous = 0.0_f64;
        for to in 1..=chain.pieces() {
            let length = path_arc_length(&chain.assemble(0, to));
            prop_assert!(
                length + CONVERGENCE_GATE * length >= previous,
                "to={to}: {length} < {previous}"
            );
            previous = length;
        }
    }

    #[test]
    fn length_is_at_least_the_chord(chain in arb_chain()) {
        for from in 0..chain.pieces() {
            for to in (from + 1)..=chain.pieces() {
                let curve = chain.assemble(from, to);
                let length = path_arc_length(&curve);
                let straight = chord(&curve);
                prop_assert!(
                    length + CONVERGENCE_GATE * length >= straight,
                    "[{from},{to}]: length {length} below chord {straight}"
                );
            }
        }
    }

    #[test]
    fn length_is_additive_across_a_split(chain in arb_chain()) {
        let whole = path_arc_length(&chain.assemble(0, chain.pieces()));
        for split in 1..chain.pieces() {
            let head = path_arc_length(&chain.assemble(0, split));
            let tail = path_arc_length(&chain.assemble(split, chain.pieces()));
            prop_assert!(
                (whole - head - tail).abs() <= 3.0 * CONVERGENCE_GATE * whole,
                "split={split}: {whole} vs {head} + {tail}"
            );
        }
    }

    #[test]
    fn straight_line_length_equals_the_chord(chain in arb_straight_chain()) {
        for to in 1..=chain.pieces() {
            let curve = chain.assemble(0, to);
            let length = path_arc_length(&curve);
            let straight = chord(&curve);
            prop_assert!(
                (length - straight).abs() <= 1e-12 * straight,
                "to={to}: collinear control points give length {length}, chord {straight}"
            );
        }
    }

    #[test]
    fn sub_chain_is_the_exact_restriction_of_the_chain(
        chain in arb_chain(),
        fractions in prop::collection::vec(0.0..=1.0_f64, 16),
    ) {
        let whole = chain.assemble(0, chain.pieces());
        for from in 0..chain.pieces() {
            for to in (from + 1)..=chain.pieces() {
                let part = chain.assemble(from, to);
                let lo = chain.breaks[from];
                let hi = chain.breaks[to];
                for fraction in &fractions {
                    let u = lo + fraction * (hi - lo);
                    prop_assert_eq!(vector_eval(&part, u), vector_eval(&whole, u), "u={}", u);
                }
            }
        }
    }
}
