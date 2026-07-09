//! Characterization grid for the G2 Hermite solver: every case is a
//! round-trip — a ground-truth clothoid pair is built forward, then the
//! solver must recover *a* pair matching its endpoint states. Cases whose
//! ground truth curls past a junction-plausible turn are filtered out, so
//! every attempted case is known feasible and any failure is the solver's.

use super::*;
use crate::path::CurvatureProfile;
use crate::path::lowering::PositionProfile;

const Z: [f64; 3] = [0.0, 0.0, 1.0];
const X: [f64; 3] = [1.0, 0.0, 0.0];

const KAPPAS: [f64; 7] = [-2.0, -1.0, -0.4, 0.0, 0.4, 1.0, 2.0];
const KAPPA_PEAKS: [f64; 7] = [-3.0, -1.5, -0.6, 0.0, 0.6, 1.5, 3.0];
const LENGTHS: [f64; 3] = [0.3, 1.0, 3.0];
const SCALES: [f64; 3] = [1e-2, 1.0, 1e2];

const TURN_MIN_RAD: f64 = 1e-3;
const TURN_MAX_RAD: f64 = std::f64::consts::PI;
const POS_REL_TOL: f64 = 1e-8;
const ANG_TOL_RAD: f64 = 1e-9;
const KAPPA_REL_TOL: f64 = 1e-9;

#[derive(Clone, Copy, Debug)]
struct Case {
    kappa_a: f64,
    kappa_b: f64,
    kappa_peak: f64,
    l1: f64,
    l2: f64,
    scale: f64,
}

impl Case {
    fn scaled(&self) -> (f64, f64, f64, f64, f64) {
        (
            self.kappa_a / self.scale,
            self.kappa_b / self.scale,
            self.kappa_peak / self.scale,
            self.l1 * self.scale,
            self.l2 * self.scale,
        )
    }
}

/// Max |heading| along the pair, in radians, from the canonical (scale=1)
/// parameters. Heading is piecewise quadratic in s; extrema sit at segment
/// ends and interior curvature zero-crossings.
fn max_heading_excursion(c: &Case) -> f64 {
    let mut worst = 0.0_f64;
    let mut theta = 0.0_f64;
    let legs = [
        (c.kappa_a, c.kappa_peak, c.l1),
        (c.kappa_peak, c.kappa_b, c.l2),
    ];
    for (k0, k1, len) in legs {
        let sigma = (k1 - k0) / len;
        if sigma.abs() > 0.0 {
            let s_star = -k0 / sigma;
            if s_star > 0.0 && s_star < len {
                let t = theta + k0 * s_star + 0.5 * sigma * s_star * s_star;
                worst = worst.max(t.abs());
            }
        }
        theta += k0 * len + 0.5 * sigma * len * len;
        worst = worst.max(theta.abs());
    }
    worst
}

struct Outcome {
    case: Case,
    turn: f64,
    result: Result<(), &'static str>,
}

fn run_case(case: Case) -> Option<Outcome> {
    let turn = max_heading_excursion(&case);
    if !(TURN_MIN_RAD..=TURN_MAX_RAD).contains(&turn) {
        return None;
    }
    let (ka, kb, kp, l1, l2) = case.scaled();
    let start = Endpoint {
        pose: [0.0; 3],
        tangent: X,
        kappa: ka,
    };
    let truth = build_pair(&start, kb, Z, kp, l1, l2)?;
    let (p_b, t_b) = pair_end(&truth);
    let chord = dist([0.0; 3], p_b);
    if chord < 1e-6 * case.scale {
        return None;
    }

    let result = match hermite_g2([0.0; 3], X, ka, p_b, t_b, kb, Z) {
        None => Err("no convergence"),
        Some(pair) => verify(&pair, ka, p_b, t_b, kb, chord),
    };
    Some(Outcome { case, turn, result })
}

fn verify(
    pair: &ClothoidPair,
    ka: f64,
    p_b: [f64; 3],
    t_b: [f64; 3],
    kb: f64,
    chord: f64,
) -> Result<(), &'static str> {
    let (k_start, _) = pair.half1.kappa_endpoints();
    let (_, k_end) = pair.half2.kappa_endpoints();
    let kappa_scale = ka.abs().max(kb.abs()).max(1.0 / chord);
    if (k_start - ka).abs() > KAPPA_REL_TOL * kappa_scale {
        return Err("entry kappa mismatch");
    }
    if (k_end - kb).abs() > KAPPA_REL_TOL * kappa_scale {
        return Err("exit kappa mismatch");
    }
    let (_, k1_end) = pair.half1.kappa_endpoints();
    let (k2_start, _) = pair.half2.kappa_endpoints();
    if (k1_end - k2_start).abs() > KAPPA_REL_TOL * kappa_scale {
        return Err("internal kappa step");
    }
    let l1 = pair.half1.s_len();
    let mid = pair.half1.point_at(l1);
    if dist(mid, pair.half2.start_pose) > POS_REL_TOL * chord {
        return Err("internal position gap");
    }
    if signed_angle(pair.half1.heading_at(l1), pair.half2.heading_at(0.0), Z).abs() > ANG_TOL_RAD {
        return Err("internal tangent kink");
    }
    let (end, heading) = pair_end(pair);
    if dist(end, p_b) > POS_REL_TOL * chord {
        return Err("endpoint position miss");
    }
    if signed_angle(heading, t_b, Z).abs() > ANG_TOL_RAD {
        return Err("endpoint tangent miss");
    }
    Ok(())
}

fn all_cases() -> impl Iterator<Item = Case> {
    SCALES.into_iter().flat_map(|scale| {
        KAPPAS.into_iter().flat_map(move |kappa_a| {
            KAPPAS.into_iter().flat_map(move |kappa_b| {
                KAPPA_PEAKS.into_iter().flat_map(move |kappa_peak| {
                    LENGTHS.into_iter().flat_map(move |l1| {
                        LENGTHS.into_iter().map(move |l2| Case {
                            kappa_a,
                            kappa_b,
                            kappa_peak,
                            l1,
                            l2,
                            scale,
                        })
                    })
                })
            })
        })
    })
}

fn turn_bucket(turn: f64) -> usize {
    if turn < 0.5 {
        0
    } else if turn < 1.5 {
        1
    } else {
        2
    }
}

#[test]
fn hermite_g2_grid_characterization() {
    let outcomes: Vec<Outcome> = all_cases().filter_map(run_case).collect();
    assert!(
        outcomes.len() > 1000,
        "grid filter left too few feasible cases: {}",
        outcomes.len()
    );

    let mut per_scale: Vec<(f64, usize, usize)> = SCALES.iter().map(|s| (*s, 0, 0)).collect();
    let mut per_turn = [(0usize, 0usize); 3];
    let mut failures: Vec<&Outcome> = Vec::new();
    for o in &outcomes {
        let si = SCALES
            .iter()
            .position(|s| *s == o.case.scale)
            .expect("case scale is from the grid");
        per_scale[si].1 += 1;
        per_turn[turn_bucket(o.turn)].0 += 1;
        if o.result.is_ok() {
            per_scale[si].2 += 1;
            per_turn[turn_bucket(o.turn)].1 += 1;
        } else {
            failures.push(o);
        }
    }

    println!("hermite_g2 grid: {} feasible cases", outcomes.len());
    for (scale, total, ok) in &per_scale {
        println!(
            "  scale {scale:>8.0e}: {ok}/{total} ok ({:.2}%)",
            100.0 * *ok as f64 / *total as f64
        );
    }
    let turn_names = ["turn <0.5", "turn 0.5-1.5", "turn >1.5"];
    for (name, (total, ok)) in turn_names.iter().zip(per_turn) {
        println!(
            "  {name:>12}: {ok}/{total} ok ({:.2}%)",
            100.0 * ok as f64 / total as f64
        );
    }
    let mut reasons: std::collections::BTreeMap<&str, usize> = Default::default();
    for f in &failures {
        *reasons.entry(f.result.unwrap_err()).or_default() += 1;
    }
    for (reason, count) in &reasons {
        println!("  failure[{reason}]: {count}");
    }
    for f in failures.iter().take(15) {
        println!(
            "    FAIL {:?} turn={:.3} case={:?}",
            f.result.unwrap_err(),
            f.turn,
            f.case
        );
    }

    assert!(
        failures.is_empty(),
        "hermite_g2 failed {} of {} known-feasible cases",
        failures.len(),
        outcomes.len()
    );
}
