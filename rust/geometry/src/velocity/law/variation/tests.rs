use crate::velocity::law::{LawSegment, RailPair, RailPairShape, ScalarLaw};

const SAMPLES: u32 = 256;

fn rails() -> Vec<LawSegment> {
    let mut rails = Vec::new();
    for accel in [100.0, 5000.0, 50_000.0] {
        for kappa0 in [0.0, 0.05, 2.0, -1.5, 400.0] {
            for sigma in [0.0, 0.3, -4.0, 120.0, -40.0] {
                for brake in [false, true] {
                    let law = ScalarLaw::DiskRail {
                        accel,
                        kappa0,
                        sigma,
                        brake,
                    };
                    let cap = if kappa0 == 0.0 {
                        f64::INFINITY
                    } else {
                        (accel / f64::abs(kappa0)).sqrt()
                    };
                    for v0 in [0.0, 0.5, 30.0, 250.0, cap] {
                        if v0 > cap || !v0.is_finite() {
                            continue;
                        }
                        for ds in [3e-4, 0.01, 0.7, 12.0] {
                            rails.extend(LawSegment::until_arc(0.0, 0.0, v0, law, ds));
                        }
                    }
                }
            }
        }
    }
    rails.push(
        LawSegment::until_arc(
            0.0,
            0.0,
            12.5556,
            ScalarLaw::DiskRail {
                accel: 70_000.0,
                kappa0: 422.616,
                sigma: 634_643.7,
                brake: false,
            },
            1e-4,
        )
        .expect("the approach fixture covers its arc"),
    );
    rails
}

/// Each rail cut inside each of its square-root pairs, with the pair the
/// cut keeps clear of its departure end: its last.
fn clipped(rails: &[LawSegment]) -> Vec<(LawSegment, RailPair)> {
    rails
        .iter()
        .filter(|seg| seg.dt > 0.0)
        .flat_map(|seg| {
            seg.rail_pairs(0.0, seg.dt)
                .filter(|pair| !matches!(pair.shape, RailPairShape::Quintic { .. }))
                .flat_map(|pair| {
                    [0.15, 0.5, 0.85].map(|share| {
                        let cut = seg.cut_at_time(pair.t_lo + share * (pair.t_hi - pair.t_lo));
                        let last = cut
                            .rail_pairs(0.0, cut.dt)
                            .last()
                            .expect("a cut keeps a pair");
                        (cut, last)
                    })
                })
                .collect::<Vec<_>>()
        })
        .collect()
}

/// The variation of the acceleration `state_at` reads over `[lo, hi]`,
/// sampled.
fn sampled_variation(seg: &LawSegment, lo: f64, hi: f64) -> (f64, f64) {
    let at = |k: u32| {
        seg.state_at(lo + (hi - lo) * f64::from(k) / f64::from(SAMPLES))
            .2
    };
    let mut variation = 0.0;
    let mut largest = at(0).abs();
    for k in 1..=SAMPLES {
        variation += (at(k) - at(k - 1)).abs();
        largest = largest.max(at(k).abs());
    }
    (variation, largest)
}

fn windows(pair: &RailPair) -> Vec<(f64, f64)> {
    let h = pair.t_hi - pair.t_lo;
    let at = |share: f64| pair.t_lo + share * h;
    vec![
        (pair.t_lo, pair.t_hi),
        (pair.t_lo, at(1.0 / 64.0)),
        (at(63.0 / 64.0), pair.t_hi),
        (at(0.2), at(0.7)),
        (at(0.45), at(0.47)),
    ]
}

#[test]
fn no_sampled_variation_of_a_rail_pair_exceeds_its_bound() {
    let rails = rails();
    let every_pair = rails
        .iter()
        .flat_map(|seg| seg.rail_pairs(0.0, seg.dt).map(move |pair| (seg, pair)));
    let clipped = clipped(&rails);
    let mut counts = [0usize; 3];
    for (seg, pair) in every_pair.chain(clipped.iter().map(|(seg, pair)| (seg, *pair))) {
        counts[match pair.shape {
            RailPairShape::Quintic { .. } => 0,
            RailPairShape::Departure { .. } => 1,
            RailPairShape::Clipped { .. } => 2,
        }] += 1;
        for (lo, hi) in windows(&pair) {
            let bound = pair.acceleration_variation(lo, hi);
            let (sampled, largest) = sampled_variation(seg, lo, hi);
            assert!(
                sampled <= bound + 1e-9 * (1.0 + largest),
                "{:?} pair {pair:?} over [{lo}, {hi}]: sampled variation {sampled} past \
                 its bound {bound}",
                seg.law
            );
        }
    }
    assert!(
        counts.iter().all(|&count| count > 20),
        "the rails cover too few pairs of a shape (quintic, departure, clipped): {counts:?}"
    );
}

/// Over a stretch short enough that the acceleration's rate is nearly
/// linear the bound sits within the `4/3` a linear rate crossing zero
/// mid-stretch costs it.
#[test]
fn a_fine_stretch_bounds_its_variation_closely() {
    for seg in rails().into_iter().step_by(5) {
        for pair in seg.rail_pairs(0.0, seg.dt) {
            let (lo, hi) = (pair.t_lo, pair.t_hi);
            let h = hi - lo;
            let (sampled, largest) = sampled_variation(&seg, lo + 0.3 * h, lo + 0.31 * h);
            let bound = pair.acceleration_variation(lo + 0.3 * h, lo + 0.31 * h);
            assert!(
                bound <= 1.34 * sampled + 1e-9 * (1.0 + largest),
                "{:?} pair {pair:?}: bound {bound} far over the sampled {sampled}",
                seg.law
            );
        }
    }
}

#[test]
fn a_pair_reads_the_acceleration_the_segment_does_inside_it() {
    let rails = rails();
    let every_pair = rails
        .iter()
        .step_by(3)
        .flat_map(|seg| seg.rail_pairs(0.0, seg.dt).map(move |pair| (seg, pair)));
    let clipped = clipped(&rails);
    for (seg, pair) in every_pair.chain(clipped.iter().map(|(seg, pair)| (seg, *pair))) {
        for share in [0.1, 0.5, 0.9] {
            let tau = pair.t_lo + share * (pair.t_hi - pair.t_lo);
            let expected = seg.state_at(tau).2;
            let read = pair.acceleration_at(tau);
            assert!(
                (read - expected).abs() <= 1e-9 * (1.0 + expected.abs()),
                "{:?} pair {pair:?} at {tau}: reads {read}, the segment {expected}",
                seg.law
            );
        }
    }
}

#[test]
fn a_stretch_outside_the_pair_varies_by_nothing() {
    let seg = rails()
        .into_iter()
        .find(|seg| seg.rail_pairs(0.0, seg.dt).count() > 1)
        .expect("a rail of several pairs");
    let pair = seg.rail_pairs(0.0, seg.dt).next().expect("a first pair");
    assert_eq!(pair.acceleration_variation(pair.t_hi, seg.dt), 0.0);
}
