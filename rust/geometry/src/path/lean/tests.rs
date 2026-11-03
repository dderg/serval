use super::*;
use crate::path::CurvatureProfile;
use crate::path::lowering::PositionProfile;

const Z: usize = 2;
const SAMPLES: usize = 20_000;

/// The lean read off the segment's own heading and unit normal at dense
/// samples — a lower bound that converges onto the exact maxima.
fn sampled_lean(segment: &Segment) -> (f64, f64) {
    let length = segment.s_len();
    let mut lean = (0.0_f64, 0.0_f64);
    for k in 0..=SAMPLES {
        let s = length * k as f64 / SAMPLES as f64;
        lean.0 = lean.0.max(segment.heading_at(s)[Z].abs());
        let turn = segment.dheading_ds(s);
        let kappa = segment.kappa(s).abs();
        if kappa > 0.0 {
            lean.1 = lean.1.max(turn[Z].abs() / kappa);
        }
    }
    lean
}

fn assert_lean_matches_samples(segment: &Segment) {
    let lean = segment.lean_on(Z);
    let (heading, normal) = sampled_lean(segment);
    assert!(
        lean.heading >= heading - 1e-12,
        "{lean:?} vs sampled heading {heading}"
    );
    assert!(
        lean.heading <= heading + 1e-6,
        "{lean:?} vs sampled heading {heading}"
    );
    assert!(
        lean.normal >= normal - 1e-12,
        "{lean:?} vs sampled normal {normal}"
    );
    assert!(
        lean.normal <= normal + 1e-6,
        "{lean:?} vs sampled normal {normal}"
    );
    assert!(lean.plane >= lean.heading.max(lean.normal) - 1e-12);
}

fn tilted_basis(tilt: f64) -> ([f64; 3], [f64; 3]) {
    let u = [libm::cos(tilt), 0.0, libm::sin(tilt)];
    let v = [0.0, 1.0, 0.0];
    (u, v)
}

#[test]
fn line_leans_by_its_z_share() {
    let line = Segment::Line(Line::try_new([0.0, 0.0, 0.0], [3.0, 0.0, 4.0]).unwrap());
    let lean = line.lean_on(Z);
    assert!((lean.heading - 0.8).abs() < 1e-12);
    assert_eq!(lean.normal, 0.0);
    assert_eq!(lean.plane, lean.heading);
}

#[test]
fn flat_curves_do_not_lean_on_z() {
    let arc = Segment::Arc(
        Arc::try_new([0.0; 3], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], 2.0, 0.3, 4.0).unwrap(),
    );
    let lean = arc.lean_on(Z);
    assert_eq!((lean.heading, lean.normal, lean.plane), (0.0, 0.0, 0.0));
}

#[test]
fn arcs_in_tilted_planes_match_their_sampled_lean() {
    for (tilt, start, sweep) in [
        (0.4, 0.0, 0.3),
        (0.4, 1.0, -0.2),
        (1.2, -2.0, 3.5),
        (std::f64::consts::FRAC_PI_2, 0.2, 0.01),
    ] {
        let (u, v) = tilted_basis(tilt);
        let arc = Segment::Arc(Arc::try_new([1.0, 2.0, 3.0], v, u, 0.7, start, sweep).unwrap());
        assert_lean_matches_samples(&arc);
    }
}

#[test]
fn clothoids_in_tilted_planes_match_their_sampled_lean() {
    for (tilt, kappa_0, sigma, length) in [
        (0.3, 0.0, 2.0, 0.5),
        (1.0, 3.0, -4.0, 1.2),
        (std::f64::consts::FRAC_PI_2, 0.0, 0.004 / 625.0, 25.0),
        (0.7, -1.0, 1.5, 2.0),
    ] {
        let (u, v) = tilted_basis(tilt);
        let clothoid =
            Segment::Clothoid(Clothoid::try_new([0.0; 3], u, v, kappa_0, sigma, length).unwrap());
        assert_lean_matches_samples(&clothoid);
    }
}
