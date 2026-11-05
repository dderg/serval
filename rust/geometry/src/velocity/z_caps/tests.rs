use super::*;
use crate::fitter::{CornerFitConfig, fit_corners};
use crate::frontend::{Move, MoveContext, corner_deviation_from_scv, line_move};
use crate::path::lowering::PositionProfile;
use crate::path::{Clothoid, Segment};
use crate::segment::SourceRange;
use crate::velocity::{VelocityProfile, plan_velocity_warm_start, warm_start_params};

const MAX_ACCEL: f64 = 70_000.0;
const MAX_Z_VELOCITY: f64 = 25.0;
const MAX_Z_ACCEL: f64 = 100.0;
const SCV: f64 = 5.0;

fn limits() -> VelocityLimits {
    VelocityLimits::try_new(
        1000.0,
        MAX_ACCEL,
        corner_deviation_from_scv(SCV, MAX_ACCEL),
        f64::INFINITY,
    )
    .unwrap()
    .with_z_caps(MAX_Z_VELOCITY, MAX_Z_ACCEL)
    .unwrap()
}

fn path(points: &[[f64; 3]], feeds: &[f64]) -> Vec<Move> {
    points
        .windows(2)
        .zip(feeds)
        .enumerate()
        .map(|(i, (pair, &feed))| {
            line_move(
                pair[0],
                pair[1],
                0.0,
                MoveContext {
                    extruder_axis: 3,
                    feedrate_mm_s: feed,
                    limits: limits(),
                    source: SourceRange {
                        start_line: i as u32,
                        end_line: i as u32,
                    },
                },
            )
            .unwrap()
        })
        .collect()
}

fn planned(moves: Vec<Move>) -> (Vec<Move>, VelocityProfile) {
    let fitted = fit_corners(&moves, CornerFitConfig::default()).unwrap();
    assert!(
        fitted
            .moves
            .iter()
            .any(|m| matches!(m.segment.spatial, Some(Segment::Clothoid(_)))),
        "the corners must blend for the blend to be under test"
    );
    let profile = plan_velocity_warm_start(&fitted, warm_start_params(1e-7)).unwrap();
    (fitted.moves, profile)
}

/// Peak z speed and z acceleration of the planned profile, each sample's
/// tangential state carried through the segment's own heading and its
/// derivative: `v·ĥ_z` and `a·ĥ_z + v²·(dĥ/ds)_z`.
fn z_peaks(moves: &[Move], profile: &VelocityProfile) -> (f64, f64) {
    let mut peaks = (0.0_f64, 0.0_f64);
    for (m, velocity) in moves.iter().zip(&profile.moves) {
        let segment = m.segment.spatial.as_ref().unwrap();
        for sample in &velocity.samples {
            let heading = segment.heading_at(sample.s)[Z_AXIS];
            let turning = segment.dheading_ds(sample.s)[Z_AXIS];
            peaks.0 = peaks.0.max((sample.v * heading).abs());
            peaks.1 = peaks
                .1
                .max((sample.a * heading + sample.v * sample.v * turning).abs());
        }
    }
    peaks
}

fn assert_z_within_caps(moves: Vec<Move>) {
    let (fitted, profile) = planned(moves);
    let (z_speed, z_accel) = z_peaks(&fitted, &profile);
    assert!(
        z_speed <= MAX_Z_VELOCITY * (1.0 + 1e-6),
        "z reaches {z_speed} mm/s against max_z_velocity {MAX_Z_VELOCITY}"
    );
    assert!(
        z_accel <= MAX_Z_ACCEL * (1.0 + 1e-6),
        "z reaches {z_accel} mm/s² against max_z_accel {MAX_Z_ACCEL}"
    );
}

/// The dense print's layer change: an extruding XY line turns back into a
/// short XYZ travel up to the next layer, which turns into the next layer's
/// first line. Both corners blend.
#[test]
fn layer_change_blends_hold_the_z_caps() {
    assert_z_within_caps(path(
        &[
            [100.0, 150.0, 40.2],
            [120.512, 149.536, 40.2],
            [120.123, 150.439, 40.4],
            [119.0, 151.0, 40.4],
        ],
        &[432.0, 1000.0, 432.0],
    ));
}

/// A z hop straight up into an XY move: the blend turns the heading from
/// pure z to pure XY.
#[test]
fn z_hop_into_xy_blend_holds_the_z_caps() {
    assert_z_within_caps(path(
        &[[10.0, 10.0, 0.0], [10.0, 10.0, 0.2], [60.0, 10.0, 0.2]],
        &[25.0, 400.0],
    ));
}

/// An XY line running on into a shallow XYZ travel: the blend's plane is
/// vertical, so all of its normal acceleration lands on z.
#[test]
fn collinear_slope_change_blend_holds_the_z_caps() {
    assert_z_within_caps(path(
        &[[0.0, 10.0, 0.2], [60.0, 10.0, 0.2], [110.0, 10.0, 0.4]],
        &[200.0, 1000.0],
    ));
}

fn gentle_vertical_blend() -> Segment {
    Segment::Clothoid(
        Clothoid::try_new(
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
            0.0,
            0.004 / (25.0 * 25.0),
            25.0,
        )
        .unwrap(),
    )
}

/// The z acceleration a profile under `(ceiling, accel)` can reach at
/// worst on `segment`.
fn worst_z_accel(segment: &Segment, ceiling: f64, accel: f64) -> f64 {
    let lean = segment.lean_on(Z_AXIS);
    let (_, kappa_peak) = crate::path::CurvatureProfile::kappa_peak(segment);
    accel * lean.heading + ceiling * ceiling * kappa_peak * lean.normal
}

/// A long gentle blend in a vertical plane — every bit of its normal
/// acceleration on z, its heading barely off horizontal — entered at a
/// printing speed whose normal load on z is small keeps that speed and
/// the tangential budget the rest of the z cap allows over its tiny
/// heading share, not the z cap itself.
#[test]
fn gentle_blend_at_printing_speed_keeps_its_speed_and_most_of_the_path_budget() {
    let gentle = gentle_vertical_blend();
    let (_, kappa_peak) = crate::path::CurvatureProfile::kappa_peak(&gentle);
    let (ceiling, accel) = z_capped(&gentle, &limits(), kappa_peak, 200.0);
    assert_eq!(ceiling, 200.0);
    let worst = worst_z_accel(&gentle, ceiling, accel);
    assert!((worst - MAX_Z_ACCEL).abs() < 1e-9 * MAX_Z_ACCEL, "{worst}");
    assert!(
        accel > 300.0 * MAX_Z_ACCEL,
        "budget {accel} collapsed toward max_z_accel"
    );
}

/// The same blend at travel speed: the normal load alone would pass the z
/// cap, so tangential and normal parts split it — the ceiling drops to
/// where the normal part fits half the cap, the disk budget to where the
/// tangential part fits the other half.
#[test]
fn gentle_blend_at_travel_speed_splits_the_z_cap() {
    let gentle = gentle_vertical_blend();
    let lean = gentle.lean_on(Z_AXIS);
    let (_, kappa_peak) = crate::path::CurvatureProfile::kappa_peak(&gentle);
    let (ceiling, accel) = z_capped(&gentle, &limits(), kappa_peak, 1000.0);
    let half = 0.5 * MAX_Z_ACCEL;
    assert!((ceiling * ceiling * kappa_peak * lean.normal - half).abs() < 1e-9 * half);
    assert!((accel * lean.heading - half).abs() < 1e-9 * half);
    assert!(ceiling > 500.0 && accel > 200.0 * MAX_Z_ACCEL);
}

/// The XY half of a tight z-hop corner: its heading climbs to 45° and all
/// of its normal acceleration lands on z. Splitting the z cap would halve
/// both parts; cutting the whole disk budget to the z cap leaves the speed
/// ceiling at the z speed cap's and crosses the corner sooner.
#[test]
fn tight_z_corner_cuts_the_whole_budget_to_the_z_cap() {
    let length = 0.084;
    let corner = Segment::Clothoid(
        Clothoid::try_new(
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
            0.0,
            std::f64::consts::FRAC_PI_2 / (length * length),
            length,
        )
        .unwrap(),
    );
    let (_, kappa_peak) = crate::path::CurvatureProfile::kappa_peak(&corner);
    let heading = corner.lean_on(Z_AXIS).heading;
    assert!((heading - std::f64::consts::FRAC_1_SQRT_2).abs() < 1e-12);
    assert_eq!(
        z_capped(&corner, &limits(), kappa_peak, 150.0),
        (MAX_Z_VELOCITY / heading, MAX_Z_ACCEL)
    );
}

/// A straight line's z caps are the per-move caps every Klipper-derived
/// planner derives from the move's z share.
#[test]
fn line_caps_are_the_z_share_caps() {
    let moves = path(&[[0.0, 0.0, 0.0], [3.0, 4.0, 1.25]], &[1000.0]);
    let segment = moves[0].segment.spatial.as_ref().unwrap();
    let z_share = 1.25 / libm::hypot(5.0, 1.25);
    let (ceiling, accel) = z_capped(segment, &limits(), 0.0, 1000.0);
    assert!((ceiling - MAX_Z_VELOCITY / z_share).abs() < 1e-9);
    assert!((accel - MAX_Z_ACCEL / z_share).abs() < 1e-9);
}
