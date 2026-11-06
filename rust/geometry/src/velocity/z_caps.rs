use super::law::RAIL_ACCELERATION_DEFECT;
use crate::frontend::VelocityLimits;
use crate::path::{AxisLean, CurvatureProfile, Segment};

const Z_AXIS: usize = 2;

/// The speed ceiling and disk budget under which no point of `segment` asks
/// the z axis for more than the move's z caps. The caps a move arrives with
/// bound the path; only the fitted segment's geometry says how much of that
/// path motion lands on z — a corner blend into a z change carries z in its
/// normal acceleration even where the heading barely climbs.
///
/// A curved segment runs on a disk rail, whose interpolated tangential
/// acceleration strays up to [`RAIL_ACCELERATION_DEFECT`] past the budget it
/// is handed: the z cap reserves that defect over its lean, so the rail's
/// motion holds z, not only its law. A straight segment's profile is closed
/// form and reserves nothing.
///
/// Two cap pairs each hold z, and the one a profile starting from rest
/// crosses the segment sooner under wins: the whole disk budget cut to the
/// z cap over the plane's lean, which keeps the speed ceiling and suits a
/// tight corner, or the z cap split between the tangential and the normal
/// part, which keeps most of the budget on a long gentle blend whose heading
/// barely leans on z.
pub(super) fn z_capped(
    segment: &Segment,
    limits: &VelocityLimits,
    kappa_peak: f64,
    path_ceiling: f64,
) -> (f64, f64) {
    let lean = segment.lean_on(Z_AXIS);
    if lean.plane == 0.0 {
        return (path_ceiling, limits.accel_mm_s2);
    }
    let max_z_accel = limits.max_z_accel_mm_s2;
    let rail_defect = if kappa_peak > 0.0 {
        RAIL_ACCELERATION_DEFECT
    } else {
        0.0
    };
    let ceiling = path_ceiling.min(limits.max_z_velocity_mm_s / lean.heading);
    let split = split_z_accel(
        lean,
        max_z_accel - rail_defect * lean.heading,
        kappa_peak,
        ceiling,
        limits.accel_mm_s2,
    );
    let plane = (
        ceiling,
        limits
            .accel_mm_s2
            .min(max_z_accel / lean.plane - rail_defect),
    );
    assert!(
        plane.1 > 0.0 && split.1 > 0.0,
        "z cap {max_z_accel} mm/s² leaves no disk budget past the rail defect \
         {rail_defect} mm/s² on a segment leaning {lean:?} on z"
    );
    let length = segment.s_len();
    if crossing_time(length, kappa_peak, plane) < crossing_time(length, kappa_peak, split) {
        plane
    } else {
        split
    }
}

/// A disk-limited profile puts at most `accel·heading` of its tangential
/// part on z and at most `ceiling²·κ_peak·normal` of its normal part. When
/// the two together can exceed `z_share`, the z cap left past the rail's
/// tangential defect, each part keeps what it needs up to half of it and
/// the other gets the rest: the tangential share lowers the disk budget,
/// the normal share the speed ceiling.
fn split_z_accel(
    lean: AxisLean,
    z_share: f64,
    kappa_peak: f64,
    ceiling: f64,
    accel: f64,
) -> (f64, f64) {
    let tangential_need = accel * lean.heading;
    let normal_need = ceiling * ceiling * kappa_peak * lean.normal;
    let half = 0.5 * z_share;
    let normal_ceiling = |share: f64| (share / (kappa_peak * lean.normal)).sqrt();
    if tangential_need + normal_need <= z_share {
        (ceiling, accel)
    } else if tangential_need < half {
        (normal_ceiling(z_share - tangential_need), accel)
    } else if normal_need < half {
        (ceiling, (z_share - normal_need) / lean.heading)
    } else {
        (normal_ceiling(half), half / lean.heading)
    }
}

/// How long a profile starting from rest takes to cross `length` under
/// `(ceiling, accel)`: accelerating to the apex speed the ceiling and the
/// disk on the curvature peak allow, then holding it.
pub(super) fn crossing_time(length: f64, kappa_peak: f64, (ceiling, accel): (f64, f64)) -> f64 {
    let apex = ceiling.min((accel / kappa_peak).sqrt());
    let ramp = apex * apex / (2.0 * accel);
    if ramp >= length {
        (2.0 * length / accel).sqrt()
    } else {
        apex / accel + (length - ramp) / apex
    }
}

#[cfg(test)]
mod tests;
