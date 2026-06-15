use crate::kinematics::SPATIAL_AXES;
use nurbs::ScalarNurbs;
use nurbs::bezier::{BezierPiece, bezier_pieces_to_nurbs};
use trajectory::ShapedSegment;

#[cfg(test)]
mod tests;

/// Returns `(accel_t, cruise_t, cruise_v)`.
/// `accel <= 0` collapses to a single constant-velocity phase (`accel_t == 0`).
pub(crate) fn calc_move_time(dist: f64, speed: f64, accel: f64) -> (f64, f64, f64) {
    let dist = dist.abs();
    if accel <= 0.0 || dist == 0.0 {
        let cruise_t = if speed > 0.0 { dist / speed } else { 0.0 };
        return (0.0, cruise_t, speed);
    }
    let max_cruise_v2 = dist * accel;
    let cruise_v = speed.min(max_cruise_v2.sqrt());
    let accel_t = cruise_v / accel;
    let accel_decel_d = accel_t * cruise_v;
    let cruise_t = (dist - accel_decel_d) / cruise_v;
    (accel_t, cruise_t.max(0.0), cruise_v)
}

fn monomial_piece(t_start: f64, t_end: f64, c0: f64, c1: f64, c2: f64) -> BezierPiece<f64> {
    BezierPiece {
        u_start: t_start,
        u_end: t_end,
        coeffs: vec![c0, c1, c2, 0.0],
    }
}

fn linear_scalar_nurbs(t_start: f64, t_end: f64, pos_start: f64, vel: f64) -> ScalarNurbs<f64> {
    bezier_pieces_to_nurbs(&[monomial_piece(t_start, t_end, pos_start, vel, 0.0)])
}

fn quad_scalar_nurbs(
    t_start: f64,
    t_end: f64,
    pos_start: f64,
    vel: f64,
    half_accel: f64,
) -> ScalarNurbs<f64> {
    bezier_pieces_to_nurbs(&[monomial_piece(t_start, t_end, pos_start, vel, half_accel)])
}

fn constant_zero_nurbs(t_start: f64, t_end: f64) -> ScalarNurbs<f64> {
    bezier_pieces_to_nurbs(&[BezierPiece::zero(t_start, t_end, 3)])
}

fn shaped_segment(
    axis_idx: usize,
    t_start: f64,
    t_end: f64,
    axis_curve: ScalarNurbs<f64>,
    motor_mask: u8,
) -> ShapedSegment {
    let axes: Vec<ScalarNurbs<f64>> = (0..SPATIAL_AXES)
        .map(|ax| {
            if ax == axis_idx {
                axis_curve.clone()
            } else {
                constant_zero_nurbs(t_start, t_end)
            }
        })
        .collect();
    ShapedSegment {
        axes,
        followers: vec![],
        t_start,
        t_end,
        motor_mask,
    }
}

/// Build the relative `0 → delta_mm` overlay as cubic `ShapedSegment`(s) on `axis_idx`,
/// stamped with `motor_mask`. No solver — closed-form box or trapezoid profile.
pub fn plan_nudge_profile(
    axis_idx: u8,
    delta_mm: f64,
    speed: f64,
    accel: f64,
    motor_mask: u8,
) -> Result<Vec<ShapedSegment>, String> {
    if !delta_mm.is_finite() || !speed.is_finite() || speed <= 0.0 {
        return Err(format!("nudge: bad speed {speed} / delta {delta_mm}"));
    }

    let ax = axis_idx as usize;
    if ax >= SPATIAL_AXES {
        return Err(format!(
            "nudge: axis_idx {axis_idx} out of range (max {})",
            SPATIAL_AXES - 1
        ));
    }

    let sign = delta_mm.signum();
    let (accel_t, cruise_t, cruise_v) = calc_move_time(delta_mm, speed, accel);

    if accel_t == 0.0 {
        let curve = linear_scalar_nurbs(0.0, cruise_t, 0.0, sign * speed);
        return Ok(vec![shaped_segment(ax, 0.0, cruise_t, curve, motor_mask)]);
    }

    let half_accel = 0.5 * accel * sign;
    let accel_end_pos = half_accel * accel_t * accel_t;

    let mut segs = Vec::with_capacity(3);

    let accel_seg = shaped_segment(
        ax,
        0.0,
        accel_t,
        quad_scalar_nurbs(0.0, accel_t, 0.0, 0.0, half_accel),
        motor_mask,
    );
    segs.push(accel_seg);

    if cruise_t > 0.0 {
        let cruise_start = accel_t;
        let cruise_end = accel_t + cruise_t;
        let curve = linear_scalar_nurbs(cruise_start, cruise_end, accel_end_pos, sign * cruise_v);
        segs.push(shaped_segment(
            ax,
            cruise_start,
            cruise_end,
            curve,
            motor_mask,
        ));
    }

    let decel_start = accel_t + cruise_t;
    let decel_end = decel_start + accel_t;
    let cruise_end_pos = accel_end_pos + sign * cruise_v * cruise_t;
    let decel_curve = quad_scalar_nurbs(
        decel_start,
        decel_end,
        cruise_end_pos,
        sign * cruise_v,
        -half_accel,
    );
    segs.push(shaped_segment(
        ax,
        decel_start,
        decel_end,
        decel_curve,
        motor_mask,
    ));

    Ok(segs)
}
