use crate::kinematics::SPATIAL_AXES;
use nurbs::bezier::BezierPiece;

#[cfg(test)]
mod tests;

pub use motion_pipeline::NudgePiece;

pub fn calc_move_time(dist: f64, speed: f64, accel: f64) -> (f64, f64, f64) {
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

fn monomial_piece(t_start: f64, t_end: f64, c0: f64, c1: f64, c2: f64) -> BezierPiece {
    BezierPiece {
        u_start: t_start,
        u_end: t_end,
        coeffs: vec![c0, c1, c2, 0.0],
    }
}

fn push_phase(out: &mut Vec<NudgePiece>, axis: u8, motor_mask: u8, piece: BezierPiece) {
    if piece.u_end > piece.u_start {
        out.push(NudgePiece {
            axis,
            motor_mask,
            piece,
        });
    }
}

pub fn plan_nudge_profile(
    axis_idx: u8,
    delta_mm: f64,
    speed: f64,
    accel: f64,
    motor_mask: u8,
    t_start_base: f64,
) -> Result<Vec<NudgePiece>, String> {
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

    let mut segs = Vec::with_capacity(3);

    if accel_t == 0.0 {
        let t0 = t_start_base;
        let t1 = t_start_base + cruise_t;
        push_phase(
            &mut segs,
            axis_idx,
            motor_mask,
            monomial_piece(t0, t1, 0.0, sign * speed, 0.0),
        );
    } else {
        let half_accel = 0.5 * accel * sign;
        let accel_end_pos = half_accel * accel_t * accel_t;

        let accel_t0 = t_start_base;
        let accel_t1 = t_start_base + accel_t;
        push_phase(
            &mut segs,
            axis_idx,
            motor_mask,
            monomial_piece(accel_t0, accel_t1, 0.0, 0.0, half_accel),
        );

        if cruise_t > 0.0 {
            let cruise_start = t_start_base + accel_t;
            let cruise_end = t_start_base + accel_t + cruise_t;
            push_phase(
                &mut segs,
                axis_idx,
                motor_mask,
                monomial_piece(
                    cruise_start,
                    cruise_end,
                    accel_end_pos,
                    sign * cruise_v,
                    0.0,
                ),
            );
        }

        let decel_start = t_start_base + accel_t + cruise_t;
        let decel_end = decel_start + accel_t;
        let cruise_end_pos = accel_end_pos + sign * cruise_v * cruise_t;
        push_phase(
            &mut segs,
            axis_idx,
            motor_mask,
            monomial_piece(
                decel_start,
                decel_end,
                cruise_end_pos,
                sign * cruise_v,
                -half_accel,
            ),
        );
    }

    if segs.is_empty() {
        return Err(format!(
            "nudge: degenerate move (delta {delta_mm}, speed {speed}, accel {accel}) produced no phases"
        ));
    }

    Ok(segs)
}
