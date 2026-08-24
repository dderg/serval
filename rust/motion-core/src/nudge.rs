use runtime::stepping_state::MAX_AXES;
use trajectory::NudgeProfile;

#[cfg(test)]
mod tests;

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

pub fn plan_nudge_profile(
    axis_idx: u8,
    delta_mm: f64,
    speed: f64,
    accel: f64,
    t_start_base: f64,
) -> Result<NudgeProfile, String> {
    if !speed.is_finite() || speed <= 0.0 {
        return Err(format!("nudge: bad speed {speed} / delta {delta_mm}"));
    }

    if axis_idx as usize >= MAX_AXES {
        return Err(format!(
            "nudge: axis_idx {axis_idx} out of range (max {})",
            MAX_AXES - 1
        ));
    }

    NudgeProfile::try_new(delta_mm, speed, accel, t_start_base)
        .map_err(|e| format!("nudge: {e} (delta {delta_mm}, speed {speed}, accel {accel})"))
}
