use runtime_contract::axes::MAX_AXES;
use trajectory::NudgeProfile;

#[cfg(test)]
mod tests;

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
