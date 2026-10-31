/// Microsteps to emit for `step_phase` mm of accumulated, not-yet-emitted
/// displacement, with half-quantum hysteresis so a phase sitting exactly on a
/// boundary never rounds past it.
#[must_use]
#[allow(clippy::cast_possible_truncation, clippy::cast_precision_loss)]
pub fn quantize_step_delta(step_phase: f32, microstep_distance: f32) -> i32 {
    let target = libm::roundf(step_phase / microstep_distance) as i32;
    if target > 0 && step_phase <= (target as f32 - 0.5) * microstep_distance {
        target - 1
    } else if target < 0 && step_phase >= (target as f32 + 0.5) * microstep_distance {
        target + 1
    } else {
        target
    }
}
