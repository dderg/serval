#[derive(Debug, Clone, Copy)]
pub struct StepMotorState {
    step_accumulator: f64,
    steps_per_mm: f32,
}

impl Default for StepMotorState {
    fn default() -> Self {
        Self {
            step_accumulator: 0.0,
            steps_per_mm: 0.0,
        }
    }
}

impl StepMotorState {
    pub fn seed(&mut self, motor_position_mm: f32) {
        self.step_accumulator = f64::from(motor_position_mm) * f64::from(self.steps_per_mm);
    }

    /// Drop the sub-step residual without zeroing `steps_per_mm`. Used by
    /// `runtime_force_idle`: the motor's position is re-anchored by the host on
    /// the next segment push, so the cross-segment accumulator is meaningless;
    /// `Default::default()` must NOT be used here because it also zeros
    /// `steps_per_mm`, and the host doesn't re-call `configure()` after a flush.
    pub fn reset_accumulator(&mut self) {
        self.step_accumulator = 0.0;
    }
}
