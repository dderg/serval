pub const ERR_ARM_SENSORLESS_BAD_THRESHOLD: i32 = -360;

#[derive(Debug, Clone, Copy)]
pub struct SensorlessArm {
    slot: u8,
    endstop_id: u8,
    torque_trip_tenth_pct: u16,
    seen_below_threshold: bool,
    fired: bool,
}

impl SensorlessArm {
    #[must_use]
    pub fn new(slot: u8, endstop_id: u8, torque_trip_tenth_pct: u16) -> Self {
        Self {
            slot,
            endstop_id,
            torque_trip_tenth_pct,
            seen_below_threshold: false,
            fired: false,
        }
    }

    #[must_use]
    pub fn slot(&self) -> u8 {
        self.slot
    }

    pub fn poll(&mut self, torque_actual: i16) -> Option<u8> {
        if self.fired {
            return None;
        }
        let magnitude = u32::from(torque_actual.unsigned_abs());
        let threshold = u32::from(self.torque_trip_tenth_pct);
        if !self.seen_below_threshold {
            if magnitude < threshold {
                self.seen_below_threshold = true;
            }
            return None;
        }
        if magnitude >= threshold {
            self.fired = true;
            Some(self.endstop_id)
        } else {
            None
        }
    }
}

#[cfg(test)]
mod tests;
