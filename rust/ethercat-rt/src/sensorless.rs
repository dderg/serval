pub const ERR_ARM_SENSORLESS_BAD_THRESHOLD: i32 = -360;
pub const ERR_ARM_SENSORLESS_AMBIGUOUS_PAIR: i32 = -361;

#[derive(Debug, Clone, Copy)]
struct SensorlessArm {
    endstop_id: u8,
    torque_trip_tenth_pct: u16,
    partner: Option<usize>,
    seen_below_threshold: bool,
    fired: bool,
}

impl SensorlessArm {
    fn new(endstop_id: u8, torque_trip_tenth_pct: u16, partner: Option<usize>) -> Self {
        Self {
            endstop_id,
            torque_trip_tenth_pct,
            partner,
            seen_below_threshold: false,
            fired: false,
        }
    }

    fn poll(&mut self, torque_actual: i16) -> Option<u8> {
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

/// One independently-armed sensorless endstop per drive slot. Each slot latches
/// its own below-then-above-threshold trip, so homing one drive on a multi-drive
/// chain never disturbs another drive's arm.
///
/// A slot armed with a partner (the other drive of an AWD belt pair) trips on
/// the pair's common-mode torque — the average of the two mechanical-frame
/// readings. A crash drives both rotors the same mechanical direction so the
/// signal comes through at full strength, while the pair's standing fight and
/// the differential damper's injection are antisymmetric and cancel exactly.
/// `torque_of` must therefore return MECHANICAL-frame torque (drive reading
/// flipped by the slot's direction sign); for an unpaired slot the flip cannot
/// change the magnitude the trip compares.
pub struct SensorlessBank {
    arms: Vec<Option<SensorlessArm>>,
}

impl SensorlessBank {
    #[must_use]
    pub fn new(slots: usize) -> Self {
        Self {
            arms: vec![None; slots],
        }
    }

    pub fn arm(
        &mut self,
        slot: usize,
        endstop_id: u8,
        torque_trip_tenth_pct: u16,
        partner: Option<usize>,
    ) {
        assert!(partner != Some(slot), "slot cannot partner itself");
        self.arms[slot] = Some(SensorlessArm::new(
            endstop_id,
            torque_trip_tenth_pct,
            partner,
        ));
    }

    pub fn disarm(&mut self, slot: usize) {
        self.arms[slot] = None;
    }

    /// Polls every armed slot against `torque_of` (mechanical frame) and calls
    /// `on_trip(slot, endstop_id, torque)` for each slot crossing its threshold
    /// this cycle. Returns whether any slot fired.
    pub fn poll(
        &mut self,
        mut torque_of: impl FnMut(usize) -> i16,
        mut on_trip: impl FnMut(usize, u8, i16),
    ) -> bool {
        let mut tripped = false;
        for slot in 0..self.arms.len() {
            if let Some(arm) = self.arms[slot].as_mut() {
                let torque = match arm.partner {
                    Some(partner) => {
                        ((i32::from(torque_of(slot)) + i32::from(torque_of(partner))) / 2) as i16
                    }
                    None => torque_of(slot),
                };
                if let Some(endstop_id) = arm.poll(torque) {
                    tripped = true;
                    on_trip(slot, endstop_id, torque);
                }
            }
        }
        tripped
    }
}

#[cfg(test)]
mod tests;
