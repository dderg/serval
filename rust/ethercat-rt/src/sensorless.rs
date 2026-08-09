use std::collections::VecDeque;

pub const ERR_ARM_SENSORLESS_BAD_THRESHOLD: i32 = -360;
pub const ERR_ARM_SENSORLESS_AMBIGUOUS_PAIR: i32 = -361;

/// Commanded-target samples kept per armed slot for the contact-clock
/// search. Covers the worst-case torque wind-up (homing_following_error
/// at the slowest homing speed) with margin: 4096 cycles is >1 s at any
/// supported cycle time.
const COMMANDED_HISTORY_LEN: usize = 4096;

#[derive(Debug, Clone)]
struct SensorlessArm {
    endstop_id: u8,
    torque_trip_tenth_pct: u16,
    partner: Option<usize>,
    seen_below_threshold: bool,
    fired: bool,
    commanded_history: VecDeque<(u64, i32)>,
}

impl SensorlessArm {
    fn new(endstop_id: u8, torque_trip_tenth_pct: u16, partner: Option<usize>) -> Self {
        Self {
            endstop_id,
            torque_trip_tenth_pct,
            partner,
            seen_below_threshold: false,
            fired: false,
            commanded_history: VecDeque::with_capacity(COMMANDED_HISTORY_LEN),
        }
    }

    fn record_commanded(&mut self, clock: u64, counts: i32) {
        if self.commanded_history.len() == COMMANDED_HISTORY_LEN {
            self.commanded_history.pop_front();
        }
        self.commanded_history.push_back((clock, counts));
    }

    /// The clock at which the commanded trajectory first reached the
    /// encoder position observed at the trip. A servo winds up following
    /// error against the crashed axis before the torque threshold fires,
    /// so the trip-time commanded position is past the wall; the commanded
    /// stream crossed the trip-time ACTUAL position back when the rotor was
    /// still tracking it — that clock reconstructs (host-side, from the
    /// commanded motion history) to the rotor's physical stall position.
    /// Falls back to `trip_clock` when there is no usable history.
    fn contact_clock(&self, actual_counts: i32, trip_clock: u64) -> u64 {
        let (Some(&(_, oldest)), Some(&(_, newest))) = (
            self.commanded_history.front(),
            self.commanded_history.back(),
        ) else {
            return trip_clock;
        };
        let dir = (i64::from(newest) - i64::from(oldest)).signum();
        if dir == 0 {
            return trip_clock;
        }
        self.commanded_history
            .iter()
            .find(|&&(_, counts)| dir * (i64::from(counts) - i64::from(actual_counts)) >= 0)
            .map_or(trip_clock, |&(clock, _)| clock)
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

    /// Records the commanded target applied at `clock` for every armed slot
    /// that has not fired yet; `commanded_of` returns None when the slot has
    /// no commanded frame this cycle (no motion streaming).
    pub fn record_commanded(
        &mut self,
        clock: u64,
        mut commanded_of: impl FnMut(usize) -> Option<i32>,
    ) {
        for slot in 0..self.arms.len() {
            if let Some(arm) = self.arms[slot].as_mut() {
                if !arm.fired {
                    if let Some(counts) = commanded_of(slot) {
                        arm.record_commanded(clock, counts);
                    }
                }
            }
        }
    }

    /// Polls every armed slot against `torque_of` (mechanical frame) and calls
    /// `on_trip(slot, endstop_id, torque, contact_clock)` for each slot
    /// crossing its threshold this cycle, where `contact_clock` is the
    /// crash-contact estimate from `contact_clock()` (drive-frame encoder
    /// counts via `position_of`). Returns whether any slot fired.
    pub fn poll(
        &mut self,
        trip_clock: u64,
        mut torque_of: impl FnMut(usize) -> i16,
        mut position_of: impl FnMut(usize) -> i32,
        mut on_trip: impl FnMut(usize, u8, i16, u64),
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
                    let contact = arm.contact_clock(position_of(slot), trip_clock);
                    on_trip(slot, endstop_id, torque, contact);
                }
            }
        }
        tripped
    }
}

#[cfg(test)]
mod tests;
