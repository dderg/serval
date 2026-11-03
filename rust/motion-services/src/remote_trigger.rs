#[derive(Debug, PartialEq, Eq)]
pub enum RelayAction {
    Fire,
    Ignore,
}

pub fn relay_decision(can_trigger: Option<u32>, already_fired: bool) -> RelayAction {
    match can_trigger {
        Some(0) if !already_fired => RelayAction::Fire,
        _ => RelayAction::Ignore,
    }
}

pub fn relay_trip_clock(clock32: u32, reference_clock64: u64) -> u64 {
    const HOST_COMMANDED_TRIGGER_CLOCK: u32 = 0;
    if clock32 == HOST_COMMANDED_TRIGGER_CLOCK {
        return reference_clock64;
    }
    let delta = checked_cast::clock32_delta(clock32, reference_clock64)
        .unwrap_or_else(|e| panic!("relay_trip_clock: {e}"));
    reference_clock64.wrapping_add_signed(delta)
}

#[cfg(test)]
mod tests;
