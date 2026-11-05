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

/// The 64-bit trip clock for a `trsync_state` report. A report without its
/// clock parameter, or a router with no clock estimate to expand it against,
/// cannot be relayed: reading either as 0 would fire at a wrong clock.
pub fn trip_clock_from_report<E: std::fmt::Debug>(
    mcu: impl std::fmt::Display,
    clock32: Option<u32>,
    reference: Result<u64, E>,
) -> u64 {
    let clock32 = clock32
        .unwrap_or_else(|| panic!("mcu {mcu}: trsync_state report carried no clock parameter"));
    let reference = reference.unwrap_or_else(|e| {
        panic!(
            "mcu {mcu}: trip relay has no clock estimate to expand the trip clock against ({e:?})"
        )
    });
    relay_trip_clock(clock32, reference)
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
