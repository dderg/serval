//! Host instant to MCU clock through a clocksync record.

use crate::clock_regression::REGRESSION_WINDOW_SECS;

/// How far before a record's reference instant a host time may lie and still
/// be projected. The reference is the regression's decay-weighted centroid,
/// which trails the newest sample by up to one regression window, so any
/// instant from the last window is legitimately earlier than it. A host time
/// further back than that has no sample behind it: a zero, a stale cached
/// time or a record from another epoch, not jitter.
pub const NEGATIVE_PROJECTION_BOUND_SECS: f64 = REGRESSION_WINDOW_SECS;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ProjectionError {
    pub host_secs: f64,
    pub clock_offset: f64,
    pub delta_secs: f64,
}

impl std::fmt::Display for ProjectionError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "host time {} is {} s from the clock record's reference {}, beyond the {} s a \
             record can project backwards",
            self.host_secs, self.delta_secs, self.clock_offset, NEGATIVE_PROJECTION_BOUND_SECS
        )
    }
}

impl std::error::Error for ProjectionError {}

/// Ticks the record puts between its reference and `host_secs`, never negative
/// inside the bound: an instant before the reference projects to the reference
/// clock itself.
pub fn ticks_from_reference(
    clock_offset: f64,
    clock_freq: f64,
    host_secs: f64,
) -> Result<f64, ProjectionError> {
    let delta_secs = host_secs - clock_offset;
    if !(delta_secs >= -NEGATIVE_PROJECTION_BOUND_SECS) {
        return Err(ProjectionError {
            host_secs,
            clock_offset,
            delta_secs,
        });
    }
    Ok((delta_secs * clock_freq).max(0.0))
}

pub fn project_clock(
    last_clock: u64,
    clock_offset: f64,
    clock_freq: f64,
    host_secs: f64,
) -> Result<u64, ProjectionError> {
    let ticks = ticks_from_reference(clock_offset, clock_freq, host_secs)?;
    #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
    Ok(last_clock.wrapping_add(ticks as u64))
}

#[cfg(test)]
mod tests;
