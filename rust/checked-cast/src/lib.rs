//! `value as i64` saturates, maps NaN to 0 and truncates silently, which turns
//! a corrupt position or duration into plausible motion. These conversions
//! take an already rounded value and refuse anything the target type cannot
//! hold exactly, naming the value.

use core::fmt;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CastError {
    pub value: f64,
    pub target: &'static str,
}

impl fmt::Display for CastError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{} is not representable as {}", self.value, self.target)
    }
}

impl std::error::Error for CastError {}

const TWO_POW_32: f64 = 4_294_967_296.0;
const TWO_POW_63: f64 = 9_223_372_036_854_775_808.0;
const TWO_POW_64: f64 = 18_446_744_073_709_551_616.0;

fn require(value: f64, lo: f64, hi_exclusive: f64, target: &'static str) -> Result<f64, CastError> {
    if value.is_finite() && value >= lo && value < hi_exclusive {
        Ok(value)
    } else {
        Err(CastError { value, target })
    }
}

#[allow(clippy::cast_possible_truncation)]
pub fn f64_to_i64(value: f64) -> Result<i64, CastError> {
    require(value, -TWO_POW_63, TWO_POW_63, "i64").map(|v| v as i64)
}

#[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
pub fn f64_to_u64(value: f64) -> Result<u64, CastError> {
    require(value, 0.0, TWO_POW_64, "u64").map(|v| v as u64)
}

#[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
pub fn f64_to_u32(value: f64) -> Result<u32, CastError> {
    require(value, 0.0, TWO_POW_32, "u32").map(|v| v as u32)
}

/// Ticks either side of the reference within which a 32-bit clock still names
/// one moment unambiguously: a quarter of the circle, half of what the wrap
/// could in principle resolve, so a reference this stale or wrong is reported
/// before the answer silently lands a whole wrap away.
pub const CLOCK32_AMBIGUITY_TICKS: i64 = 1 << 30;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ClockWrapError {
    pub clock32: u32,
    pub reference: u64,
    pub delta: i64,
}

impl fmt::Display for ClockWrapError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "32-bit clock {:#x} is {} ticks from the 64-bit reference {:#x}; beyond ±{} ticks \
             the wrap is ambiguous, so the reference is stale or wrong",
            self.clock32, self.delta, self.reference, CLOCK32_AMBIGUITY_TICKS
        )
    }
}

impl std::error::Error for ClockWrapError {}

/// The signed distance from `reference` to the moment `clock32` names, taking
/// the nearest wrap. Refuses a gap past [`CLOCK32_AMBIGUITY_TICKS`].
pub fn clock32_delta(clock32: u32, reference: u64) -> Result<i64, ClockWrapError> {
    #[allow(clippy::cast_possible_truncation, clippy::cast_possible_wrap)]
    let delta = i64::from(clock32.wrapping_sub(reference as u32) as i32);
    if delta.abs() >= CLOCK32_AMBIGUITY_TICKS {
        return Err(ClockWrapError {
            clock32,
            reference,
            delta,
        });
    }
    Ok(delta)
}

#[cfg(test)]
mod tests;
