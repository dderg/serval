//! Which transport each lane is streaming through *right now*.
//!
//! A lane's kind is a config-time fact, but a phase-capable lane whose motor
//! also carries a classic step/dir binding (sensorless homing needs the
//! StallGuard trip to run against the classic step queue) alternates between
//! two transports over its life. The endpoints hold both bindings; this table
//! is the single place that says which one owns the lane at any instant, and
//! both the pump thread and the pyo3 thread read it.

use crate::mcu_config::McuAxisConfig;
use crate::types::AxisKey;
use std::collections::HashMap;
use std::sync::atomic::{AtomicU8, Ordering};

pub const TRANSPORT_PULSE: u8 = 0;
pub const TRANSPORT_PHASE: u8 = 1;

#[must_use]
pub fn transport_name(mode: u8) -> &'static str {
    match mode {
        TRANSPORT_PULSE => "pulse",
        TRANSPORT_PHASE => "phase",
        _ => "unknown",
    }
}

struct LaneTransport {
    mode: AtomicU8,
    pulse_capable: bool,
    phase_capable: bool,
}

impl LaneTransport {
    fn supports(&self, mode: u8) -> bool {
        match mode {
            TRANSPORT_PULSE => self.pulse_capable,
            TRANSPORT_PHASE => self.phase_capable,
            _ => false,
        }
    }

    fn capabilities(&self) -> String {
        let mut names = Vec::new();
        if self.pulse_capable {
            names.push("pulse");
        }
        if self.phase_capable {
            names.push("phase");
        }
        names.join("+")
    }
}

#[derive(Default)]
pub struct AxisTransports {
    lanes: HashMap<AxisKey, LaneTransport>,
}

impl AxisTransports {
    #[must_use]
    pub fn from_configs(configs: &[McuAxisConfig]) -> Self {
        let mut lanes = HashMap::new();
        for cfg in configs {
            for (lane, &axis) in cfg.axes.iter().enumerate() {
                let phase_capable = cfg.phase_capable(lane);
                let key = AxisKey {
                    mcu_id: cfg.mcu_id,
                    axis: axis as u8,
                };
                lanes.insert(
                    key,
                    LaneTransport {
                        mode: AtomicU8::new(if phase_capable {
                            TRANSPORT_PHASE
                        } else {
                            TRANSPORT_PULSE
                        }),
                        pulse_capable: cfg.pulse_capable(lane),
                        phase_capable,
                    },
                );
            }
        }
        Self { lanes }
    }

    /// The transport the lane streams through now. An axis this host was never
    /// told about is not routed at all; the caller's endpoint lookup fails
    /// loudly on its own, so the neutral answer here is the pulse default.
    #[must_use]
    pub fn mode(&self, key: AxisKey) -> u8 {
        self.lanes
            .get(&key)
            .map_or(TRANSPORT_PULSE, |lane| lane.mode.load(Ordering::Acquire))
    }

    #[must_use]
    pub fn is_pulse(&self, key: AxisKey) -> bool {
        self.mode(key) == TRANSPORT_PULSE
    }

    #[must_use]
    pub fn is_phase(&self, key: AxisKey) -> bool {
        self.mode(key) == TRANSPORT_PHASE
    }

    #[must_use]
    pub fn supports(&self, key: AxisKey, mode: u8) -> bool {
        self.lanes.get(&key).is_some_and(|lane| lane.supports(mode))
    }

    /// Adopt `mode` for `key`, returning the mode it replaced. Refuses a mode
    /// the lane has no binding for: routing a stream to a transport the mcu
    /// was never configured with is the failure this table exists to catch.
    pub fn adopt(&self, key: AxisKey, mode: u8) -> Result<u8, String> {
        let lane = self.lanes.get(&key).ok_or_else(|| {
            format!(
                "transport switch: mcu {} axis {} is not a lane of this host",
                key.mcu_id, key.axis
            )
        })?;
        if !lane.supports(mode) {
            return Err(format!(
                "transport switch: mcu {} axis {} cannot stream through the {} transport; it \
                 is bound as {} only. A phase motor needs a classic step/dir binding at config \
                 time before it can be switched to pulse mode",
                key.mcu_id,
                key.axis,
                transport_name(mode),
                lane.capabilities()
            ));
        }
        Ok(lane.mode.swap(mode, Ordering::AcqRel))
    }
}

#[cfg(test)]
#[path = "axis_transport_tests.rs"]
mod axis_transport_tests;
