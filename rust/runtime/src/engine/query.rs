use core::sync::atomic::Ordering;

use crate::stepping_state::MAX_AXES;

use super::{Engine, RuntimeStatus};

impl Engine {
    pub fn status(&self) -> RuntimeStatus {
        RuntimeStatus::from_u8(self.status.load(Ordering::Acquire))
    }

    pub fn last_error(&self) -> i32 {
        self.last_error.load(Ordering::Acquire)
    }

    pub fn tick_counter(&self) -> u32 {
        self.tick_counter.snapshot()
    }

    #[cfg(feature = "sample-stepping")]
    pub fn retired_counts(&self) -> [u32; MAX_AXES] {
        let mut out = [0u32; MAX_AXES];
        for (slot, lane) in out.iter_mut().zip(self.sample_lanes.iter()) {
            *slot = lane.retired();
        }
        out
    }

    #[cfg(not(feature = "sample-stepping"))]
    pub fn retired_counts(&self) -> [u32; MAX_AXES] {
        [0u32; MAX_AXES]
    }

    #[cfg(feature = "sample-stepping")]
    pub fn playback_clocks(&self) -> [u64; MAX_AXES] {
        let mut out = [0u64; MAX_AXES];
        for (slot, lane) in out.iter_mut().zip(self.sample_lanes.iter()) {
            *slot = lane.playback_clock();
        }
        out
    }

    #[cfg(not(feature = "sample-stepping"))]
    pub fn playback_clocks(&self) -> [u64; MAX_AXES] {
        [0u64; MAX_AXES]
    }

    #[cfg(feature = "sample-stepping")]
    pub fn occupancy_counts(&self) -> [u32; MAX_AXES] {
        let mut out = [0u32; MAX_AXES];
        for (slot, lane) in out.iter_mut().zip(self.sample_lanes.iter()) {
            #[allow(clippy::cast_possible_truncation)]
            {
                *slot = lane.depth() as u32;
            }
        }
        out
    }

    #[cfg(not(feature = "sample-stepping"))]
    pub fn occupancy_counts(&self) -> [u32; MAX_AXES] {
        [0u32; MAX_AXES]
    }

    #[cfg(feature = "sample-stepping")]
    pub fn head_window(&self, axis_idx: usize) -> Option<(u64, u64)> {
        self.sample_lanes.get(axis_idx)?.front_window()
    }

    #[cfg(not(feature = "sample-stepping"))]
    pub fn head_window(&self, _axis_idx: usize) -> Option<(u64, u64)> {
        None
    }

    pub fn motor_state(&self, i: usize) -> Option<(f32, f32)> {
        self.stepping_axes
            .get(i)
            .and_then(|s| s.as_ref())
            .map(|axis| (axis.p_prev, axis.v_prev))
    }
}
