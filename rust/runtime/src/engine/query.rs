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

    pub fn retired_counts(&self) -> [u32; MAX_AXES] {
        let mut out = [0u32; MAX_AXES];
        for (slot, entry) in out.iter_mut().zip(self.stepping_axes.iter()) {
            if let Some(axis) = entry {
                *slot = axis.ring.retired_count();
            }
        }
        out
    }

    pub fn armed_window(&self, axis_idx: usize) -> Option<(u64, u64)> {
        self.stepping_axes
            .get(axis_idx)?
            .as_ref()?
            .armed
            .as_ref()
            .map(|p| (p.piece_start_cycles, p.piece_end_cycles))
    }

    pub fn occupancy_counts(&self) -> [u32; MAX_AXES] {
        let mut out = [0u32; MAX_AXES];
        for (slot, entry) in out.iter_mut().zip(self.stepping_axes.iter()) {
            if let Some(axis) = entry {
                #[allow(clippy::cast_possible_truncation)]
                {
                    *slot = axis.ring.len() as u32;
                }
            }
        }
        out
    }

    pub fn motor_state(&self, i: usize) -> Option<(f32, f32)> {
        self.stepping_axes
            .get(i)
            .and_then(|s| s.as_ref())
            .map(|axis| (axis.p_prev, axis.v_prev))
    }
}
