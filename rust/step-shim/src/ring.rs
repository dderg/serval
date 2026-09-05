use std::collections::VecDeque;

use trajectory::ClockedMotorSpan;

use crate::ShimError;

/// Two roundings of one shared stream instant: the end of a view and the
/// start of its successor are each `round(start_clock_exact + dt * freq)`.
pub const SEAM_ROUNDING_CYCLES: u64 = 2;

#[derive(Debug)]
pub struct SpanQueue {
    views: VecDeque<ClockedMotorSpan>,
    capacity: u32,
    converted: u32,
    abandoned: u32,
    seam: Option<u64>,
}

impl SpanQueue {
    pub fn new(capacity: u32) -> Self {
        Self {
            views: VecDeque::with_capacity(capacity as usize),
            capacity,
            converted: 0,
            abandoned: 0,
            seam: None,
        }
    }

    pub fn push(&mut self, motor: usize, view: ClockedMotorSpan) -> Result<(), ShimError> {
        if self.views.len() as u32 >= self.capacity {
            return Err(ShimError::QueueFull { motor });
        }
        admissible(motor, &view, self.seam)?;
        self.seam = Some(view.end_clock);
        self.views.push_back(view);
        Ok(())
    }

    /// Whether this run may be appended as one batch. The admissibility walk
    /// runs first so a malformed view is reported as such even when the batch
    /// also overflows the ring: a degenerate clock range is a producer bug,
    /// while a full queue is backpressure the caller retries.
    pub fn validate(&self, motor: usize, views: &[ClockedMotorSpan]) -> Result<(), ShimError> {
        let mut seam = self.seam;
        for view in views {
            admissible(motor, view, seam)?;
            seam = Some(view.end_clock);
        }
        if self.views.len() + views.len() > self.capacity as usize {
            return Err(ShimError::QueueFull { motor });
        }
        Ok(())
    }

    pub fn active(&self) -> Option<&ClockedMotorSpan> {
        self.views.front()
    }

    pub fn release_active(&mut self) {
        if self.views.pop_front().is_some() {
            self.converted = self.converted.wrapping_add(1);
        }
    }

    /// A cut drops every view the cursor never resolved. They free their room
    /// but are not converted motion, so they never claim playback.
    pub fn abandon_all(&mut self) {
        let dropped = u32::try_from(self.views.len()).expect("a queue holds at most u32 views");
        self.views.clear();
        self.abandoned = self.abandoned.wrapping_add(dropped);
        self.seam = None;
    }

    pub fn accept_forward_gap(
        &mut self,
        motor: usize,
        at_start_clock: u64,
    ) -> Result<(), ShimError> {
        if let Some(expected) = self.seam {
            if at_start_clock.saturating_add(SEAM_ROUNDING_CYCLES) < expected {
                return Err(ShimError::SpanGap {
                    motor,
                    expected,
                    got: at_start_clock,
                    tolerance: SEAM_ROUNDING_CYCLES,
                });
            }
        }
        self.seam = None;
        Ok(())
    }

    pub fn detach_seam(&mut self, motor: usize) -> Result<(), ShimError> {
        if !self.views.is_empty() {
            return Err(ShimError::QueueFull { motor });
        }
        self.seam = None;
        Ok(())
    }

    pub fn released(&self) -> u32 {
        self.converted.wrapping_add(self.abandoned)
    }

    pub fn len(&self) -> usize {
        self.views.len()
    }

    pub fn is_empty(&self) -> bool {
        self.views.is_empty()
    }
}

fn admissible(motor: usize, view: &ClockedMotorSpan, seam: Option<u64>) -> Result<(), ShimError> {
    if view.end_clock <= view.start_clock {
        return Err(ShimError::SpanClockDegenerate {
            motor,
            start_clock: view.start_clock,
            end_clock: view.end_clock,
        });
    }
    if let Some(expected) = seam {
        if view.start_clock.abs_diff(expected) > SEAM_ROUNDING_CYCLES {
            return Err(ShimError::SpanGap {
                motor,
                expected,
                got: view.start_clock,
                tolerance: SEAM_ROUNDING_CYCLES,
            });
        }
    }
    Ok(())
}
