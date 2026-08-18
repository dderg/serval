pub const ERR_PIECES_WHILE_HALTED: i32 = -315;
pub const ERR_RESUME_STREAM_NOT_HALTED: i32 = -316;

/// An endstop trip or host `Stop` halts the stream, and only `ResumeStream`
/// reopens it. While halted every `PushSampleRuns` is rejected, so setpoints
/// the pump drips during the trip→Stop round-trip cannot restart motion.
#[derive(Debug, Default)]
pub struct StreamHalt {
    halted: bool,
}

impl StreamHalt {
    pub fn halt(&mut self) {
        self.halted = true;
    }

    pub fn resume(&mut self) -> Result<(), i32> {
        if !self.halted {
            return Err(ERR_RESUME_STREAM_NOT_HALTED);
        }
        self.halted = false;
        Ok(())
    }

    pub fn check_push_allowed(&self) -> Result<(), i32> {
        if self.halted {
            Err(ERR_PIECES_WHILE_HALTED)
        } else {
            Ok(())
        }
    }

    #[cfg(test)]
    #[must_use]
    pub fn is_halted(&self) -> bool {
        self.halted
    }
}

#[cfg(test)]
mod tests;
