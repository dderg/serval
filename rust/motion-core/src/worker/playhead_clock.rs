//! The clock the playhead advances on: the pacer, the intake and the
//! dispatcher read it and the ingress times its waits on it. Production
//! runs on the wall clock; tests run on one they control, so a pacing
//! property holds however the machine schedules the threads.

use std::time::Instant;

use crossbeam_channel::{Receiver, RecvTimeoutError};

use super::StreamMsg;

pub(crate) trait PlayheadClock: Send + Sync {
    fn now(&self) -> Instant;

    fn recv_until(
        &self,
        rx: &Receiver<StreamMsg>,
        deadline: Instant,
    ) -> Result<StreamMsg, RecvTimeoutError>;
}

pub(crate) struct WallClock;

impl PlayheadClock for WallClock {
    fn now(&self) -> Instant {
        Instant::now()
    }

    fn recv_until(
        &self,
        rx: &Receiver<StreamMsg>,
        deadline: Instant,
    ) -> Result<StreamMsg, RecvTimeoutError> {
        rx.recv_deadline(deadline)
    }
}
