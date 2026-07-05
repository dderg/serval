//! Background execution for EtherCAT endpoint round-trips.
//!
//! Endpoint calls (torque enable, drive limits, sensorless arm, home seed)
//! are request/response over the endpoint socket and can take hundreds of
//! milliseconds. They must never run on the klippy reactor thread: a blocked
//! reactor stops heater PWM refreshes, and the MCU shuts down with "Timer
//! too close" as soon as one lands past its deadline. Callers start the call
//! here and poll `done` from a reactor-friendly loop.

use std::collections::HashMap;
use std::sync::Mutex;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::mpsc::{Receiver, TryRecvError, channel};

#[derive(Debug, Default)]
pub struct BgCalls {
    next_id: AtomicU64,
    pending: Mutex<HashMap<u64, Receiver<Result<(), String>>>>,
}

impl BgCalls {
    pub fn start(
        &self,
        what: &'static str,
        call: impl FnOnce() -> Result<(), String> + Send + 'static,
    ) -> u64 {
        let (tx, rx) = channel();
        std::thread::Builder::new()
            .name(format!("bg-{what}"))
            .spawn(move || {
                let _ = tx.send(call());
            })
            .unwrap_or_else(|e| panic!("failed to spawn bg-{what} thread: {e}"));
        let id = self.next_id.fetch_add(1, Ordering::Relaxed) + 1;
        self.pending
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .insert(id, rx);
        id
    }

    /// `Ok(false)` while the call is still running. `Ok(true)` once it
    /// finished successfully, `Err` once it failed — both consume the id.
    pub fn done(&self, id: u64) -> Result<bool, String> {
        let mut pending = self.pending.lock().unwrap_or_else(|p| p.into_inner());
        let Some(rx) = pending.get(&id) else {
            return Err(format!("endpoint call {id}: unknown or already consumed"));
        };
        match rx.try_recv() {
            Err(TryRecvError::Empty) => Ok(false),
            Ok(result) => {
                pending.remove(&id);
                result.map(|()| true)
            }
            Err(TryRecvError::Disconnected) => {
                pending.remove(&id);
                Err(format!(
                    "endpoint call {id}: worker thread died without a result"
                ))
            }
        }
    }
}

#[cfg(test)]
mod tests;
