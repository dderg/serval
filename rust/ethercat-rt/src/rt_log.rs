//! Stderr logging for the RT thread without the stderr syscall.
//!
//! `eprintln!` on the DC thread writes into the journald pipe and can block
//! when the pipe backs up — an unbounded stall on the deadline path. The RT
//! side only formats (bounded, allocator-fast) and sends the line over a
//! channel; a companion thread does the actual write. Before `init` runs
//! (unit tests, the claim phase) lines fall through to plain `eprintln!`.
//!
//! Exit paths that are about to kill the process must keep using direct
//! `eprintln!` — queued lines die with the process.

use std::sync::mpsc::{channel, Sender};
use std::sync::OnceLock;

static TX: OnceLock<Sender<String>> = OnceLock::new();

/// Spawn the writer thread. Call before the DC loop starts pumping — under
/// mlockall(MCL_FUTURE) a thread spawn prefaults multiple milliseconds.
pub fn init() {
    let (tx, rx) = channel::<String>();
    if TX.set(tx).is_err() {
        return;
    }
    let _ = std::thread::Builder::new()
        .name("ec-rt-log".into())
        .spawn(move || {
            while let Ok(line) = rx.recv() {
                eprintln!("{line}");
            }
        });
}

pub fn log(line: String) {
    match TX.get() {
        Some(tx) => {
            let _ = tx.send(line);
        }
        None => eprintln!("{line}"),
    }
}

#[macro_export]
macro_rules! rt_eprintln {
    ($($arg:tt)*) => {
        $crate::rt_log::log(format!($($arg)*))
    };
}
