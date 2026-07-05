use std::io::{self, Read};
use std::time::{Duration, Instant};

use serialport::SerialPort;

use mcu_transport::demux::{Demuxer, PollOutcome};

use crate::transport::TransportError;

pub struct SerialFrameIo {
    port: Box<dyn SerialPort>,
    demuxer: Demuxer,
    scratch: [u8; 1024],
}

impl SerialFrameIo {
    pub fn new(port: Box<dyn SerialPort>) -> Self {
        Self {
            port,
            demuxer: Demuxer::new(),
            scratch: [0u8; 1024],
        }
    }

    pub fn poll_frames_until(&mut self, deadline: Instant) -> Result<PollOutcome, TransportError> {
        let now = Instant::now();
        let remaining = deadline.saturating_duration_since(now);
        if let Err(e) = self.port.set_timeout(remaining) {
            return Err(TransportError::Io(io::Error::new(
                io::ErrorKind::Other,
                e.to_string(),
            )));
        }
        match self.port.read(&mut self.scratch) {
            // USB-CDC Ok(0) is an idle timeout, not a half-close; treat as Timeout so the reactor
            // stays alive across idle windows. Real disconnects arrive as Err(ENODEV).
            Ok(0) => Ok(PollOutcome::Timeout),
            Ok(n) => {
                let (frames, errors) = self.demuxer.feed_slice(&self.scratch[..n]);
                Ok(PollOutcome::Frames { frames, errors })
            }
            Err(e)
                if matches!(
                    e.kind(),
                    io::ErrorKind::TimedOut
                        | io::ErrorKind::Interrupted
                        | io::ErrorKind::WouldBlock
                ) =>
            {
                Ok(PollOutcome::Timeout)
            }
            Err(e) => Err(TransportError::Io(e)),
        }
    }

    // A single stalled poll must not kill the session: the MCU buffers ~2 s of
    // motion, and observed link stalls (100–337 ms, self-healing) are far below
    // that. Writes retry through transient TimedOut polls — each retry logged —
    // and only a stall past WRITE_STALL_LIMIT (half the MCU motion buffer) is a
    // transport fault.
    const WRITE_STALL_LIMIT: Duration = Duration::from_millis(1000);
    const WRITE_POLL: Duration = Duration::from_millis(25);

    pub fn write_all(&mut self, bytes: &[u8]) -> Result<(), TransportError> {
        let deadline = Instant::now() + Self::WRITE_STALL_LIMIT;
        if let Err(e) = self.port.set_timeout(Self::WRITE_POLL) {
            return Err(TransportError::Io(io::Error::other(e.to_string())));
        }
        let mut off = 0;
        let mut stalled_polls = 0u32;
        while off < bytes.len() {
            match io::Write::write(&mut self.port, &bytes[off..]) {
                Ok(0) => {
                    return Err(TransportError::Io(io::Error::new(
                        io::ErrorKind::WriteZero,
                        "serial write returned 0 bytes",
                    )));
                }
                Ok(n) => off += n,
                Err(e)
                    if matches!(
                        e.kind(),
                        io::ErrorKind::TimedOut
                            | io::ErrorKind::Interrupted
                            | io::ErrorKind::WouldBlock
                    ) =>
                {
                    stalled_polls += 1;
                    tracing::warn!(
                        subsystem = "mcu-comms",
                        event = "write_stall_retry",
                        written = off,
                        total = bytes.len(),
                        stalled_polls,
                        outq = ?self.port.bytes_to_write(),
                        "serial write poll stalled; retrying within stall limit"
                    );
                    if Instant::now() >= deadline {
                        return Err(TransportError::Io(io::Error::new(
                            io::ErrorKind::TimedOut,
                            format!(
                                "serial write stalled past {:?}: {off}/{} bytes written",
                                Self::WRITE_STALL_LIMIT,
                                bytes.len()
                            ),
                        )));
                    }
                }
                Err(e) => return Err(TransportError::Io(e)),
            }
        }
        Ok(())
    }

    /// Bytes queued in the kernel tty out-buffer, not yet on the wire.
    pub fn bytes_to_write(&self) -> Result<u32, TransportError> {
        self.port
            .bytes_to_write()
            .map_err(|e| TransportError::Io(io::Error::other(e.to_string())))
    }

    // NOT `self.port.flush()`: that is tcdrain(), whose in-kernel wait sleeps
    // in whole jiffies — 4 ms at HZ=250 — so every frame paid a 4–12 ms stall
    // on a raw tty (and the reactor cannot read while it waits). Polling the
    // kernel's unsent-byte count with hrtimer-backed sleeps keeps the same
    // drained-on-return backpressure at microsecond granularity.
    pub fn flush(&mut self) -> Result<(), TransportError> {
        const DRAIN_POLL: Duration = Duration::from_micros(200);
        const DRAIN_TIMEOUT: Duration = Duration::from_secs(2);
        let deadline = Instant::now() + DRAIN_TIMEOUT;
        loop {
            let pending = self
                .port
                .bytes_to_write()
                .map_err(|e| TransportError::Io(io::Error::other(e.to_string())))?;
            if pending == 0 {
                return Ok(());
            }
            if Instant::now() >= deadline {
                return Err(TransportError::Io(io::Error::new(
                    io::ErrorKind::TimedOut,
                    format!("serial drain stalled: {pending} bytes unsent after {DRAIN_TIMEOUT:?}"),
                )));
            }
            std::thread::sleep(DRAIN_POLL);
        }
    }

    #[cfg(any(test, feature = "test-harness"))]
    pub fn port_mut(&mut self) -> &mut Box<dyn SerialPort> {
        &mut self.port
    }
}

#[cfg(test)]
mod tests;
