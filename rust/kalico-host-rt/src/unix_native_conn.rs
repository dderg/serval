//! `UnixNativeConn`: a blocking same-host Unix-socket client speaking pure
//! kalico-native frames. Implements [`NativeCall`] so the curve/segment
//! producers drive an EtherCAT RT endpoint exactly as they drive a serial
//! `KalicoHostIo`. Same-host => no clock-sync round-trips; the caller stamps
//! segment times on the shared `CLOCK_MONOTONIC` (see `EtherCatNode`).

use std::io::{Read, Write};
use std::os::unix::net::UnixStream;
use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::Mutex;
use std::time::{Duration, Instant};

use kalico_native_transport::demux::{Demuxer, Frame};
use kalico_native_transport::wire_helpers::decode_message_header;
use kalico_protocol::MessageKind;

use crate::host_io::kalico_native::build_kalico_control_frame;
use crate::native_call::NativeCall;
use crate::transport::TransportError;

/// Mutable I/O state guarded together so `kalico_call(&self, ...)` is `Sync`.
struct ConnState {
    stream: UnixStream,
    demux: Demuxer,
    buf: [u8; 4096],
}

pub struct UnixNativeConn {
    state: Mutex<ConnState>,
    next_cid: AtomicU32,
}

impl UnixNativeConn {
    /// Connect to a listening kalico-native endpoint at `path`.
    pub fn connect(path: &str) -> std::io::Result<Self> {
        let stream = UnixStream::connect(path)?;
        Ok(Self::from_stream(stream))
    }

    /// Wrap an already-connected stream (used by tests via `UnixStream::pair`).
    pub fn from_stream(stream: UnixStream) -> Self {
        Self {
            state: Mutex::new(ConnState {
                stream,
                demux: Demuxer::new(),
                buf: [0u8; 4096],
            }),
            // Start at 1 so a zero correlation id never collides with a
            // freshly-zeroed field on the wire.
            next_cid: AtomicU32::new(1),
        }
    }
}

impl NativeCall for UnixNativeConn {
    fn kalico_call(
        &self,
        kind: MessageKind,
        body: Vec<u8>,
        timeout: Duration,
    ) -> Result<(MessageKind, Vec<u8>), TransportError> {
        let cid = self.next_cid.fetch_add(1, Ordering::Relaxed);
        let frame = build_kalico_control_frame(kind, cid, &body);

        let mut st = self.state.lock().unwrap_or_else(|p| p.into_inner());

        st.stream.write_all(&frame).map_err(TransportError::Io)?;

        // Bound each blocking read so the deadline is honoured even if the
        // peer goes silent.
        st.stream
            .set_read_timeout(Some(Duration::from_millis(50)))
            .map_err(TransportError::Io)?;

        let deadline = Instant::now() + timeout;
        loop {
            if Instant::now() >= deadline {
                return Err(TransportError::Timeout);
            }
            let ConnState { stream, demux, buf } = &mut *st;
            let n = match stream.read(buf) {
                Ok(0) => return Err(TransportError::Closed),
                Ok(n) => n,
                Err(ref e)
                    if e.kind() == std::io::ErrorKind::WouldBlock
                        || e.kind() == std::io::ErrorKind::TimedOut =>
                {
                    continue;
                }
                Err(e) => return Err(TransportError::Io(e)),
            };
            let (frames, _errs) = demux.feed_slice(&buf[..n]);
            for f in frames {
                if let Frame::Kalico { payload, .. } = f {
                    if let Some((hdr, resp_body)) = decode_message_header(&payload) {
                        if hdr.correlation_id == cid {
                            let resp_kind = MessageKind::from_u16(hdr.kind_raw)
                                .ok_or_else(|| {
                                    TransportError::Parse(format!(
                                        "unknown response kind 0x{:04x}",
                                        hdr.kind_raw
                                    ))
                                })?;
                            return Ok((resp_kind, resp_body.to_vec()));
                        }
                        // Different correlation id (e.g. an async event):
                        // ignore and keep reading for ours.
                    }
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use kalico_native_transport::frame::{encode_frame, CHANNEL_CONTROL};
    use kalico_native_transport::wire_helpers::{encode_message_header, MESSAGE_VERSION_DEFAULT};
    use std::thread;

    /// Stub endpoint: read one framed request, reply with `reply_kind` echoing
    /// the request's correlation id and a fixed body.
    fn spawn_stub(mut peer: UnixStream, reply_kind: MessageKind, reply_body: Vec<u8>) {
        thread::spawn(move || {
            let mut demux = Demuxer::new();
            let mut buf = [0u8; 4096];
            loop {
                let n = match peer.read(&mut buf) {
                    Ok(0) | Err(_) => return,
                    Ok(n) => n,
                };
                let (frames, _e) = demux.feed_slice(&buf[..n]);
                for f in frames {
                    if let Frame::Kalico { payload, .. } = f {
                        let (hdr, _b) = decode_message_header(&payload).unwrap();
                        let mut out = encode_message_header(
                            reply_kind,
                            MESSAGE_VERSION_DEFAULT,
                            hdr.correlation_id,
                        )
                        .to_vec();
                        out.extend_from_slice(&reply_body);
                        let frame = encode_frame(CHANNEL_CONTROL, &out);
                        peer.write_all(&frame).unwrap();
                        return;
                    }
                }
            }
        });
    }

    #[test]
    fn round_trips_a_call_by_correlation_id() {
        let (client, server) = UnixStream::pair().unwrap();
        spawn_stub(server, MessageKind::LoadCurveResponse, vec![1, 2, 3, 4]);
        let conn = UnixNativeConn::from_stream(client);
        let (kind, body) = conn
            .kalico_call(MessageKind::LoadCurveCubic, vec![9, 9], Duration::from_secs(2))
            .expect("call ok");
        assert_eq!(kind, MessageKind::LoadCurveResponse);
        assert_eq!(body, vec![1, 2, 3, 4]);
    }

    #[test]
    fn times_out_when_peer_silent() {
        let (client, _server) = UnixStream::pair().unwrap();
        // _server never replies.
        let conn = UnixNativeConn::from_stream(client);
        let r = conn.kalico_call(MessageKind::PushSegment, vec![], Duration::from_millis(150));
        assert!(matches!(r, Err(TransportError::Timeout)));
    }
}
