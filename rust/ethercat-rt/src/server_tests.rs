use std::io::{self, Write};
use std::sync::Arc;
use std::time::{Duration, Instant};

use parking_lot::Mutex;

use super::{FrameServer, MAX_PENDING_BYTES};

#[derive(Default)]
struct MockState {
    sink: Vec<u8>,
    accept_budget: usize,
    fail: Option<io::ErrorKind>,
}

#[derive(Clone)]
struct MockWriter {
    state: Arc<Mutex<MockState>>,
}

impl MockWriter {
    fn new(accept_budget: usize) -> (Self, Arc<Mutex<MockState>>) {
        let state = Arc::new(Mutex::new(MockState {
            accept_budget,
            ..MockState::default()
        }));
        (
            Self {
                state: Arc::clone(&state),
            },
            state,
        )
    }
}

impl Write for MockWriter {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        let mut s = self.state.lock();
        if let Some(kind) = s.fail {
            return Err(io::Error::new(kind, "mock failure"));
        }
        if s.accept_budget == 0 {
            return Err(io::Error::new(
                io::ErrorKind::WouldBlock,
                "mock buffer full",
            ));
        }
        let n = buf.len().min(s.accept_budget);
        s.sink.extend_from_slice(&buf[..n]);
        s.accept_budget -= n;
        Ok(n)
    }

    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

fn server_with(accept_budget: usize) -> (FrameServer, Arc<Mutex<MockState>>) {
    let (writer, state) = MockWriter::new(accept_budget);
    (FrameServer::with_writer_for_test(Box::new(writer)), state)
}

#[test]
fn wouldblock_midframe_preserves_byte_exact_ordering() {
    let (mut server, state) = server_with(3);

    server.respond(&[0, 1, 2, 3, 4]);
    assert_eq!(state.lock().sink, vec![0, 1, 2]);
    assert_eq!(server.pending_len(), 2);
    assert!(!server.session_ended());

    state.lock().accept_budget = 0;
    server.respond(&[5, 6, 7]);
    assert_eq!(state.lock().sink, vec![0, 1, 2]);
    assert_eq!(server.pending_len(), 5);

    state.lock().accept_budget = usize::MAX;
    server.pump();
    assert_eq!(server.pending_len(), 0);
    server.respond(&[8, 9]);

    assert_eq!(state.lock().sink, vec![0, 1, 2, 3, 4, 5, 6, 7, 8, 9]);
    assert!(!server.session_ended());
    assert!(!server.backpressure_active());
}

#[test]
fn response_metrics_reset_and_track_backpressure() {
    let (mut server, _state) = server_with(0);

    server.reset_response_metrics();
    server.respond(&[1, 2, 3]);
    server.respond(&[4, 5]);

    let metrics = server.response_metrics();
    assert_eq!(metrics.count, 2);
    assert!(metrics.total_ns >= metrics.max_ns);
    assert_eq!(metrics.pending_bytes, 5);

    server.reset_response_metrics();
    assert_eq!(
        server.response_metrics(),
        super::ResponseMetrics {
            pending_bytes: 5,
            ..super::ResponseMetrics::default()
        }
    );
}

#[test]
fn real_write_error_ends_session_immediately() {
    let (mut server, state) = server_with(usize::MAX);
    state.lock().fail = Some(io::ErrorKind::BrokenPipe);

    server.respond(&[1, 2, 3]);

    assert!(server.session_ended());
    assert!(!server.client_connected());
    assert_eq!(server.pending_len(), 0);
}

#[test]
fn exceeding_max_pending_bytes_ends_session() {
    let (mut server, _state) = server_with(0);

    let oversized = vec![0u8; MAX_PENDING_BYTES + 1];
    server.respond(&oversized);

    assert!(server.session_ended());
    assert!(!server.client_connected());
    assert_eq!(server.pending_len(), 0);
}

#[test]
fn deadline_exceeded_ends_session() {
    let (mut server, _state) = server_with(0);

    server.respond(&[1, 2, 3]);
    assert!(server.backpressure_active());
    assert!(!server.session_ended());

    server.force_pending_since(Instant::now() - Duration::from_secs(6));
    server.pump();

    assert!(server.session_ended());
    assert!(!server.client_connected());
}

#[test]
fn recovery_clears_the_stall_clock() {
    let (mut server, state) = server_with(0);

    server.respond(&[0, 1, 2]);
    assert!(server.backpressure_active());
    assert_eq!(server.pending_len(), 3);
    assert!(!server.session_ended());

    state.lock().accept_budget = usize::MAX;
    server.pump();

    assert!(!server.backpressure_active());
    assert_eq!(server.pending_len(), 0);
    assert!(!server.session_ended());
    assert_eq!(state.lock().sink, vec![0, 1, 2]);
}
