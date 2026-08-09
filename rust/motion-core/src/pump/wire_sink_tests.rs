use super::{McuTransport, WireSink};
use crate::pump::{AxisFrame, AxisKey, SendError};
use host_rt::transport::TransportError;
use std::collections::HashMap;
use std::os::unix::net::UnixStream;
use std::sync::Arc;
use std::time::Duration;

fn closed_conn() -> Arc<host_rt::mcu_serial_conn::McuSerialConn> {
    let (client, _peer_kept_alive_for_from_stream) = UnixStream::pair().unwrap();
    Arc::new(host_rt::mcu_serial_conn::McuSerialConn::from_stream(client).expect("from_stream"))
}

fn key() -> AxisKey {
    AxisKey { mcu_id: 0, axis: 0 }
}

fn one_piece() -> Vec<runtime::piece_ring::PieceEntry> {
    vec![runtime::piece_ring::PieceEntry {
        start_time: 1000,
        duration: 0.001,
        ..runtime::piece_ring::PieceEntry::zeroed()
    }]
}

#[test]
fn closed_peer_yields_fatal_send_error() {
    let conn = closed_conn();
    let sink = WireSink {
        transports: {
            let mut m = HashMap::new();
            m.insert(0, McuTransport::EtherCat(Arc::downgrade(&conn)));
            m
        },
        timeout: Duration::from_millis(50),
        clock_of: Arc::new(|_| None),
    };
    let frame = AxisFrame {
        axis: key().axis,
        pieces: one_piece(),
        start_slot: 0,
        new_head: 1,
        room: 8,
    };
    match sink.call_push_pieces(key().mcu_id, std::slice::from_ref(&frame)) {
        Err(SendError::Fatal(_)) => {}
        other => panic!("expected Fatal for closed EtherCAT peer, got {other:?}"),
    }
}

#[test]
fn detached_ethercat_conn_yields_fatal_send_error() {
    let weak_to_already_dropped_conn = Arc::downgrade(&closed_conn());
    let sink = WireSink {
        transports: {
            let mut m = HashMap::new();
            m.insert(0, McuTransport::EtherCat(weak_to_already_dropped_conn));
            m
        },
        timeout: Duration::from_millis(50),
        clock_of: Arc::new(|_| None),
    };
    let frame = AxisFrame {
        axis: key().axis,
        pieces: one_piece(),
        start_slot: 0,
        new_head: 1,
        room: 8,
    };
    match sink.call_push_pieces(key().mcu_id, std::slice::from_ref(&frame)) {
        Err(SendError::Fatal(_)) => {}
        other => panic!("expected Fatal for detached EtherCAT conn, got {other:?}"),
    }
}

#[test]
fn timeout_yields_transient_send_error() {
    let e = TransportError::Timeout;
    let is_fatal = matches!(e, TransportError::Closed | TransportError::Io(_));
    assert!(!is_fatal, "Timeout must not be fatal");
}

#[test]
fn parse_error_yields_transient_send_error() {
    let e = TransportError::Parse("bad frame".to_owned());
    let is_fatal = matches!(e, TransportError::Closed | TransportError::Io(_));
    assert!(!is_fatal, "Parse must not be fatal");
}

#[test]
fn io_error_yields_fatal_send_error() {
    let e = TransportError::Io(std::io::Error::from(std::io::ErrorKind::BrokenPipe));
    let is_fatal = matches!(e, TransportError::Closed | TransportError::Io(_));
    assert!(is_fatal, "Io must be fatal");
}

#[test]
fn closed_variant_is_fatal() {
    let e = TransportError::Closed;
    let is_fatal = matches!(e, TransportError::Closed | TransportError::Io(_));
    assert!(is_fatal, "Closed must be fatal");
}
