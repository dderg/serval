use super::{EtherCatRing, WireSink};
use crate::pump::{AxisFrame, AxisKey, PieceSink, SendError};
use std::collections::HashMap;
use std::os::unix::net::UnixStream;
use std::sync::Arc;
use std::sync::Mutex;
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
        start_time: 1_000,
        duration: 0.001,
        coeff_count: 2,
        ..runtime::piece_ring::PieceEntry::zeroed()
    }]
}

fn frame() -> AxisFrame {
    AxisFrame {
        axis: key().axis,
        pieces: one_piece(),
        new_head: 1,
        room: 8,
        guard_recorded_ns: 0,
        guard_mcu_clock: 0,
    }
}

fn ring_filler() -> super::RingFiller {
    use ethercat_rt::setpoint_fill::{ChainFiller, LaneSpec};
    Arc::new(Mutex::new(ChainFiller::new(
        &[LaneSpec {
            axis: key().axis,
            cmd_counts_per_mm: 1_000.0,
            ff_lead_ns: 0,
        }],
        None,
        250_000,
        1,
    )))
}

#[test]
fn detached_ethercat_conn_yields_fatal_send_error() {
    let weak_to_already_dropped_conn = Arc::downgrade(&closed_conn());
    let sink = WireSink {
        stepcompress: HashMap::new(),
        samples: HashMap::new(),
        ethercat: HashMap::from([(
            key().mcu_id,
            EtherCatRing {
                conn: weak_to_already_dropped_conn,
                ring: ring_filler(),
            },
        )]),
        timeout: Duration::from_millis(50),
        transports: Arc::new(crate::axis_transport::AxisTransports::from_configs(&[])),
    };
    let frame = frame();
    match sink.send_mcu_frames(key().mcu_id, std::slice::from_ref(&frame)) {
        Err(SendError::Fatal(_)) => {}
        other => panic!("expected Fatal for a detached EtherCAT conn, got {other:?}"),
    }
}

#[test]
fn a_lane_in_no_endpoint_map_is_fatal_and_names_it() {
    let sink = WireSink {
        stepcompress: HashMap::new(),
        samples: HashMap::new(),
        ethercat: HashMap::new(),
        timeout: Duration::from_millis(50),
        transports: Arc::new(crate::axis_transport::AxisTransports::from_configs(&[])),
    };
    let frame = frame();
    let error = sink
        .send_mcu_frames(key().mcu_id, std::slice::from_ref(&frame))
        .expect_err("a lane with no transport must not be silently dropped");
    let SendError::Fatal(message) = error else {
        panic!("a lane with no transport is a wiring bug, so it must be Fatal: {error:?}");
    };
    assert!(
        message.contains("mcu 0") && message.contains("axis 0"),
        "the fatal must name the unrouted lane: {message}"
    );
}

#[test]
fn an_empty_bundle_reaches_no_transport() {
    let sink = WireSink {
        stepcompress: HashMap::new(),
        samples: HashMap::new(),
        ethercat: HashMap::new(),
        timeout: Duration::from_millis(50),
        transports: Arc::new(crate::axis_transport::AxisTransports::from_configs(&[])),
    };
    sink.send_mcu_frames(key().mcu_id, &[])
        .expect("an empty bundle asks nothing of any endpoint");
}
