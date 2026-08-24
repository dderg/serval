use super::{EtherCatRing, WireSink};
use crate::pump::{AxisFrame, AxisKey, SendError, SpanSink};
use ethercat_rt::setpoint_fill::CLOCK_FREQ_HZ;
use std::collections::HashMap;
use std::os::unix::net::UnixStream;
use std::sync::Arc;
use std::sync::Mutex;
use std::time::Duration;
use trajectory::{
    ClockedMotorSpan, ContinuousAxis, MotorGroup, MotorSpan, MotorTerm, NudgeProfile,
};

fn closed_conn() -> Arc<host_rt::mcu_serial_conn::McuSerialConn> {
    let (client, _peer_kept_alive_for_from_stream) = UnixStream::pair().unwrap();
    Arc::new(host_rt::mcu_serial_conn::McuSerialConn::from_stream(client).expect("from_stream"))
}

fn key() -> AxisKey {
    AxisKey { mcu_id: 0, axis: 0 }
}

fn linear_span(start_ns: u64, duration_s: f64, from_mm: f64, to_mm: f64) -> ClockedMotorSpan {
    let delta = to_mm - from_mm;
    let profile =
        NudgeProfile::try_new(delta, delta.abs() / duration_s, 0.0, 0.0).expect("cruise profile");
    let duration = profile.duration();
    let groups: Arc<[MotorGroup]> = Arc::from([
        MotorGroup::Independent(MotorTerm {
            source_axis: 0,
            axis: ContinuousAxis::Hold {
                position: from_mm,
                t_start: 0.0,
                t_end: duration,
            },
            scale: 1.0,
        }),
        MotorGroup::Independent(MotorTerm {
            source_axis: 0,
            axis: ContinuousAxis::Nudge(profile),
            scale: 1.0,
        }),
    ]);
    let signal = Arc::new(MotorSpan::try_new(groups, 0.0, duration, 0, 0, false).expect("span"));
    #[allow(clippy::cast_precision_loss)]
    let start_clock_exact = start_ns as f64;
    let start_host = start_clock_exact / CLOCK_FREQ_HZ;
    ClockedMotorSpan::try_new(
        Arc::clone(&signal),
        signal.t_start,
        signal.t_end,
        start_host,
        start_host + duration,
        start_clock_exact,
        CLOCK_FREQ_HZ,
    )
    .expect("a positive-duration view on the nanosecond DC clock")
}

fn one_span() -> Vec<ClockedMotorSpan> {
    vec![linear_span(1_000, 0.001, 0.0, 1.0)]
}

fn frame() -> AxisFrame {
    AxisFrame {
        axis: key().axis,
        spans: one_span(),
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
