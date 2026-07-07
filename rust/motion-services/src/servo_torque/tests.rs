use super::*;
use mcu_protocol::codec::Decode as _;
use mcu_transport::demux::{Demuxer, Frame};
use mcu_transport::frame::{CHANNEL_CONTROL, encode_frame};
use mcu_transport::wire_helpers::{
    MESSAGE_VERSION_DEFAULT, decode_message_header, encode_message_header,
};
use std::io::{Read, Write};
use std::os::unix::net::UnixStream;

fn spawn_endpoint(peer: UnixStream, result: i32) -> std::sync::mpsc::Receiver<SetTorque> {
    spawn_endpoint_with_kind(peer, MessageKind::SetTorqueResponse, {
        let resp = SetTorqueResponse { result };
        resp.encoded_to_vec()
    })
}

fn spawn_endpoint_with_kind(
    mut peer: UnixStream,
    kind: MessageKind,
    body: Vec<u8>,
) -> std::sync::mpsc::Receiver<SetTorque> {
    let (tx, rx) = std::sync::mpsc::channel();
    std::thread::spawn(move || {
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
                    let (hdr, _body) =
                        decode_message_header(&payload).expect("valid message header");
                    let msg = SetTorque::decode(_body).expect("valid SetTorque body");
                    let _ = tx.send(msg);
                    let mut out =
                        encode_message_header(kind, MESSAGE_VERSION_DEFAULT, hdr.correlation_id)
                            .to_vec();
                    out.extend_from_slice(&body);
                    let frame = encode_frame(CHANNEL_CONTROL, &out);
                    peer.write_all(&frame).unwrap();
                    return;
                }
            }
        }
    });
    rx
}

#[test]
fn round_trips_enable_and_result() {
    let (client, server) = UnixStream::pair().unwrap();
    let rx = spawn_endpoint(server, 0);
    let conn = McuSerialConn::from_stream(client).expect("from_stream");
    let result = send_set_torque(&conn, true, 42_000).expect("call");
    assert_eq!(result, 0);
    let seen = rx.recv().expect("endpoint saw the command");
    assert_eq!(seen.value, 1);
    assert_eq!(seen.execute_at_ns, 42_000);
}

#[test]
fn surfaces_nonzero_result() {
    let (client, server) = UnixStream::pair().unwrap();
    let _rx = spawn_endpoint(server, -312);
    let conn = McuSerialConn::from_stream(client).expect("from_stream");
    assert_eq!(send_set_torque(&conn, false, 99).expect("call"), -312);
}

#[test]
fn transport_error_is_an_err() {
    let (client, live_peer) = UnixStream::pair().unwrap();
    let conn = McuSerialConn::from_stream(client).expect("from_stream needs the peer alive");
    drop(live_peer);
    assert!(send_set_torque(&conn, true, 1).is_err());
}

#[test]
fn wrong_kind_response_is_rejected() {
    let (client, server) = UnixStream::pair().unwrap();
    let _rx = spawn_endpoint_with_kind(server, MessageKind::PushPiecesResponse, vec![0u8; 20]);
    let conn = McuSerialConn::from_stream(client).expect("from_stream");
    let err = send_set_torque(&conn, true, 42_000).expect_err("should error on wrong kind");
    assert!(err.contains("unexpected response kind"));
}

fn spawn_stop_endpoint(
    mut peer: UnixStream,
    reply_kind: MessageKind,
    body: Vec<u8>,
) -> std::sync::mpsc::Receiver<u16> {
    let (tx, rx) = std::sync::mpsc::channel();
    std::thread::spawn(move || {
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
                    let (hdr, _body) =
                        decode_message_header(&payload).expect("valid message header");
                    let _ = tx.send(hdr.kind_raw);
                    let mut out = encode_message_header(
                        reply_kind,
                        MESSAGE_VERSION_DEFAULT,
                        hdr.correlation_id,
                    )
                    .to_vec();
                    out.extend_from_slice(&body);
                    peer.write_all(&encode_frame(CHANNEL_CONTROL, &out))
                        .unwrap();
                    return;
                }
            }
        }
    });
    rx
}

fn stop_response_body(result: i32, discard_clock: u64) -> Vec<u8> {
    StopResponse {
        result,
        discard_clock,
    }
    .encoded_to_vec()
}

#[test]
fn stop_round_trips_kind_and_result() {
    let (client, server) = UnixStream::pair().unwrap();
    let rx = spawn_stop_endpoint(
        server,
        MessageKind::StopResponse,
        stop_response_body(0, 12_345),
    );
    let conn = McuSerialConn::from_stream(client).expect("from_stream");
    let result = send_stop(&conn).expect("call");
    assert_eq!(result, 0);
    assert_eq!(
        rx.recv().expect("endpoint saw the command"),
        MessageKind::Stop.as_u16()
    );
}

#[test]
fn stop_surfaces_nonzero_result() {
    let (client, server) = UnixStream::pair().unwrap();
    let _rx = spawn_stop_endpoint(server, MessageKind::StopResponse, stop_response_body(-5, 0));
    let conn = McuSerialConn::from_stream(client).expect("from_stream");
    assert_eq!(send_stop(&conn).expect("call"), -5);
}

#[test]
fn stop_transport_error_is_an_err() {
    let (client, live_peer) = UnixStream::pair().unwrap();
    let conn = McuSerialConn::from_stream(client).expect("from_stream needs the peer alive");
    drop(live_peer);
    assert!(send_stop(&conn).is_err());
}

#[test]
fn stop_wrong_kind_response_is_rejected() {
    let (client, server) = UnixStream::pair().unwrap();
    let _rx = spawn_stop_endpoint(server, MessageKind::PushPiecesResponse, vec![0u8; 20]);
    let conn = McuSerialConn::from_stream(client).expect("from_stream");
    let err = send_stop(&conn).expect_err("should error on wrong kind");
    assert!(err.contains("unexpected response kind"));
}

fn spawn_buzz_endpoint(
    mut peer: UnixStream,
    result: i32,
) -> std::sync::mpsc::Receiver<ResonanceBuzz> {
    let (tx, rx) = std::sync::mpsc::channel();
    std::thread::spawn(move || {
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
                    let (hdr, body) =
                        decode_message_header(&payload).expect("valid message header");
                    let msg = ResonanceBuzz::decode(body).expect("valid ResonanceBuzz body");
                    let _ = tx.send(msg);
                    let mut out = encode_message_header(
                        MessageKind::ResonanceBuzzResponse,
                        MESSAGE_VERSION_DEFAULT,
                        hdr.correlation_id,
                    )
                    .to_vec();
                    out.extend_from_slice(&ResonanceBuzzResponse { result }.encoded_to_vec());
                    peer.write_all(&encode_frame(CHANNEL_CONTROL, &out))
                        .unwrap();
                    return;
                }
            }
        }
    });
    rx
}

#[test]
fn resonance_buzz_round_trips_args_and_result() {
    let (client, server) = UnixStream::pair().unwrap();
    let rx = spawn_buzz_endpoint(server, 0);
    let conn = McuSerialConn::from_stream(client).expect("from_stream");
    let result = send_resonance_buzz(
        &conn,
        ResonanceBuzz {
            axis_mask: 0b001,
            sign_mask: 0b010,
            freq_start_millihz: 5_000,
            freq_end_millihz: 300_000,
            amplitude_nm: 4_200,
            duration_ms: 3_000,
            ramp_ms: 300,
        },
    )
    .expect("call");
    assert_eq!(result, 0);
    let seen = rx.recv().expect("endpoint saw the command");
    assert_eq!(seen.axis_mask, 0b001);
    assert_eq!(seen.sign_mask, 0b010);
    assert_eq!(seen.freq_start_millihz, 5_000);
    assert_eq!(seen.freq_end_millihz, 300_000);
    assert_eq!(seen.amplitude_nm, 4_200);
    assert_eq!(seen.duration_ms, 3_000);
    assert_eq!(seen.ramp_ms, 300);
}

#[test]
fn resonance_buzz_surfaces_nonzero_result() {
    let (client, server) = UnixStream::pair().unwrap();
    let _rx = spawn_buzz_endpoint(server, -1);
    let conn = McuSerialConn::from_stream(client).expect("from_stream");
    assert_eq!(
        send_resonance_buzz(
            &conn,
            ResonanceBuzz {
                axis_mask: 0b001,
                sign_mask: 0,
                freq_start_millihz: 5_000,
                freq_end_millihz: 5_000,
                amplitude_nm: 100,
                duration_ms: 1_000,
                ramp_ms: 100,
            },
        )
        .expect("call"),
        -1
    );
}

fn spawn_arm_endpoint(
    mut peer: UnixStream,
    result: i32,
) -> std::sync::mpsc::Receiver<ArmSensorlessEndstop> {
    let (tx, rx) = std::sync::mpsc::channel();
    std::thread::spawn(move || {
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
                    let (hdr, body) =
                        decode_message_header(&payload).expect("valid message header");
                    let msg = ArmSensorlessEndstop::decode(body)
                        .expect("valid ArmSensorlessEndstop body");
                    let _ = tx.send(msg);
                    let mut out = encode_message_header(
                        MessageKind::ArmSensorlessEndstopResponse,
                        MESSAGE_VERSION_DEFAULT,
                        hdr.correlation_id,
                    )
                    .to_vec();
                    out.extend_from_slice(
                        &ArmSensorlessEndstopResponse { result }.encoded_to_vec(),
                    );
                    peer.write_all(&encode_frame(CHANNEL_CONTROL, &out))
                        .unwrap();
                    return;
                }
            }
        }
    });
    rx
}

#[test]
fn arm_sensorless_endstop_round_trips_args_and_result() {
    let (client, server) = UnixStream::pair().unwrap();
    let rx = spawn_arm_endpoint(server, 0);
    let conn = McuSerialConn::from_stream(client).expect("from_stream");
    let result = send_arm_sensorless_endstop(&conn, 0, 4, 500, true).expect("call");
    assert_eq!(result, 0);
    let seen = rx.recv().expect("endpoint saw the command");
    assert_eq!(seen.endstop_id, 4);
    assert_eq!(seen.torque_trip_tenth_pct, 500);
    assert_eq!(seen.enable, 1);
}

#[test]
fn arm_sensorless_endstop_surfaces_nonzero_result() {
    let (client, server) = UnixStream::pair().unwrap();
    let _rx = spawn_arm_endpoint(server, -360);
    let conn = McuSerialConn::from_stream(client).expect("from_stream");
    assert_eq!(
        send_arm_sensorless_endstop(&conn, 0, 3, 0, true).expect("call"),
        -360
    );
}
