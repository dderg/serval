//! Sends one gentle there-and-back move to a running kalico-ethercat-rt endpoint.
//!
//! Usage: ec-test-client [--socket PATH] [--mm F] [--secs F]
//!
//! Time-domain note: both this client and the endpoint read the host-wide
//! `CLOCK_MONOTONIC` epoch directly (via `kalico_ethercat_rt::clock::monotonic_ns`),
//! which is shared by every process on the machine — unlike `std::time::Instant`,
//! whose value is opaque and anchored per-process and therefore NOT comparable
//! across the socket. The client stamps `t_start = monotonic_ns() + LEAD_NS` so the
//! segment arrives, arms, and pre-rolls at the start position before play begins.
//! `CountMap` captures the origin on the first sample, so there is no position jump
//! regardless of where the rotor sits at arm time. When Plan 2 wires `motion-bridge`,
//! it will negotiate the host↔endpoint reference on this same `CLOCK_MONOTONIC`
//! primitive.

use std::io::{Read, Write};
use std::os::unix::net::UnixStream;
use std::time::{Duration, Instant};

use kalico_ethercat_rt::clock::monotonic_ns;
use kalico_ethercat_rt::wire::control_frame;
use kalico_native_transport::demux::{Demuxer, Frame};
use kalico_native_transport::wire_helpers::decode_message_header;
use kalico_protocol::codec::{Decode, Encode};
use kalico_protocol::messages::{LoadCurveCubic, LoadCurveResponse, MessageKind, PushSegment};

fn piece_bytes(bp: [f32; 4], dur: f32, out: &mut Vec<u8>) {
    for x in bp {
        out.extend_from_slice(&x.to_bits().to_le_bytes());
    }
    out.extend_from_slice(&dur.to_bits().to_le_bytes());
}

fn arg_val(args: &[String], key: &str) -> Option<String> {
    args.iter().position(|a| a == key).and_then(|i| args.get(i + 1).cloned())
}

/// Read bytes from `stream` into a `Demuxer` until we get a `LoadCurveResponse`
/// or the deadline passes. Returns the `curve_handle_packed` on success.
fn read_load_curve_response(stream: &mut UnixStream, deadline: Instant) -> Option<u32> {
    let mut demux = Demuxer::new();
    let mut buf = [0u8; 1024];
    loop {
        if Instant::now() >= deadline {
            eprintln!("client: timed out waiting for LoadCurveResponse");
            return None;
        }
        match stream.read(&mut buf) {
            Ok(0) => {
                eprintln!("client: server closed connection while waiting for LoadCurveResponse");
                return None;
            }
            Ok(n) => {
                let (frames, errs) = demux.feed_slice(&buf[..n]);
                for e in &errs {
                    eprintln!("client: demux error: {e:?}");
                }
                for f in frames {
                    if let Frame::Kalico { payload, .. } = f {
                        let Some((hdr, body)) = decode_message_header(&payload) else {
                            continue;
                        };
                        if MessageKind::from_u16(hdr.kind_raw) == Some(MessageKind::LoadCurveResponse) {
                            match LoadCurveResponse::decode(body) {
                                Ok(resp) => {
                                    eprintln!(
                                        "client: LoadCurveResponse result={} handle=0x{:08x}",
                                        resp.result, resp.curve_handle_packed
                                    );
                                    return Some(resp.curve_handle_packed);
                                }
                                Err(e) => {
                                    eprintln!("client: failed to decode LoadCurveResponse: {e}");
                                    return None;
                                }
                            }
                        }
                    }
                }
            }
            Err(ref e)
                if e.kind() == std::io::ErrorKind::WouldBlock
                    || e.kind() == std::io::ErrorKind::TimedOut =>
            {
                // No bytes yet; loop back and check deadline.
            }
            Err(e) => {
                eprintln!("client: read error waiting for LoadCurveResponse: {e}");
                return None;
            }
        }
    }
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let socket =
        arg_val(&args, "--socket").unwrap_or_else(|| "/tmp/kalico-ethercat.sock".into());
    let mm: f32 = arg_val(&args, "--mm").and_then(|s| s.parse().ok()).unwrap_or(20.0);
    let secs: f32 = arg_val(&args, "--secs").and_then(|s| s.parse().ok()).unwrap_or(2.0);

    let mut stream = UnixStream::connect(&socket).expect("connect");

    // Build piece bytes: ease 0 -> mm over secs, then ease mm -> 0 over secs.
    // Bernstein form [a, a, b, b] gives zero velocity at both endpoints.
    let mut pieces = Vec::new();
    piece_bytes([0.0, 0.0, mm, mm], secs, &mut pieces);
    piece_bytes([mm, mm, 0.0, 0.0], secs, &mut pieces);

    let load = LoadCurveCubic {
        slot_idx: 0,
        axis_idx: 0,
        piece_count: 2,
        pieces_bytes: pieces,
    };
    stream
        .write_all(&control_frame(MessageKind::LoadCurveCubic, 1, &load.encoded_to_vec()))
        .expect("write LoadCurveCubic");

    // Wait for the LoadCurveResponse to obtain the real curve handle assigned by
    // the endpoint. The endpoint stores into slot 0 with generation 1 on the
    // first load, so the packed handle is (1 << 16) | 0 = 0x0001_0000, but we
    // decode it from the response rather than hardcoding it so this client stays
    // correct if the pool policy ever changes.
    stream.set_read_timeout(Some(Duration::from_millis(500))).expect("set_read_timeout");
    let resp_deadline = Instant::now() + Duration::from_millis(500);
    let curve_handle_packed = read_load_curve_response(&mut stream, resp_deadline)
        .unwrap_or_else(|| {
            // Fallback: first load into slot 0 yields generation 1.
            // CurveHandle::pack = (gen << 16) | slot, gen=1, slot=0 => 0x0001_0000.
            eprintln!("client: falling back to hardcoded handle 0x0001_0000");
            0x0001_0000
        });

    // Stamp the segment on the shared host-wide CLOCK_MONOTONIC timeline (the
    // same clock the endpoint reads), with a 150 ms lead so it arrives, arms,
    // and pre-rolls at the start position before play begins. Total play time is
    // both ease pieces: 2 * secs.
    const LEAD_NS: u64 = 150_000_000;
    let t_start: u64 = monotonic_ns() + LEAD_NS;
    let t_end: u64 = t_start + (2.0 * secs * 1e9) as u64;

    let seg = PushSegment {
        id: 1,
        handle_x: curve_handle_packed,
        handle_y: 0,
        handle_z: 0,
        handle_e: 0,
        t_start,
        t_end,
        kinematics: 0,
        e_mode: 0,
        extrusion_ratio: 0.0,
    };
    stream
        .write_all(&control_frame(MessageKind::PushSegment, 2, &seg.encoded_to_vec()))
        .expect("write PushSegment");

    // Drain response bytes for ~500 ms then exit.
    stream.set_read_timeout(Some(Duration::from_millis(500))).unwrap();
    let mut buf = [0u8; 1024];
    let drain_deadline = Instant::now() + Duration::from_millis(500);
    while Instant::now() < drain_deadline {
        match stream.read(&mut buf) {
            Ok(0) => break,
            Ok(n) => eprintln!("client: {n} response bytes"),
            Err(_) => break,
        }
    }
    eprintln!("client: sent load + push (mm={mm}, secs={secs})");
}
