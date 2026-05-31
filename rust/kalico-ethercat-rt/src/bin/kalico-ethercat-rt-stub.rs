//! kalico-ethercat-rt-stub: no-hardware endpoint. Binds the kalico-native
//! socket and answers Identify/LoadCurveCubic/PushSegment/ResetCurvePool,
//! exercising the curve store + sampler, but drives NO hardware. For drive-off
//! integration testing (the real endpoint is `required-features = ["hw"]`).
//!
//! Usage: kalico-ethercat-rt-stub [--socket PATH] [--handle x|y|z|e]
//!
//! Keep the command-handling loop in sync with `kalico-ethercat-rt.rs` (the real bin) — minus FFI.

use std::thread::sleep;
use std::time::Duration;

use kalico_ethercat_rt::clock::monotonic_ns;
use kalico_ethercat_rt::curves::{ChannelTrack, CurveStore};
use kalico_ethercat_rt::server::FrameServer;
use kalico_ethercat_rt::wire::{
    Command, identify_response_frame, load_curve_response_frame, push_segment_response_frame,
    reset_pool_response_frame,
};
use kalico_protocol::messages::PushSegment;

fn arg_val(args: &[String], key: &str) -> Option<String> {
    args.iter().position(|a| a == key).and_then(|i| args.get(i + 1).cloned())
}

/// Select the curve handle for the configured axis slot from a `PushSegment`.
fn pick_handle(seg: &PushSegment, handle_sel: &str) -> u32 {
    match handle_sel {
        "y" => seg.handle_y,
        "z" => seg.handle_z,
        "e" => seg.handle_e,
        _ => seg.handle_x,
    }
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let socket =
        arg_val(&args, "--socket").unwrap_or_else(|| "/tmp/kalico-ethercat.sock".into());
    let handle_sel = arg_val(&args, "--handle").unwrap_or_else(|| "x".into());

    let store = CurveStore::new();
    let mut track: Option<ChannelTrack> = None;
    // Pending segment: (handle_packed, t_start_ns, t_end_ns).
    let mut pending: Option<(u32, u64, u64)> = None;

    let mut server = FrameServer::bind(&socket).expect("bind socket");
    eprintln!("ec-rt-stub: socket {socket}, handle {handle_sel} (NO HARDWARE)");

    loop {
        // 1) Service socket commands.
        for cmd in server.poll_commands() {
            match cmd {
                Command::Identify { correlation_id, proto_version } => {
                    server.respond(&identify_response_frame(correlation_id, proto_version));
                }
                Command::LoadCurve { correlation_id, msg } => {
                    match store.load(msg.slot_idx, msg.piece_count, &msg.pieces_bytes) {
                        Ok(handle) => {
                            server.respond(&load_curve_response_frame(correlation_id, 0, handle));
                        }
                        Err(e) => {
                            eprintln!("ec-rt-stub: load err {e:?}");
                            server.respond(&load_curve_response_frame(correlation_id, -1, 0));
                        }
                    }
                }
                Command::PushSegment { correlation_id, msg } => {
                    let h = pick_handle(&msg, &handle_sel);
                    pending = Some((h, msg.t_start, msg.t_end));
                    server.respond(&push_segment_response_frame(correlation_id, 0, msg.id));
                }
                Command::ResetPool { correlation_id } => {
                    store.reset();
                    track = None;
                    pending = None;
                    server.respond(&reset_pool_response_frame(correlation_id, 0));
                }
                Command::Unknown { kind_raw, .. } => {
                    eprintln!("ec-rt-stub: ignoring kind 0x{kind_raw:04x}");
                }
            }
        }

        let now = monotonic_ns();

        // 2) Arm a pending segment.
        if let Some((h, t0, t1)) = pending.take() {
            track = Some(ChannelTrack::arm(h, t0, t1));
        }

        // 3) Sample trajectory; discard position — no hardware to command.
        if let Some(tr) = track.as_mut() {
            let _ = tr.sample(&store, now); // exercise sampler; discard position
            if tr.is_done(now) {
                track = None;
            }
        }

        // 4) 1 ms stub cycle (replaces ec_rt_cycle).
        sleep(Duration::from_millis(1));
    }
}
