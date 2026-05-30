//! kalico-ethercat-rt: bring up the A6-EC in CSP/DC and stream the kalico-native
//! piece trajectory to it as encoder counts.
//!
//! Usage: kalico-ethercat-rt <ifname> [--socket PATH] [--cycle-us N]
//!        [--counts-per-mm F] [--rt-cpu N] [--rt-prio N] [--handle x|y|z|e]
#![allow(unsafe_code)]

use std::ffi::CString;
use std::sync::OnceLock;
use std::time::Instant;

use kalico_ethercat_rt::curves::{ChannelTrack, CurveStore};
use kalico_ethercat_rt::ffi;
use kalico_ethercat_rt::scale::CountMap;
use kalico_ethercat_rt::server::FrameServer;
use kalico_ethercat_rt::wire::{
    Command, identify_response_frame, load_curve_response_frame, push_segment_response_frame,
    reset_pool_response_frame,
};
use kalico_protocol::messages::PushSegment;

/// Nanoseconds since process start (CLOCK_MONOTONIC via `Instant`).
fn monotonic_ns() -> u64 {
    static START: OnceLock<Instant> = OnceLock::new();
    let start = START.get_or_init(Instant::now);
    // elapsed() wraps Duration; as_nanos() returns u128. Cap at u64::MAX
    // (~584 years) which is beyond any practical run time.
    start.elapsed().as_nanos() as u64
}

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
    let ifname = args.get(1).cloned().unwrap_or_else(|| "eth0".into());
    let socket =
        arg_val(&args, "--socket").unwrap_or_else(|| "/tmp/kalico-ethercat.sock".into());
    let cycle_us: i64 =
        arg_val(&args, "--cycle-us").and_then(|s| s.parse().ok()).unwrap_or(1000);
    let counts_per_mm: f64 =
        arg_val(&args, "--counts-per-mm").and_then(|s| s.parse().ok()).unwrap_or(3276.8);
    let rt_cpu: i32 =
        arg_val(&args, "--rt-cpu").and_then(|s| s.parse().ok()).unwrap_or(3);
    let rt_prio: i32 =
        arg_val(&args, "--rt-prio").and_then(|s| s.parse().ok()).unwrap_or(80);
    let handle_sel = arg_val(&args, "--handle").unwrap_or_else(|| "x".into());
    let cycle_ns = cycle_us * 1000;
    // Cycles per 0.5 s telemetry period. cycle_us is positive by invariant.
    let telemetry_period = u64::try_from(cycle_us)
        .map(|u| (500_000u64 / u).max(1))
        .unwrap_or(500);

    let store = CurveStore::new();
    let mut track: Option<ChannelTrack> = None;
    // Pending segment: (handle_packed, t_start_ns, t_end_ns).
    let mut pending: Option<(u32, u64, u64)> = None;
    let mut cmap: Option<CountMap> = None;

    let mut server = FrameServer::bind(&socket).expect("bind socket");
    eprintln!(
        "ec-rt: socket {socket}, cycle {cycle_us}us, counts/mm {counts_per_mm}, handle {handle_sel}"
    );

    // Bring up the drive (blocks until CiA402 operation-enabled).
    let cif = CString::new(ifname.clone()).expect("ifname must not contain NUL");
    let rc = unsafe { ffi::ec_rt_bringup(cif.as_ptr(), cycle_ns, rt_cpu, rt_prio) };
    if rc != 0 {
        eprintln!("ec-rt: bringup failed rc={rc}");
        std::process::exit(1);
    }
    eprintln!("ec-rt: drive enabled, entering DC loop");

    let mut prdiv = 0u64;
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
                            eprintln!("ec-rt: load err {e:?}");
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
                    cmap = None;
                    server.respond(&reset_pool_response_frame(correlation_id, 0));
                }
                Command::Unknown { kind_raw, .. } => {
                    eprintln!("ec-rt: ignoring kind 0x{kind_raw:04x}");
                }
            }
        }

        let now = monotonic_ns();

        // 2) Arm a pending segment once we have the DC clock reference.
        if let Some((h, t0, t1)) = pending.take() {
            track = Some(ChannelTrack::arm(h, t0, t1));
        }

        // 3) Sample trajectory -> encoder counts -> stage target.
        if let Some(tr) = track.as_mut() {
            if let Some(pos_mm) = tr.sample(&store, now) {
                let map = cmap.get_or_insert_with(|| {
                    // Capture origin on first sample: no startup jump regardless
                    // of where the rotor sits at arm time.
                    let actual = unsafe { ffi::ec_rt_get_position_actual() };
                    CountMap::new(counts_per_mm, actual, f64::from(pos_mm))
                });
                let counts = map.target_counts(f64::from(pos_mm));
                unsafe { ffi::ec_rt_set_target_position(counts) };
            }
            if tr.is_done(now) {
                track = None;
            }
        }

        // 4) One DC cycle. Pacing (sleep-to-deadline) is inside ec_rt_cycle.
        let mut toff = 0i64;
        let wkc = unsafe { ffi::ec_rt_cycle(&mut toff) };

        // 5) Telemetry every ~0.5 s.
        prdiv += 1;
        if prdiv >= telemetry_period {
            prdiv = 0;
            let (sw, err, pos, ferr) = unsafe {
                (
                    ffi::ec_rt_get_statusword(),
                    ffi::ec_rt_get_error_code(),
                    ffi::ec_rt_get_position_actual(),
                    ffi::ec_rt_get_following_error(),
                )
            };
            eprintln!(
                "ec-rt: wkc={wkc} sw=0x{sw:04x} err=0x{err:04x} pos={pos} ferr={ferr} toff={toff} active={}",
                track.is_some()
            );
            if err != 0 {
                eprintln!("ec-rt: DRIVE FAULT err=0x{err:04x}, disabling");
                break;
            }
        }
    }

    unsafe {
        ffi::ec_rt_disable();
        ffi::ec_rt_shutdown();
    }
    eprintln!("ec-rt: shutdown complete");
}
