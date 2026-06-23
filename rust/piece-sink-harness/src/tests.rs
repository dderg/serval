use mcu_protocol::codec::{Decode, Encode};
use mcu_protocol::messages::{AxisDiag, AxisPieces, MessageKind, PushPieces, PushPiecesResponse};
use std::sync::Mutex;

unsafe extern "C" {
    fn piece_sink_feed(b: u8);
    fn piece_sink_commit();
    fn harness_reset();
    fn harness_write_count() -> i32;
    fn harness_write_axis(i: i32) -> u8;
    fn harness_write_slot(i: i32) -> u16;
    fn harness_write_idx(i: i32) -> u8;
    fn harness_commit_count() -> i32;
    fn harness_commit_axis(i: i32) -> u8;
    fn harness_commit_head(i: i32) -> u32;
    fn harness_resp(out: *mut u8, maxlen: i32) -> i32;
    fn harness_set_commit_rc(rc: i32);
    fn harness_set_runtime_null(is_null: i32);
}

// The C parser + stub keep global state; serialize every run.
static LOCK: Mutex<()> = Mutex::new(());

#[derive(Debug, Default)]
struct Outcome {
    writes: Vec<(u8, u16, u8)>, // (axis, start_slot, index)
    commits: Vec<(u8, u32)>,    // (axis, new_head)
    resp: Vec<u8>,              // raw response payload (7B msg header + body)
}

/// Feed a frame payload (the CRC-covered bytes the demuxer hands the sink: the
/// 7-byte per-message header followed by the body) and commit.
fn run_with(frame: &[u8], commit_rc: i32, runtime_null: bool) -> Outcome {
    let _g = LOCK.lock().unwrap_or_else(|e| e.into_inner());
    unsafe {
        harness_reset();
        harness_set_commit_rc(commit_rc);
        harness_set_runtime_null(i32::from(runtime_null));
        for &b in frame {
            piece_sink_feed(b);
        }
        piece_sink_commit();

        let wc = harness_write_count().max(0);
        let writes = (0..wc)
            .map(|i| {
                (
                    harness_write_axis(i),
                    harness_write_slot(i),
                    harness_write_idx(i),
                )
            })
            .collect();
        let cc = harness_commit_count().max(0);
        let commits = (0..cc)
            .map(|i| (harness_commit_axis(i), harness_commit_head(i)))
            .collect();
        let mut buf = [0u8; 256];
        let n = harness_resp(buf.as_mut_ptr(), 256).max(0) as usize;
        Outcome {
            writes,
            commits,
            resp: buf[..n].to_vec(),
        }
    }
}

fn run(frame: &[u8]) -> Outcome {
    run_with(frame, 0, false)
}

/// 7-byte per-message header for the pieces channel: kind | version | corr_id.
fn msg_header(corr: u32) -> Vec<u8> {
    let kind = MessageKind::PushPieces.as_u16();
    let mut h = vec![
        (kind & 0xFF) as u8,
        (kind >> 8) as u8,
        0x01, // MESSAGE_VERSION_DEFAULT
    ];
    h.extend_from_slice(&corr.to_le_bytes());
    h
}

fn frame_for(corr: u32, body: &[u8]) -> Vec<u8> {
    let mut f = msg_header(corr);
    f.extend_from_slice(body);
    f
}

/// A 32-byte piece whose first 8 bytes (the start_time the parser echoes) are
/// `start_time`.
fn piece(start_time: u64) -> Vec<u8> {
    let mut p = vec![0u8; 32];
    p[..8].copy_from_slice(&start_time.to_le_bytes());
    p
}

fn resp_result(resp: &[u8]) -> i32 {
    PushPiecesResponse::decode(&resp[7..])
        .expect("response body decodes")
        .result
}

const OK: i32 = 0;

#[test]
fn happy_path_one_axis() {
    let body = PushPieces::single(2, 1, 7, 5, piece(0xAB)).encoded_to_vec();
    let out = run(&frame_for(0, &body));
    assert_eq!(out.writes, vec![(2, 7, 0)]);
    assert_eq!(out.commits, vec![(2, 5)]);
    let r = PushPiecesResponse::decode(&out.resp[7..]).unwrap();
    assert_eq!(r.result, OK);
    assert_eq!(r.arrival_clock, 0x1234_5678);
    assert_eq!(
        r.axes,
        vec![AxisDiag {
            axis_idx: 2,
            front_start_time: 0xAB
        }]
    );
}

#[test]
fn differential_three_axes_round_trips() {
    // host encoder -> C parser -> host decoder must agree end to end.
    let msg = PushPieces {
        axes: vec![
            AxisPieces {
                axis_idx: 0,
                piece_count: 1,
                start_slot: 10,
                new_head: 1,
                pieces_bytes: piece(0x111),
            },
            AxisPieces {
                axis_idx: 1,
                piece_count: 2,
                start_slot: 20,
                new_head: 2,
                pieces_bytes: [piece(0x222), piece(0x999)].concat(),
            },
            AxisPieces {
                axis_idx: 2,
                piece_count: 1,
                start_slot: 30,
                new_head: 3,
                pieces_bytes: piece(0x333),
            },
        ],
    };
    let out = run(&frame_for(0xDEAD_BEEF, &msg.encoded_to_vec()));

    // writes: axis 0 [idx0], axis 1 [idx0, idx1], axis 2 [idx0], each at its slot.
    assert_eq!(
        out.writes,
        vec![(0, 10, 0), (1, 20, 0), (1, 20, 1), (2, 30, 0)]
    );
    // one commit per axis, with its new_head.
    assert_eq!(out.commits, vec![(0, 1), (1, 2), (2, 3)]);

    let r = PushPiecesResponse::decode(&out.resp[7..]).unwrap();
    assert_eq!(r.result, OK);
    assert_eq!(
        r.axes,
        vec![
            AxisDiag {
                axis_idx: 0,
                front_start_time: 0x111
            },
            AxisDiag {
                axis_idx: 1,
                front_start_time: 0x222
            }, // FRONT piece of axis 1
            AxisDiag {
                axis_idx: 2,
                front_start_time: 0x333
            },
        ]
    );
}

#[test]
fn axis_count_zero_rejected_no_commit() {
    let mut body = vec![0u8]; // axis_count = 0
    body.extend_from_slice(&[0u8; 4]);
    let out = run(&frame_for(0, &body));
    assert!(out.commits.is_empty());
    assert_ne!(resp_result(&out.resp), OK);
}

#[test]
fn axis_count_over_max_rejected_no_commit() {
    let out = run(&frame_for(0, &[99u8]));
    assert!(out.commits.is_empty());
    assert_ne!(resp_result(&out.resp), OK);
}

#[test]
fn duplicate_axis_idx_rejected_no_commit() {
    // axis_count=2; both blocks axis_idx=1, piece_count=0.
    let mut body = vec![2u8];
    for _ in 0..2 {
        body.extend_from_slice(&[1, 0]); // axis_idx=1, piece_count=0
        body.extend_from_slice(&0u16.to_le_bytes());
        body.extend_from_slice(&0u32.to_le_bytes());
    }
    let out = run(&frame_for(0, &body));
    assert!(out.commits.is_empty());
    assert_ne!(resp_result(&out.resp), OK);
}

#[test]
fn truncated_frame_rejected_no_commit() {
    let full = frame_for(
        0,
        &PushPieces::single(0, 1, 0, 1, piece(7)).encoded_to_vec(),
    );
    let out = run(&full[..full.len() - 8]); // ends mid-piece
    assert!(out.commits.is_empty());
    assert_ne!(resp_result(&out.resp), OK);
}

#[test]
fn commit_error_surfaces_frame_level() {
    let body = PushPieces::single(0, 1, 0, 9, piece(1)).encoded_to_vec();
    let out = run_with(&frame_for(0, &body), -309 /* RING_FULL */, false);
    assert_eq!(resp_result(&out.resp), -309);
}

#[test]
fn runtime_not_init_rejected() {
    let body = PushPieces::single(0, 1, 0, 1, piece(1)).encoded_to_vec();
    let out = run_with(&frame_for(0, &body), 0, true);
    assert!(out.commits.is_empty());
    assert_ne!(resp_result(&out.resp), OK);
}

#[test]
fn fuzz_arbitrary_bytes_never_panics_or_runs_away() {
    // Deterministic LCG; no crash, no runaway, malformed never over-commits.
    // (The silent-OOB guarantee is scripts/fuzz-piece-sink.sh, the same shape
    // under AddressSanitizer.)
    let mut s: u64 = 0x9E37_79B9_7F4A_7C15;
    let mut next = || {
        s = s
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        (s >> 33) as u32
    };
    for _ in 0..50_000 {
        let n = (next() % 320) as usize;
        let frame: Vec<u8> = (0..n).map(|_| (next() & 0xFF) as u8).collect();
        let out = run(&frame);
        assert!(
            out.commits.len() <= 8,
            "commits bounded by MCU_MAX_FRAME_AXES"
        );
        assert!(
            out.writes.len() <= 8 * 255,
            "writes bounded by axes*piece_count"
        );
    }
}
