//! A lane that holds while the other axes print gets its whole hold collapsed
//! into one merged piece. The merge rewrites that piece's `duration` from a
//! tick span, and the stepcompress shim later turns that `duration` back into
//! ticks to decide where the next piece must start. These tests pin the two
//! halves of that round trip to the same basis.

use super::pump_loop::Pump;
use super::sched::{SeamBasis, append_pieces_merging_holds};
use super::stall::ConsumptionStallWatch;
use super::{AxisKey, AxisQueue, EnqueueMsg, PieceSink, PumpCallbacks, SendError};
use crate::pump::MAX_LEAD_SECS;
use runtime::piece_ring::PieceEntry;
use std::collections::{BTreeMap, VecDeque};
use std::sync::Arc;
use std::sync::atomic::AtomicU64;
use step_shim::{MotorConfig, ShimError, StepShim};

/// The Voron 0 main mcu: an F103 at 72 MHz.
const EPOCH_FREQ: f64 = 72_000_000.0;
/// What the serial clock estimator reports a few minutes of heater load later.
const DRIFTED_FREQ: f64 = EPOCH_FREQ * (1.0 + 2e-6);
const ANCHOR: u64 = 869_400_000_000;
const HOLD_SECS: f32 = 0.005;
const HOLD_TICKS: u64 = 360_000;
/// 20 s of first-layer motion on X/Y with Z parked.
const HOLD_COUNT: u64 = 4_000;

fn hold(index: u64) -> (PieceEntry, f64) {
    let mut p = PieceEntry::zeroed();
    p.start_time = ANCHOR + index * HOLD_TICKS;
    p.duration = HOLD_SECS;
    p.coeffs[0] = 3.25;
    (p, index as f64 * f64::from(HOLD_SECS))
}

/// The move that ends the hold: three coefficients, so it never merges and its
/// start is seam-checked against wherever the merged hold projected its end.
fn move_after_the_hold() -> (PieceEntry, f64) {
    let mut p = PieceEntry::zeroed();
    p.start_time = ANCHOR + HOLD_COUNT * HOLD_TICKS;
    p.duration = HOLD_SECS;
    p.coeff_count = 3;
    p.coeffs[0] = 3.25;
    p.coeffs[1] = 0.05;
    (p, HOLD_COUNT as f64 * f64::from(HOLD_SECS))
}

fn hold_run() -> Vec<(PieceEntry, f64)> {
    let mut out: Vec<_> = (0..HOLD_COUNT).map(hold).collect();
    out.push(move_after_the_hold());
    out
}

fn shim_frozen_at_epoch() -> StepShim {
    StepShim::new(
        vec![MotorConfig {
            oid: 7,
            microstep_distance: 0.0125,
            invert_dir: false,
            max_steps_per_sample: 64,
            sample_rate_hz: 20_000.0,
            cycles_per_second: EPOCH_FREQ,
            min_rearm_cycles: 0,
            encoder: step_shim::StepEncoder::Classic {
                max_error_ticks: step_shim::compress::DEFAULT_MAX_ERROR_TICKS,
            },
        }],
        super::stepcompress_sink::SHIM_RING_DEPTH,
    )
}

fn stepcompress_basis(freq: f64) -> SeamBasis {
    SeamBasis {
        freq,
        skew_budget_cycles: step_shim::MAX_SEAM_SKEW_CYCLES / 2,
    }
}

fn push_through_shim(pieces: &VecDeque<(PieceEntry, f64)>) -> Result<(), ShimError> {
    let staged: Vec<PieceEntry> = pieces.iter().map(|(p, _)| *p).collect();
    shim_frozen_at_epoch().push_pieces(0, &staged)
}

fn merged_on(basis: SeamBasis) -> VecDeque<(PieceEntry, f64)> {
    let mut queue = VecDeque::new();
    for piece in hold_run() {
        append_pieces_merging_holds(&mut queue, vec![piece], basis, true);
    }
    queue
}

#[test]
fn merging_a_long_hold_on_the_live_clock_breaks_the_shim_seam() {
    let unbounded_live_clock = SeamBasis {
        freq: DRIFTED_FREQ,
        skew_budget_cycles: u64::MAX,
    };
    let queue = merged_on(unbounded_live_clock);
    assert_eq!(
        queue.len(),
        2,
        "the whole hold run collapses into one piece plus the trailing move"
    );

    match push_through_shim(&queue) {
        Err(ShimError::PieceGap {
            expected,
            got,
            tolerance,
            ..
        }) => {
            assert!(
                tolerance < 1_000,
                "the derived tolerance covers f32 reprojection, not a slope error; got {tolerance}"
            );
            assert!(
                expected.abs_diff(got) > 1_000,
                "a 2 ppm slope error over a 20 s hold is thousands of cycles, not rounding: \
                 expected {expected}, got {got}"
            );
        }
        other => panic!(
            "merging on a slope the shim does not share must land the merged end away from the \
             next piece's start; got {other:?}"
        ),
    }
}

#[test]
fn merging_a_long_hold_on_the_epoch_basis_keeps_the_shim_seam() {
    let queue = merged_on(stepcompress_basis(EPOCH_FREQ));
    push_through_shim(&queue).expect("every seam is inside the shim's tolerance");
    assert!(
        queue.len() < 32,
        "20 s of 5 ms holds must still collapse hard; got {} pieces",
        queue.len()
    );
}

#[test]
fn the_f32_duration_round_trip_bounds_a_merge_the_shim_will_reproject() {
    let queue = merged_on(stepcompress_basis(EPOCH_FREQ));
    let budget = step_shim::MAX_SEAM_SKEW_CYCLES / 2;
    #[allow(clippy::cast_possible_truncation)]
    let freq32 = EPOCH_FREQ as f32;
    for (piece, _) in &queue {
        let reprojected = piece.end_time(freq32);
        let planned = piece.start_time + (f64::from(piece.duration) * EPOCH_FREQ) as u64;
        assert!(
            reprojected.abs_diff(planned) <= budget,
            "duration {} s reprojects {} cycles off the span it was built from",
            piece.duration,
            reprojected.abs_diff(planned)
        );
    }
}

#[test]
fn a_hold_that_cannot_round_trip_stays_a_separate_piece() {
    // One 20 s hold followed by an abutting 5 ms hold: absorbing it would put
    // the merged end 2^24 cycles out, where an f32 duration cannot name the
    // seam within the shim's tolerance.
    let long_secs = 20.0_f32;
    let mut long = PieceEntry::zeroed();
    long.start_time = ANCHOR;
    long.duration = long_secs;
    long.coeffs[0] = 3.25;
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let mut next = long;
    next.start_time = long.end_time(EPOCH_FREQ as f32);
    next.duration = HOLD_SECS;

    let mut queue = VecDeque::from([(long, 0.0)]);
    append_pieces_merging_holds(
        &mut queue,
        vec![(next, f64::from(long_secs))],
        stepcompress_basis(EPOCH_FREQ),
        true,
    );
    assert_eq!(
        queue.len(),
        2,
        "a merge whose reprojection misses the seam must be refused, not rounded away"
    );

    // The same pair on the wire-walker basis, whose consumer tolerates 200 us,
    // still merges — piece mode keeps its wire savings.
    let mut wire_queue = VecDeque::from([(long, 0.0)]);
    append_pieces_merging_holds(
        &mut wire_queue,
        vec![(next, f64::from(long_secs))],
        SeamBasis::wire_walker(EPOCH_FREQ),
        true,
    );
    assert_eq!(wire_queue.len(), 1);
}

/// Two multi-second holds — one lane parked across a first layer, split where
/// the segment boundary fell. Merging them costs almost nothing by the merge's
/// own measure, because it weighs the rewritten `duration` against the end of
/// the piece it absorbs. The shim weighs it against the start of the piece
/// that *follows*, one whole absorbed duration further out, where the f32
/// `duration` grid is tens of cycles coarse. That is the bench's residual
/// `PieceGap`: both halves inside their own budget, the sum outside a flat
/// tolerance.
#[test]
fn a_merge_of_two_multi_second_holds_stays_inside_the_shim_seam() {
    const FIRST_SECS: f64 = 2.454;
    const SECOND_SECS: f64 = 4.352;

    #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
    let project = |secs: f64| ANCHOR + (secs * EPOCH_FREQ).round() as u64;
    #[allow(clippy::cast_possible_truncation)]
    let parked_lane = |start: u64, secs: f64| {
        let mut p = PieceEntry::zeroed();
        p.start_time = start;
        p.duration = secs as f32;
        p.coeffs[0] = 3.25;
        (p, 0.0_f64)
    };

    let mut queue = VecDeque::from([parked_lane(project(0.0), FIRST_SECS)]);
    append_pieces_merging_holds(
        &mut queue,
        vec![parked_lane(project(FIRST_SECS), SECOND_SECS)],
        stepcompress_basis(EPOCH_FREQ),
        true,
    );
    assert_eq!(queue.len(), 1, "two abutting holds must collapse into one");

    let mut resume = move_after_the_hold().0;
    resume.start_time = project(FIRST_SECS + SECOND_SECS);
    queue.push_back((resume, 0.0));

    #[allow(clippy::cast_possible_truncation)]
    let merged_end = queue[0].0.end_time(EPOCH_FREQ as f32);
    assert!(
        merged_end.abs_diff(resume.start_time) > step_shim::MAX_SEAM_SKEW_CYCLES,
        "this case only bites when the reprojection lands outside the flat tolerance; \
         merged end {merged_end} vs resume {}",
        resume.start_time
    );

    push_through_shim(&queue).expect("an in-budget merge must survive the shim's seam check");
}

#[derive(Clone, Copy)]
struct EpochBasisSink;

impl PieceSink for EpochBasisSink {
    fn send_frame(
        &self,
        _key: AxisKey,
        _pieces: &[PieceEntry],
        _start_slot: u16,
        _new_head: u32,
        _room: u32,
    ) -> Result<i32, SendError> {
        Ok(mcu_protocol::result_codes::OK)
    }

    fn seam_basis(&self, _key: AxisKey) -> Option<SeamBasis> {
        Some(stepcompress_basis(EPOCH_FREQ))
    }
}

fn pump_with(sink: EpochBasisSink, callbacks: PumpCallbacks) -> Pump<EpochBasisSink> {
    Pump {
        queues: BTreeMap::new(),
        junctions: super::JunctionTracker::default(),
        cohort: None,
        halted: BTreeMap::new(),
        sink,
        callbacks,
        history: None,
        ledger: Arc::new(crate::drain::DrainLedger::new()),
        pending_barrier_acks: Vec::new(),
        backlog: Arc::new(AtomicU64::new(0)),
        holding_ahead: false,
        data_open: true,
        intake_batch_open: false,
        consumption_stall: ConsumptionStallWatch::new(std::time::Duration::from_secs(60)),
        mem_probe: super::memstat::MemPressureProbe::new(),
    }
}

#[test]
fn the_pump_merges_holds_on_the_transports_basis_not_the_live_clock() {
    let key = AxisKey { mcu_id: 0, axis: 2 };
    let mut pump = pump_with(
        EpochBasisSink,
        PumpCallbacks {
            mcu_clock_of: Box::new(|_| Some((ANCHOR, DRIFTED_FREQ))),
            ..PumpCallbacks::noop(super::stepcompress_sink::SHIM_RING_DEPTH)
        },
    );

    for piece in hold_run() {
        pump.enqueue(EnqueueMsg {
            epoch_freq: None,
            key,
            pieces: vec![piece],
            epoch: crate::anchor::StreamEpoch::Continuation,
            lead_secs: MAX_LEAD_SECS,
            source_line: u32::MAX,
            batch_end: true,
        });
    }

    let queue: &AxisQueue = pump.queues.get(&key).expect("the lane was staged");
    push_through_shim(&queue.pieces)
        .expect("the pump must merge on the slope the shim reprojects with");
}
