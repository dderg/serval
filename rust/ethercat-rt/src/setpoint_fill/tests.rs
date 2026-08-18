use mcu_protocol::messages::LANE_RUN_FLAG_REANCHOR;
use runtime::motion_core::arm_piece;
use runtime::piece_ring::PieceEntry;

use super::*;

const INTERVAL: u64 = 250_000;
const CPM: f64 = 3_276.8;
const GRID_INDEX: u64 = 1_000;
const GRID_CLOCK: u64 = 4_000_000_000;

fn linear_piece(start_ns: u64, duration_s: f32, from_mm: f32, to_mm: f32) -> PieceEntry {
    let mut entry = PieceEntry {
        start_time: start_ns,
        duration: duration_s,
        coeff_count: 2,
        ..PieceEntry::zeroed()
    };
    entry.coeffs[0] = (from_mm + to_mm) / 2.0;
    entry.coeffs[1] = (to_mm - from_mm) / 2.0;
    entry
}

fn filler(lanes: usize) -> ChainFiller {
    let specs: Vec<LaneSpec> = (0..lanes)
        .map(|axis| LaneSpec {
            axis: axis as u8,
            cmd_counts_per_mm: CPM,
            ff_lead_ns: 0,
        })
        .collect();
    let mut f = ChainFiller::new(&specs, None, INTERVAL, 400);
    f.observe_grid(GRID_INDEX, GRID_CLOCK)
        .expect("the grid only advances");
    f
}

#[test]
fn a_lane_run_starts_anchored_on_the_grid_index_covering_the_piece() {
    let mut f = filler(1);
    f.push_pieces(
        0,
        &[linear_piece(GRID_CLOCK + INTERVAL * 4, 0.001, 0.0, 1.0)],
    );
    let runs = f.drain().expect("fill");
    assert_eq!(runs.len(), 1);
    let run = &runs[0];
    assert_eq!(run.start_index, GRID_INDEX + 4);
    assert_eq!(run.interval_ticks, INTERVAL as u32);
    assert_eq!(run.flags & LANE_RUN_FLAG_REANCHOR, LANE_RUN_FLAG_REANCHOR);
    assert_eq!(run.sample_count as usize, run.samples.len());
    assert_eq!(run.samples.len(), 4);
    assert_eq!(run.samples[0].pos_counts, 0);
}

#[test]
fn positions_are_the_analytic_trajectory_in_anchored_counts() {
    let start = GRID_CLOCK + INTERVAL * 4;
    let piece = linear_piece(start, 0.001, 0.0, 1.0);
    let mut f = filler(1);
    f.push_pieces(0, &[piece]);
    let runs = f.drain().expect("fill");
    let armed = arm_piece(&piece, crate::setpoint_fill::CLOCK_FREQ_HZ);
    let origin = f64::from(armed.eval_pos_vel(start).0);
    for (step, sample) in runs[0].samples.iter().enumerate() {
        let clock = start + INTERVAL * step as u64;
        let pos_mm = f64::from(armed.eval_pos_vel(clock).0);
        assert_eq!(
            sample.pos_counts,
            crate::scale::mm_to_counts(pos_mm - origin, CPM)
        );
        let vel_mm_s = f64::from(armed.eval_pos_vel(clock).1);
        assert_eq!(sample.vel_ff, (vel_mm_s * CPM).round() as i32);
    }
    assert_eq!(
        runs[0].origin_mm_q16,
        (origin * 65536.0).round() as i32,
        "the anchor origin rides the wire so the endpoint can rebuild host mm"
    );
}

#[test]
fn the_next_drain_abuts_without_re_anchoring() {
    let start = GRID_CLOCK + INTERVAL * 4;
    let mut f = filler(1);
    f.push_pieces(0, &[linear_piece(start, 0.001, 0.0, 1.0)]);
    let first = f.drain().expect("first fill");
    let next_index = first[0].start_index + u64::from(first[0].sample_count);
    f.push_pieces(0, &[linear_piece(start + 1_000_000, 0.001, 1.0, 2.0)]);
    let second = f.drain().expect("second fill");
    assert_eq!(second.len(), 1);
    assert_eq!(second[0].start_index, next_index);
    assert_eq!(second[0].flags & LANE_RUN_FLAG_REANCHOR, 0);
    assert_eq!(second[0].origin_mm_q16, first[0].origin_mm_q16);
}

#[test]
fn a_coverage_gap_closes_the_run_and_the_resume_re_anchors() {
    let start = GRID_CLOCK + INTERVAL * 4;
    let mut f = filler(1);
    f.push_pieces(0, &[linear_piece(start, 0.001, 0.0, 1.0)]);
    let gap_start = start + 1_000_000 + INTERVAL * 8;
    f.push_pieces(0, &[linear_piece(gap_start, 0.001, 5.0, 6.0)]);
    let first = f.drain().expect("first fill");
    assert_eq!(first.len(), 1);
    assert_eq!(first[0].samples.len(), 4, "run stops where coverage stops");
    let second = f.drain().expect("second fill");
    assert_eq!(second.len(), 1);
    assert_eq!(
        second[0].flags & LANE_RUN_FLAG_REANCHOR,
        LANE_RUN_FLAG_REANCHOR,
        "resuming across a gap is a new anchor epoch"
    );
    assert_eq!(second[0].samples[0].pos_counts, 0);
}

#[test]
fn one_drain_never_exceeds_the_per_frame_cap() {
    let start = GRID_CLOCK + INTERVAL;
    let mut f = filler(1);
    f.push_pieces(0, &[linear_piece(start, 1.0, 0.0, 100.0)]);
    let runs = f.drain().expect("fill");
    assert_eq!(runs[0].samples.len(), MAX_FILL_CYCLES);
    assert!(f.wants_drain());
}

#[test]
fn every_lane_of_the_chain_fills_from_one_drain() {
    let start = GRID_CLOCK + INTERVAL * 2;
    let mut f = filler(3);
    f.push_pieces(0, &[linear_piece(start, 0.001, 0.0, 1.0)]);
    f.push_pieces(2, &[linear_piece(start, 0.001, 0.0, -1.0)]);
    let runs = f.drain().expect("fill");
    let axes: Vec<u8> = runs.iter().map(|r| r.axis_idx).collect();
    assert_eq!(axes, vec![0, 2]);
    assert!(runs[1].samples[3].pos_counts < 0);
}

#[test]
fn a_buzz_streams_through_the_same_runs() {
    let mut f = filler(2);
    assert_eq!(f.arm_buzz(0b01, 0, 40_000, 40_000, 20_000, 500, 5), 0);
    assert!(f.buzz_active());
    let runs = f.drain().expect("buzz fill");
    assert_eq!(runs.len(), 1, "only the driven lane gets samples");
    assert_eq!(runs[0].axis_idx, 0);
    assert_eq!(runs[0].start_index, GRID_INDEX + 400);
    assert_eq!(
        runs[0].flags & LANE_RUN_FLAG_REANCHOR,
        LANE_RUN_FLAG_REANCHOR
    );
    assert_eq!(
        runs[0].samples.len(),
        MAX_FILL_CYCLES,
        "a 500 ms buzz outlives one frame and keeps the pump draining"
    );
    assert!(runs[0].samples.iter().any(|s| s.pos_counts != 0));
    assert!(f.buzz_active());
    assert!(f.wants_drain());
}

#[test]
fn a_reset_makes_the_next_run_re_anchor() {
    let start = GRID_CLOCK + INTERVAL * 4;
    let mut f = filler(1);
    f.push_pieces(0, &[linear_piece(start, 0.001, 0.0, 1.0)]);
    f.drain().expect("first fill");
    f.reset();
    assert!(!f.wants_drain());
    f.push_pieces(0, &[linear_piece(start + 1_000_000, 0.001, 9.0, 10.0)]);
    let runs = f.drain().expect("post-reset fill");
    assert_eq!(
        runs[0].flags & LANE_RUN_FLAG_REANCHOR,
        LANE_RUN_FLAG_REANCHOR
    );
}

#[test]
fn nothing_is_filled_before_the_endpoint_grid_is_known() {
    let specs = [LaneSpec {
        axis: 0,
        cmd_counts_per_mm: CPM,
        ff_lead_ns: 0,
    }];
    let mut f = ChainFiller::new(&specs, None, INTERVAL, 400);
    f.push_pieces(0, &[linear_piece(GRID_CLOCK, 0.001, 0.0, 1.0)]);
    assert!(f.drain().expect("no grid, no runs").is_empty());
}
