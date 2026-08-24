use mcu_protocol::messages::{LaneRun, PushSampleRuns, SetpointSample};

use super::*;

fn frame(samples: usize) -> PushSampleRuns {
    PushSampleRuns {
        lanes: vec![LaneRun {
            axis_idx: 0,
            slot_idx: 0,
            flags: 0,
            origin_mm_q16: 0,
            start_index: 0,
            interval_ticks: 1_000_000,
            samples: vec![
                SetpointSample {
                    pos_counts: 0,
                    vel_ff: 0,
                    torque_ff: 0,
                    acc_mm_s2: 0.0,
                };
                samples
            ],
        }],
    }
}

#[test]
fn dispose_never_blocks_even_with_ring_overrun() {
    let mut reclaim = Reclaim::spawn();
    let started = std::time::Instant::now();
    for _ in 0..(RECLAIM_RING_CAPACITY * 4) {
        reclaim.dispose(frame(256));
    }
    assert!(
        started.elapsed() < Duration::from_secs(1),
        "dispose must never block the caller"
    );
}

#[test]
fn drop_joins_the_janitor_after_draining() {
    let mut reclaim = Reclaim::spawn();
    for _ in 0..16 {
        reclaim.dispose(frame(64));
    }
    drop(reclaim);
}
