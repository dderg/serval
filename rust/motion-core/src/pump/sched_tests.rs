use super::*;
use std::collections::BTreeMap;
use std::sync::Arc;
use trajectory::{ClockedMotorSpan, ContinuousAxis, MotorGroup, MotorSpan, MotorTerm};

const FREQ: f64 = 1_000_000.0;
const SPAN_SECS: f64 = 0.001;
const SOURCE_LINE: u32 = 11;

fn span(start_clock: u64, start_host: f64) -> ClockedMotorSpan {
    let t_start = start_host;
    let t_end = t_start + SPAN_SECS;
    let groups: Arc<[MotorGroup]> = Arc::from(vec![MotorGroup::Independent(MotorTerm {
        source_axis: 0,
        axis: ContinuousAxis::Hold {
            position: 0.0,
            t_start,
            t_end,
        },
        scale: 1.0,
    })]);
    let signal = MotorSpan::try_new(groups, t_start, t_end, 0, SOURCE_LINE, true)
        .expect("a hold motor span is dispatchable");
    #[allow(clippy::cast_precision_loss)]
    let start_clock_exact = start_clock as f64;
    ClockedMotorSpan::try_new(
        Arc::new(signal),
        t_start,
        t_end,
        t_start,
        t_end,
        start_clock_exact,
        FREQ,
    )
    .expect("the projected view spans at least one clock")
}

fn q_with_host(ring_depth: u32, starts: &[(u64, f64)]) -> AxisQueue {
    let mut q = AxisQueue::new(ring_depth);
    for &(clock, host) in starts {
        q.spans.push_back(span(clock, host));
    }
    q
}

#[allow(clippy::cast_precision_loss)]
fn q_with(ring_depth: u32, starts: &[u64]) -> AxisQueue {
    let pairs: Vec<(u64, f64)> = starts.iter().map(|&c| (c, c as f64 / FREQ)).collect();
    q_with_host(ring_depth, &pairs)
}

fn no_cap(_: &AxisKey) -> usize {
    usize::MAX
}

fn limits(spans_per_axis: usize) -> impl Fn(u32) -> BundleLimits {
    move |_| BundleLimits { spans_per_axis }
}

/// The one-instant snapshot `schedule` judges a pass against, derived from a
/// per-lane rule the way the pump derives it from a live clock read.
fn horizons(
    queues: &BTreeMap<AxisKey, AxisQueue>,
    of: impl Fn(&AxisKey, &AxisQueue) -> Option<u64>,
) -> ReleaseHorizons {
    let mut horizons = ReleaseHorizons::default();
    horizons.resample(queues, |_| None, |key, q, _| of(key, q));
    horizons
}

fn unbounded(queues: &BTreeMap<AxisKey, AxisQueue>) -> ReleaseHorizons {
    horizons(queues, |_, _| None)
}

/// The pump's own rule: one clock reading per mcu, each lane's horizon
/// derived from its own staged lead.
fn lead_horizons(queues: &BTreeMap<AxisKey, AxisQueue>, ack_now: u64) -> ReleaseHorizons {
    let mut horizons = ReleaseHorizons::default();
    horizons.resample(
        queues,
        |_| Some((ack_now, FREQ)),
        |_, q, clock| {
            clock.map(|(ack, freq)| {
                #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
                let lead_ticks = (q.lead_secs * freq) as u64;
                ack + lead_ticks
            })
        },
    );
    horizons
}

#[test]
fn idle_when_empty() {
    let queues: BTreeMap<AxisKey, AxisQueue> = BTreeMap::new();
    assert!(matches!(
        schedule(&queues, limits(255), &unbounded(&queues), no_cap),
        Schedule::Idle
    ));
}

#[test]
fn full_ring_does_not_block_another_mcu() {
    let mut queues = BTreeMap::new();
    let mut a = q_with(2, &[10]);
    a.pushed = 2;
    queues.insert(AxisKey { mcu_id: 1, axis: 0 }, a);
    queues.insert(AxisKey { mcu_id: 2, axis: 0 }, q_with(8, &[20]));
    match schedule(&queues, limits(255), &unbounded(&queues), no_cap) {
        Schedule::Send(frames) => {
            assert_eq!(frames.len(), 1);
            assert_eq!(frames[0].key, AxisKey { mcu_id: 2, axis: 0 });
        }
        other => panic!("expected ready MCU to send, got {other:?}"),
    }
}

#[test]
fn stalls_when_every_ring_is_full() {
    let mut queues = BTreeMap::new();
    let mut a = q_with(2, &[10]);
    a.pushed = 2;
    queues.insert(AxisKey { mcu_id: 1, axis: 0 }, a);
    let mut b = q_with(3, &[20]);
    b.pushed = 3;
    queues.insert(AxisKey { mcu_id: 2, axis: 0 }, b);
    assert!(matches!(
        schedule(&queues, limits(255), &unbounded(&queues), no_cap),
        Schedule::StallFull(AxisKey { mcu_id: 1, axis: 0 })
    ));
}

/// The ring room a lane regains the moment its endpoint reports the view
/// consumed — playback (retirement) trails it, and a scheduler that waited
/// for retirement would idle a ring slot the endpoint already released.
#[test]
fn a_consumed_view_frees_the_ring_slot_ahead_of_retirement() {
    let key = AxisKey { mcu_id: 1, axis: 0 };
    let mut queues = BTreeMap::new();
    let mut q = q_with(1, &[10]);
    q.pushed = 1;
    queues.insert(key, q);
    assert!(matches!(
        schedule(&queues, limits(255), &unbounded(&queues), no_cap),
        Schedule::StallFull(_)
    ));

    queues.get_mut(&key).unwrap().credit(RetiredBy::Pulse, 1, 0);
    match schedule(&queues, limits(255), &unbounded(&queues), no_cap) {
        Schedule::Send(frames) => assert_eq!(frames[0].key, key),
        other => panic!("a consumed view must free its slot before playback, got {other:?}"),
    }
}

#[test]
fn batches_head_mcu_past_other_mcu_interleave() {
    let mut queues = BTreeMap::new();
    queues.insert(AxisKey { mcu_id: 1, axis: 0 }, q_with(8, &[0, 3]));
    queues.insert(AxisKey { mcu_id: 1, axis: 1 }, q_with(8, &[1]));
    queues.insert(AxisKey { mcu_id: 2, axis: 0 }, q_with(8, &[2]));
    let s = schedule(&queues, limits(255), &unbounded(&queues), no_cap);
    match s {
        Schedule::Send(frames) => {
            let ax: Vec<_> = frames.iter().map(|f| (f.key, f.spans.len())).collect();
            assert!(
                ax.contains(&(AxisKey { mcu_id: 1, axis: 0 }, 2)),
                "head-MCU batch must not stop at another MCU's interleaved span: {ax:?}"
            );
            assert!(ax.contains(&(AxisKey { mcu_id: 1, axis: 1 }, 1)));
            assert!(!ax.iter().any(|(k, _)| k.mcu_id == 2));
        }
        other => panic!("expected Send, got {other:?}"),
    }
}

/// The Neptune 2026-07-02 in-past abort: one trajectory fanned out to two MCUs
/// produces span streams with identical host times, so a scheduler that stops
/// batching at the first cross-MCU span degenerates to one span per frame —
/// and one serial round trip per ~5 ms span cannot keep up with real time.
/// Each frame must instead carry the head MCU's whole eligible prefix.
#[test]
fn fanned_out_trajectory_still_batches_full_frames() {
    let mut queues = BTreeMap::new();
    let starts: Vec<u64> = (0..40u64).map(|i| i * 10).collect();
    for mcu_id in [1u32, 2] {
        for axis in [0u8, 1] {
            queues.insert(AxisKey { mcu_id, axis }, q_with(64, &starts));
        }
    }
    let s = schedule(&queues, limits(32), &unbounded(&queues), no_cap);
    match s {
        Schedule::Send(frames) => {
            assert!(frames.iter().all(|f| f.key.mcu_id == 1));
            assert_eq!(frames.len(), 2, "both axes of the head MCU ship together");
            for f in &frames {
                assert_eq!(
                    f.spans.len(),
                    32,
                    "frame for {:?} must batch to max_per_frame",
                    f.key
                );
            }
        }
        other => panic!("expected Send, got {other:?}"),
    }
}

#[test]
fn frame_cap_splits() {
    let mut queues = BTreeMap::new();
    queues.insert(AxisKey { mcu_id: 1, axis: 0 }, q_with(8, &[0, 1, 2, 3]));
    let s = schedule(&queues, limits(2), &unbounded(&queues), no_cap);
    match s {
        Schedule::Send(frames) => {
            assert_eq!(frames.len(), 1);
            assert_eq!(frames[0].spans.len(), 2);
        }
        other => panic!("expected Send, got {other:?}"),
    }
}

#[test]
fn serial_limits_amortize_one_transaction_across_each_axis_queue() {
    let mut queues = BTreeMap::new();
    queues.insert(AxisKey { mcu_id: 1, axis: 0 }, q_with(8, &[0, 1, 2, 3]));
    queues.insert(AxisKey { mcu_id: 1, axis: 1 }, q_with(8, &[1, 2]));
    match schedule(
        &queues,
        |_| super::messages::SERIAL_BUNDLE_LIMITS,
        &unbounded(&queues),
        no_cap,
    ) {
        Schedule::Send(frames) => {
            assert_eq!(frames.len(), 2, "both axes of the mcu ship together");
            assert_eq!(frames[0].spans.len(), 4);
            assert_eq!(frames[1].spans.len(), 2);
        }
        other => panic!("expected Send, got {other:?}"),
    }
}

#[test]
fn full_axis_does_not_block_same_mcu_sibling() {
    let mut q: BTreeMap<AxisKey, AxisQueue> = BTreeMap::new();
    let yq = q_with(8, &[0, 2]);
    let mut xq = q_with(1, &[1]);
    xq.pushed = 1;
    q.insert(AxisKey { mcu_id: 1, axis: 1 }, yq);
    q.insert(AxisKey { mcu_id: 1, axis: 0 }, xq);
    match schedule(&q, limits(255), &unbounded(&q), no_cap) {
        Schedule::Send(frames) => {
            let yf = frames
                .iter()
                .find(|f| f.key == AxisKey { mcu_id: 1, axis: 1 });
            assert!(yf.is_some(), "Y should be batched despite full sibling X");
            assert!(
                !frames
                    .iter()
                    .any(|f| f.key == AxisKey { mcu_id: 1, axis: 0 }),
                "full X must not appear in the batch"
            );
        }
        other => panic!("expected Send, got {other:?}"),
    }
}

#[test]
fn time_gate_blocks_span_beyond_horizon() {
    let mut queues = BTreeMap::new();
    queues.insert(AxisKey { mcu_id: 1, axis: 0 }, q_with(8, &[100]));
    queues.insert(AxisKey { mcu_id: 1, axis: 1 }, q_with(8, &[200]));
    match schedule(
        &queues,
        limits(255),
        &horizons(&queues, |_, _| Some(150)),
        no_cap,
    ) {
        Schedule::Send(frames) => {
            assert_eq!(frames.len(), 1, "only axis 0 should be batched");
            assert_eq!(frames[0].key, AxisKey { mcu_id: 1, axis: 0 });
            assert_eq!(frames[0].spans.len(), 1);
        }
        other => panic!("expected Send, got {other:?}"),
    }
}

#[test]
fn all_beyond_horizon_returns_stall_ahead() {
    let mut queues = BTreeMap::new();
    queues.insert(AxisKey { mcu_id: 1, axis: 0 }, q_with(8, &[1000]));
    assert!(
        matches!(
            schedule(
                &queues,
                limits(255),
                &horizons(&queues, |_, _| Some(500)),
                no_cap
            ),
            Schedule::StallAhead(AxisKey { mcu_id: 1, axis: 0 })
        ),
        "expected StallAhead when the sole span is beyond horizon"
    );
}

#[test]
fn no_horizon_none_uses_count_only_gate() {
    let mut queues = BTreeMap::new();
    queues.insert(AxisKey { mcu_id: 1, axis: 0 }, q_with(8, &[1 << 40]));
    match schedule(&queues, limits(255), &unbounded(&queues), no_cap) {
        Schedule::Send(frames) => {
            assert_eq!(frames.len(), 1);
            assert_eq!(frames[0].spans.len(), 1);
        }
        other => panic!("expected Send (no time gate), got {other:?}"),
    }
}

#[test]
fn cross_mcu_host_time_ordering_bench_regression() {
    let f446_tick: u64 = 4_790_000_000_000;
    let h7_tick: u64 = 13_800_000_000_000;

    let f446_host: f64 = 1_000.0;
    let h7_host: f64 = 1.0;

    let mut queues = BTreeMap::new();
    queues.insert(
        AxisKey { mcu_id: 1, axis: 2 },
        q_with_host(8, &[(f446_tick, f446_host)]),
    );
    queues.insert(
        AxisKey { mcu_id: 0, axis: 0 },
        q_with_host(8, &[(h7_tick, h7_host)]),
    );

    let horizons = horizons(&queues, |k, _| {
        if k.mcu_id == 0 {
            Some(h7_tick + 1_000_000)
        } else {
            Some(f446_tick - 1)
        }
    });

    match schedule(&queues, limits(255), &horizons, no_cap) {
        Schedule::Send(frames) => {
            assert_eq!(frames.len(), 1);
            assert_eq!(
                frames[0].key.mcu_id, 0,
                "H7 (mcu0) should be selected, not F446 (mcu1)"
            );
        }
        other => {
            panic!("expected Send(mcu0) — cross-MCU host-time ordering regression, got {other:?}")
        }
    }
}

#[test]
fn homing_lead_gates_span_release() {
    let ack_now: u64 = 0;

    let inside = (25_000_u64, 0.025_f64);
    let beyond = (75_000_u64, 0.075_f64);

    let mut queues = BTreeMap::new();
    let key = AxisKey { mcu_id: 1, axis: 0 };
    let mut q = q_with_host(8, &[inside, beyond]);
    q.lead_secs = 0.05;
    queues.insert(key, q);

    match schedule(
        &queues,
        limits(255),
        &lead_horizons(&queues, ack_now),
        no_cap,
    ) {
        Schedule::Send(frames) => {
            assert_eq!(frames.len(), 1);
            assert_eq!(
                frames[0].spans.len(),
                1,
                "only the inside-50ms span must release"
            );
            assert_eq!(frames[0].spans[0].start_clock, 25_000);
        }
        other => panic!("expected Send with one span, got {other:?}"),
    }

    let mut queues2 = BTreeMap::new();
    let mut q2 = q_with_host(8, &[inside, beyond]);
    q2.lead_secs = MAX_LEAD_SECS;
    queues2.insert(key, q2);

    match schedule(
        &queues2,
        limits(255),
        &lead_horizons(&queues2, ack_now),
        no_cap,
    ) {
        Schedule::Send(frames) => {
            assert_eq!(frames.len(), 1);
            assert_eq!(
                frames[0].spans.len(),
                2,
                "both spans must release under MAX_LEAD_SECS"
            );
        }
        other => panic!("expected Send with two spans, got {other:?}"),
    }
}

#[test]
fn cross_lead_per_queue_horizon_independent() {
    let ack_now: u64 = 0;

    let key_a = AxisKey { mcu_id: 1, axis: 0 };
    let key_b = AxisKey { mcu_id: 1, axis: 1 };

    let mut queues = BTreeMap::new();

    let mut qa = q_with_host(8, &[(25_000, 0.025), (75_000, 0.075)]);
    qa.lead_secs = 0.05;
    queues.insert(key_a, qa);

    let mut qb = q_with_host(8, &[(75_000, 0.075)]);
    qb.lead_secs = MAX_LEAD_SECS;
    queues.insert(key_b, qb);

    match schedule(
        &queues,
        limits(255),
        &lead_horizons(&queues, ack_now),
        no_cap,
    ) {
        Schedule::Send(frames) => {
            let a_frame = frames
                .iter()
                .find(|f| f.key == key_a)
                .expect("queue A must have a frame");
            assert_eq!(
                a_frame.spans.len(),
                1,
                "A should send only the inside-50ms span; got {} spans",
                a_frame.spans.len()
            );
            assert_eq!(
                a_frame.spans[0].start_clock, 25_000,
                "A's sent span must be the inside-horizon one"
            );

            let b_frame = frames
                .iter()
                .find(|f| f.key == key_b)
                .expect("queue B must have a frame (MAX_LEAD_SECS horizon)");
            assert_eq!(
                b_frame.spans.len(),
                1,
                "B should send its span (within MAX_LEAD_SECS horizon); got {} spans",
                b_frame.spans.len()
            );
            assert_eq!(b_frame.spans[0].start_clock, 75_000);
        }
        other => panic!("expected Send with both A-inside and B spans; got {other:?}"),
    }
}

#[test]
fn full_earliest_ring_does_not_starve_later_mcu() {
    let mut queues = BTreeMap::new();

    let mut mcu0_q = q_with_host(2, &[(100, 1.0)]);
    mcu0_q.pushed = 2;
    queues.insert(AxisKey { mcu_id: 0, axis: 0 }, mcu0_q);

    queues.insert(AxisKey { mcu_id: 1, axis: 0 }, q_with_host(8, &[(50, 5.0)]));

    match schedule(&queues, limits(255), &unbounded(&queues), no_cap) {
        Schedule::Send(frames) => {
            assert_eq!(frames.len(), 1);
            assert_eq!(frames[0].key, AxisKey { mcu_id: 1, axis: 0 });
        }
        other => panic!("expected later ready MCU to send, got {other:?}"),
    }
}

/// The releasable cap the drip path imposes on top of the ring and the
/// horizon: a lane whose cap is spent stalls ahead instead of releasing.
#[test]
fn releasable_cap_bounds_the_frame() {
    let key = AxisKey { mcu_id: 1, axis: 0 };
    let mut queues = BTreeMap::new();
    queues.insert(key, q_with(8, &[0, 1, 2]));

    match schedule(&queues, limits(255), &unbounded(&queues), |_| 2) {
        Schedule::Send(frames) => assert_eq!(frames[0].spans.len(), 2),
        other => panic!("expected Send bounded by the cap, got {other:?}"),
    }

    assert!(matches!(
        schedule(&queues, limits(255), &unbounded(&queues), |_| 0),
        Schedule::StallAhead(_)
    ));
}
