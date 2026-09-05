use std::collections::{BTreeMap, BTreeSet, VecDeque};
use std::sync::Arc;

use motion_core::pump::{
    AxisKey, AxisQueue, BundleLimits, LaneRelease, ReleasePlan, Schedule,
    append_spans_merging_holds, schedule,
};
use proptest::prelude::*;
use proptest::test_runner::FileFailurePersistence;
use trajectory::{
    ClockedMotorSpan, ContinuousAxis, MAX_SPAN_SECS, MotorGroup, MotorSpan, MotorTerm,
};

const FREQ: f64 = 1_000_000.0;
const SOURCE_LINE: u32 = 11;
const MAX_FRAME_VIEWS: usize = u8::MAX as usize;
const HOST_SLOTS: u8 = 4;

fn view(
    start_clock: u64,
    ticks: u64,
    stream_t_start: f64,
    start_host: f64,
    position: f64,
    motor_mask: u8,
    freq: f64,
    explicit_hold: bool,
) -> ClockedMotorSpan {
    let t_start = stream_t_start;
    let t_end = t_start + ticks as f64 / freq;
    let groups: Arc<[MotorGroup]> = Arc::from(vec![MotorGroup::Independent(MotorTerm {
        source_axis: 0,
        axis: ContinuousAxis::Hold {
            position,
            t_start,
            t_end,
        },
        scale: 1.0,
    })]);
    let signal = MotorSpan::try_new(
        groups,
        t_start,
        t_end,
        motor_mask,
        SOURCE_LINE,
        explicit_hold,
    )
    .expect("a positive-duration hold signal is dispatchable");
    ClockedMotorSpan::try_new(
        Arc::new(signal),
        t_start,
        t_end,
        start_host,
        start_host + (t_end - t_start),
        start_clock as f64,
        freq,
    )
    .expect("an integral anchor over one or more ticks projects onto the clock")
}

#[derive(Debug, Clone, Copy)]
struct ViewSpec {
    start_clock: u64,
    host_slot: u8,
}

impl ViewSpec {
    fn build(self) -> ClockedMotorSpan {
        view(
            self.start_clock,
            1_000,
            0.0,
            f64::from(self.host_slot) * 0.5,
            0.0,
            0,
            FREQ,
            true,
        )
    }
}

#[derive(Debug, Clone)]
struct LaneSpec {
    ring_depth: u32,
    pushed: u32,
    consumed: u32,
    views: Vec<ViewSpec>,
    release: LaneRelease,
}

impl LaneSpec {
    fn build(&self) -> AxisQueue {
        let mut queue = AxisQueue::new(self.ring_depth);
        queue.pushed = self.pushed;
        queue.consumed = self.consumed;
        for spec in &self.views {
            queue.spans.push_back(spec.build());
        }
        queue
    }
}

#[derive(Debug, Clone)]
struct Pass {
    lanes: Vec<(AxisKey, LaneSpec)>,
    spans_per_axis: usize,
}

struct Staged {
    queues: BTreeMap<AxisKey, AxisQueue>,
    release: BTreeMap<AxisKey, LaneRelease>,
    spans_per_axis: usize,
}

impl Pass {
    fn stage(&self) -> Staged {
        self.stage_relabelled(|key| key)
    }

    fn stage_relabelled(&self, relabel: impl Fn(AxisKey) -> AxisKey) -> Staged {
        let mut queues = BTreeMap::new();
        let mut release = BTreeMap::new();
        for (key, spec) in &self.lanes {
            let key = relabel(*key);
            queues.insert(key, spec.build());
            release.insert(key, spec.release);
        }
        Staged {
            queues,
            release,
            spans_per_axis: self.spans_per_axis,
        }
    }
}

impl Staged {
    fn plan(&self) -> ReleasePlan {
        let mut plan = ReleasePlan::default();
        plan.resample(&self.queues, |_| None, |key, _, _| self.release[key]);
        plan
    }

    fn limits(&self) -> impl Fn(u32) -> BundleLimits + use<'_> {
        move |_| BundleLimits {
            spans_per_axis: self.spans_per_axis,
        }
    }

    fn frame_cap(&self) -> usize {
        self.spans_per_axis.min(MAX_FRAME_VIEWS)
    }

    fn run(&self) -> Schedule {
        schedule(&self.queues, self.limits(), &self.plan())
    }
}

/// How many leading views a lane could release this pass if the frame cap
/// were unbounded: the ring's room, the drip cap, and the run of leading
/// views the horizon admits, whichever runs out first.
fn releasable(queue: &AxisQueue, release: LaneRelease) -> usize {
    let admitted_by_horizon = queue
        .spans
        .iter()
        .position(|span| {
            release
                .horizon
                .is_some_and(|horizon| span.start_clock > horizon)
        })
        .unwrap_or(queue.spans.len());
    admitted_by_horizon
        .min(queue.room() as usize)
        .min(release.cap)
}

#[derive(Debug, PartialEq, Eq)]
enum LaneState {
    Empty,
    /// The ring is full: no elapsed time releases this lane, only a report.
    NoRoom,
    /// Room exists but the cap is spent or the head view is past the horizon.
    Held,
    Ready(usize),
}

fn lane_state(queue: &AxisQueue, release: LaneRelease) -> LaneState {
    if queue.spans.is_empty() {
        return LaneState::Empty;
    }
    if queue.room() == 0 {
        return LaneState::NoRoom;
    }
    match releasable(queue, release) {
        0 => LaneState::Held,
        admitted => LaneState::Ready(admitted),
    }
}

fn earliest(lanes: impl Iterator<Item = (AxisKey, f64)>) -> Option<AxisKey> {
    lanes
        .min_by(|(ka, ha), (kb, hb)| ha.total_cmp(hb).then(ka.cmp(kb)))
        .map(|(key, _)| key)
}

fn head_lane(staged: &Staged) -> Option<AxisKey> {
    earliest(
        staged
            .queues
            .iter()
            .filter(|(key, queue)| {
                matches!(lane_state(queue, staged.release[key]), LaneState::Ready(_))
            })
            .map(|(key, queue)| (*key, queue.spans[0].start_host)),
    )
}

/// Every lane's released view count as the release bounds alone dictate it,
/// derived without reference to the scheduler's two selection phases.
fn expected_frames(staged: &Staged) -> BTreeMap<AxisKey, usize> {
    let Some(head) = head_lane(staged) else {
        return BTreeMap::new();
    };
    staged
        .queues
        .iter()
        .filter(|(key, _)| key.mcu_id == head.mcu_id)
        .filter_map(
            |(key, queue)| match lane_state(queue, staged.release[key]) {
                LaneState::Ready(admitted) => Some((*key, admitted.min(staged.frame_cap()))),
                _ => None,
            },
        )
        .collect()
}

fn taken_counts(schedule: &Schedule) -> BTreeMap<AxisKey, usize> {
    match schedule {
        Schedule::Send(frames) => frames.iter().map(|f| (f.key, f.spans.len())).collect(),
        Schedule::Stall { .. } => BTreeMap::new(),
    }
}

fn arb_lane_spec() -> impl Strategy<Value = LaneSpec> {
    (
        0u32..=6,
        0u32..=10,
        0u32..=10,
        prop::collection::vec(
            (prop_oneof![Just(1u64), 1u64..40_000], 0u8..HOST_SLOTS).prop_map(
                |(start_clock, host_slot)| ViewSpec {
                    start_clock,
                    host_slot,
                },
            ),
            0..=6,
        ),
        prop_oneof![Just(None), (1u64..40_000).prop_map(Some)],
        prop_oneof![Just(usize::MAX), 0usize..=6],
    )
        .prop_map(
            |(ring_depth, pushed, consumed, views, horizon, cap)| LaneSpec {
                ring_depth,
                pushed,
                consumed,
                views,
                release: LaneRelease { horizon, cap },
            },
        )
}

fn arb_pass() -> impl Strategy<Value = Pass> {
    (
        prop::collection::btree_map(
            (0u32..3, 0u8..4).prop_map(|(mcu_id, axis)| AxisKey { mcu_id, axis }),
            arb_lane_spec(),
            0..=6,
        ),
        prop_oneof![1usize..=8, Just(255), Just(300)],
    )
        .prop_map(|(lanes, spans_per_axis)| Pass {
            lanes: lanes.into_iter().collect(),
            spans_per_axis,
        })
}

/// A relabelling of the axis ids inside each mcu: the lane contents and the
/// per-mcu lane population are untouched, only the tie-break keys move.
fn arb_relabelled_pass() -> impl Strategy<Value = (Pass, BTreeMap<AxisKey, AxisKey>)> {
    arb_pass().prop_flat_map(|pass| {
        let mcus: BTreeSet<u32> = pass.lanes.iter().map(|(key, _)| key.mcu_id).collect();
        let widths: Vec<usize> = mcus
            .iter()
            .map(|mcu| {
                pass.lanes
                    .iter()
                    .filter(|(key, _)| key.mcu_id == *mcu)
                    .count()
            })
            .collect();
        let rotations = widths
            .iter()
            .map(|width| 0usize..width.max(&1) + 4)
            .collect::<Vec<_>>();
        (Just(pass), Just(mcus), rotations).prop_map(|(pass, mcus, rotations)| {
            let mut relabel = BTreeMap::new();
            for (mcu, rotation) in mcus.iter().zip(rotations) {
                let axes: Vec<u8> = pass
                    .lanes
                    .iter()
                    .filter(|(key, _)| key.mcu_id == *mcu)
                    .map(|(key, _)| key.axis)
                    .collect();
                for (index, axis) in axes.iter().enumerate() {
                    let target = axes[(index + rotation) % axes.len()];
                    relabel.insert(
                        AxisKey {
                            mcu_id: *mcu,
                            axis: *axis,
                        },
                        AxisKey {
                            mcu_id: *mcu,
                            axis: target,
                        },
                    );
                }
            }
            (pass, relabel)
        })
    })
}

#[derive(Debug, Clone, Copy)]
struct MergeSpec {
    ticks: u64,
    gap_ticks: u64,
    position_slot: u8,
    mask_slot: u8,
    freq_slot: u8,
    explicit_hold: bool,
}

impl MergeSpec {
    fn position(self) -> f64 {
        f64::from(self.position_slot) * -1.25
    }

    fn freq(self) -> f64 {
        if self.freq_slot == 0 {
            FREQ
        } else {
            2.0 * FREQ
        }
    }
}

fn arb_merge_spec() -> impl Strategy<Value = MergeSpec> {
    (
        1u64..30_000,
        prop_oneof![9 => Just(0u64), 1 => 1u64..64],
        0u8..3,
        0u8..2,
        0u8..2,
        prop_oneof![3 => Just(true), 1 => Just(false)],
    )
        .prop_map(
            |(ticks, gap_ticks, position_slot, mask_slot, freq_slot, explicit_hold)| MergeSpec {
                ticks,
                gap_ticks,
                position_slot,
                mask_slot,
                freq_slot,
                explicit_hold,
            },
        )
}

/// A clock-contiguous-by-default chain of views: the stream domain mirrors
/// the clock so every view is a faithful projection of one anchor.
fn build_chain(specs: &[MergeSpec], first_clock: u64) -> Vec<ClockedMotorSpan> {
    let mut clock = first_clock;
    let mut built = Vec::with_capacity(specs.len());
    for spec in specs {
        clock += spec.gap_ticks;
        let freq = spec.freq();
        let stream_t_start = (clock - first_clock) as f64 / freq;
        let span = view(
            clock,
            spec.ticks,
            stream_t_start,
            stream_t_start,
            spec.position(),
            spec.mask_slot,
            freq,
            spec.explicit_hold,
        );
        clock = span.end_clock;
        built.push(span);
    }
    built
}

fn covered_ticks(spans: &[ClockedMotorSpan]) -> u64 {
    spans.iter().map(|s| s.end_clock - s.start_clock).sum()
}

fn hold_position(span: &ClockedMotorSpan) -> f64 {
    span.signal
        .position(span.stream_t_start)
        .expect("a staged view evaluates at its own start")
}

proptest! {
    #![proptest_config(ProptestConfig {
        cases: 512,
        failure_persistence: Some(Box::new(FileFailurePersistence::Direct(
            "proptest-regressions/sched_fuzz.txt",
        ))),
        ..ProptestConfig::default()
    })]

    /// Safety half, read off the send alone: one mcu, queue prefixes only,
    /// and no lane past its ring, its cap, the frame cap or its horizon.
    #[test]
    fn a_send_stays_inside_every_release_bound(pass in arb_pass()) {
        let staged = pass.stage();
        let Schedule::Send(frames) = staged.run() else {
            return Ok(());
        };
        prop_assert!(!frames.is_empty(), "a send carries at least one lane");
        let mcu = frames[0].key.mcu_id;
        for frame in &frames {
            let queue = &staged.queues[&frame.key];
            let release = staged.release[&frame.key];
            prop_assert_eq!(frame.key.mcu_id, mcu, "a bundle is one mcu transaction");
            prop_assert!(!frame.spans.is_empty(), "an empty frame is not work");
            prop_assert!(frame.spans.len() <= queue.room() as usize, "{:?} overran its ring", frame.key);
            prop_assert!(frame.spans.len() <= release.cap, "{:?} overran its drip cap", frame.key);
            prop_assert!(
                frame.spans.len() <= staged.frame_cap(),
                "{:?} overran the frame cap {}",
                frame.key,
                staged.frame_cap()
            );
            for (released, staged_view) in frame.spans.iter().zip(&queue.spans) {
                prop_assert_eq!(
                    released.start_clock,
                    staged_view.start_clock,
                    "{:?} released out of queue order",
                    frame.key
                );
                if let Some(horizon) = release.horizon {
                    prop_assert!(
                        released.start_clock <= horizon,
                        "{:?} released a view at {} past its horizon {horizon}",
                        frame.key,
                        released.start_clock
                    );
                }
            }
        }
    }

    /// The endpoint ring-overflow guard, `room()` composed with the pass: a
    /// send may never leave a lane holding more views than its mcu ring. A
    /// lane whose odometers have desynced — a transport handover credits views
    /// this `pushed` never counted — reads as an empty ring by construction
    /// and stands outside the guard.
    #[test]
    fn a_send_never_oversubscribes_an_endpoint_ring(pass in arb_pass()) {
        let staged = pass.stage();
        let Schedule::Send(frames) = staged.run() else {
            return Ok(());
        };
        for frame in &frames {
            let queue = &staged.queues[&frame.key];
            let in_flight = queue.pushed.wrapping_sub(queue.consumed);
            if in_flight > queue.ring_depth {
                continue;
            }
            prop_assert!(
                in_flight as usize + frame.spans.len() <= queue.ring_depth as usize,
                "{:?} would hold {} of {} ring slots",
                frame.key,
                in_flight as usize + frame.spans.len(),
                queue.ring_depth
            );
        }
    }

    /// Completeness half: the pass ships every view the head mcu's lanes may
    /// release, and picks the head by earliest host time.
    #[test]
    fn a_pass_ships_every_releasable_view_of_the_head_mcu(pass in arb_pass()) {
        let staged = pass.stage();
        let outcome = staged.run();
        let expected = expected_frames(&staged);

        prop_assert_eq!(
            matches!(outcome, Schedule::Send(_)),
            head_lane(&staged).is_some(),
            "a pass sends exactly when some lane cleared room, cap and horizon"
        );
        prop_assert_eq!(taken_counts(&outcome), expected);
    }

    /// The stall report is the pump's stall watch input: it must name the
    /// earliest ring-full lane and flag any lane only elapsed time frees.
    #[test]
    fn a_stall_names_the_full_lane_and_the_holding_flag(pass in arb_pass()) {
        let staged = pass.stage();
        let Schedule::Stall { full, holding } = staged.run() else {
            return Ok(());
        };
        let expected_full = earliest(
            staged
                .queues
                .iter()
                .filter(|(key, queue)| {
                    lane_state(queue, staged.release[key]) == LaneState::NoRoom
                })
                .map(|(key, queue)| (*key, queue.spans[0].start_host)),
        );
        let expected_holding = staged
            .queues
            .iter()
            .any(|(key, queue)| lane_state(queue, staged.release[key]) == LaneState::Held);

        prop_assert_eq!(full, expected_full);
        prop_assert_eq!(holding, expected_holding);
    }

    /// A lane's released count answers to its own bounds and the head mcu,
    /// never to which sibling happens to sort first.
    #[test]
    fn relabelling_axes_inside_an_mcu_moves_the_same_views(
        (pass, relabel) in arb_relabelled_pass(),
    ) {
        let straight = pass.stage();
        let permuted = pass.stage_relabelled(|key| relabel[&key]);

        let moved: BTreeMap<AxisKey, usize> = taken_counts(&straight.run())
            .into_iter()
            .map(|(key, taken)| (relabel[&key], taken))
            .collect();
        prop_assert_eq!(moved, taken_counts(&permuted.run()));
    }

    /// Coalescing may only fuse abutting hold views, and the fused view must
    /// cover exactly the clock range of the pair it replaced.
    #[test]
    fn appending_only_coalesces_abutting_identical_holds(
        staged in prop::collection::vec(arb_merge_spec(), 0..=3),
        incoming in prop::collection::vec(arb_merge_spec(), 0..=8),
        allow_tail_merge in any::<bool>(),
        first_clock in 1u64..1_000_000,
    ) {
        let chain = build_chain(
            &staged.iter().copied().chain(incoming.iter().copied()).collect::<Vec<_>>(),
            first_clock,
        );
        let (staged_views, incoming_views) = chain.split_at(staged.len());
        let inputs = chain.clone();

        let mut queue: VecDeque<ClockedMotorSpan> = staged_views.iter().cloned().collect();
        append_spans_merging_holds(&mut queue, incoming_views.to_vec(), allow_tail_merge);
        let output: Vec<ClockedMotorSpan> = queue.into_iter().collect();

        prop_assert_eq!(
            covered_ticks(&output),
            covered_ticks(&inputs),
            "coalescing changed the covered clock range"
        );

        let mut absorbed = inputs.iter();
        for merged in &output {
            let first = absorbed.next().expect("every output view absorbs an input");
            prop_assert_eq!(merged.start_clock, first.start_clock);
            let mut fused = vec![first];
            while fused.last().expect("a run starts with one view").end_clock != merged.end_clock {
                let next = absorbed.next().expect("a fused run stays inside the input");
                prop_assert_eq!(
                    fused.last().expect("a run starts with one view").end_clock,
                    next.start_clock,
                    "a fused run must be contiguous in clock"
                );
                fused.push(next);
            }
            if fused.len() == 1 {
                continue;
            }
            prop_assert!(merged.signal.is_explicit_hold, "only holds may fuse");
            for part in &fused {
                prop_assert!(part.signal.is_explicit_hold, "a non-hold view was dropped");
                prop_assert_eq!(part.signal.motor_mask, merged.signal.motor_mask);
                prop_assert_eq!(part.clock_freq_hz, merged.clock_freq_hz);
                prop_assert_eq!(hold_position(part).to_bits(), hold_position(merged).to_bits());
            }
            prop_assert!(
                merged.stream_t_end - merged.stream_t_start <= MAX_SPAN_SECS,
                "a fused hold must stay inside the dispatchable view length"
            );
        }
        prop_assert!(absorbed.next().is_none(), "an input view went missing");
    }

    /// The reason coalescing exists: a stationary axis crossing many planner
    /// segments must reach the wire as one entry, not one per segment.
    #[test]
    fn an_abutting_hold_run_collapses_to_a_single_view(
        ticks in prop::collection::vec(1u64..3_000, 2..=8),
        first_clock in 1u64..1_000_000,
    ) {
        let specs: Vec<MergeSpec> = ticks
            .iter()
            .map(|&ticks| MergeSpec {
                ticks,
                gap_ticks: 0,
                position_slot: 1,
                mask_slot: 0,
                freq_slot: 0,
                explicit_hold: true,
            })
            .collect();
        let run = build_chain(&specs, first_clock);
        let total_ticks = covered_ticks(&run);
        prop_assert!(
            total_ticks as f64 / FREQ <= MAX_SPAN_SECS,
            "the generated run must fit one dispatchable view for the collapse to be required"
        );

        let mut queue = VecDeque::new();
        append_spans_merging_holds(&mut queue, run, true);

        prop_assert_eq!(queue.len(), 1, "an abutting hold run must ship as one view");
        let merged = &queue[0];
        prop_assert_eq!(merged.start_clock, first_clock);
        prop_assert_eq!(merged.end_clock - merged.start_clock, total_ticks);
    }
}
