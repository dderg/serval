use std::sync::Arc;

use trajectory::{
    ClockedMotorSpan, ContinuousAxis, MotorGroup, MotorSpan, MotorTerm, NudgeProfile,
};

use super::compress::StepMove;
use super::{MotorConfig, ShimError, StepEncoder, StepFrame, StepShim};

const CYCLES_PER_SECOND: f64 = 1_000_000.0;
const MICROSTEP: f64 = 0.01;
const OID: u32 = 7;

/// A quarter of a microstep. Seeding a lane at this offset from its commanded
/// lattice puts every integer crossing half a cycle away from a clock edge, so
/// the roots the cursor solves are exact integers with room to spare — and a
/// half-step lattice would land them 50 cycles earlier.
const LATTICE_OFFSET: f64 = 0.00255;

fn cfg() -> MotorConfig {
    MotorConfig {
        oid: OID,
        microstep_distance: MICROSTEP,
        invert_dir: false,
        cycles_per_second: CYCLES_PER_SECOND,
        encoder: StepEncoder::Classic { max_error_ticks: 0 },
        min_rearm_cycles: 0,
    }
}

fn hp_cfg() -> MotorConfig {
    MotorConfig {
        encoder: StepEncoder::HighPrecision,
        ..cfg()
    }
}

fn seeded(cfg: MotorConfig, queue_depth: u32) -> StepShim {
    let mut shim = StepShim::new(vec![cfg], queue_depth);
    shim.reset_position(0, 0);
    shim
}

fn base_group(position: f64, t_start: f64, t_end: f64) -> MotorGroup {
    MotorGroup::Independent(MotorTerm {
        source_axis: 0,
        axis: ContinuousAxis::Hold {
            position,
            t_start,
            t_end,
        },
        scale: 1.0,
    })
}

fn ramp_group(delta_mm: f64, duration: f64, t_start: f64, scale: f64) -> MotorGroup {
    let profile = NudgeProfile::try_new(delta_mm, delta_mm.abs() / duration, 0.0, t_start)
        .expect("a constant-velocity nudge");
    MotorGroup::Independent(MotorTerm {
        source_axis: 0,
        axis: ContinuousAxis::Nudge(profile),
        scale,
    })
}

fn signal(groups: Vec<MotorGroup>, t_start: f64, t_end: f64, motor_mask: u8) -> Arc<MotorSpan> {
    Arc::new(
        MotorSpan::try_new(groups.into(), t_start, t_end, motor_mask, 0, false)
            .expect("a dispatchable motor span"),
    )
}

fn clocked(signal: Arc<MotorSpan>, start_clock: u64, freq: f64) -> ClockedMotorSpan {
    let (t_start, t_end) = (signal.t_start, signal.t_end);
    ClockedMotorSpan::try_new(
        signal,
        t_start,
        t_end,
        t_start,
        t_end,
        start_clock as f64,
        freq,
    )
    .expect("a representable clocked view")
}

fn window(start_clock: u64, cycles: u64) -> (f64, f64) {
    let t_start = start_clock as f64 / CYCLES_PER_SECOND;
    (t_start, t_start + cycles as f64 / CYCLES_PER_SECOND)
}

/// A constant-velocity view on the lane's own clock timeline, so consecutive
/// views abut exactly.
fn span(start_clock: u64, from_mm: f64, delta_mm: f64, cycles: u64) -> ClockedMotorSpan {
    let (t_start, t_end) = window(start_clock, cycles);
    let groups = vec![
        base_group(from_mm, t_start, t_end),
        ramp_group(delta_mm, t_end - t_start, t_start, 1.0),
    ];
    clocked(
        signal(groups, t_start, t_end, 0),
        start_clock,
        CYCLES_PER_SECOND,
    )
}

fn hold_span(start_clock: u64, position: f64, cycles: u64) -> ClockedMotorSpan {
    let (t_start, t_end) = window(start_clock, cycles);
    let groups = vec![base_group(position, t_start, t_end)];
    clocked(
        signal(groups, t_start, t_end, 0),
        start_clock,
        CYCLES_PER_SECOND,
    )
}

/// The CoreXY case a single motor cannot tell from a hold: two source axes
/// move, and this motor's two terms cancel to the cycle.
fn cancelling_span(
    start_clock: u64,
    position: f64,
    delta_mm: f64,
    cycles: u64,
) -> ClockedMotorSpan {
    let (t_start, t_end) = window(start_clock, cycles);
    let duration = t_end - t_start;
    let groups = vec![
        base_group(position, t_start, t_end),
        ramp_group(delta_mm, duration, t_start, 1.0),
        ramp_group(delta_mm, duration, t_start, -1.0),
    ];
    clocked(
        signal(groups, t_start, t_end, 0),
        start_clock,
        CYCLES_PER_SECOND,
    )
}

/// A motor-local overlay (`motor_mask != 0`): the cursor walks it on its own
/// lattice, seeded where the overlay signal starts rather than where the drain
/// happens to resume.
fn overlay_span(start_clock: u64, position: f64, delta_mm: f64, cycles: u64) -> ClockedMotorSpan {
    let (t_start, t_end) = window(start_clock, cycles);
    let groups = vec![
        base_group(position, t_start, t_end),
        ramp_group(delta_mm, t_end - t_start, t_start, 1.0),
    ];
    clocked(
        signal(groups, t_start, t_end, 1),
        start_clock,
        CYCLES_PER_SECOND,
    )
}

fn queue_step_count(frames: &[StepFrame]) -> u32 {
    frames
        .iter()
        .map(|f| match f {
            StepFrame::QueueStep { count, .. } | StepFrame::QueueStepHp { count, .. } => {
                u32::from(*count)
            }
            _ => 0,
        })
        .sum()
}

fn dir_frame_indices(frames: &[StepFrame]) -> Vec<usize> {
    frames
        .iter()
        .enumerate()
        .filter_map(|(index, f)| matches!(f, StepFrame::SetNextStepDir { .. }).then_some(index))
        .collect()
}

fn reset_clocks(frames: &[StepFrame]) -> Vec<u64> {
    frames
        .iter()
        .filter_map(|f| match *f {
            StepFrame::ResetStepClock { clock, .. } => Some(u64::from(clock)),
            _ => None,
        })
        .collect()
}

/// The step clocks the mcu will execute, walked the way its stepper walks
/// them: an anchor from `reset_step_clock`, then every interval accumulated.
/// A zero-error classic encoder reproduces the requested roots exactly, so
/// this is the wire's view of the lattice.
fn replayed_step_clocks(frames: &[StepFrame]) -> Vec<u64> {
    let mut cursor = 0_u64;
    let mut clocks = Vec::new();
    for frame in frames {
        match *frame {
            StepFrame::ResetStepClock { clock, .. } => cursor = u64::from(clock),
            StepFrame::SetNextStepDir { .. } => {}
            StepFrame::QueueStep {
                interval,
                count,
                add,
                ..
            } => {
                let mv = StepMove {
                    interval,
                    count,
                    add,
                };
                clocks.extend((1..=count).map(|nth| mv.step_clock(cursor, nth)));
                cursor = mv.last_clock(cursor);
            }
            StepFrame::QueueStepHp { .. } => {
                panic!("the classic clock replay cannot walk an hp frame")
            }
        }
    }
    clocks
}

/// The clocks the cursor solved, read off the commanded step counter one cycle
/// at a time. Independent of the encoder: the counter moves when a root is
/// resolved, not when a frame is packed.
fn solved_root_clocks(cfg: MotorConfig, spans: &[ClockedMotorSpan]) -> Vec<u64> {
    let mut shim = seeded(cfg, 16);
    shim.push_spans(0, spans).expect("a contiguous stream");
    let first = spans[0].start_clock;
    let last = spans[spans.len() - 1].end_clock;
    let mut clocks = Vec::new();
    let mut seen = 0_i64;
    for clock in first..=last {
        shim.drain(clock).expect("a paced drain");
        let now = shim.commanded_steps(0);
        while seen != now {
            clocks.push(clock);
            seen += if now > seen { 1 } else { -1 };
        }
    }
    clocks
}

#[test]
fn roots_land_on_the_exact_integer_step_lattice() {
    let start = 1_000;
    let view = span(start, LATTICE_OFFSET, 0.1, 1_000);
    let clocks = solved_root_clocks(cfg(), &[view]);

    let expected: Vec<u64> = (1..=10).map(|k| start + 100 * k - 25).collect();
    assert_eq!(
        clocks, expected,
        "every root must be the first clock whose position reaches \
         (commanded_step + direction) * microstep_distance"
    );
}

#[test]
fn a_half_microstep_of_travel_never_moves_the_commanded_counter() {
    let start = 1_000;
    let mut shim = seeded(cfg(), 8);
    shim.push_spans(0, &[span(start, LATTICE_OFFSET, 0.005, 500)])
        .unwrap();
    let frames = shim.drain(u64::MAX).unwrap();

    assert_eq!(
        shim.commanded_steps(0),
        0,
        "the lane travels to 0.00755 mm, short of the 0.01 mm lattice line"
    );
    assert!(frames.is_empty(), "{frames:?}");
}

#[test]
fn commanded_position_tracks_the_step_lattice() {
    let mut shim = seeded(cfg(), 8);
    shim.push_spans(0, &[span(1_000, LATTICE_OFFSET, 0.1, 1_000)])
        .unwrap();
    shim.drain(u64::MAX).unwrap();

    assert_eq!(shim.commanded_steps(0), 10);
    assert!(
        (shim.commanded_position(0) - 10.0 * MICROSTEP).abs() < 1e-12,
        "commanded position {} is off the lattice",
        shim.commanded_position(0)
    );
}

/// A view's last clock is shared with its successor's first. The earlier view
/// owns it: a root that lands exactly there must reach the wire when that view
/// is converted, whether or not a successor ever arrives.
#[test]
fn a_root_on_the_seam_clock_belongs_to_the_earlier_view() {
    let start = 1_000;
    let view = span(start, 0.0, 1.0, 10_000);
    assert_eq!(view.end_clock, start + 10_000);

    let mut shim = seeded(cfg(), 8);
    shim.push_spans(0, &[view.clone()]).unwrap();
    shim.drain(view.end_clock - 1).unwrap();
    assert_eq!(
        shim.commanded_steps(0),
        99,
        "the hundredth microstep is only reached on the seam clock itself"
    );
    assert_eq!(shim.queued_spans(), 1, "the view is not converted yet");

    shim.drain(view.end_clock).unwrap();
    assert_eq!(
        shim.commanded_steps(0),
        100,
        "the seam root must be emitted by the view that ends on it"
    );
    assert_eq!(shim.consumed_counts(), vec![1]);

    shim.push_spans(0, &[span(start + 10_000, 1.0, 1.0, 10_000)])
        .unwrap();
    shim.drain(u64::MAX).unwrap();
    assert_eq!(
        shim.commanded_steps(0),
        200,
        "the successor must neither replay nor lose the seam root"
    );
}

#[test]
fn first_emission_resets_the_step_clock_then_sets_dir() {
    let mut shim = seeded(cfg(), 8);
    shim.push_spans(0, &[span(1_000, LATTICE_OFFSET, 0.1, 1_000)])
        .unwrap();
    let frames = shim.drain(u64::MAX).unwrap();

    assert!(
        matches!(frames[0], StepFrame::ResetStepClock { oid: OID, .. }),
        "{:?}",
        frames[0]
    );
    assert_eq!(frames[1], StepFrame::SetNextStepDir { oid: OID, dir: 1 });
    assert!(matches!(frames[2], StepFrame::QueueStep { .. }));
    assert_eq!(reset_clocks(&frames).len(), 1);

    let clocks = replayed_step_clocks(&frames);
    assert_eq!(
        clocks,
        solved_root_clocks(cfg(), &[span(1_000, LATTICE_OFFSET, 0.1, 1_000)])
    );
    assert_eq!(reset_clocks(&frames)[0] + 1, clocks[0]);
}

#[test]
fn a_second_drain_does_not_reset_the_step_clock_again() {
    let mut shim = seeded(cfg(), 8);
    shim.push_spans(
        0,
        &[
            span(1_000, LATTICE_OFFSET, 0.1, 1_000),
            span(2_000, 0.1 + LATTICE_OFFSET, 0.1, 1_000),
        ],
    )
    .unwrap();

    let first = shim.drain(1_600).unwrap();
    let second = shim.drain(u64::MAX).unwrap();
    assert_eq!(reset_clocks(&first).len(), 1);
    assert!(reset_clocks(&second).is_empty());
    assert_eq!(queue_step_count(&first) + queue_step_count(&second), 20);
    assert_eq!(shim.commanded_steps(0), 20);
}

#[test]
fn direction_reversal_latches_dir_ahead_of_the_run_it_applies_to() {
    let mut shim = seeded(cfg(), 8);
    shim.push_spans(
        0,
        &[
            span(1_000, LATTICE_OFFSET, 0.1, 1_000),
            span(2_000, 0.1 + LATTICE_OFFSET, -0.11, 1_100),
        ],
    )
    .unwrap();
    let frames = shim.drain(u64::MAX).unwrap();

    let dirs: Vec<u8> = frames
        .iter()
        .filter_map(|f| match f {
            StepFrame::SetNextStepDir { dir, .. } => Some(*dir),
            _ => None,
        })
        .collect();
    assert_eq!(dirs, vec![1, 0]);
    for index in dir_frame_indices(&frames) {
        assert!(
            matches!(frames[index + 1], StepFrame::QueueStep { .. }),
            "a dir latch must head the run it turns around: {:?}",
            &frames[index..]
        );
    }

    assert_eq!(queue_step_count(&frames), 20);
    assert_eq!(
        shim.commanded_steps(0),
        0,
        "ten microsteps out and ten back leave the counter where it started"
    );
    let clocks = replayed_step_clocks(&frames);
    assert!(
        clocks.windows(2).all(|w| w[0] < w[1]),
        "a reversed stream must stay monotonic in time: {clocks:?}"
    );
}

#[test]
fn a_reversal_inside_one_view_reverses_the_lattice_walk() {
    let start = 1_000;
    let up = span(start, LATTICE_OFFSET, 0.1, 1_000);
    let down = span(start + 1_000, 0.1 + LATTICE_OFFSET, -0.11, 1_100);
    let clocks = solved_root_clocks(cfg(), &[up, down]);

    let mut expected: Vec<u64> = (1..=10).map(|k| start + 100 * k - 25).collect();
    expected.extend((0..10).map(|k| start + 1_000 + 100 * k + 126));
    assert_eq!(
        clocks, expected,
        "the falling walk must cross (commanded_step - 1) * microstep_distance"
    );
}

#[test]
fn a_hold_span_emits_no_roots_and_keeps_the_anchor() {
    let mut shim = seeded(cfg(), 8);
    shim.push_spans(
        0,
        &[
            span(1_000, LATTICE_OFFSET, 0.1, 1_000),
            hold_span(2_000, 0.1 + LATTICE_OFFSET, 2_000),
            span(4_000, 0.1 + LATTICE_OFFSET, 0.1, 1_000),
        ],
    )
    .unwrap();

    let mut frames = shim.drain(2_500).unwrap();
    assert_eq!(shim.commanded_steps(0), 10, "the hold must not step");
    frames.extend(shim.drain(u64::MAX).unwrap());

    assert_eq!(reset_clocks(&frames).len(), 1, "the hold is inside reach");
    assert_eq!(
        dir_frame_indices(&frames).len(),
        1,
        "the hold never reverses"
    );
    assert_eq!(queue_step_count(&frames), 20);
    assert_eq!(shim.commanded_steps(0), 20);
    assert_eq!(shim.consumed_counts(), vec![3]);
}

#[test]
fn a_cancelling_span_emits_no_roots() {
    let mut shim = seeded(cfg(), 8);
    shim.push_spans(
        0,
        &[
            span(1_000, LATTICE_OFFSET, 0.1, 1_000),
            cancelling_span(2_000, 0.1 + LATTICE_OFFSET, 0.5, 1_000),
        ],
    )
    .unwrap();
    let frames = shim.drain(u64::MAX).unwrap();

    assert_eq!(
        shim.commanded_steps(0),
        10,
        "two terms that cancel move this motor nowhere"
    );
    assert_eq!(queue_step_count(&frames), 10);
    assert_eq!(shim.consumed_counts(), vec![2]);
}

#[test]
fn a_collapsed_clock_range_fails_loud() {
    let mut view = span(1_000, LATTICE_OFFSET, 0.1, 1_000);
    view.end_clock = view.start_clock;
    let mut shim = seeded(cfg(), 8);

    match shim.push_spans(0, &[view]).unwrap_err() {
        ShimError::SpanClockDegenerate {
            motor,
            start_clock,
            end_clock,
        } => assert_eq!((motor, start_clock, end_clock), (0, 1_000, 1_000)),
        other => panic!("expected SpanClockDegenerate, got {other}"),
    }
}

#[test]
fn a_span_on_a_foreign_clock_slope_fails_loud() {
    let mut shim = seeded(cfg(), 8);
    shim.push_spans(0, &[span(1_000, LATTICE_OFFSET, 0.1, 1_000)])
        .unwrap();
    shim.set_motor_cycles_per_second(0, CYCLES_PER_SECOND * 1.004);

    match shim.drain(u64::MAX).unwrap_err() {
        ShimError::SpanFrequencyMismatch {
            motor,
            expected,
            got,
        } => {
            assert_eq!(motor, 0);
            assert_eq!(got, CYCLES_PER_SECOND);
            assert_eq!(expected, CYCLES_PER_SECOND * 1.004);
        }
        other => panic!("expected SpanFrequencyMismatch, got {other}"),
    }
}

#[test]
fn changing_the_clock_slope_keeps_the_commanded_count() {
    let mut shim = seeded(cfg(), 8);
    shim.push_spans(0, &[span(1_000, LATTICE_OFFSET, 0.1, 1_000)])
        .unwrap();
    shim.drain(u64::MAX).unwrap();
    assert_eq!(shim.commanded_steps(0), 10);

    shim.set_motor_cycles_per_second(0, CYCLES_PER_SECOND * 1.004);
    assert_eq!(shim.commanded_steps(0), 10);
    assert_eq!(shim.motor_cycles_per_second(0), CYCLES_PER_SECOND * 1.004);
    assert!((shim.commanded_position(0) - 0.1).abs() < 1e-12);
}

#[test]
fn a_queue_beyond_its_depth_fails_loud() {
    let mut shim = seeded(cfg(), 2);
    let spans = [
        span(1_000, LATTICE_OFFSET, 0.1, 1_000),
        span(2_000, 0.1 + LATTICE_OFFSET, 0.1, 1_000),
        span(3_000, 0.2 + LATTICE_OFFSET, 0.1, 1_000),
    ];

    assert!(matches!(
        shim.push_spans(0, &spans).unwrap_err(),
        ShimError::QueueFull { motor: 0 }
    ));
    assert_eq!(shim.queue_depth(), 2);
}

#[test]
fn a_seam_gap_fails_loud_until_it_is_sanctioned() {
    let mut shim = seeded(cfg(), 8);
    shim.push_spans(0, &[span(1_000, LATTICE_OFFSET, 0.1, 1_000)])
        .unwrap();

    let ahead = span(2_500, 0.1 + LATTICE_OFFSET, 0.1, 1_000);
    match shim.push_spans(0, &[ahead.clone()]).unwrap_err() {
        ShimError::SpanGap {
            motor,
            expected,
            got,
            ..
        } => assert_eq!((motor, expected, got), (0, 2_000, 2_500)),
        other => panic!("expected SpanGap, got {other}"),
    }

    assert!(matches!(
        shim.accept_forward_seam_gap(0, 1_500).unwrap_err(),
        ShimError::SpanGap { .. }
    ));
    shim.accept_forward_seam_gap(0, 2_500)
        .expect("a forward dwell is sanctionable");
    shim.push_spans(0, &[ahead]).unwrap();
    shim.drain(u64::MAX).unwrap();
    assert_eq!(shim.commanded_steps(0), 20);
}

#[test]
fn a_seam_within_the_clock_map_rounding_is_accepted() {
    for offset in [-2_i64, -1, 0, 1, 2] {
        let mut shim = seeded(cfg(), 8);
        shim.push_spans(0, &[span(1_000, LATTICE_OFFSET, 0.1, 1_000)])
            .unwrap();
        let start = (2_000_i64 + offset) as u64;
        shim.push_spans(0, &[span(start, 0.1 + LATTICE_OFFSET, 0.1, 1_000)])
            .unwrap_or_else(|e| panic!("seam skew of {offset} cycles must be tolerated: {e}"));
    }
}

#[test]
fn detaching_the_seam_requires_a_drained_queue() {
    let mut shim = seeded(cfg(), 8);
    shim.push_spans(0, &[span(1_000, LATTICE_OFFSET, 0.1, 1_000)])
        .unwrap();
    assert!(matches!(
        shim.detach_span_seam(0).unwrap_err(),
        ShimError::QueueFull { motor: 0 }
    ));

    shim.drain(u64::MAX).unwrap();
    shim.detach_span_seam(0).expect("a drained queue detaches");
    shim.push_spans(0, &[span(50_000, 0.1 + LATTICE_OFFSET, 0.1, 1_000)])
        .expect("a detached seam accepts any start clock");
}

#[test]
fn halt_reports_executed_steps_and_frees_queue_credit() {
    let mut shim = seeded(cfg(), 8);
    shim.push_spans(
        0,
        &[
            span(1_000, LATTICE_OFFSET, 0.1, 1_000),
            span(2_000, 0.1 + LATTICE_OFFSET, 0.1, 1_000),
        ],
    )
    .unwrap();
    shim.drain(u64::MAX).unwrap();

    let (executed, tail) = shim.halt_at(0, u64::MAX).unwrap();
    assert_eq!(executed, 20);
    assert!(
        tail.is_empty(),
        "every root was already on the wire: {tail:?}"
    );
    assert_eq!(shim.consumed_counts(), vec![2]);
    assert_eq!(shim.pending_roots(), 0);
}

#[test]
fn halt_with_executed_count_uses_the_external_seed() {
    let mut shim = seeded(cfg(), 8);
    shim.push_spans(0, &[span(1_000, LATTICE_OFFSET, 0.1, 1_000)])
        .unwrap();
    shim.drain(u64::MAX).unwrap();

    let expected = shim.expected_halt_count(0, u64::MAX);
    assert_eq!(expected, 10);
    let (derived, _) = shim
        .halt_at_with_executed(0, 3_000, 37)
        .expect("an external count can reseed a drained lane");
    assert_eq!(derived, expected);
    assert_eq!(shim.commanded_steps(0), 37);

    shim.push_spans(0, &[span(5_000, 0.37 + LATTICE_OFFSET, 0.1, 1_000)])
        .unwrap();
    shim.drain(u64::MAX).unwrap();
    assert_eq!(shim.halt_at(0, u64::MAX).unwrap().0, 47);
}

#[test]
fn halt_discards_queued_work_and_re_anchors_the_resumed_lane() {
    let mut shim = seeded(cfg(), 8);
    shim.push_spans(
        0,
        &[
            span(1_000, LATTICE_OFFSET, 0.1, 1_000),
            span(2_000, 0.1 + LATTICE_OFFSET, 0.1, 1_000),
        ],
    )
    .unwrap();
    shim.drain(1_500).unwrap();

    let (executed, _) = shim.halt_at(0, 1_500).unwrap();
    assert!(executed > 0 && executed < 20, "cut mid-stream: {executed}");
    assert_eq!(shim.queued_spans(), 0, "a cut abandons the queue");
    assert_eq!(shim.consumed_counts(), vec![2]);

    shim.reset_position(0, 200);
    shim.push_spans(0, &[span(50_000, 2.0 + LATTICE_OFFSET, 0.1, 1_000)])
        .unwrap();
    let frames = shim.drain(u64::MAX).unwrap();
    let resets = reset_clocks(&frames);
    assert_eq!(resets.len(), 1);
    let clocks = replayed_step_clocks(&frames);
    assert_eq!(resets[0] + 1, clocks[0]);
    assert!(
        resets[0] >= 50_000,
        "reset {} predates the view the lane resumed on",
        resets[0]
    );
    assert_eq!(shim.commanded_steps(0), 210);
}

#[test]
fn a_cut_never_replays_steps_before_the_cut_clock() {
    let cut_at = 1_500;
    let mut shim = seeded(cfg(), 8);
    shim.push_spans(0, &[span(1_000, LATTICE_OFFSET, 0.1, 1_000)])
        .unwrap();
    let before = replayed_step_clocks(&shim.drain(cut_at).unwrap());
    assert!(!before.is_empty(), "the first drain must emit steps");

    shim.halt_at(0, cut_at).unwrap();
    shim.push_spans(0, &[span(1_000, LATTICE_OFFSET, 0.1, 1_000)])
        .unwrap();
    let after = replayed_step_clocks(&shim.drain(u64::MAX).unwrap());
    assert!(
        after.iter().all(|clock| *clock > cut_at),
        "a step at or before the cut was replayed: {after:?}"
    );
}

#[test]
fn reset_position_reseeds_the_step_counter() {
    let mut shim = StepShim::new(vec![cfg()], 8);
    shim.reset_position(0, -400);
    shim.push_spans(0, &[span(1_000, -4.0 + LATTICE_OFFSET, 0.1, 1_000)])
        .unwrap();
    let frames = shim.drain(u64::MAX).unwrap();

    assert_eq!(queue_step_count(&frames), 10);
    assert_eq!(shim.commanded_steps(0), -390);
    assert!((shim.commanded_position(0) + 3.9).abs() < 1e-12);
}

#[test]
fn an_inverted_lane_flips_the_dir_bit_only() {
    let mut inverted = cfg();
    inverted.invert_dir = true;
    let mut shim = seeded(inverted, 8);
    shim.push_spans(0, &[span(1_000, LATTICE_OFFSET, 0.1, 1_000)])
        .unwrap();
    let frames = shim.drain(u64::MAX).unwrap();

    assert!(shim.invert_dir(0));
    assert_eq!(frames[1], StepFrame::SetNextStepDir { oid: OID, dir: 0 });
    assert_eq!(shim.commanded_steps(0), 10);
    assert_eq!(
        replayed_step_clocks(&frames),
        solved_root_clocks(cfg(), &[span(1_000, LATTICE_OFFSET, 0.1, 1_000)]),
        "inverting the driver must not move a root"
    );
}

/// Root solving happens before encoding, so both encoders are handed exactly
/// the same clocks. Only the packing differs.
#[test]
fn classic_and_hp_request_the_same_step_clocks() {
    let spans = [
        span(1_000, LATTICE_OFFSET, 0.1, 1_000),
        span(2_000, 0.1 + LATTICE_OFFSET, -0.11, 1_100),
    ];
    let classic = solved_root_clocks(cfg(), &spans);
    let hp = solved_root_clocks(hp_cfg(), &spans);
    assert_eq!(classic, hp);
    assert_eq!(classic.len(), 20);

    let mut classic_shim = seeded(cfg(), 8);
    classic_shim.push_spans(0, &spans).unwrap();
    let classic_frames = classic_shim.drain(u64::MAX).unwrap();
    assert_eq!(
        replayed_step_clocks(&classic_frames),
        classic,
        "a zero-error classic encoder must reproduce the roots exactly"
    );

    let mut hp_shim = seeded(hp_cfg(), 8);
    hp_shim.push_spans(0, &spans).unwrap();
    let hp_frames = hp_shim.drain(u64::MAX).unwrap();
    assert!(
        hp_frames
            .iter()
            .any(|f| matches!(f, StepFrame::QueueStepHp { .. }))
            && !hp_frames
                .iter()
                .any(|f| matches!(f, StepFrame::QueueStep { .. })),
        "an hp lane packs hp moves only: {hp_frames:?}"
    );
    assert_eq!(queue_step_count(&hp_frames), classic.len() as u32);
    assert_eq!(hp_shim.commanded_steps(0), classic_shim.commanded_steps(0));
    assert_eq!(hp_shim.motor_encoder(0), StepEncoder::HighPrecision);
}

#[test]
fn finish_drains_every_pending_root() {
    let mut shim = seeded(cfg(), 8);
    shim.push_spans(0, &[span(1_000, LATTICE_OFFSET, 0.1, 1_000)])
        .unwrap();
    shim.drain(u64::MAX).unwrap();
    assert_eq!(shim.pending_roots(), 0);
    assert!(shim.finish(0).unwrap().is_empty());
    assert_eq!(shim.emitted_clock(0), 1_975);
    assert_eq!(shim.motor_microstep_distance(0), MICROSTEP);
}

/// A new overlay is entered one clock after its signal starts: the seam clock
/// belongs to the previous view, so the drain resumes at `start_clock + 1`.
/// Seeding the overlay lattice there would swallow the first clock of travel —
/// at 0.985 microsteps per clock that is a whole crossing, and every remaining
/// crossing slides a clock late.
#[test]
fn an_overlay_keeps_the_travel_of_its_first_clock() {
    let overlay_start = 2_000;
    let cycles = 100;
    let mut shim = seeded(cfg(), 8);
    shim.push_spans(
        0,
        &[
            hold_span(1_000, 0.0, 1_000),
            overlay_span(overlay_start, 0.0, 0.985, cycles),
        ],
    )
    .unwrap();
    let frames = shim.drain(u64::MAX).unwrap();

    let expected: Vec<u64> = (1..=98)
        .map(|k| overlay_start + (200 * k + 196) / 197)
        .collect();
    assert_eq!(
        replayed_step_clocks(&frames),
        expected,
        "every microstep the overlay travels is a crossing, counted from the \
         overlay's own start clock"
    );
    assert_eq!(
        shim.commanded_steps(0),
        0,
        "an overlay walks its own lattice and never moves the commanded lane"
    );
}
