use super::*;
use crate::kinematics::KinematicsModule;
use crate::mcu_config::{AXIS_X, AXIS_Y, KINEMATICS_COREXY};
use geometry::path::{Line, PathSegment, Segment};
use geometry::{Move, SourceRange, StraightPhase, VelocityLimits};
use trajectory::{MAX_SPAN_SECS, SurfaceMode};

const CLOCK_FREQ_HZ: f64 = 1_000_000.0;
const T0: f64 = 100.0;

fn linear_span(delta: [f64; 3], duration: f64) -> Arc<AnalyticMoveSpan> {
    let line = Line::try_new([0.0, 0.0, 0.0], delta).expect("a nonzero line");
    let length = line.length();
    let source = Move {
        segment: PathSegment::try_new(Segment::Line(line), vec![]).expect("a follower-free path"),
        feedrate_mm_s: 100.0,
        limits: VelocityLimits::try_new(100.0, 1_000.0, 0.1, f64::INFINITY).expect("limits"),
        source: SourceRange {
            start_line: 0,
            end_line: 0,
        },
    };
    Arc::new(
        AnalyticMoveSpan::try_new(
            source,
            Arc::from([StraightPhase {
                t0: 0.0,
                dt: duration,
                s0: 0.0,
                v0: length / duration,
                a0: 0.0,
                j: 0.0,
            }]),
            0.0,
            0.0,
            duration,
            Arc::from([0.0, 0.0, 0.0, 0.0]),
            SurfaceMode::None,
        )
        .expect("a constant-velocity line span"),
    )
}

fn analytic_seg(delta: [f64; 3], duration: f64, motor_mask: u8) -> ContinuousSegment {
    let span = linear_span(delta, duration);
    ContinuousSegment {
        axes: (0..3)
            .map(|axis| ContinuousAxis::Analytic {
                span: Arc::clone(&span),
                axis,
            })
            .collect(),
        followers: Arc::from([]),
        spatial_path: true,
        t_start: 0.0,
        t_end: duration,
        motor_mask,
        source_line: 0,
        rest_at_end: true,
    }
}

fn hold_seg(positions: [f64; 3], duration: f64) -> ContinuousSegment {
    ContinuousSegment {
        axes: positions
            .into_iter()
            .map(|position| ContinuousAxis::Hold {
                position,
                t_start: 0.0,
                t_end: duration,
            })
            .collect(),
        followers: Arc::from([]),
        spatial_path: false,
        t_start: 0.0,
        t_end: duration,
        motor_mask: 0,
        source_line: 0,
        rest_at_end: true,
    }
}

fn constant_spline(value: f64, duration: f64) -> ScalarNurbs {
    ScalarNurbs::try_new(1, vec![0.0, 0.0, duration, duration], vec![value, value])
        .expect("a degree-1 constant spline")
}

fn seg_x_move() -> ContinuousSegment {
    analytic_seg([10.0, 0.0, 0.0], 1.0, 0)
}

fn ctx_projected<P: Fn(u32, f64) -> f64>(
    epoch: crate::anchor::StreamEpoch,
    project_exact: P,
) -> EnqueueCtx<'static, P> {
    EnqueueCtx {
        epoch_freq: &|_| None,
        clock_freq_hz: &|_| CLOCK_FREQ_HZ,
        lane_is_phase: &|_| false,
        t0: T0,
        epoch,
        host_now: 0.0,
        lead_secs: crate::pump::MAX_LEAD_SECS,
        project_exact,
    }
}

fn ctx_with_epoch(
    epoch: crate::anchor::StreamEpoch,
) -> EnqueueCtx<'static, impl Fn(u32, f64) -> f64> {
    ctx_projected(epoch, |_mcu, host_secs| host_secs * CLOCK_FREQ_HZ)
}

fn test_ctx() -> EnqueueCtx<'static, impl Fn(u32, f64) -> f64> {
    ctx_with_epoch(crate::anchor::StreamEpoch::Reposition)
}

fn cartesian_cfg(mcu_id: u32, axes: Vec<usize>, ceiling: f64) -> Vec<McuAxisConfig> {
    let count = axes.len();
    vec![McuAxisConfig {
        ethercat: false,
        mcu_id,
        axes,
        kinematics: 1,
        max_motor_velocity: vec![ceiling; count],
        ..Default::default()
    }]
}

fn ec_cfg() -> Vec<McuAxisConfig> {
    vec![McuAxisConfig {
        ethercat: true,
        mcu_id: 9,
        axes: vec![AXIS_X, AXIS_Y],
        kinematics: 1,
        max_motor_velocity: vec![f64::INFINITY; 2],
        ..Default::default()
    }]
}

#[test]
fn cartesian_x_axis_yields_views_anchored_on_the_exact_projection() {
    let cfg = cartesian_cfg(7, vec![AXIS_X, AXIS_Y, 2], f64::INFINITY);
    let msgs = enqueue_segment(&seg_x_move(), &cfg, &test_ctx()).expect("enqueue must succeed");

    let x = msgs
        .iter()
        .find(|m| m.key == AxisKey { mcu_id: 7, axis: 0 })
        .expect("X axis EnqueueMsg must be present");

    let first = x.spans.first().expect("X must have at least one view");
    assert_eq!(
        first.start_clock,
        (T0 * CLOCK_FREQ_HZ) as u64,
        "the first view must start at project_exact(t0)"
    );
    assert!(
        x.spans
            .iter()
            .all(|v| v.end_clock > v.start_clock && v.stream_t_end > v.stream_t_start),
        "every view must span a positive duration and at least one clock"
    );
    assert!(
        msgs.iter().any(|m| m.key == AxisKey { mcu_id: 7, axis: 1 }),
        "Y axis must be emitted"
    );
    assert!(
        msgs.iter().any(|m| m.key == AxisKey { mcu_id: 7, axis: 2 }),
        "Z axis must be emitted"
    );
    assert!(
        msgs.last().expect("at least one msg").batch_end,
        "only the last message closes the batch"
    );
    assert_eq!(
        msgs.iter().filter(|m| m.batch_end).count(),
        1,
        "exactly one batch_end per dispatch"
    );
}

/// An explicitly held (or otherwise servo-stationary) lane must enqueue no
/// views for an ethercat lane: a parked drive receiving them trips the torque
/// gate's motion-while-parked fault, and an enabled drive already holds that
/// exact target.
#[test]
fn ethercat_explicit_hold_lane_is_skipped() {
    let seg = hold_seg([25.0, -7.5, 0.0], 1.2);
    let msgs = enqueue_segment(
        &seg,
        &ec_cfg(),
        &ctx_with_epoch(crate::anchor::StreamEpoch::Continuation),
    )
    .expect("enqueue must succeed");
    assert!(
        msgs.is_empty(),
        "explicitly held ethercat lanes must enqueue nothing on a continuation, got {} msgs",
        msgs.len()
    );

    let msgs = enqueue_segment(&seg, &ec_cfg(), &test_ctx()).expect("enqueue must succeed");
    assert_eq!(
        msgs.len(),
        2,
        "a Reposition must reach every ethercat lane so the pump forgets its junction baseline"
    );
    assert!(
        msgs.iter().all(|m| m.spans.is_empty()
            && m.epoch == crate::anchor::StreamEpoch::Reposition
            && m.key.mcu_id == 9),
        "held Reposition carriers must be view-free epoch markers"
    );
}

/// A lane whose analytic projection happens to be zero is not an explicit
/// hold: the endpoint still owns a trajectory it must evaluate on its own
/// clock, and hold merging may not rewrite its time domain.
#[test]
fn ethercat_analytic_lanes_always_stream_even_when_projection_is_zero() {
    let msgs = enqueue_segment(
        &seg_x_move(),
        &ec_cfg(),
        &ctx_with_epoch(crate::anchor::StreamEpoch::Continuation),
    )
    .expect("enqueue must succeed");
    for axis in [0u8, 1] {
        let msg = msgs
            .iter()
            .find(|m| m.key == AxisKey { mcu_id: 9, axis })
            .unwrap_or_else(|| panic!("analytic lane {axis} must stream to the ethercat slot"));
        assert!(!msg.spans.is_empty());
        assert!(
            !msg.spans[0].signal.is_explicit_hold,
            "an analytic lane is never classified as an explicit hold"
        );
    }
}

#[test]
fn serial_explicit_hold_lane_still_streams() {
    let mut cfg = ec_cfg();
    cfg[0].ethercat = false;
    let msgs = enqueue_segment(&hold_seg([25.0, -7.5, 0.0], 1.2), &cfg, &test_ctx())
        .expect("enqueue must succeed");
    let held = msgs
        .iter()
        .find(|m| m.key == AxisKey { mcu_id: 9, axis: 0 })
        .expect("serial stepper lanes keep their held views");
    assert!(!held.spans.is_empty());
    assert!(held.spans.iter().all(|v| v.signal.is_explicit_hold));
}

#[test]
fn corexy_motor_lanes_are_the_sum_and_difference_of_the_axes() {
    let cfg = vec![McuAxisConfig {
        ethercat: false,
        mcu_id: 1,
        axes: vec![AXIS_X, AXIS_Y],
        kinematics: KINEMATICS_COREXY,
        max_motor_velocity: vec![f64::INFINITY; 2],
        ..Default::default()
    }];
    let seg = analytic_seg([10.0, 4.0, 0.0], 1.0, 0);
    let msgs = enqueue_segment(&seg, &cfg, &test_ctx()).expect("enqueue must succeed");

    for (axis, expected) in [(0u8, 14.0_f64), (1, 6.0)] {
        let msg = msgs
            .iter()
            .find(|m| m.key == AxisKey { mcu_id: 1, axis })
            .unwrap_or_else(|| panic!("motor lane {axis} must be present"));
        let signal = &msg.spans.last().expect("a trailing view").signal;
        let end = signal.position(signal.t_end).expect("finite endpoint");
        assert!(
            (end - expected).abs() < 1e-9,
            "motor-{axis} endpoint expected {expected}, got {end}"
        );
    }
}

#[test]
fn corexy_lane_is_one_correlated_analytic_group() {
    let module = KinematicsModule::from_tag(KINEMATICS_COREXY).expect("corexy tag");
    let seg = analytic_seg([10.0, 4.0, 0.0], 1.0, 0);

    for (lane, expected) in [
        (AXIS_X, vec![(0usize, 1.0_f64), (1, 1.0)]),
        (AXIS_Y, vec![(0, 1.0), (1, -1.0)]),
    ] {
        let span = lane_span(&module, &seg, lane).expect("a valid motor span");
        assert_eq!(
            span.groups.len(),
            1,
            "both terms share one AnalyticMoveSpan, so they must coalesce into one group"
        );
        match &span.groups[0] {
            MotorGroup::Analytic { terms, .. } => assert_eq!(
                terms
                    .iter()
                    .map(|t| (t.source_axis, t.scale))
                    .collect::<Vec<_>>(),
                expected,
                "lane {lane} must carry the kinematic weights as correlated terms"
            ),
            other => panic!("lane {lane} must be one analytic group, got {other:?}"),
        }
    }
}

/// The correlation must survive into the bounds: a pure-Z move leaves both
/// CoreXY motors exactly stationary, and independent per-axis boxes would
/// instead bound them by the sum of two full spatial projections.
#[test]
fn corexy_pure_z_move_cancels_exactly_in_both_motor_lanes() {
    let module = KinematicsModule::from_tag(KINEMATICS_COREXY).expect("corexy tag");
    let seg = analytic_seg([0.0, 0.0, 5.0], 0.5, 0);

    for lane in [AXIS_X, AXIS_Y] {
        let span = lane_span(&module, &seg, lane).expect("a valid motor span");
        let bounds = span.pva_bounds(span.t_start, span.t_end).expect("bounds");
        assert_eq!(
            (bounds.velocity_min, bounds.velocity_max),
            (0.0, 0.0),
            "lane {lane} must cancel to exactly zero velocity"
        );
        assert_eq!(bounds.acceleration_abs_max, 0.0);
        assert!(
            !span.is_explicit_hold,
            "kinematic cancellation must not become hold-merge eligible"
        );
    }
}

#[test]
fn cartesian_lane_is_a_single_unweighted_term_of_its_own_axis() {
    let module = KinematicsModule::from_tag(1).expect("cartesian tag");
    let seg = analytic_seg([10.0, 4.0, 7.0], 1.0, 0);
    for lane in [AXIS_X, AXIS_Y, 2] {
        let span = lane_span(&module, &seg, lane).expect("a valid motor span");
        match &span.groups[..] {
            [MotorGroup::Analytic { terms, .. }] => {
                assert_eq!(terms.len(), 1);
                assert_eq!((terms[0].source_axis, terms[0].scale), (lane, 1.0));
                assert_eq!(terms[0].axis, seg.axes[lane]);
            }
            other => panic!("cartesian lane {lane} must pass its axis through, got {other:?}"),
        }
    }
}

#[test]
fn follower_lanes_never_pass_through_the_spatial_matrix() {
    let module = KinematicsModule::from_tag(KINEMATICS_COREXY).expect("corexy tag");
    let mut seg = analytic_seg([10.0, 4.0, 7.0], 1.0, 0);
    let mut axes = seg.axes.to_vec();
    axes.push(ContinuousAxis::Spline(Arc::new(constant_spline(2.0, 1.0))));
    seg.axes = axes.into();

    let span = lane_span(&module, &seg, 3).expect("a valid motor span");
    match &span.groups[..] {
        [
            MotorGroup::Spline {
                curve,
                summed_scale,
            },
        ] => {
            assert_eq!(**curve, constant_spline(2.0, 1.0));
            assert_eq!(*summed_scale, 1.0);
        }
        other => panic!("follower lane 3 must bypass the spatial matrix, got {other:?}"),
    }
}

#[test]
fn only_time_domain_holds_and_bit_identical_splines_are_explicit_holds() {
    let module = KinematicsModule::from_tag(1).expect("cartesian tag");
    let duration = 0.4;

    let held = hold_seg([5.0, 5.0, 5.0], duration);
    assert!(
        lane_span(&module, &held, AXIS_X)
            .expect("span")
            .is_explicit_hold,
        "a time-domain hold is an explicit hold"
    );

    let mut spline_seg = held.clone();
    spline_seg.axes = (0..3)
        .map(|_| ContinuousAxis::Spline(Arc::new(constant_spline(5.0, duration))))
        .collect();
    assert!(
        lane_span(&module, &spline_seg, AXIS_X)
            .expect("span")
            .is_explicit_hold,
        "a spline whose control points are bit-identical is an explicit hold"
    );

    let mut drifting = held.clone();
    drifting.axes = (0..3)
        .map(|_| {
            ContinuousAxis::Spline(Arc::new(
                ScalarNurbs::try_new(
                    1,
                    vec![0.0, 0.0, duration, duration],
                    vec![5.0, 5.000_000_001],
                )
                .expect("a degree-1 spline"),
            ))
        })
        .collect();
    assert!(
        !lane_span(&module, &drifting, AXIS_X)
            .expect("span")
            .is_explicit_hold,
        "a super-resolution constant step is real motion, not a hold"
    );

    let tiny = analytic_seg([1e-9, 0.0, 0.0], duration, 0);
    assert!(
        !lane_span(&module, &tiny, AXIS_X)
            .expect("span")
            .is_explicit_hold,
        "a numerically small analytic move must not be classified as a hold"
    );
}

#[test]
fn views_are_bounded_at_25ms_and_share_one_signal() {
    let cfg = cartesian_cfg(7, vec![AXIS_X], f64::INFINITY);
    let msgs = enqueue_segment(&analytic_seg([10.0, 0.0, 0.0], 0.2, 0), &cfg, &test_ctx())
        .expect("enqueue must succeed");
    let spans = &msgs
        .iter()
        .find(|m| m.key == AxisKey { mcu_id: 7, axis: 0 })
        .expect("X axis must be present")
        .spans;

    assert_eq!(spans.len(), 8, "0.2 s / 0.025 s = 8 views");
    let signal = Arc::clone(&spans[0].signal);
    for view in spans {
        assert!(Arc::ptr_eq(&view.signal, &signal), "views are zero-copy");
        assert!(view.stream_t_end - view.stream_t_start <= MAX_SPAN_SECS + 1e-12);
    }
    for pair in spans.windows(2) {
        assert_eq!(
            pair[0].end_clock, pair[1].start_clock,
            "abutting views must share their seam clock exactly"
        );
        assert_eq!(pair[0].stream_t_end, pair[1].stream_t_start);
    }
    assert_eq!(spans[0].stream_t_start, 0.0);
    assert!((spans[7].stream_t_end - 0.2).abs() < 1e-12);
}

#[test]
fn rounded_endpoints_come_from_the_exact_anchor_not_from_each_other() {
    let cfg = cartesian_cfg(7, vec![AXIS_X], f64::INFINITY);
    let ctx = ctx_projected(
        crate::anchor::StreamEpoch::Reposition,
        |_mcu, host_secs: f64| host_secs * CLOCK_FREQ_HZ + 0.5,
    );
    let msgs = enqueue_segment(&analytic_seg([10.0, 0.0, 0.0], 0.07, 0), &cfg, &ctx)
        .expect("enqueue must succeed");
    let spans = &msgs[0].spans;

    let base = (T0 * CLOCK_FREQ_HZ) + 0.5;
    for view in spans {
        assert_eq!(view.start_clock, view.start_clock_exact.round() as u64);
        let expected_end =
            view.start_clock_exact + (view.stream_t_end - view.stream_t_start) * view.clock_freq_hz;
        assert_eq!(view.end_clock, expected_end.round() as u64);
        assert!(
            (view.start_clock_exact - (base + view.stream_t_start * CLOCK_FREQ_HZ)).abs() < 1e-6,
            "every view's exact anchor stays on the dispatch's affine map"
        );
    }
}

#[test]
fn a_nonpositive_projected_clock_is_rejected() {
    let cfg = cartesian_cfg(7, vec![AXIS_X], f64::INFINITY);
    let ctx = ctx_projected(
        crate::anchor::StreamEpoch::Reposition,
        |_mcu, _host_secs| 0.0,
    );
    assert!(
        matches!(
            enqueue_segment(&seg_x_move(), &cfg, &ctx),
            Err(ContinuousError::InvalidSpan {
                reason: "projected start clock must be positive"
            })
        ),
        "a clock at or before zero is not a dispatchable anchor"
    );
}

#[test]
fn a_sub_clock_span_emits_no_device_view() {
    let cfg = cartesian_cfg(7, vec![AXIS_X], f64::INFINITY);
    let mut ctx = test_ctx();
    ctx.clock_freq_hz = &|_| 1.0;
    let messages = enqueue_segment(&analytic_seg([10.0, 0.0, 0.0], 0.02, 0), &cfg, &ctx).unwrap();
    assert!(messages.is_empty());
}

#[test]
fn enqueue_stamps_the_motor_mask_onto_every_view() {
    let cfg = cartesian_cfg(1, vec![2], f64::INFINITY);
    let msgs = enqueue_segment(
        &analytic_seg([0.0, 0.0, 10.0], 0.1, 0b0000_0010),
        &cfg,
        &test_ctx(),
    )
    .expect("enqueue must succeed");
    let spans: Vec<_> = msgs.iter().flat_map(|m| m.spans.iter()).collect();
    assert!(!spans.is_empty());
    assert!(spans.iter().all(|v| v.signal.motor_mask == 0b0000_0010));
}

#[test]
fn step_rate_within_ceiling_enqueues() {
    let cfg = cartesian_cfg(7, vec![AXIS_X, AXIS_Y, 2], 50.0);
    let msgs = enqueue_segment(&seg_x_move(), &cfg, &test_ctx()).expect("enqueue must succeed");
    assert!(!msgs.is_empty(), "10 mm/s is comfortably under 50 mm/s");
}

#[test]
#[should_panic(expected = "step rate exceeds MCU ceiling (-307)")]
fn step_rate_over_ceiling_fails_loud() {
    let cfg = cartesian_cfg(7, vec![AXIS_X, AXIS_Y, 2], 5.0);
    let _ = enqueue_segment(&seg_x_move(), &cfg, &test_ctx());
}

#[test]
fn step_rate_over_ceiling_is_ignored_on_a_phase_routed_lane() {
    // The Trident full-G28 crash of 2026-08-20: the pulse-path step-rate
    // ceiling (step-pulse cost) does not bound a lane executing on the
    // phase transport - coil writes carry no step pulses. The same
    // over-ceiling demand that aborts a pulse lane must enqueue cleanly
    // when the lane is phase-routed.
    let cfg = cartesian_cfg(7, vec![AXIS_X, AXIS_Y, 2], 5.0);
    let mut ctx = test_ctx();
    ctx.lane_is_phase = &|_| true;
    let msgs = enqueue_segment(&seg_x_move(), &cfg, &ctx).expect("enqueue must succeed");
    assert!(!msgs.is_empty());
}
