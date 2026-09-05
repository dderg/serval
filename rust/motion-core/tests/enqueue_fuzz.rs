//! `enqueue_segment` over the axis kinds the lowerer and shaper actually put
//! into a `ContinuousSegment`: one shared `AnalyticMoveSpan` across the
//! spatial axes (`motion-pipeline/src/lower_stage.rs:150`), holds, and the
//! spline forms the shaper replaces them with. `ContinuousAxis::Buzz` and
//! `::Nudge` are deliberately absent — they never reach this entry point, the
//! sinks build them straight into a `MotorGroup` (`worker/pump_sink.rs:527`,
//! `pump/stepcompress_sink.rs:2162`).

use std::sync::Arc;

use geometry::path::{Line, PathSegment, Segment};
use geometry::{LawSegment, Move, ScalarLaw, SourceRange, VelocityLimits};
use motion_core::anchor::StreamEpoch;
use motion_core::enqueue::{EnqueueCtx, enqueue_segment};
use motion_core::mcu_config::McuAxisConfig;
use motion_core::pump::{EnqueueMsg, MAX_LEAD_SECS};
use motion_core::types::AxisKey;
use nurbs::ScalarNurbs;
use proptest::prelude::*;
use proptest::test_runner::FileFailurePersistence;
use runtime::segment::KinematicTag;
use step_shim::ring::SEAM_ROUNDING_CYCLES;
use trajectory::{
    AnalyticMoveSpan, ClockedMotorSpan, ContinuousAxis, ContinuousError, ContinuousSegment,
    MAX_SPAN_SECS, MotorSpan, Pva, PvaBounds, RelativeSplinePiece, SurfaceMode,
};

const MAX_DEGREE: usize = 4;
const MAX_AXES: usize = 4;
const SPATIAL_AXES: usize = 3;
const SAMPLES_PER_WINDOW: usize = 8;
const RELATIVE_SLACK: f64 = 1e-9;

#[derive(Debug, Clone)]
struct SplineShape {
    degree: usize,
    interior: Vec<(f64, usize)>,
    offsets: Vec<f64>,
    base_mm: f64,
    amplitude_mm: f64,
}

impl SplineShape {
    fn curve(&self, t_start: f64, t_end: f64) -> Arc<ScalarNurbs> {
        let order = self.degree + 1;
        let mut knots = vec![t_start; order];
        for &(fraction, multiplicity) in &self.interior {
            let knot = t_start + fraction * (t_end - t_start);
            knots.extend(std::iter::repeat_n(knot, multiplicity));
        }
        knots.extend(std::iter::repeat_n(t_end, order));
        let control_points = self
            .offsets
            .iter()
            .take(knots.len() - order)
            .map(|u| self.base_mm + self.amplitude_mm * u)
            .collect::<Vec<f64>>();
        Arc::new(
            ScalarNurbs::try_new(self.degree as u8, knots, control_points)
                .expect("a clamped spline"),
        )
    }
}

fn arb_spline_shape() -> impl Strategy<Value = SplineShape> {
    (1..=MAX_DEGREE, 0usize..4).prop_flat_map(|(degree, joints)| {
        let interior =
            prop::collection::vec((0.05f64..0.95, 1..=degree), joints).prop_map(|mut joints| {
                joints.sort_by(|a, b| a.0.total_cmp(&b.0));
                joints.dedup_by(|a, b| (a.0 - b.0).abs() < 0.02);
                joints
            });
        let control_points = degree + 1 + joints * degree;
        (
            interior,
            prop::collection::vec(-1.0..=1.0, control_points),
            -350.0..350.0,
            prop_oneof![Just(0.0), 1e-9..1e-3, 1e-3..1.0, 1.0..50.0],
        )
            .prop_map(
                move |(interior, offsets, base_mm, amplitude_mm)| SplineShape {
                    degree,
                    interior,
                    offsets,
                    base_mm,
                    amplitude_mm,
                },
            )
    })
}

#[derive(Debug, Clone)]
enum AxisShape {
    Spline(SplineShape),
    RelativeSpline { base_mm: f64, shape: SplineShape },
    Piecewise(Vec<(f64, SplineShape)>),
    Hold(f64),
}

impl AxisShape {
    fn axis(&self, t_start: f64, t_end: f64) -> ContinuousAxis {
        let duration = t_end - t_start;
        match self {
            Self::Spline(shape) => ContinuousAxis::Spline(shape.curve(t_start, t_end)),
            Self::RelativeSpline { base_mm, shape } => ContinuousAxis::RelativeSpline {
                base_position: *base_mm,
                curve: shape.curve(t_start, t_end),
            },
            Self::Piecewise(pieces) => {
                let count = pieces.len() as f64;
                ContinuousAxis::PiecewiseRelativeSpline(Arc::from(
                    pieces
                        .iter()
                        .enumerate()
                        .map(|(index, (base_mm, shape))| {
                            let piece_start = t_start + duration * index as f64 / count;
                            let piece_end = t_start + duration * (index + 1) as f64 / count;
                            RelativeSplinePiece {
                                base_position: *base_mm,
                                curve: shape.curve(piece_start, piece_end),
                                t_start: piece_start,
                                t_end: piece_end,
                            }
                        })
                        .collect::<Vec<_>>(),
                ))
            }
            Self::Hold(position) => ContinuousAxis::Hold {
                position: *position,
                t_start,
                t_end,
            },
        }
    }
}

fn arb_axis_shape() -> impl Strategy<Value = AxisShape> {
    prop_oneof![
        arb_spline_shape().prop_map(AxisShape::Spline),
        (-350.0..350.0, arb_spline_shape())
            .prop_map(|(base_mm, shape)| AxisShape::RelativeSpline { base_mm, shape }),
        prop::collection::vec((-350.0..350.0, arb_spline_shape()), 1..=3)
            .prop_map(AxisShape::Piecewise),
        (-350.0..350.0).prop_map(AxisShape::Hold),
    ]
}

/// One planned move as the lowerer hands it over: a straight path whose
/// tangential velocity is a chain of constant-acceleration phases, shared by
/// every spatial axis of the segment.
#[derive(Debug, Clone)]
struct AnalyticShape {
    direction: [f64; SPATIAL_AXES],
    velocity_knots: Vec<f64>,
    phase_weights: Vec<f64>,
    axis_starts: Vec<f64>,
}

impl AnalyticShape {
    fn span(&self, t_start: f64, t_end: f64) -> Arc<AnalyticMoveSpan> {
        let duration = t_end - t_start;
        let total_weight: f64 = self.phase_weights.iter().sum();
        let mut phases: Vec<LawSegment> = Vec::with_capacity(self.phase_weights.len());
        let mut local_t = 0.0;
        let mut distance = 0.0;
        for (index, weight) in self.phase_weights.iter().enumerate() {
            let dt = duration * weight / total_weight;
            let v0 = self.velocity_knots[index];
            let v1 = self.velocity_knots[index + 1];
            let segment = LawSegment::new(
                local_t,
                dt,
                distance,
                v0,
                ScalarLaw::ConstAccel { a0: (v1 - v0) / dt },
            );
            (distance, _, _) = segment.end_state();
            local_t = segment.end_time();
            phases.push(segment);
        }
        let norm = self
            .direction
            .iter()
            .map(|d| d * d)
            .sum::<f64>()
            .sqrt()
            .max(f64::MIN_POSITIVE);
        let delta = self.direction.map(|d| d / norm * distance);
        let line = Line::try_new([0.0, 0.0, 0.0], delta).expect("a nonzero line");
        let source = Move {
            segment: PathSegment::try_new(Segment::Line(line), vec![])
                .expect("a follower-free path"),
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
                Arc::from(phases),
                0.0,
                t_start,
                t_end,
                Arc::from(self.axis_starts.clone()),
                SurfaceMode::None,
            )
            .expect("a phase chain that tiles its own time and distance"),
        )
    }
}

fn arb_analytic_shape() -> impl Strategy<Value = AnalyticShape> {
    (1usize..=3)
        .prop_flat_map(|phases| {
            (
                prop::collection::vec(-1.0f64..1.0, SPATIAL_AXES)
                    .prop_filter("a nonzero direction", |d| d.iter().any(|v| v.abs() > 1e-3)),
                prop::collection::vec(0.0f64..300.0, phases + 1)
                    .prop_filter("a nonzero displacement", |v| v.iter().any(|v| *v > 1.0)),
                prop::collection::vec(0.1f64..1.0, phases),
                prop::collection::vec(-350.0f64..350.0, MAX_AXES),
            )
        })
        .prop_map(
            |(direction, velocity_knots, phase_weights, axis_starts)| AnalyticShape {
                direction: [direction[0], direction[1], direction[2]],
                velocity_knots,
                phase_weights,
                axis_starts,
            },
        )
}

#[derive(Debug, Clone)]
struct Scenario {
    t_start: f64,
    duration: f64,
    analytic: Option<AnalyticShape>,
    shapes: Vec<AxisShape>,
    corexy: bool,
    lane_lists: Vec<Vec<usize>>,
    t0: f64,
    clock_freq_hz: f64,
    clock_phase: f64,
    ethercat: bool,
    sample_fractions: Vec<f64>,
}

impl Scenario {
    fn t_end(&self) -> f64 {
        self.t_start + self.duration
    }

    fn kinematics(&self) -> u8 {
        if self.corexy && self.shapes.len() >= 2 {
            KinematicTag::CoreXy as u8
        } else {
            KinematicTag::Cartesian as u8
        }
    }

    fn segment(&self) -> ContinuousSegment {
        let (t_start, t_end) = (self.t_start, self.t_end());
        let analytic = self
            .analytic
            .as_ref()
            .map(|shape| shape.span(t_start, t_end));
        let axes = self
            .shapes
            .iter()
            .enumerate()
            .map(|(axis, shape)| match &analytic {
                Some(span) if axis < SPATIAL_AXES => ContinuousAxis::Analytic {
                    span: Arc::clone(span),
                    axis,
                },
                _ => shape.axis(t_start, t_end),
            })
            .collect();
        ContinuousSegment {
            axes,
            followers: Arc::from([]),
            spatial_path: analytic.is_some(),
            t_start,
            t_end,
            motor_mask: 0,
            source_line: 7,
            rest_at_end: true,
        }
    }

    fn configs(&self, ceilings: &[Vec<f64>]) -> Vec<McuAxisConfig> {
        self.lane_lists
            .iter()
            .zip(ceilings)
            .enumerate()
            .map(|(index, (axes, max_motor_velocity))| McuAxisConfig {
                ethercat: self.ethercat,
                mcu_id: index as u32,
                axes: axes.clone(),
                kinematics: self.kinematics(),
                max_motor_velocity: max_motor_velocity.clone(),
                ..Default::default()
            })
            .collect()
    }

    fn unbounded_ceilings(&self) -> Vec<Vec<f64>> {
        self.lane_lists
            .iter()
            .map(|axes| vec![f64::INFINITY; axes.len()])
            .collect()
    }

    fn enqueue(&self, ceilings: &[Vec<f64>]) -> Result<Vec<EnqueueMsg>, ContinuousError> {
        let freq = self.clock_freq_hz;
        let phase = self.clock_phase;
        let clock_freq_hz = move |_: u32| freq;
        let epoch_freq = move |_: u32| None;
        let lane_is_phase = move |_: AxisKey| false;
        let ctx = EnqueueCtx {
            t0: self.t0,
            epoch: StreamEpoch::Reposition,
            host_now: 0.0,
            lead_secs: MAX_LEAD_SECS,
            project_exact: move |_mcu: u32, host_secs: f64| host_secs * freq + phase,
            clock_freq_hz: &clock_freq_hz,
            epoch_freq: &epoch_freq,
            lane_is_phase: &lane_is_phase,
        };
        enqueue_segment(&self.segment(), &self.configs(ceilings), &ctx)
    }

    /// Both ends of the interval, both one-ulp-inward neighbours, the drawn
    /// interior fractions, and every breakpoint the interval straddles.
    fn closed_samples(&self, signal: &MotorSpan, t0: f64, t1: f64) -> Vec<f64> {
        let mut times = vec![t0, next_toward(t0, t1), next_toward(t1, t0), t1];
        times.extend(
            self.sample_fractions
                .iter()
                .map(|fraction| t0 + fraction * (t1 - t0)),
        );
        for &knot in signal.breakpoints.iter() {
            if knot > t0 && knot < t1 {
                times.extend([knot, next_toward(knot, t0), next_toward(knot, t1)]);
            }
        }
        times.retain(|t| *t >= t0 && *t <= t1);
        times
    }
}

fn next_toward(value: f64, target: f64) -> f64 {
    if target > value {
        f64::from_bits(value.to_bits() + 1)
    } else if target < value {
        f64::from_bits(value.to_bits() - 1)
    } else {
        value
    }
}

/// The windows whose closed interval carries `t`: the one it was drawn from,
/// plus the neighbour it shares the breakpoint with when it landed on one.
fn carrying_windows(breakpoints: &[f64], t: f64, drawn_from: usize) -> Vec<usize> {
    let mut carriers = vec![drawn_from];
    if t == breakpoints[drawn_from] && drawn_from > 0 {
        carriers.push(drawn_from - 1);
    }
    if t == breakpoints[drawn_from + 1] && drawn_from + 2 < breakpoints.len() {
        carriers.push(drawn_from + 1);
    }
    carriers
}

fn carries(bounds: &PvaBounds, pva: &Pva, slack: f64) -> bool {
    pva.velocity >= bounds.velocity_min - slack
        && pva.velocity <= bounds.velocity_max + slack
        && pva.acceleration.abs() <= bounds.acceleration_abs_max + slack
}

fn demand(bounds: &PvaBounds) -> f64 {
    bounds.velocity_min.abs().max(bounds.velocity_max.abs())
}

/// A non-empty ordered subset of the segment's axes, so a lane's ceiling can
/// only be found by its position in `axes`, never by the axis index.
fn arb_lane_list(axis_count: usize) -> impl Strategy<Value = Vec<usize>> {
    prop::collection::vec(0usize..MAX_AXES, 1..=MAX_AXES).prop_map(move |picks| {
        let mut lanes: Vec<usize> = Vec::with_capacity(picks.len());
        for pick in picks {
            let lane = pick % axis_count;
            if !lanes.contains(&lane) {
                lanes.push(lane);
            }
        }
        lanes
    })
}

fn arb_scenario() -> impl Strategy<Value = Scenario> {
    prop::collection::vec(arb_axis_shape(), 1..=MAX_AXES).prop_flat_map(|shapes| {
        let axis_count = shapes.len();
        (
            Just(shapes),
            prop_oneof![Just(0.0), 1e-3..1.0, 1.0..40.0],
            prop_oneof![1e-4..1e-2, 1e-2..0.2, 0.2..1.0],
            prop::option::of(arb_analytic_shape()),
            any::<bool>(),
            prop::collection::vec(arb_lane_list(axis_count), 1..=2),
            prop_oneof![Just(0.5), Just(100.0), Just(3600.0)],
            prop_oneof![
                Just(1.0e6),
                Just(64.0e6),
                Just(168.0e6),
                Just(400.0e6),
                Just(520.0e6)
            ],
            -0.5f64..0.5,
            prop_oneof![9 => Just(false), 1 => Just(true)],
            prop::collection::vec(0.0..=1.0, SAMPLES_PER_WINDOW),
        )
            .prop_map(
                |(
                    shapes,
                    t_start,
                    duration,
                    analytic,
                    corexy,
                    lane_lists,
                    t0,
                    clock_freq_hz,
                    clock_phase,
                    ethercat,
                    sample_fractions,
                )| Scenario {
                    t_start,
                    duration,
                    analytic,
                    shapes,
                    corexy,
                    lane_lists,
                    t0,
                    clock_freq_hz,
                    clock_phase,
                    ethercat,
                    sample_fractions,
                },
            )
    })
}

/// The step-rate demand `check_step_rate_ceiling` reads off one lane: the
/// widest velocity magnitude its per-breakpoint bounds report.
fn window_demand(signal: &MotorSpan) -> f64 {
    signal
        .breakpoints
        .windows(2)
        .map(|window| {
            demand(
                &signal
                    .pva_bounds(window[0], window[1])
                    .expect("a breakpoint window lies inside its own span"),
            )
        })
        .fold(0.0_f64, f64::max)
}

fn lane_signals(messages: &[EnqueueMsg]) -> Vec<(u32, u8, Arc<MotorSpan>)> {
    messages
        .iter()
        .filter_map(|msg| {
            msg.spans
                .first()
                .map(|view| (msg.key.mcu_id, msg.key.axis, Arc::clone(&view.signal)))
        })
        .collect()
}

/// Per-lane ceilings sitting one relative ulp above each lane's own demand, so
/// a ceiling looked up against the wrong lane trips the guard.
fn snug_ceilings(scenario: &Scenario, messages: &[EnqueueMsg]) -> Vec<Vec<f64>> {
    let signals = lane_signals(messages);
    scenario
        .lane_lists
        .iter()
        .enumerate()
        .map(|(mcu_index, axes)| {
            axes.iter()
                .map(|axis| {
                    signals
                        .iter()
                        .find(|(mcu_id, lane, _)| {
                            *mcu_id == mcu_index as u32 && usize::from(*lane) == *axis
                        })
                        .map_or(f64::INFINITY, |(_, _, signal)| {
                            window_demand(signal) * (1.0 + 1e-9) + f64::MIN_POSITIVE
                        })
                })
                .collect()
        })
        .collect()
}

fn magnitude_scale(bounds: &PvaBounds, samples: &[Pva]) -> f64 {
    samples
        .iter()
        .flat_map(|pva| [pva.velocity.abs(), pva.acceleration.abs()])
        .chain([
            bounds.velocity_min.abs(),
            bounds.velocity_max.abs(),
            bounds.acceleration_abs_max,
        ])
        .fold(1.0_f64, f64::max)
}

fn position_at(view: &ClockedMotorSpan, clock: u64) -> Result<f64, TestCaseError> {
    view.position_at_clock(clock)
        .map_err(|error| TestCaseError::fail(format!("view does not carry clock {clock}: {error}")))
}

proptest! {
    #![proptest_config(ProptestConfig {
        cases: 256,
        failure_persistence: Some(Box::new(FileFailurePersistence::Direct(
            "proptest-regressions/enqueue_fuzz.txt",
        ))),
        ..ProptestConfig::default()
    })]

    /// Every lane's views tile its segment exactly, in stream time and on the
    /// mcu clock, and every anchor comes off the one affine map of the dispatch
    /// instead of the previous view's rounded clock.
    #[test]
    fn every_lane_tiles_its_segment_in_stream_time_and_on_the_clock(
        scenario in arb_scenario(),
    ) {
        let messages = scenario
            .enqueue(&scenario.unbounded_ceilings())
            .map_err(|error| TestCaseError::fail(format!("valid segment rejected: {error}")))?;
        let seg = scenario.segment();
        let base = (scenario.t0 + seg.t_start) * scenario.clock_freq_hz + scenario.clock_phase;

        for msg in &messages {
            let Some(first) = msg.spans.first() else {
                continue;
            };
            let signal = Arc::clone(&first.signal);
            prop_assert_eq!(first.stream_t_start, seg.t_start);
            prop_assert_eq!(
                msg.spans.last().expect("a non-empty run").stream_t_end,
                seg.t_end
            );
            prop_assert_eq!(
                first.start_clock_exact,
                base,
                "the run must be anchored on the dispatch's own unrounded projection"
            );
            for view in &msg.spans {
                prop_assert!(Arc::ptr_eq(&view.signal, &signal), "views are zero-copy");
                prop_assert!(view.end_clock > view.start_clock);
                prop_assert!(
                    view.stream_t_end - view.stream_t_start <= MAX_SPAN_SECS * (1.0 + 1e-12),
                    "a view outran the dispatchable length"
                );
                let carried = view.start_clock_exact - base;
                let projected = (view.stream_t_start - seg.t_start) * scenario.clock_freq_hz;
                prop_assert!(
                    (carried - projected).abs()
                        <= 64.0 * f64::EPSILON * view.start_clock_exact.abs().max(1.0),
                    "anchor {} sits at {carried} ticks off the base where the map says {projected}",
                    view.start_clock_exact
                );
            }
            for pair in msg.spans.windows(2) {
                prop_assert_eq!(pair[0].stream_t_end, pair[1].stream_t_start);
                prop_assert!(
                    pair[0].end_clock.abs_diff(pair[1].start_clock) <= SEAM_ROUNDING_CYCLES,
                    "view seam {} -> {} is past the {SEAM_ROUNDING_CYCLES} tick budget the shim \
                     admits for two roundings of one stream instant",
                    pair[0].end_clock,
                    pair[1].start_clock
                );
            }
        }
    }

    /// The soundness the -307 guard rests on. Inside a window the reported hull
    /// must contain the track: that is what the guard's per-window read
    /// certifies. On a breakpoint the profile picks one of the two branches
    /// meeting there, so the guarantee is the weaker one the guard consumes —
    /// no wider in magnitude than the widest of the windows meeting there.
    #[test]
    fn lane_bounds_carry_every_sample_of_the_motor_span(scenario in arb_scenario()) {
        let messages = scenario
            .enqueue(&scenario.unbounded_ceilings())
            .map_err(|error| TestCaseError::fail(format!("valid segment rejected: {error}")))?;

        for (_, _, signal) in lane_signals(&messages) {
            let breakpoints: Vec<f64> = signal.breakpoints.to_vec();
            let bounds: Vec<PvaBounds> = breakpoints
                .windows(2)
                .map(|window| {
                    signal
                        .pva_bounds(window[0], window[1])
                        .expect("a breakpoint window lies inside its own span")
                })
                .collect();

            for (index, window) in breakpoints.windows(2).enumerate() {
                let (t0, t1) = (window[0], window[1]);
                let times = scenario.closed_samples(&signal, t0, t1);
                let samples = times
                    .iter()
                    .map(|&t| signal.eval_pva(t).expect("a sample inside the span"))
                    .collect::<Vec<Pva>>();
                let slack = RELATIVE_SLACK * magnitude_scale(&bounds[index], &samples);
                prop_assert!(
                    bounds[index].velocity_min <= bounds[index].velocity_max + slack,
                    "reversed velocity bounds over [{t0}, {t1}]: {:?}",
                    bounds[index]
                );
                for (t, pva) in times.iter().zip(&samples) {
                    if *t > t0 && *t < t1 {
                        prop_assert!(
                            carries(&bounds[index], pva, slack),
                            "sample {pva:?} at t={t} escapes its own window {:?}",
                            bounds[index]
                        );
                        continue;
                    }
                    let carriers = carrying_windows(&breakpoints, *t, index);
                    let widest = carriers.iter().map(|&at| demand(&bounds[at])).fold(0.0, f64::max);
                    let steepest = carriers
                        .iter()
                        .map(|&at| bounds[at].acceleration_abs_max)
                        .fold(0.0, f64::max);
                    prop_assert!(
                        pva.velocity.abs() <= widest + slack
                            && pva.acceleration.abs() <= steepest + slack,
                        "breakpoint sample {pva:?} at t={t} outruns every window meeting it: {:?}",
                        carriers.iter().map(|&at| bounds[at]).collect::<Vec<PvaBounds>>()
                    );
                }
            }
        }
    }

    /// A lane enqueued against a ceiling that just clears its own demand must
    /// enqueue, and nothing it then ships may outrun that ceiling.
    #[test]
    fn no_released_view_outruns_its_own_lane_ceiling(scenario in arb_scenario()) {
        let unbounded = scenario
            .enqueue(&scenario.unbounded_ceilings())
            .map_err(|error| TestCaseError::fail(format!("valid segment rejected: {error}")))?;
        let ceilings = snug_ceilings(&scenario, &unbounded);
        let messages = scenario
            .enqueue(&ceilings)
            .map_err(|error| TestCaseError::fail(format!("snug ceiling rejected: {error}")))?;

        for msg in &messages {
            let Some(first) = msg.spans.first() else {
                continue;
            };
            let lane_index = scenario.lane_lists[msg.key.mcu_id as usize]
                .iter()
                .position(|axis| *axis == usize::from(msg.key.axis))
                .expect("an enqueued lane is configured");
            let ceiling = ceilings[msg.key.mcu_id as usize][lane_index];
            let signal = Arc::clone(&first.signal);
            for view in &msg.spans {
                for t in scenario.closed_samples(&signal, view.stream_t_start, view.stream_t_end) {
                    let velocity = signal.eval_pva(t).expect("a sample inside the view").velocity;
                    prop_assert!(
                        velocity.abs() <= ceiling * (1.0 + RELATIVE_SLACK),
                        "lane {:?} runs at {velocity} mm/s past its {ceiling} mm/s ceiling",
                        msg.key
                    );
                }
            }
        }
    }

    /// Both sides of a view seam read the shared clock back to the same point
    /// on the track: at worst one clock tick of travel apart.
    #[test]
    fn positions_agree_across_every_view_seam(scenario in arb_scenario()) {
        let messages = scenario
            .enqueue(&scenario.unbounded_ceilings())
            .map_err(|error| TestCaseError::fail(format!("valid segment rejected: {error}")))?;

        for msg in &messages {
            let Some(first) = msg.spans.first() else {
                continue;
            };
            let signal = Arc::clone(&first.signal);
            for pair in msg.spans.windows(2) {
                let seam = pair[0].end_clock;
                let before = position_at(&pair[0], seam)?;
                let after = position_at(&pair[1], seam)?;
                let travel_per_tick = demand(
                    &signal
                        .pva_bounds(pair[0].stream_t_start, pair[1].stream_t_end)
                        .expect("a seam-straddling window lies inside the span"),
                ) / pair[0].clock_freq_hz;
                let tolerance =
                    travel_per_tick + RELATIVE_SLACK * before.abs().max(after.abs()).max(1.0);
                prop_assert!(
                    (before - after).abs() <= tolerance,
                    "seam clock {seam}: {before} vs {after} exceeds {tolerance}"
                );
            }
        }
    }
}
