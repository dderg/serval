use std::f64::consts::FRAC_1_SQRT_2;
use std::sync::Arc;

use geometry::path::{Arc as PathArc, Clothoid, CurvatureProfile, Line, PathSegment, Segment};
use geometry::{FollowerDemand, LawSegment, Move, ScalarLaw, SourceRange, VelocityLimits};
use proptest::prelude::*;
use proptest::test_runner::FileFailurePersistence;
use trajectory::{
    AnalyticMoveSpan, ContinuousAxis, MotorGroup, MotorSpan, MotorTerm, Pva, PvaBounds, SurfaceMode,
};

const SAMPLES_PER_INTERVAL: usize = 64;
const RELATIVE_SLACK: f64 = 1e-9;
const AXIS_SLOTS: usize = 6;
const HELD_AXIS: usize = 5;
const MAX_FOLLOWERS: usize = 2;
const MAX_PHASES: usize = 3;
const ONE_OVER_SQRT_3: f64 = 0.577_350_269_189_625_8;

fn plane_basis(index: usize) -> ([f64; 3], [f64; 3]) {
    match index % 5 {
        0 => ([1.0, 0.0, 0.0], [0.0, 1.0, 0.0]),
        1 => ([0.0, 1.0, 0.0], [0.0, 0.0, 1.0]),
        2 => ([0.0, 0.0, 1.0], [1.0, 0.0, 0.0]),
        3 => (
            [FRAC_1_SQRT_2, FRAC_1_SQRT_2, 0.0],
            [-FRAC_1_SQRT_2, FRAC_1_SQRT_2, 0.0],
        ),
        _ => ([ONE_OVER_SQRT_3; 3], [FRAC_1_SQRT_2, -FRAC_1_SQRT_2, 0.0]),
    }
}

#[derive(Debug, Clone)]
enum SegmentShape {
    Line {
        plane: usize,
        length: f64,
    },
    Arc {
        plane: usize,
        radius: f64,
        length: f64,
        clockwise: bool,
    },
    Clothoid {
        plane: usize,
        kappa_start: f64,
        kappa_end: f64,
        length: f64,
    },
}

impl SegmentShape {
    fn build(&self) -> Segment {
        match self {
            Self::Line { plane, length } => {
                let (u, _) = plane_basis(*plane);
                let start = [0.4, -1.2, 3.1];
                let end = std::array::from_fn(|axis| start[axis] + length * u[axis]);
                Segment::Line(Line::try_new(start, end).expect("a positive-length line"))
            }
            Self::Arc {
                plane,
                radius,
                length,
                clockwise,
            } => {
                let (u, v) = plane_basis(*plane);
                let sweep = if *clockwise {
                    -length / radius
                } else {
                    length / radius
                };
                Segment::Arc(
                    PathArc::try_new([0.4, -1.2, 3.1], u, v, *radius, 0.7, sweep)
                        .expect("a positive-length arc"),
                )
            }
            Self::Clothoid {
                plane,
                kappa_start,
                kappa_end,
                length,
            } => {
                let (u, v) = plane_basis(*plane);
                Segment::Clothoid(
                    Clothoid::try_new(
                        [0.4, -1.2, 3.1],
                        u,
                        v,
                        *kappa_start,
                        (kappa_end - kappa_start) / length,
                        *length,
                    )
                    .expect("a finite clothoid"),
                )
            }
        }
    }
}

fn arb_segment_shape() -> impl Strategy<Value = SegmentShape> {
    let length = || prop_oneof![0.05f64..0.5, 0.5f64..1.5];
    prop_oneof![
        (0usize..5, length()).prop_map(|(plane, length)| SegmentShape::Line { plane, length }),
        (0usize..5, 0.15f64..40.0, length(), any::<bool>()).prop_map(
            |(plane, radius, length, clockwise)| SegmentShape::Arc {
                plane,
                radius,
                length,
                clockwise,
            }
        ),
        (0usize..5, -4.0f64..4.0, -4.0f64..4.0, length()).prop_map(
            |(plane, kappa_start, kappa_end, length)| SegmentShape::Clothoid {
                plane,
                kappa_start,
                kappa_end,
                length,
            }
        ),
    ]
}

#[derive(Debug, Clone)]
struct PhaseShape {
    rail: bool,
    brake: bool,
    accel_fraction: f64,
    arc_fraction: f64,
}

fn arb_phase_shape() -> impl Strategy<Value = PhaseShape> {
    (any::<bool>(), any::<bool>(), 0.0f64..=1.0, 0.02f64..=0.98).prop_map(
        |(rail, brake, accel_fraction, arc_fraction)| PhaseShape {
            rail,
            brake,
            accel_fraction,
            arc_fraction,
        },
    )
}

#[derive(Debug, Clone)]
struct Case {
    segment: SegmentShape,
    followers: Vec<(f64, f64)>,
    phases: Vec<PhaseShape>,
    accel_budget: f64,
    entry_speed_fraction: f64,
    t_start: f64,
    distance_origin: f64,
    surface_offset: Option<f64>,
    axis_starts: Vec<f64>,
    window: (f64, f64),
    sample_fractions: Vec<f64>,
    group_axes: Vec<(usize, f64)>,
}

/// Speed the curvature cap admits at the acceleration budget: above
/// `sqrt(accel/|kappa|)` the normal acceleration alone exceeds the budget and
/// the disk rail has no tangential headroom left.
fn curvature_capped_speed(segment: &Segment, accel_budget: f64) -> f64 {
    let (_, kappa_peak) = segment.kappa_peak();
    if kappa_peak > 0.0 {
        (accel_budget / kappa_peak).sqrt()
    } else {
        500.0
    }
}

fn phase_boundaries(length: f64, phases: &[PhaseShape]) -> Vec<f64> {
    let mut fractions = phases
        .iter()
        .skip(1)
        .map(|phase| phase.arc_fraction)
        .collect::<Vec<f64>>();
    fractions.sort_by(f64::total_cmp);
    let mut boundaries = vec![0.0];
    for fraction in fractions {
        let arc = length * fraction;
        if arc > boundaries[boundaries.len() - 1] + 1e-3 * length && arc < length - 1e-3 * length {
            boundaries.push(arc);
        }
    }
    boundaries.push(length);
    boundaries
}

/// The rail follows its law only while the state stays under the local
/// curvature cap `|kappa|*v^2 <= accel`; above the cap the tangential term
/// saturates at zero, the law kinks, and the dense reconstruction stops being
/// monotone. `v^2 <= entry^2 + 2*accel*arc_span` bounds the phase exit, so a
/// budget above this floor keeps the whole phase under the cap.
fn cap_clearing_budget(
    curvature_abs_max: f64,
    entry_speed: f64,
    arc_span: f64,
    requested: f64,
) -> Option<f64> {
    let headroom = 1.0 - 2.0 * curvature_abs_max * arc_span;
    if headroom < 0.5 {
        return None;
    }
    Some(requested.max(1.25 * curvature_abs_max * entry_speed * entry_speed / headroom))
}

fn rail_law(
    segment: &Segment,
    shape: &PhaseShape,
    arc_start: f64,
    arc_span: f64,
    entry_speed: f64,
    accel_budget: f64,
) -> Option<ScalarLaw> {
    if shape.brake && entry_speed == 0.0 {
        return None;
    }
    let curvature_abs_max = segment
        .kappa(arc_start)
        .abs()
        .max(segment.kappa(arc_start + arc_span).abs());
    let budget = cap_clearing_budget(curvature_abs_max, entry_speed, arc_span, accel_budget)?;
    if shape.brake && budget > 0.45 * entry_speed * entry_speed / arc_span {
        return None;
    }
    Some(ScalarLaw::DiskRail {
        accel: budget,
        kappa0: segment.kappa(arc_start),
        sigma: segment.dkappa_ds(arc_start),
        brake: shape.brake,
    })
}

fn phase_law(
    segment: &Segment,
    shape: &PhaseShape,
    arc_start: f64,
    arc_span: f64,
    entry_speed: f64,
    accel_budget: f64,
) -> ScalarLaw {
    if shape.rail {
        if let Some(law) = rail_law(
            segment,
            shape,
            arc_start,
            arc_span,
            entry_speed,
            accel_budget,
        ) {
            return law;
        }
    }
    let reachable_decel = -0.95 * entry_speed * entry_speed / (2.0 * arc_span);
    let a0 = if shape.brake && entry_speed > 0.0 {
        reachable_decel * shape.accel_fraction
    } else {
        accel_budget * shape.accel_fraction.max(0.05)
    };
    ScalarLaw::ConstAccel { a0 }
}

impl Case {
    fn followers(&self) -> Vec<FollowerDemand> {
        self.followers
            .iter()
            .enumerate()
            .map(|(index, (ratio, ratio_end))| FollowerDemand {
                axis_index: 3 + index,
                ratio: *ratio,
                ratio_end: *ratio_end,
            })
            .collect()
    }

    fn span(&self) -> Arc<AnalyticMoveSpan> {
        let segment = self.segment.build();
        let length = segment.s_len();
        let entry_speed =
            self.entry_speed_fraction * curvature_capped_speed(&segment, self.accel_budget);
        let boundaries = phase_boundaries(length, &self.phases);
        let mut phases = Vec::with_capacity(boundaries.len() - 1);
        let mut speed = entry_speed;
        let mut chain_time = 0.0;
        for (index, window) in boundaries.windows(2).enumerate() {
            let (arc_start, arc_end) = (window[0], window[1]);
            let law = phase_law(
                &segment,
                &self.phases[index.min(self.phases.len() - 1)],
                arc_start,
                arc_end - arc_start,
                speed,
                self.accel_budget,
            );
            let phase = LawSegment::until_arc(
                chain_time,
                self.distance_origin + arc_start,
                speed,
                law,
                arc_end - arc_start,
            )
            .expect("a stall-free phase");
            let (_, end_speed, _) = phase.end_state();
            chain_time = phase.end_time();
            speed = end_speed;
            phases.push(phase);
        }
        let surface = match self.surface_offset {
            None => SurfaceMode::None,
            Some(offset) => SurfaceMode::Constant(offset),
        };
        Arc::new(
            AnalyticMoveSpan::try_new(
                Move {
                    segment: PathSegment::try_new(segment, self.followers())
                        .expect("valid follower demands"),
                    feedrate_mm_s: 100.0,
                    limits: VelocityLimits::try_new(500.0, self.accel_budget, 0.1, f64::INFINITY)
                        .expect("valid limits"),
                    source: SourceRange {
                        start_line: 7,
                        end_line: 7,
                    },
                },
                Arc::from(phases),
                self.distance_origin,
                self.t_start,
                self.t_start + chain_time,
                Arc::from(self.axis_starts.clone()),
                surface,
            )
            .expect("a coverage-consistent analytic span"),
        )
    }

    fn interval(&self, span: &AnalyticMoveSpan) -> (f64, f64) {
        let width = span.t_end - span.t_start;
        let (a, b) = self.window;
        (
            span.t_start + a.min(b) * width,
            span.t_start + a.max(b) * width,
        )
    }

    fn sample_times(&self, breakpoints: &[f64], t0: f64, t1: f64) -> Vec<f64> {
        let mut times = vec![t0, t1];
        times.extend(self.sample_fractions.iter().map(|f| t0 + f * (t1 - t0)));
        for &knot in breakpoints {
            if knot > t0 && knot < t1 {
                times.extend([knot, next_toward(knot, t0), next_toward(knot, t1)]);
            }
        }
        times
    }

    fn terms(&self, span: &Arc<AnalyticMoveSpan>) -> Vec<MotorTerm> {
        self.group_axes
            .iter()
            .map(|(axis, scale)| MotorTerm {
                source_axis: *axis,
                axis: ContinuousAxis::Analytic {
                    span: Arc::clone(span),
                    axis: *axis,
                },
                scale: *scale,
            })
            .collect()
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

fn arb_case() -> impl Strategy<Value = Case> {
    (
        arb_segment_shape(),
        prop::collection::vec(
            (
                prop_oneof![-0.4f64..-1e-3, 1e-3f64..0.4],
                prop_oneof![-0.4f64..-1e-3, 1e-3f64..0.4],
            ),
            0..=MAX_FOLLOWERS,
        ),
        prop::collection::vec(arb_phase_shape(), 1..=MAX_PHASES),
        prop_oneof![100.0f64..5_000.0, 5_000.0f64..100_000.0],
        prop_oneof![Just(0.0), 1e-3f64..1.0],
        prop_oneof![Just(0.0), 0.0f64..1_000.0],
        prop_oneof![Just(0.0), -1_000.0f64..10_000.0],
        prop::option::of(-2.0f64..2.0),
        prop::collection::vec(-350.0f64..350.0, AXIS_SLOTS),
        prop_oneof![Just((0.0, 1.0)), (0.0f64..=1.0, 0.0f64..=1.0)],
        prop::collection::vec(0.0f64..=1.0, SAMPLES_PER_INTERVAL),
        prop::collection::vec(
            (
                0usize..AXIS_SLOTS,
                prop_oneof![Just(1.0), Just(-1.0), -3.0f64..3.0],
            ),
            1..=4,
        ),
    )
        .prop_map(
            |(
                segment,
                followers,
                phases,
                accel_budget,
                entry_speed_fraction,
                t_start,
                distance_origin,
                surface_offset,
                axis_starts,
                window,
                sample_fractions,
                group_axes,
            )| Case {
                segment,
                followers,
                phases,
                accel_budget,
                entry_speed_fraction,
                t_start,
                distance_origin,
                surface_offset,
                axis_starts,
                window,
                sample_fractions,
                group_axes,
            },
        )
}

fn magnitude_scale(bounds: &PvaBounds, samples: &[Pva]) -> f64 {
    samples
        .iter()
        .flat_map(|pva| {
            [
                pva.position.abs(),
                pva.velocity.abs(),
                pva.acceleration.abs(),
            ]
        })
        .chain([
            bounds.velocity_min.abs(),
            bounds.velocity_max.abs(),
            bounds.acceleration_abs_max,
        ])
        .fold(1.0_f64, f64::max)
}

fn check_bounds_contain_samples(
    bounds: &PvaBounds,
    times: &[f64],
    samples: &[Pva],
    t0: f64,
) -> Result<(), TestCaseError> {
    let slack = RELATIVE_SLACK * magnitude_scale(bounds, samples);
    prop_assert!(
        bounds.velocity_min <= bounds.velocity_max + slack,
        "velocity bounds are reversed: {bounds:?}"
    );
    for (t, pva) in times.iter().zip(samples) {
        prop_assert!(
            pva.velocity >= bounds.velocity_min - slack
                && pva.velocity <= bounds.velocity_max + slack,
            "velocity {} at t={t} escapes {bounds:?}",
            pva.velocity
        );
        prop_assert!(
            pva.acceleration.abs() <= bounds.acceleration_abs_max + slack,
            "acceleration {} at t={t} escapes {bounds:?}",
            pva.acceleration
        );
        if bounds.velocity_continuous {
            let drift = bounds.acceleration_abs_max * (t - t0).abs() + slack;
            prop_assert!(
                (pva.velocity - samples[0].velocity).abs() <= drift,
                "velocity {} at t={t} drifted more than {drift} from {} despite continuity: {bounds:?}",
                pva.velocity,
                samples[0].velocity
            );
        }
    }
    Ok(())
}

/// A single disk-rail phase over a clothoid whose curvature passes through
/// zero: the tangential acceleration `sqrt(A^2 - (kappa*v^2)^2)` peaks at the
/// inflection, strictly inside the phase, while both phase ends sit at the
/// curvature extremes. `entry_load` is the entry's normal acceleration as a
/// fraction of the budget; the arc is scaled against the headroom that keeps
/// the exit under the cap.
#[derive(Debug, Clone)]
struct InflectionCase {
    plane: usize,
    spatial_axis: usize,
    kappa: f64,
    length_fraction: f64,
    accel_budget: f64,
    entry_load: f64,
    follower_ratio: f64,
    follower_scale: f64,
    window: (f64, f64),
    sample_fractions: Vec<f64>,
}

const INFLECTION_EXIT_LOAD_CEILING: f64 = 0.9;

impl InflectionCase {
    /// `v^2 <= entry^2 + 2*accel*length` keeps the exit load below
    /// `INFLECTION_EXIT_LOAD_CEILING` times the budget.
    fn length(&self) -> f64 {
        self.length_fraction * (INFLECTION_EXIT_LOAD_CEILING - self.entry_load) / (2.0 * self.kappa)
    }

    fn span(&self) -> Arc<AnalyticMoveSpan> {
        let (u, v) = plane_basis(self.plane);
        let segment = Segment::Clothoid(
            Clothoid::try_new(
                [0.0, 0.0, 0.0],
                u,
                v,
                -self.kappa,
                2.0 * self.kappa / self.length(),
                self.length(),
            )
            .expect("a finite clothoid"),
        );
        let length = segment.s_len();
        let entry_speed = (self.entry_load * self.accel_budget / self.kappa).sqrt();
        let phase = LawSegment::until_arc(
            0.0,
            0.0,
            entry_speed,
            ScalarLaw::DiskRail {
                accel: self.accel_budget,
                kappa0: segment.kappa(0.0),
                sigma: segment.dkappa_ds(0.0),
                brake: false,
            },
            length,
        )
        .expect("an accelerating rail never stalls");
        let duration = phase.end_time();
        Arc::new(
            AnalyticMoveSpan::try_new(
                Move {
                    segment: PathSegment::try_new(
                        segment,
                        vec![FollowerDemand::constant(3, self.follower_ratio)],
                    )
                    .expect("valid follower demands"),
                    feedrate_mm_s: 100.0,
                    limits: VelocityLimits::try_new(500.0, self.accel_budget, 0.1, f64::INFINITY)
                        .expect("valid limits"),
                    source: SourceRange {
                        start_line: 7,
                        end_line: 7,
                    },
                },
                Arc::from([phase]),
                0.0,
                0.0,
                duration,
                Arc::from([0.0, 0.0, 0.0, 0.0]),
                SurfaceMode::None,
            )
            .expect("a coverage-consistent analytic span"),
        )
    }

    fn interval_around_inflection(&self, span: &AnalyticMoveSpan) -> (f64, f64) {
        let inflection = span.phases[0]
            .time_at_distance(0.5 * span.source.segment.s_len())
            .expect("the inflection lies inside the phase");
        let (before, after) = self.window;
        (
            span.t_start + inflection * (1.0 - before),
            inflection + after * (span.t_end - inflection),
        )
    }

    fn terms(&self, span: &Arc<AnalyticMoveSpan>) -> Vec<MotorTerm> {
        [(self.spatial_axis, 1.0), (3, self.follower_scale)]
            .into_iter()
            .map(|(axis, scale)| MotorTerm {
                source_axis: axis,
                axis: ContinuousAxis::Analytic {
                    span: Arc::clone(span),
                    axis,
                },
                scale,
            })
            .collect()
    }
}

fn arb_inflection_case() -> impl Strategy<Value = InflectionCase> {
    (
        0usize..5,
        0usize..3,
        0.2f64..5.0,
        0.1f64..=1.0,
        prop_oneof![1_000.0f64..20_000.0, 20_000.0f64..100_000.0],
        0.05f64..0.85,
        prop_oneof![-0.5f64..-1e-3, 1e-3f64..0.5],
        prop_oneof![Just(1.0), Just(-1.0), -2.0f64..2.0],
        (0.0f64..=1.0, 0.0f64..=1.0),
        prop::collection::vec(0.0f64..=1.0, SAMPLES_PER_INTERVAL),
    )
        .prop_map(
            |(
                plane,
                spatial_axis,
                kappa,
                length_fraction,
                accel_budget,
                entry_load,
                follower_ratio,
                follower_scale,
                window,
                sample_fractions,
            )| InflectionCase {
                plane,
                spatial_axis,
                kappa,
                length_fraction,
                accel_budget,
                entry_load,
                follower_ratio,
                follower_scale,
                window,
                sample_fractions,
            },
        )
}

proptest! {
    #![proptest_config(ProptestConfig {
        cases: 256,
        failure_persistence: Some(Box::new(FileFailurePersistence::Direct(
            "proptest-regressions/analytic_bounds_fuzz.txt",
        ))),
        ..ProptestConfig::default()
    })]

    #[test]
    fn analytic_axis_bounds_contain_every_sample(case in arb_case()) {
        let span = case.span();
        let (t0, t1) = case.interval(&span);
        for axis in 0..AXIS_SLOTS {
            let carrier = ContinuousAxis::Analytic {
                span: Arc::clone(&span),
                axis,
            };
            let bounds = carrier.pva_bounds(t0, t1).expect("bounds over a sub-interval");
            let times = case.sample_times(&carrier.breakpoints(), t0, t1);
            let samples = times
                .iter()
                .map(|&t| carrier.eval_pva(t).expect("a sample inside the span"))
                .collect::<Vec<Pva>>();

            check_bounds_contain_samples(&bounds, &times, &samples, t0)?;
        }
    }

    #[test]
    fn analytic_group_bounds_contain_every_sample(case in arb_case()) {
        let span = case.span();
        let group = MotorGroup::Analytic {
            span: Arc::clone(&span),
            terms: Arc::from(case.terms(&span)),
        };
        let motor = MotorSpan::try_new(
            Arc::from([group]),
            span.t_start,
            span.t_end,
            0,
            7,
            false,
        )
        .expect("a dispatchable analytic motor span");
        let (t0, t1) = case.interval(&span);
        let bounds = motor.pva_bounds(t0, t1).expect("bounds over a sub-interval");
        let times = case.sample_times(&motor.breakpoints, t0, t1);
        let samples = times
            .iter()
            .map(|&t| motor.eval_pva(t).expect("a sample inside the span"))
            .collect::<Vec<Pva>>();

        check_bounds_contain_samples(&bounds, &times, &samples, t0)?;
    }

    #[test]
    fn held_axis_slots_never_move(case in arb_case()) {
        let span = case.span();
        let carrier = ContinuousAxis::Analytic {
            span: Arc::clone(&span),
            axis: HELD_AXIS,
        };
        let (t0, t1) = case.interval(&span);
        let start = case.axis_starts[HELD_AXIS];
        for &t in &case.sample_times(&carrier.breakpoints(), t0, t1) {
            let sample = carrier.eval_pva(t).expect("a sample inside the span");
            prop_assert_eq!(sample.position, start);
            prop_assert_eq!(sample.velocity, 0.0);
            prop_assert_eq!(sample.acceleration, 0.0);
        }
    }

    #[test]
    fn rail_bounds_contain_the_inflection_peak(case in arb_inflection_case()) {
        let span = case.span();
        let motor = MotorSpan::try_new(
            Arc::from([MotorGroup::Analytic {
                span: Arc::clone(&span),
                terms: Arc::from(case.terms(&span)),
            }]),
            span.t_start,
            span.t_end,
            0,
            7,
            false,
        )
        .expect("a dispatchable analytic motor span");
        let (t0, t1) = case.interval_around_inflection(&span);
        let bounds = motor.pva_bounds(t0, t1).expect("bounds over a sub-interval");
        let mut times = vec![t0, t1];
        times.extend(case.sample_fractions.iter().map(|f| t0 + f * (t1 - t0)));
        let samples = times
            .iter()
            .map(|&t| motor.eval_pva(t).expect("a sample inside the span"))
            .collect::<Vec<Pva>>();

        check_bounds_contain_samples(&bounds, &times, &samples, t0)?;
    }
}
