use std::sync::Arc;

use nurbs::ScalarNurbs;
use proptest::prelude::*;
use proptest::test_runner::FileFailurePersistence;
use trajectory::{
    BuzzProfile, ContinuousAxis, MotorGroup, MotorSpan, MotorTerm, NudgeProfile, Pva, PvaBounds,
    RelativeSplinePiece,
};

const MAX_DEGREE: usize = 4;
const SAMPLES_PER_INTERVAL: usize = 64;
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
    RelativeSpline {
        base_mm: f64,
        shape: SplineShape,
    },
    Piecewise(Vec<(f64, SplineShape)>),
    Hold(f64),
    Nudge {
        delta_mm: f64,
        speed_mm_s: f64,
        accel_mm_s2: f64,
    },
    Buzz {
        amplitude_mm: f64,
        freq_start_hz: f64,
        freq_end_hz: f64,
        ramp: f64,
        sign: f64,
    },
}

#[derive(Debug, Clone)]
struct Case {
    t_start: f64,
    duration: f64,
    shape: AxisShape,
    window: (f64, f64),
    sample_fractions: Vec<f64>,
}

impl Case {
    fn t_end(&self) -> f64 {
        self.t_start + self.duration
    }

    fn axis(&self) -> ContinuousAxis {
        let (t_start, t_end) = (self.t_start, self.t_end());
        match &self.shape {
            AxisShape::Spline(shape) => ContinuousAxis::Spline(shape.curve(t_start, t_end)),
            AxisShape::RelativeSpline { base_mm, shape } => ContinuousAxis::RelativeSpline {
                base_position: *base_mm,
                curve: shape.curve(t_start, t_end),
            },
            AxisShape::Piecewise(pieces) => {
                let count = pieces.len() as f64;
                ContinuousAxis::PiecewiseRelativeSpline(Arc::from(
                    pieces
                        .iter()
                        .enumerate()
                        .map(|(index, (base_mm, shape))| {
                            let piece_start = t_start + self.duration * index as f64 / count;
                            let piece_end = t_start + self.duration * (index + 1) as f64 / count;
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
            AxisShape::Hold(position) => ContinuousAxis::Hold {
                position: *position,
                t_start,
                t_end,
            },
            AxisShape::Nudge {
                delta_mm,
                speed_mm_s,
                accel_mm_s2,
            } => ContinuousAxis::Nudge(
                NudgeProfile::try_new(*delta_mm, *speed_mm_s, *accel_mm_s2, t_start)
                    .expect("a nudge profile"),
            ),
            AxisShape::Buzz {
                amplitude_mm,
                freq_start_hz,
                freq_end_hz,
                ramp,
                sign,
            } => ContinuousAxis::Buzz {
                base_position: 0.0,
                sign: *sign,
                profile: Arc::new(
                    BuzzProfile::try_new(
                        *amplitude_mm,
                        *freq_start_hz,
                        *freq_end_hz,
                        self.duration,
                        *ramp,
                        t_start,
                    )
                    .expect("a buzz profile"),
                ),
            },
        }
    }

    fn interval(&self, axis: &ContinuousAxis) -> (f64, f64) {
        let (domain_start, domain_end) = axis.domain();
        let span = domain_end - domain_start;
        let (a, b) = self.window;
        (
            domain_start + a.min(b) * span,
            domain_start + a.max(b) * span,
        )
    }

    fn sample_times(&self, axis: &ContinuousAxis, t0: f64, t1: f64) -> Vec<f64> {
        let mut times = vec![t0, t1];
        times.extend(self.sample_fractions.iter().map(|f| t0 + f * (t1 - t0)));
        for knot in axis.breakpoints() {
            if knot > t0 && knot < t1 {
                times.extend([knot, next_toward(knot, t0), next_toward(knot, t1)]);
            }
        }
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

fn arb_axis_shape() -> impl Strategy<Value = AxisShape> {
    let spline = arb_spline_shape().prop_map(AxisShape::Spline);
    let relative = (-350.0..350.0, arb_spline_shape())
        .prop_map(|(base_mm, shape)| AxisShape::RelativeSpline { base_mm, shape });
    let piecewise = prop::collection::vec((-350.0..350.0, arb_spline_shape()), 1..=3)
        .prop_map(AxisShape::Piecewise);
    let hold = (-350.0..350.0).prop_map(AxisShape::Hold);
    let nudge = (
        prop_oneof![-5.0..-1e-3, 1e-3..5.0],
        1e-2..500.0,
        prop_oneof![Just(0.0), 1.0..50_000.0],
    )
        .prop_map(|(delta_mm, speed_mm_s, accel_mm_s2)| AxisShape::Nudge {
            delta_mm,
            speed_mm_s,
            accel_mm_s2,
        });
    let buzz = (
        1e-3..2.0,
        1.0..400.0,
        1.0..400.0,
        0.0..0.5,
        prop_oneof![Just(-1.0), Just(1.0)],
    )
        .prop_map(
            |(amplitude_mm, freq_start_hz, freq_end_hz, ramp, sign)| AxisShape::Buzz {
                amplitude_mm,
                freq_start_hz,
                freq_end_hz,
                ramp,
                sign,
            },
        );
    prop_oneof![spline, relative, piecewise, hold, nudge, buzz]
}

fn arb_case() -> impl Strategy<Value = Case> {
    (
        0.0..1000.0,
        prop_oneof![1e-4..1e-2, 1e-2..1.0, 1.0..5.0],
        arb_axis_shape(),
        (0.0..=1.0, 0.0..=1.0),
        prop::collection::vec(0.0..=1.0, SAMPLES_PER_INTERVAL),
    )
        .prop_map(
            |(t_start, duration, shape, window, sample_fractions)| Case {
                t_start,
                duration,
                shape,
                window,
                sample_fractions,
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

proptest! {
    #![proptest_config(ProptestConfig {
        cases: 512,
        failure_persistence: Some(Box::new(FileFailurePersistence::Direct(
            "proptest-regressions/bounds_fuzz.txt",
        ))),
        ..ProptestConfig::default()
    })]

    #[test]
    fn axis_bounds_contain_every_sample(case in arb_case()) {
        let axis = case.axis();
        let (t0, t1) = case.interval(&axis);
        let bounds = axis.pva_bounds(t0, t1).expect("bounds over a sub-interval");
        let times = case.sample_times(&axis, t0, t1);
        let samples = times
            .iter()
            .map(|&t| axis.eval_pva(t).expect("a sample inside the domain"))
            .collect::<Vec<Pva>>();

        check_bounds_contain_samples(&bounds, &times, &samples, t0)?;
    }

    #[test]
    fn motor_span_bounds_contain_every_sample(
        case in arb_case(),
        scale in prop_oneof![Just(1.0), -3.0..3.0],
        summed in prop::option::of(arb_spline_shape()),
    ) {
        let axis = case.axis();
        let (t_start, t_end) = axis.domain();
        let mut groups = vec![MotorGroup::Independent(MotorTerm {
            source_axis: 0,
            axis,
            scale,
        })];
        if let Some(shape) = summed {
            groups.push(MotorGroup::Spline {
                curve: shape.curve(t_start, t_end),
                summed_scale: scale,
            });
        }
        let span = MotorSpan::try_new(Arc::from(groups), t_start, t_end, 0, 0, false)
            .expect("a dispatchable motor span");
        let (t0, t1) = {
            let (a, b) = case.window;
            let width = t_end - t_start;
            (t_start + a.min(b) * width, t_start + a.max(b) * width)
        };
        let bounds = span.pva_bounds(t0, t1).expect("bounds over a sub-interval");
        let mut times = vec![t0, t1];
        times.extend(case.sample_fractions.iter().map(|f| t0 + f * (t1 - t0)));
        for &knot in span.breakpoints.iter() {
            if knot > t0 && knot < t1 {
                times.extend([knot, next_toward(knot, t0), next_toward(knot, t1)]);
            }
        }
        let samples = times
            .iter()
            .map(|&t| span.eval_pva(t).expect("a sample inside the span"))
            .collect::<Vec<Pva>>();

        check_bounds_contain_samples(&bounds, &times, &samples, t0)?;
    }
}
