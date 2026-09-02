use std::f64::consts::PI;
use std::sync::Arc;

use proptest::prelude::*;
use proptest::test_runner::FileFailurePersistence;
use trajectory::continuous::{
    interior_time_above, interior_time_below, ProfileError, ProfileSample,
};
use trajectory::{
    BuzzProfile, ContinuousAxis, MotorGroup, MotorSpan, MotorTerm, NudgeProfile, Pva, PvaBounds,
};

const DENSE_SAMPLES: usize = 256;
const RELATIVE_SLACK: f64 = 1e-9;
const ENDPOINT_SLACK: f64 = 1e-12;
const DERIVATIVE_SLACK: f64 = 1e-6;
/// A central difference over this much carrier phase keeps both the quadratic
/// truncation term and the subtractive cancellation far below
/// `DERIVATIVE_SLACK`.
const DERIVATIVE_PHASE_STEP: f64 = 1e-4;

fn sample_times(breakpoints: &[f64], t_start: f64, t_end: f64, fractions: &[f64]) -> Vec<f64> {
    let mut times = vec![t_start, t_end];
    times.extend(fractions.iter().map(|f| t_start + f * (t_end - t_start)));
    for &knot in breakpoints {
        times.push(knot);
        if knot > t_start {
            times.push(interior_time_below(knot));
        }
        if knot < t_end {
            times.push(interior_time_above(knot));
        }
    }
    times.retain(|t| *t >= t_start && *t <= t_end);
    times
}

fn motor_span(axis: ContinuousAxis) -> Arc<MotorSpan> {
    let (t_start, t_end) = axis.domain();
    Arc::new(
        MotorSpan::try_new(
            Arc::from([MotorGroup::Independent(MotorTerm {
                source_axis: 0,
                axis,
                scale: 1.0,
            })]),
            t_start,
            t_end,
            1,
            41,
            false,
        )
        .expect("a dispatchable motor span"),
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

/// Mirrors `check_bounds_contain_samples` in `bounds_fuzz`: every sample inside
/// the reported band, ordered bounds, and a Lipschitz drift no larger than the
/// reported acceleration whenever continuity is claimed.
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

#[derive(Debug, Clone)]
struct NudgeCase {
    delta_mm: f64,
    speed_mm_s: f64,
    accel_mm_s2: f64,
    t_start: f64,
    fractions: Vec<f64>,
    window: (f64, f64),
}

impl NudgeCase {
    fn profile(&self) -> NudgeProfile {
        NudgeProfile::try_new(
            self.delta_mm,
            self.speed_mm_s,
            self.accel_mm_s2,
            self.t_start,
        )
        .expect("a nonzero displacement at a nonzero speed")
    }

    /// The trapezoid speed is the least of the acceleration ramp, the requested
    /// cruise and the deceleration ramp — an envelope that never names
    /// `peak_speed` or the phase durations the profile derives.
    fn oracle_speed(&self, profile: &NudgeProfile, t: f64) -> f64 {
        if self.accel_mm_s2 == 0.0 {
            return profile.speed_mm_s();
        }
        let rise = self.accel_mm_s2 * (t - profile.t_start());
        let fall = self.accel_mm_s2 * (profile.t_end() - t);
        rise.min(profile.speed_mm_s()).min(fall)
    }

    /// The profile measures its phases from `t - t_start` while the oracle
    /// measures them from the absolute ends: the two differ by a few ulps of
    /// the stream time, which the ramp turns into this much speed.
    fn time_quantization_speed(&self, profile: &NudgeProfile) -> f64 {
        self.accel_mm_s2 * 8.0 * f64::EPSILON * profile.t_end().abs().max(1.0)
    }
}

fn arb_nudge_case() -> impl Strategy<Value = NudgeCase> {
    (
        prop_oneof![-20.0f64..-1e-3, 1e-3f64..20.0],
        prop_oneof![1e-2f64..1.0, 1.0f64..500.0],
        prop_oneof![Just(0.0), 1.0f64..1_000.0, 1_000.0f64..100_000.0],
        prop_oneof![Just(0.0), 0.0f64..1_000.0],
        prop::collection::vec(0.0f64..=1.0, DENSE_SAMPLES),
        prop_oneof![
            Just((0.0, 1.0)),
            (Just(0.0), 0.0f64..=1.0),
            (0.0f64..=1.0, Just(1.0)),
            (0.0f64..=1.0, 0.0f64..=1.0)
        ],
    )
        .prop_map(
            |(delta_mm, speed_mm_s, accel_mm_s2, t_start, fractions, window)| NudgeCase {
                delta_mm,
                speed_mm_s,
                accel_mm_s2,
                t_start,
                fractions,
                window,
            },
        )
}

#[derive(Debug, Clone)]
struct BuzzCase {
    amplitude_mm: f64,
    freq_start_hz: f64,
    freq_end_hz: f64,
    duration: f64,
    ramp: f64,
    t_start: f64,
    sign: f64,
    base_position: f64,
    fractions: Vec<f64>,
    window: (f64, f64),
}

impl BuzzCase {
    fn profile(&self) -> BuzzProfile {
        BuzzProfile::try_new(
            self.amplitude_mm,
            self.freq_start_hz,
            self.freq_end_hz,
            self.duration,
            self.ramp,
            self.t_start,
        )
        .expect("positive frequencies over a positive duration")
    }

    /// The carrier holds the velocity amplitude `A*omega_start` as the sweep
    /// moves, so the displacement amplitude scales as `omega_start/omega(t)`
    /// and peaks where the frequency is lowest.
    fn position_ceiling(&self) -> f64 {
        self.amplitude_mm.abs() * (self.freq_start_hz / self.freq_end_hz).max(1.0)
    }

    /// A central difference is only second-order accurate below the fastest
    /// timescale present: the carrier `omega`, the sweep's amplitude decay
    /// `|sweep_rate|/omega_min`, and the envelope's `1/ramp` slope.
    fn derivative_step(&self) -> f64 {
        let omega_start = 2.0 * PI * self.freq_start_hz;
        let omega_end = 2.0 * PI * self.freq_end_hz;
        let sweep_rate = (omega_end - omega_start) / self.duration;
        let rate = omega_start
            .max(omega_end)
            .max(sweep_rate.abs() / omega_start.min(omega_end))
            .max(if self.ramp > 0.0 {
                1.0 / self.ramp
            } else {
                0.0
            });
        (DERIVATIVE_PHASE_STEP / rate).min(1e-3 * self.duration)
    }
}

fn arb_buzz_case() -> impl Strategy<Value = BuzzCase> {
    (
        prop_oneof![Just(0.0), 1e-4f64..0.1, 0.1f64..2.0],
        1.0f64..400.0,
        1.0f64..400.0,
        prop_oneof![1e-4f64..1e-2, 1e-2f64..0.15],
        prop_oneof![Just(0.0), 1e-6f64..0.1],
        prop_oneof![Just(0.0), 0.0f64..1_000.0],
        prop_oneof![Just(1.0), Just(-1.0)],
        prop_oneof![Just(0.0), -350.0f64..350.0],
        prop::collection::vec(0.0f64..=1.0, DENSE_SAMPLES),
        prop_oneof![
            Just((0.0, 1.0)),
            (Just(0.0), 0.0f64..=1.0),
            (0.0f64..=1.0, Just(1.0)),
            (0.0f64..=1.0, 0.0f64..=1.0)
        ],
    )
        .prop_map(
            |(
                amplitude_mm,
                freq_start_hz,
                freq_end_hz,
                duration,
                ramp,
                t_start,
                sign,
                base_position,
                fractions,
                window,
            )| BuzzCase {
                amplitude_mm,
                freq_start_hz,
                freq_end_hz,
                duration,
                ramp,
                t_start,
                sign,
                base_position,
                fractions,
                window,
            },
        )
}

fn window_of(domain: (f64, f64), window: (f64, f64)) -> (f64, f64) {
    let (start, end) = domain;
    let span = end - start;
    let (a, b) = window;
    (start + a.min(b) * span, start + a.max(b) * span)
}

proptest! {
    #![proptest_config(ProptestConfig {
        cases: 320,
        failure_persistence: Some(Box::new(FileFailurePersistence::Direct(
            "proptest-regressions/profiles_fuzz.txt",
        ))),
        ..ProptestConfig::default()
    })]

    #[test]
    fn nudge_tracks_the_closed_form_trapezoid(case in arb_nudge_case()) {
        let profile = case.profile();
        let (t_start, t_end) = (profile.t_start(), profile.t_end());
        let sign = case.delta_mm.signum();

        prop_assert_eq!(profile.eval(t_start).position, 0.0);
        prop_assert_eq!(profile.eval(t_start).velocity, 0.0);
        prop_assert_eq!(profile.eval(t_end).velocity, 0.0);
        prop_assert!(
            (profile.eval(t_end).position - case.delta_mm).abs()
                <= ENDPOINT_SLACK * case.delta_mm.abs(),
            "the nudge must land on {}, got {}",
            case.delta_mm,
            profile.eval(t_end).position
        );

        let mut times = sample_times(profile.breakpoints(), t_start, t_end, &case.fractions);
        times.sort_by(f64::total_cmp);
        let oracle_slack =
            RELATIVE_SLACK * profile.speed_mm_s() + case.time_quantization_speed(&profile);
        let mut previous_position = 0.0;
        for &t in &times {
            let sample = profile.eval(t);
            let oracle = case.oracle_speed(&profile, t);
            if t > t_start && t < t_end {
                prop_assert!(
                    (sample.velocity - sign * oracle).abs() <= oracle_slack,
                    "velocity {} at t={t} leaves the trapezoid envelope {}",
                    sample.velocity,
                    sign * oracle
                );
                prop_assert!(
                    sample.velocity * sign > 0.0,
                    "velocity {} at t={t} moves against the displacement",
                    sample.velocity
                );
            }
            prop_assert!(
                sample.velocity.abs() <= profile.speed_mm_s() + oracle_slack,
                "velocity {} at t={t} exceeds the requested speed {}",
                sample.velocity,
                profile.speed_mm_s()
            );
            prop_assert!(
                sample.acceleration.abs()
                    <= case.accel_mm_s2 + RELATIVE_SLACK * case.accel_mm_s2.max(1.0),
                "acceleration {} at t={t} exceeds the budget {}",
                sample.acceleration,
                case.accel_mm_s2
            );
            prop_assert!(
                sign * (sample.position - previous_position) >= -ENDPOINT_SLACK * case.delta_mm.abs(),
                "position {} at t={t} moved back from {previous_position}",
                sample.position
            );
            previous_position = sample.position;
            prop_assert_eq!(profile.jerk(t), 0.0);
        }
    }

    #[test]
    fn nudge_phases_reconstruct_the_displacement(case in arb_nudge_case()) {
        let profile = case.profile();
        let breakpoints = profile.breakpoints();
        let (accel_time, cruise_time) = match breakpoints.len() {
            2 => (0.0, profile.duration()),
            3 => ((breakpoints[1] - breakpoints[0]), 0.0),
            _ => (
                breakpoints[1] - breakpoints[0],
                breakpoints[2] - breakpoints[1],
            ),
        };
        let peak_speed = if cruise_time > 0.0 {
            case.speed_mm_s.abs()
        } else {
            case.accel_mm_s2 * accel_time
        };
        let reconstructed = case.accel_mm_s2 * accel_time * accel_time + peak_speed * cruise_time;
        let time_ulp = 8.0 * f64::EPSILON * profile.t_end().abs().max(1.0);
        let slack = ENDPOINT_SLACK * case.delta_mm.abs() + 3.0 * profile.speed_mm_s() * time_ulp;
        prop_assert!(
            (reconstructed - case.delta_mm.abs()).abs() <= slack,
            "phases cover {reconstructed} of the {} the nudge must travel (slack {slack})",
            case.delta_mm.abs()
        );
        prop_assert!(
            2.0 * accel_time + cruise_time
                <= profile.duration() * (1.0 + ENDPOINT_SLACK) + ENDPOINT_SLACK,
            "phases last longer than the profile"
        );
    }

    #[test]
    fn nudge_position_differentiates_to_its_velocity(case in arb_nudge_case()) {
        let profile = case.profile();
        let (t_start, t_end) = (profile.t_start(), profile.t_end());
        for &fraction in &case.fractions {
            let t = t_start + fraction * (t_end - t_start);
            let reach = profile
                .breakpoints()
                .iter()
                .fold(f64::INFINITY, |nearest, knot| nearest.min((t - knot).abs()));
            let step = 0.25 * reach;
            if step <= 0.0 {
                continue;
            }
            let (lo, hi) = (t - step, t + step);
            let numeric = (profile.position(hi) - profile.position(lo)) / (hi - lo);
            let exact = profile.velocity(t);
            let cancellation = 8.0
                * f64::EPSILON
                * (profile.position(lo).abs() + profile.position(hi).abs())
                / (hi - lo);
            prop_assert!(
                (numeric - exact).abs() <= DERIVATIVE_SLACK * exact.abs() + cancellation,
                "numeric velocity {numeric} at t={t} disagrees with {exact}"
            );
        }
    }

    #[test]
    fn buzz_stays_inside_its_reported_envelope(case in arb_buzz_case()) {
        let profile = case.profile();
        let (t_start, t_end) = (profile.t_start(), profile.t_end());
        let zero = ProfileSample {
            position: 0.0,
            velocity: 0.0,
            acceleration: 0.0,
        };
        prop_assert_eq!(profile.eval(t_start), zero);
        prop_assert_eq!(profile.eval(t_end), zero);

        let (velocity_min, velocity_max) = profile.velocity_bounds();
        let (acceleration_min, acceleration_max) = profile.acceleration_bounds();
        let ceiling = case.position_ceiling();
        let times = sample_times(profile.breakpoints(), t_start, t_end, &case.fractions);
        let scale = velocity_max
            .abs()
            .max(velocity_min.abs())
            .max(acceleration_max.abs())
            .max(acceleration_min.abs())
            .max(ceiling)
            .max(1.0);
        let slack = RELATIVE_SLACK * scale;
        for &t in &times {
            let sample = profile.eval(t);
            prop_assert!(
                sample.position.abs() <= ceiling + slack,
                "position {} at t={t} leaves the {ceiling} envelope",
                sample.position
            );
            prop_assert!(
                sample.velocity >= velocity_min - slack && sample.velocity <= velocity_max + slack,
                "velocity {} at t={t} escapes [{velocity_min}, {velocity_max}]",
                sample.velocity
            );
            prop_assert!(
                sample.acceleration >= acceleration_min - slack
                    && sample.acceleration <= acceleration_max + slack,
                "acceleration {} at t={t} escapes [{acceleration_min}, {acceleration_max}]",
                sample.acceleration
            );
        }

        for &knot in profile.breakpoints() {
            if knot <= t_start || knot >= t_end {
                continue;
            }
            let at = profile.eval(knot).position;
            for neighbour in [interior_time_below(knot), interior_time_above(knot)] {
                prop_assert!(
                    (profile.eval(neighbour).position - at).abs() <= slack,
                    "position steps at the knee {knot}: {} vs {at}",
                    profile.eval(neighbour).position
                );
            }
        }
    }

    #[test]
    fn buzz_position_differentiates_to_its_velocity(case in arb_buzz_case()) {
        let profile = case.profile();
        let (t_start, t_end) = (profile.t_start(), profile.t_end());
        let step = case.derivative_step();
        for &fraction in &case.fractions {
            let t = t_start + fraction * (t_end - t_start);
            let far_from_knots = profile
                .breakpoints()
                .iter()
                .all(|knot| (t - knot).abs() > 4.0 * step);
            if !far_from_knots {
                continue;
            }
            let (lo, hi) = (t - step, t + step);
            let numeric = (profile.position(hi) - profile.position(lo)) / (hi - lo);
            let exact = profile.velocity(t);
            let velocity_scale = case.position_ceiling() * 2.0 * PI * case.freq_start_hz;
            prop_assert!(
                (numeric - exact).abs() <= DERIVATIVE_SLACK * exact.abs().max(velocity_scale),
                "numeric velocity {numeric} at t={t} disagrees with {exact}"
            );
        }
    }

    #[test]
    fn profile_axis_bounds_contain_every_sample(
        nudge in arb_nudge_case(),
        buzz in arb_buzz_case(),
    ) {
        for axis in [
            ContinuousAxis::Nudge(nudge.profile()),
            ContinuousAxis::Buzz {
                base_position: buzz.base_position,
                sign: buzz.sign,
                profile: Arc::new(buzz.profile()),
            },
        ] {
            let fractions = match &axis {
                ContinuousAxis::Nudge(_) => &nudge.fractions,
                _ => &buzz.fractions,
            };
            let window = match &axis {
                ContinuousAxis::Nudge(_) => nudge.window,
                _ => buzz.window,
            };
            let span = motor_span(axis.clone());
            let (t0, t1) = window_of(axis.domain(), window);
            let times = sample_times(&axis.breakpoints(), t0, t1, fractions);
            for source in [BoundsSource::Axis(&axis), BoundsSource::Span(&span)] {
                let bounds = source.bounds(t0, t1).expect("bounds over a sub-interval");
                let samples = times
                    .iter()
                    .map(|&t| source.sample(t).expect("a sample inside the domain"))
                    .collect::<Vec<Pva>>();
                check_bounds_contain_samples(&bounds, &times, &samples, t0)?;
            }
        }
    }

    #[test]
    fn degenerate_profiles_are_rejected(case in arb_nudge_case()) {
        prop_assert_eq!(
            NudgeProfile::try_new(0.0, case.speed_mm_s, case.accel_mm_s2, case.t_start).err(),
            Some(ProfileError::ZeroDisplacement)
        );
        prop_assert_eq!(
            NudgeProfile::try_new(case.delta_mm, 0.0, case.accel_mm_s2, case.t_start).err(),
            Some(ProfileError::ZeroSpeed)
        );
        prop_assert_eq!(
            NudgeProfile::try_new(
                case.delta_mm,
                case.speed_mm_s,
                -case.accel_mm_s2 - 1.0,
                case.t_start
            )
            .err(),
            Some(ProfileError::NegativeAcceleration)
        );
        prop_assert_eq!(
            BuzzProfile::try_new(1.0, 0.0, 1.0, 1.0, 0.0, case.t_start).err(),
            Some(ProfileError::NonPositiveFrequency)
        );
        prop_assert_eq!(
            BuzzProfile::try_new(1.0, 1.0, 1.0, 0.0, 0.0, case.t_start).err(),
            Some(ProfileError::NonPositiveDuration)
        );
        prop_assert_eq!(
            BuzzProfile::try_new(1.0, 1.0, 1.0, 1.0, -1.0, case.t_start).err(),
            Some(ProfileError::NegativeRamp)
        );
        for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            prop_assert!(
                NudgeProfile::try_new(bad, case.speed_mm_s, case.accel_mm_s2, case.t_start)
                    .is_err()
            );
            prop_assert!(
                NudgeProfile::try_new(case.delta_mm, bad, case.accel_mm_s2, case.t_start).is_err()
            );
            prop_assert!(BuzzProfile::try_new(1.0, 1.0, bad, 1.0, 0.0, case.t_start).is_err());
            prop_assert!(BuzzProfile::try_new(1.0, 1.0, 1.0, 1.0, bad, case.t_start).is_err());
        }
    }
}

enum BoundsSource<'a> {
    Axis(&'a ContinuousAxis),
    Span(&'a MotorSpan),
}

impl BoundsSource<'_> {
    fn bounds(&self, t0: f64, t1: f64) -> Result<PvaBounds, trajectory::ContinuousError> {
        match self {
            Self::Axis(axis) => axis.pva_bounds(t0, t1),
            Self::Span(span) => span.pva_bounds(t0, t1),
        }
    }

    fn sample(&self, t: f64) -> Result<Pva, trajectory::ContinuousError> {
        match self {
            Self::Axis(axis) => axis.eval_pva(t),
            Self::Span(span) => span.eval_pva(t),
        }
    }
}
