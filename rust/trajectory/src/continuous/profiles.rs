use core::f64::consts::PI;

#[derive(Clone, Copy, Debug, PartialEq, thiserror::Error)]
pub enum ProfileError {
    #[error("{0} must be finite")]
    NonFinite(&'static str),
    #[error("displacement must be nonzero")]
    ZeroDisplacement,
    #[error("speed must be nonzero")]
    ZeroSpeed,
    #[error("acceleration must be nonnegative")]
    NegativeAcceleration,
    #[error("frequency must be positive")]
    NonPositiveFrequency,
    #[error("duration must be positive")]
    NonPositiveDuration,
    #[error("ramp duration must be nonnegative")]
    NegativeRamp,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ProfileSample {
    pub position: f64,
    pub velocity: f64,
    pub acceleration: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct NudgeProfile {
    delta_mm: f64,
    speed_mm_s: f64,
    accel_mm_s2: f64,
    t_start: f64,
    t_accel: f64,
    t_cruise: f64,
    duration: f64,
    peak_speed_mm_s: f64,
    breakpoints: Vec<f64>,
}

impl NudgeProfile {
    pub fn try_new(
        delta_mm: f64,
        speed_mm_s: f64,
        accel_mm_s2: f64,
        t_start: f64,
    ) -> Result<Self, ProfileError> {
        require_finite(delta_mm, "delta_mm")?;
        require_finite(speed_mm_s, "speed_mm_s")?;
        require_finite(accel_mm_s2, "accel_mm_s2")?;
        require_finite(t_start, "t_start")?;
        if delta_mm == 0.0 {
            return Err(ProfileError::ZeroDisplacement);
        }
        if speed_mm_s == 0.0 {
            return Err(ProfileError::ZeroSpeed);
        }
        if accel_mm_s2 < 0.0 {
            return Err(ProfileError::NegativeAcceleration);
        }

        let distance = delta_mm.abs();
        let requested_speed = speed_mm_s.abs();
        let (t_accel, t_cruise, duration, peak_speed_mm_s) = if accel_mm_s2 == 0.0 {
            let duration = distance / requested_speed;
            (0.0, duration, duration, requested_speed)
        } else {
            let speed_limited_accel_time = requested_speed / accel_mm_s2;
            let speed_limited_accel_distance = accel_mm_s2 * speed_limited_accel_time.powi(2);
            if speed_limited_accel_distance >= distance {
                let t_accel = (distance / accel_mm_s2).sqrt();
                (t_accel, 0.0, 2.0 * t_accel, accel_mm_s2 * t_accel)
            } else {
                let t_cruise = (distance - speed_limited_accel_distance) / requested_speed;
                (
                    speed_limited_accel_time,
                    t_cruise,
                    2.0 * speed_limited_accel_time + t_cruise,
                    requested_speed,
                )
            }
        };
        if !duration.is_finite() || duration <= 0.0 || !peak_speed_mm_s.is_finite() {
            return Err(ProfileError::NonFinite("derived nudge duration"));
        }
        let t_end = t_start + duration;
        if !t_end.is_finite() {
            return Err(ProfileError::NonFinite("nudge end time"));
        }
        let mut breakpoints = Vec::with_capacity(4);
        breakpoints.push(t_start);
        if accel_mm_s2 > 0.0 {
            breakpoints.push(t_start + t_accel);
            if t_cruise > 0.0 {
                breakpoints.push(t_start + t_accel + t_cruise);
            }
        }
        breakpoints.push(t_end);

        Ok(Self {
            delta_mm,
            speed_mm_s: requested_speed,
            accel_mm_s2,
            t_start,
            t_accel,
            t_cruise,
            duration,
            peak_speed_mm_s,
            breakpoints,
        })
    }

    #[must_use]
    pub fn eval(&self, t: f64) -> ProfileSample {
        self.validate_eval_time(t);
        if t == self.t_start {
            return ProfileSample {
                position: 0.0,
                velocity: 0.0,
                acceleration: 0.0,
            };
        }
        if t == self.t_end() {
            return ProfileSample {
                position: self.delta_mm,
                velocity: 0.0,
                acceleration: 0.0,
            };
        }
        let sign = self.delta_mm.signum();
        let local_t = t - self.t_start;
        if self.accel_mm_s2 == 0.0 {
            return ProfileSample {
                position: sign * self.speed_mm_s * local_t,
                velocity: sign * self.speed_mm_s,
                acceleration: 0.0,
            };
        }
        if local_t < self.t_accel {
            return ProfileSample {
                position: sign * 0.5 * self.accel_mm_s2 * local_t * local_t,
                velocity: sign * self.accel_mm_s2 * local_t,
                acceleration: sign * self.accel_mm_s2,
            };
        }
        let accel_distance = 0.5 * self.accel_mm_s2 * self.t_accel * self.t_accel;
        if local_t < self.t_accel + self.t_cruise {
            let cruise_t = local_t - self.t_accel;
            return ProfileSample {
                position: sign * (accel_distance + self.peak_speed_mm_s * cruise_t),
                velocity: sign * self.peak_speed_mm_s,
                acceleration: 0.0,
            };
        }
        let remaining = self.t_end() - t;
        ProfileSample {
            position: self.delta_mm - sign * 0.5 * self.accel_mm_s2 * remaining * remaining,
            velocity: sign * self.accel_mm_s2 * remaining,
            acceleration: -sign * self.accel_mm_s2,
        }
    }

    #[must_use]
    pub fn position(&self, t: f64) -> f64 {
        self.eval(t).position
    }

    #[must_use]
    pub fn velocity(&self, t: f64) -> f64 {
        self.eval(t).velocity
    }

    #[must_use]
    pub fn acceleration(&self, t: f64) -> f64 {
        self.eval(t).acceleration
    }

    /// The nudge acceleration is piecewise constant, so the jerk is exactly zero
    /// everywhere inside a phase; the phase boundaries carry impulses instead.
    #[must_use]
    pub fn jerk(&self, t: f64) -> f64 {
        self.validate_eval_time(t);
        0.0
    }

    #[must_use]
    pub fn delta_mm(&self) -> f64 {
        self.delta_mm
    }

    #[must_use]
    pub fn speed_mm_s(&self) -> f64 {
        self.speed_mm_s
    }

    #[must_use]
    pub fn accel_mm_s2(&self) -> f64 {
        self.accel_mm_s2
    }

    #[must_use]
    pub fn t_start(&self) -> f64 {
        self.t_start
    }

    #[must_use]
    pub fn duration(&self) -> f64 {
        self.duration
    }

    #[must_use]
    pub fn t_end(&self) -> f64 {
        self.t_start + self.duration
    }

    #[must_use]
    pub fn end_time(&self) -> f64 {
        self.t_end()
    }

    #[must_use]
    pub fn breakpoints(&self) -> &[f64] {
        &self.breakpoints
    }

    /// A nudge without acceleration cruises at `sign*speed` from the first
    /// instant, while `eval` parks a zero sample at both domain ends: velocity
    /// jumps at each end. A ramped nudge leaves and lands at rest, so its
    /// velocity is continuous everywhere.
    #[must_use]
    pub fn velocity_step_inside(&self, t0: f64, t1: f64) -> bool {
        self.accel_mm_s2 == 0.0
            && [self.t_start, self.t_end()]
                .into_iter()
                .any(|end| end >= t0 && end <= t1)
    }

    #[must_use]
    pub fn velocity_bounds(&self) -> (f64, f64) {
        let signed_peak = self.delta_mm.signum() * self.peak_speed_mm_s;
        (signed_peak.min(0.0), signed_peak.max(0.0))
    }

    #[must_use]
    pub fn acceleration_bounds(&self) -> (f64, f64) {
        if self.accel_mm_s2 == 0.0 {
            (0.0, 0.0)
        } else {
            (-self.accel_mm_s2, self.accel_mm_s2)
        }
    }

    fn validate_eval_time(&self, t: f64) {
        assert!(t.is_finite(), "evaluation time must be finite");
        assert!(
            t >= self.t_start && t <= self.t_end(),
            "evaluation time is outside the nudge profile"
        );
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
enum EnvelopeInterval {
    Rising,
    Flat,
    Falling,
}

#[derive(Clone, Copy)]
enum Derivative {
    Velocity,
    Acceleration,
    Jerk,
}

#[derive(Clone, Debug, PartialEq)]
pub struct BuzzProfile {
    amplitude_mm: f64,
    omega_start: f64,
    sweep_rate: f64,
    duration: f64,
    ramp: f64,
    t_start: f64,
    breakpoints: Vec<f64>,
    velocity_extrema_times: Vec<f64>,
    acceleration_extrema_times: Vec<f64>,
    velocity_bounds: (f64, f64),
    acceleration_bounds: (f64, f64),
}

impl BuzzProfile {
    pub fn try_new(
        amplitude_mm: f64,
        freq_start_hz: f64,
        freq_end_hz: f64,
        duration: f64,
        ramp: f64,
        t_start: f64,
    ) -> Result<Self, ProfileError> {
        require_finite(amplitude_mm, "amplitude_mm")?;
        require_finite(freq_start_hz, "freq_start_hz")?;
        require_finite(freq_end_hz, "freq_end_hz")?;
        require_finite(duration, "duration")?;
        require_finite(ramp, "ramp")?;
        require_finite(t_start, "t_start")?;
        if freq_start_hz <= 0.0 || freq_end_hz <= 0.0 {
            return Err(ProfileError::NonPositiveFrequency);
        }
        if duration <= 0.0 {
            return Err(ProfileError::NonPositiveDuration);
        }
        if ramp < 0.0 {
            return Err(ProfileError::NegativeRamp);
        }
        let t_end = t_start + duration;
        if !t_end.is_finite() {
            return Err(ProfileError::NonFinite("buzz end time"));
        }
        let omega_start = 2.0 * PI * freq_start_hz;
        let omega_end = 2.0 * PI * freq_end_hz;
        let sweep_rate = (omega_end - omega_start) / duration;
        if !omega_start.is_finite() || !omega_end.is_finite() || !sweep_rate.is_finite() {
            return Err(ProfileError::NonFinite("derived buzz frequency"));
        }
        let ramp = ramp.min(0.5 * duration);
        let mut breakpoints = Vec::with_capacity(4);
        breakpoints.push(t_start);
        let first_knee = t_start + ramp;
        let second_knee = t_end - ramp;
        if first_knee > t_start && first_knee < t_end {
            breakpoints.push(first_knee);
        }
        if second_knee > first_knee && second_knee < t_end {
            breakpoints.push(second_knee);
        }
        breakpoints.push(t_end);

        let mut profile = Self {
            amplitude_mm,
            omega_start,
            sweep_rate,
            duration,
            ramp,
            t_start,
            breakpoints,
            velocity_extrema_times: Vec::new(),
            acceleration_extrema_times: Vec::new(),
            velocity_bounds: (0.0, 0.0),
            acceleration_bounds: (0.0, 0.0),
        };
        profile.velocity_extrema_times = profile.isolate_extrema(Derivative::Acceleration);
        profile.acceleration_extrema_times = profile.isolate_extrema(Derivative::Jerk);
        profile.velocity_bounds = profile.compute_bounds(Derivative::Velocity);
        profile.acceleration_bounds = profile.compute_bounds(Derivative::Acceleration);
        Ok(profile)
    }

    #[must_use]
    pub fn eval(&self, t: f64) -> ProfileSample {
        self.validate_eval_time(t);
        if self.at_a_domain_end(t) {
            return ProfileSample {
                position: 0.0,
                velocity: 0.0,
                acceleration: 0.0,
            };
        }
        let local_t = t - self.t_start;
        let interval = self.interval_at(local_t);
        self.sample_local(local_t, interval)
    }

    /// `t_end() - t_start` is only `duration` when the sum is exact, so the
    /// parked ends are recognised by the times themselves.
    fn at_a_domain_end(&self, t: f64) -> bool {
        t == self.t_start || t == self.t_end()
    }

    #[must_use]
    pub fn position(&self, t: f64) -> f64 {
        self.eval(t).position
    }

    #[must_use]
    pub fn velocity(&self, t: f64) -> f64 {
        self.eval(t).velocity
    }

    #[must_use]
    pub fn acceleration(&self, t: f64) -> f64 {
        self.eval(t).acceleration
    }

    #[must_use]
    pub fn jerk(&self, t: f64) -> f64 {
        self.validate_eval_time(t);
        if self.at_a_domain_end(t) {
            return 0.0;
        }
        let local_t = t - self.t_start;
        self.jerk_local(local_t, self.interval_at(local_t))
    }

    #[must_use]
    pub fn amplitude_mm(&self) -> f64 {
        self.amplitude_mm
    }

    #[must_use]
    pub fn freq_start_hz(&self) -> f64 {
        self.omega_start / (2.0 * PI)
    }

    #[must_use]
    pub fn freq_end_hz(&self) -> f64 {
        (self.omega_start + self.sweep_rate * self.duration) / (2.0 * PI)
    }

    #[must_use]
    pub fn ramp(&self) -> f64 {
        self.ramp
    }

    #[must_use]
    pub fn t_start(&self) -> f64 {
        self.t_start
    }

    #[must_use]
    pub fn duration(&self) -> f64 {
        self.duration
    }

    #[must_use]
    pub fn t_end(&self) -> f64 {
        self.t_start + self.duration
    }

    #[must_use]
    pub fn end_time(&self) -> f64 {
        self.t_end()
    }

    #[must_use]
    pub fn breakpoints(&self) -> &[f64] {
        &self.breakpoints
    }

    /// The trapezoid envelope's slope steps at each knee, and `eval` parks a
    /// zero sample at both domain ends while the carrier arrives there with
    /// `envelope_rate*A*sin(phase) + envelope*A*omega*cos(phase)`: velocity
    /// jumps at every such instant.
    #[must_use]
    pub fn velocity_step_inside(&self, t0: f64, t1: f64) -> bool {
        self.breakpoints[1..self.breakpoints.len() - 1]
            .iter()
            .any(|&knee| knee > t0 && knee < t1)
            || [0.0, self.duration].into_iter().any(|local_t| {
                let end = self.t_start + local_t;
                end >= t0 && end <= t1 && self.carrier_velocity_at(local_t) != 0.0
            })
    }

    fn carrier_velocity_at(&self, local_t: f64) -> f64 {
        self.sample_local(local_t, self.interval_at(local_t))
            .velocity
    }

    #[must_use]
    pub fn velocity_bounds(&self) -> (f64, f64) {
        self.velocity_bounds
    }

    #[must_use]
    pub fn acceleration_bounds(&self) -> (f64, f64) {
        self.acceleration_bounds
    }

    fn envelope_at(&self, t: f64, interval: EnvelopeInterval) -> (f64, f64) {
        if self.ramp == 0.0 {
            return (1.0, 0.0);
        }
        match interval {
            EnvelopeInterval::Rising => (t / self.ramp, 1.0 / self.ramp),
            EnvelopeInterval::Flat => (1.0, 0.0),
            EnvelopeInterval::Falling => ((self.duration - t) / self.ramp, -1.0 / self.ramp),
        }
    }

    fn sample_local(&self, t: f64, interval: EnvelopeInterval) -> ProfileSample {
        let (envelope, envelope_rate) = self.envelope_at(t, interval);
        let omega = self.omega_start + self.sweep_rate * t;
        let amplitude = if self.sweep_rate == 0.0 {
            self.amplitude_mm
        } else {
            self.amplitude_mm * self.omega_start / omega
        };
        let amplitude_rate = if self.sweep_rate == 0.0 {
            0.0
        } else {
            -self.amplitude_mm * self.omega_start * self.sweep_rate / (omega * omega)
        };
        let amplitude_accel = if self.sweep_rate == 0.0 {
            0.0
        } else {
            2.0 * self.amplitude_mm * self.omega_start * self.sweep_rate.powi(2) / omega.powi(3)
        };
        let phase = (self.omega_start + 0.5 * self.sweep_rate * t) * t;
        let sin = libm::sin(phase);
        let cos = libm::cos(phase);
        let position = envelope * amplitude * sin;
        let velocity = envelope_rate * amplitude * sin
            + envelope * amplitude_rate * sin
            + envelope * amplitude * omega * cos;
        let sin_coeff = 2.0 * envelope_rate * amplitude_rate + envelope * amplitude_accel
            - envelope * amplitude * omega * omega;
        let cos_coeff = 2.0 * envelope_rate * amplitude * omega
            + 2.0 * envelope * amplitude_rate * omega
            + envelope * amplitude * self.sweep_rate;
        ProfileSample {
            position,
            velocity,
            acceleration: sin_coeff * sin + cos_coeff * cos,
        }
    }

    fn interval_at(&self, t: f64) -> EnvelopeInterval {
        if t < self.ramp {
            EnvelopeInterval::Rising
        } else if t > self.duration - self.ramp {
            EnvelopeInterval::Falling
        } else {
            EnvelopeInterval::Flat
        }
    }

    fn isolate_extrema(&self, zero_of: Derivative) -> Vec<f64> {
        let mut extrema = Vec::new();
        extrema.push(self.t_start);
        if self.amplitude_mm == 0.0 {
            extrema.push(self.t_end());
            return extrema;
        }
        for (left, right, interval) in self.envelope_intervals() {
            if right <= left {
                continue;
            }
            self.isolate_zeros(left, right, interval, zero_of, &mut extrema);
            push_distinct(&mut extrema, self.t_start + right);
        }
        extrema
    }

    fn isolate_zeros(
        &self,
        left: f64,
        right: f64,
        interval: EnvelopeInterval,
        derivative: Derivative,
        roots: &mut Vec<f64>,
    ) {
        let phase_span = self.phase(right) - self.phase(left);
        let partitions = (phase_span.abs() / (PI / 16.0)).ceil().max(1.0) as usize;
        let mut x0 = left;
        let mut y0 = self.value_local(x0, interval, derivative);
        for index in 1..=partitions {
            let x1 = left + (right - left) * index as f64 / partitions as f64;
            let y1 = self.value_local(x1, interval, derivative);
            if y0 == 0.0 {
                push_distinct(roots, self.t_start + x0);
            }
            if y0.signum() != y1.signum() {
                let root = self.bisect_zero(x0, x1, y0, interval, derivative);
                push_distinct(roots, self.t_start + root);
            }
            x0 = x1;
            y0 = y1;
        }
        if y0 == 0.0 {
            push_distinct(roots, self.t_start + right);
        }
    }

    fn bisect_zero(
        &self,
        mut left: f64,
        mut right: f64,
        mut left_value: f64,
        interval: EnvelopeInterval,
        derivative: Derivative,
    ) -> f64 {
        for _ in 0..64 {
            let middle = 0.5 * (left + right);
            if middle == left || middle == right {
                break;
            }
            let value = self.value_local(middle, interval, derivative);
            if value == 0.0 {
                return middle;
            }
            if left_value.signum() == value.signum() {
                left = middle;
                left_value = value;
            } else {
                right = middle;
            }
        }
        0.5 * (left + right)
    }

    fn compute_bounds(&self, derivative: Derivative) -> (f64, f64) {
        let extrema = match derivative {
            Derivative::Velocity => &self.velocity_extrema_times,
            Derivative::Acceleration => &self.acceleration_extrema_times,
            Derivative::Jerk => unreachable!(),
        };
        let mut minimum: f64 = 0.0;
        let mut maximum: f64 = 0.0;
        for &time in extrema {
            let local_t = time - self.t_start;
            let value = self.value_local(local_t, self.interval_at(local_t), derivative);
            minimum = minimum.min(value);
            maximum = maximum.max(value);
        }
        for (left, right, interval) in self.envelope_intervals() {
            if right <= left {
                continue;
            }
            for local_t in [left, right] {
                let value = self.value_local(local_t, interval, derivative);
                minimum = minimum.min(value);
                maximum = maximum.max(value);
            }
        }
        (minimum, maximum)
    }

    fn value_local(&self, t: f64, interval: EnvelopeInterval, derivative: Derivative) -> f64 {
        let sample = self.sample_local(t, interval);
        match derivative {
            Derivative::Velocity => sample.velocity,
            Derivative::Acceleration => sample.acceleration,
            Derivative::Jerk => self.jerk_local(t, interval),
        }
    }

    fn jerk_local(&self, t: f64, interval: EnvelopeInterval) -> f64 {
        let (envelope, envelope_rate) = self.envelope_at(t, interval);
        let omega = self.omega_start + self.sweep_rate * t;
        let base = self.amplitude_mm * self.omega_start;
        let (amplitude, amplitude_rate, amplitude_accel, amplitude_jerk) = if self.sweep_rate == 0.0
        {
            (self.amplitude_mm, 0.0, 0.0, 0.0)
        } else {
            (
                base / omega,
                -base * self.sweep_rate / omega.powi(2),
                2.0 * base * self.sweep_rate.powi(2) / omega.powi(3),
                -6.0 * base * self.sweep_rate.powi(3) / omega.powi(4),
            )
        };
        let b = envelope * amplitude;
        let b_rate = envelope_rate * amplitude + envelope * amplitude_rate;
        let b_accel = 2.0 * envelope_rate * amplitude_rate + envelope * amplitude_accel;
        let b_jerk = 3.0 * envelope_rate * amplitude_accel + envelope * amplitude_jerk;
        let sin_coeff = b_jerk - 3.0 * b_rate * omega * omega - 3.0 * b * omega * self.sweep_rate;
        let cos_coeff = 3.0 * b_accel * omega - b * omega.powi(3) + 3.0 * b_rate * self.sweep_rate;
        let phase = self.phase(t);
        sin_coeff * libm::sin(phase) + cos_coeff * libm::cos(phase)
    }

    fn envelope_intervals(&self) -> [(f64, f64, EnvelopeInterval); 3] {
        [
            (0.0, self.ramp, EnvelopeInterval::Rising),
            (self.ramp, self.duration - self.ramp, EnvelopeInterval::Flat),
            (
                self.duration - self.ramp,
                self.duration,
                EnvelopeInterval::Falling,
            ),
        ]
    }

    fn phase(&self, t: f64) -> f64 {
        (self.omega_start + 0.5 * self.sweep_rate * t) * t
    }

    fn validate_eval_time(&self, t: f64) {
        assert!(t.is_finite(), "evaluation time must be finite");
        assert!(
            t >= self.t_start && t <= self.t_end(),
            "evaluation time is outside the buzz profile"
        );
    }
}

fn require_finite(value: f64, name: &'static str) -> Result<(), ProfileError> {
    if value.is_finite() {
        Ok(())
    } else {
        Err(ProfileError::NonFinite(name))
    }
}

fn push_distinct(values: &mut Vec<f64>, value: f64) {
    let tolerance = 32.0 * f64::EPSILON * value.abs().max(1.0);
    if values
        .last()
        .is_none_or(|previous| (value - previous).abs() > tolerance)
    {
        values.push(value);
    }
}
