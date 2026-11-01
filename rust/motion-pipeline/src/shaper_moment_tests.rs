//! The moment-translation and Taylor-shift kernels against the versions
//! they replaced, which must agree bit for bit.

use super::{MOMENT_POWER_CAPACITY, accumulate_translated_moments, taylor_shift};

fn accumulate_translated_moments_dividing_per_term(
    source_origin: f64,
    source: &[f64],
    target_origin: f64,
    target: &mut [f64],
) {
    assert_eq!(source.len(), target.len());
    assert!(target.len() <= MOMENT_POWER_CAPACITY);
    let delta = source_origin - target_origin;
    let mut delta_powers = [1.0; MOMENT_POWER_CAPACITY];
    for power in 1..target.len() {
        delta_powers[power] = delta_powers[power - 1] * delta;
    }
    for (moment_power, moment) in target.iter_mut().enumerate() {
        let mut choose = 1.0;
        for source_power in 0..=moment_power {
            *moment += choose * delta_powers[moment_power - source_power] * source[source_power];
            if source_power < moment_power {
                choose *= (moment_power - source_power) as f64 / (source_power + 1) as f64;
            }
        }
    }
}

fn taylor_shift_copying_down(coefficients: &[f64], shift: f64, shifted: &mut [f64]) {
    assert_eq!(coefficients.len(), shifted.len());
    assert!(coefficients.len() <= MOMENT_POWER_CAPACITY);
    let mut work = [0.0; MOMENT_POWER_CAPACITY];
    work[..coefficients.len()].copy_from_slice(coefficients);
    let mut remaining = coefficients.len();
    for target in shifted.iter_mut() {
        for power in (1..remaining).rev() {
            work[power - 1] = nurbs::fmadd(shift, work[power], work[power - 1]);
        }
        *target = work[0];
        work.copy_within(1..remaining, 0);
        remaining -= 1;
    }
}

struct Lcg(u64);

impl Lcg {
    fn next_f64(&mut self) -> f64 {
        self.0 = self
            .0
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        let unit = (self.0 >> 11) as f64 / (1u64 << 53) as f64;
        let magnitude = 10f64.powi((self.0 % 13) as i32 - 6);
        (2.0 * unit - 1.0) * magnitude
    }

    fn values(&mut self, len: usize) -> Vec<f64> {
        (0..len).map(|_| self.next_f64()).collect()
    }
}

fn bits(values: &[f64]) -> Vec<u64> {
    values.iter().map(|v| v.to_bits()).collect()
}

#[test]
fn translating_moments_through_the_binomial_table_matches_the_running_product() {
    let mut rng = Lcg(7);
    for len in 1..=MOMENT_POWER_CAPACITY {
        for _ in 0..200 {
            let source = rng.values(len);
            let (source_origin, target_origin) = (rng.next_f64(), rng.next_f64());
            let start = rng.values(len);
            let mut table = start.clone();
            let mut running = start;
            accumulate_translated_moments(source_origin, &source, target_origin, &mut table);
            accumulate_translated_moments_dividing_per_term(
                source_origin,
                &source,
                target_origin,
                &mut running,
            );
            assert_eq!(bits(&table), bits(&running), "len {len}");
        }
    }
}

#[test]
fn shifting_in_place_matches_shifting_by_copying_down() {
    let mut rng = Lcg(11);
    for len in 1..=MOMENT_POWER_CAPACITY {
        for _ in 0..200 {
            let coefficients = rng.values(len);
            let shift = rng.next_f64();
            let mut in_place = vec![0.0; len];
            let mut copied = vec![0.0; len];
            taylor_shift(&coefficients, shift, &mut in_place);
            taylor_shift_copying_down(&coefficients, shift, &mut copied);
            assert_eq!(bits(&in_place), bits(&copied), "len {len}");
        }
    }
}
