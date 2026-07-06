#![allow(unsafe_code)]

#[derive(Debug, Clone, PartialEq)]
pub struct ArcLengthTable {
    s: Vec<f64>,
    u: Vec<f64>,
}

impl ArcLengthTable {
    #[must_use]
    pub fn new(s: Vec<f64>, u: Vec<f64>) -> Self {
        debug_assert_eq!(s.len(), u.len());
        debug_assert!(s.len() >= 2);
        Self { s, u }
    }

    #[must_use]
    pub fn s(&self) -> &[f64] {
        &self.s
    }
    #[must_use]
    pub fn u(&self) -> &[f64] {
        &self.u
    }
    #[must_use]
    pub fn s_max(&self) -> f64 {
        *self.s.last().expect("table is non-empty")
    }
    #[must_use]
    pub fn u_max(&self) -> f64 {
        *self.u.last().expect("table is non-empty")
    }
    #[must_use]
    pub fn sample_count(&self) -> usize {
        self.s.len()
    }

    #[inline]
    #[must_use]
    pub fn as_view(&self) -> ArcLengthTableRef<'_> {
        ArcLengthTableRef {
            s: &self.s,
            u: &self.u,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct ArcLengthTableRef<'a> {
    pub(crate) s: &'a [f64],
    pub(crate) u: &'a [f64],
}

impl<'a> ArcLengthTableRef<'a> {
    pub fn new(s: &'a [f64], u: &'a [f64]) -> Self {
        debug_assert_eq!(s.len(), u.len());
        debug_assert!(s.len() >= 2);
        Self { s, u }
    }

    #[must_use]
    pub fn s(&self) -> &[f64] {
        self.s
    }
    #[must_use]
    pub fn u(&self) -> &[f64] {
        self.u
    }
    #[must_use]
    pub fn s_max(&self) -> f64 {
        *self.s.last().expect("table is non-empty")
    }
    #[must_use]
    pub fn u_max(&self) -> f64 {
        *self.u.last().expect("table is non-empty")
    }
}

const GAUSS_LEGENDRE_5_NODES: [f64; 5] = [
    -0.906_179_845_938_664,
    -0.538_469_310_105_683_1,
    0.0,
    0.538_469_310_105_683_1,
    0.906_179_845_938_664,
];
const GAUSS_LEGENDRE_5_WEIGHTS: [f64; 5] = [
    0.236_926_885_056_189_1,
    0.478_628_670_499_366_5,
    0.568_888_888_888_888_9,
    0.478_628_670_499_366_5,
    0.236_926_885_056_189_1,
];

pub(crate) fn integrate_arc_length<F: Fn(f64) -> f64>(
    integrand: F,
    u_start: f64,
    u_end: f64,
    quadrature_points: usize,
) -> f64 {
    debug_assert_eq!(
        quadrature_points, 5,
        "v1 supports only 5-point Gauss-Legendre"
    );

    let half_range = (u_end - u_start) * (0.5);
    let midpoint = (u_start + u_end) * (0.5);

    let mut sum = 0.0;
    for i in 0..5 {
        let node = GAUSS_LEGENDRE_5_NODES[i];
        let weight = GAUSS_LEGENDRE_5_WEIGHTS[i];
        let u = midpoint + half_range * node;
        sum = integrand(u).mul_add(weight, sum);
    }

    sum * half_range
}

use crate::MIN_PARAMETRIC_SPEED;
use crate::eval::{vector_derivative, vector_eval};
use crate::{ArcLengthError, VectorNurbsView};

#[inline]
pub fn param_from_arc_length(table: &ArcLengthTableRef<'_>, s: f64) -> f64 {
    debug_assert!(s >= 0.0);
    debug_assert!(s <= table.s_max());
    let s_clamped = s.max(0.0).min(table.s_max());

    let s_arr = table.s();
    let u_arr = table.u();
    debug_assert!(s_arr.len() >= 2);
    debug_assert_eq!(s_arr.len(), u_arr.len());
    if s_clamped <= unsafe { *s_arr.get_unchecked(0) } {
        return unsafe { *u_arr.get_unchecked(0) };
    }
    let last = s_arr.len() - 1;
    if s_clamped >= unsafe { *s_arr.get_unchecked(last) } {
        return unsafe { *u_arr.get_unchecked(last) };
    }

    let mut lo = 0usize;
    let mut hi = last;
    while hi - lo > 1 {
        let mid = usize::midpoint(lo, hi);
        if unsafe { *s_arr.get_unchecked(mid) } <= s_clamped {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    debug_assert!(lo < last);
    let s_lo = unsafe { *s_arr.get_unchecked(lo) };
    let s_hi = unsafe { *s_arr.get_unchecked(lo + 1) };
    let u_lo = unsafe { *u_arr.get_unchecked(lo) };
    let u_hi = unsafe { *u_arr.get_unchecked(lo + 1) };

    let span = s_hi - s_lo;
    let floor = MIN_PARAMETRIC_SPEED;
    let frac = (s_clamped - s_lo) / span.max(floor);
    u_lo + (u_hi - u_lo) * frac
}

#[inline]
pub fn arc_length_from_param(table: &ArcLengthTableRef<'_>, u: f64) -> f64 {
    debug_assert!(u >= 0.0);
    debug_assert!(u <= table.u_max());
    let u_clamped = u.max(0.0).min(table.u_max());

    let s_arr = table.s();
    let u_arr = table.u();
    debug_assert!(u_arr.len() >= 2);
    debug_assert_eq!(s_arr.len(), u_arr.len());
    if u_clamped <= unsafe { *u_arr.get_unchecked(0) } {
        return unsafe { *s_arr.get_unchecked(0) };
    }
    let last = u_arr.len() - 1;
    if u_clamped >= unsafe { *u_arr.get_unchecked(last) } {
        return unsafe { *s_arr.get_unchecked(last) };
    }

    let mut lo = 0usize;
    let mut hi = last;
    while hi - lo > 1 {
        let mid = usize::midpoint(lo, hi);
        if unsafe { *u_arr.get_unchecked(mid) } <= u_clamped {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    debug_assert!(lo < last);
    let u_lo = unsafe { *u_arr.get_unchecked(lo) };
    let u_hi = unsafe { *u_arr.get_unchecked(lo + 1) };
    let s_lo = unsafe { *s_arr.get_unchecked(lo) };
    let s_hi = unsafe { *s_arr.get_unchecked(lo + 1) };

    let span = u_hi - u_lo;
    let floor = MIN_PARAMETRIC_SPEED;
    let frac = (u_clamped - u_lo) / span.max(floor);
    s_lo + (s_hi - s_lo) * frac
}

pub fn build_arc_length_table_vector<V: VectorNurbsView<3>>(
    curve: &V,
    tolerance: f64,
    max_samples: usize,
) -> Result<ArcLengthTable, ArcLengthError> {
    let h = 1e-6;
    let knots = curve.knots();
    let u_start = knots[0];
    let u_end = knots[knots.len() - 1];

    let integrand = |u: f64| {
        let u_safe = u.max(u_start + h).min(u_end - h);
        let plus = vector_eval(curve, u_safe + h);
        let minus = vector_eval(curve, u_safe - h);
        let two_h = h + h;
        let dx = (plus[0] - minus[0]) / two_h;
        let dy = (plus[1] - minus[1]) / two_h;
        let dz = (plus[2] - minus[2]) / two_h;
        (dx * dx + dy * dy + dz * dz).sqrt()
    };

    build_table_via_integrand(integrand, u_start, u_end, tolerance, max_samples)
}

#[must_use]
pub fn path_arc_length(xyz: &crate::VectorNurbs<3>) -> f64 {
    let knots = xyz.knots();
    let u_start = knots[0];
    let u_end = knots[knots.len() - 1];

    let deriv = vector_derivative(xyz);

    let speed = |u: f64| -> f64 {
        let d = vector_eval(&deriv, u);
        (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt()
    };

    let span = u_end - u_start;
    let mut prev_estimate: Option<f64> = None;
    let mut subintervals: usize = 1;

    loop {
        let mut sum = 0.0_f64;
        for i in 0..subintervals {
            let a = u_start + span * (i as f64) / (subintervals as f64);
            let b = u_start + span * ((i + 1) as f64) / (subintervals as f64);
            sum += integrate_arc_length(speed, a, b, 5);
        }

        if let Some(prev) = prev_estimate {
            let tol = 1e-9 * sum.abs().max(1e-300);
            if (sum - prev).abs() < tol {
                return sum;
            }
        }

        if subintervals >= 64 {
            return sum;
        }

        prev_estimate = Some(sum);
        subintervals *= 2;
    }
}

fn build_table_via_integrand<F: Fn(f64) -> f64 + Copy>(
    integrand: F,
    u_start: f64,
    u_end: f64,
    tolerance: f64,
    max_samples: usize,
) -> Result<ArcLengthTable, ArcLengthError> {
    let mut count = 8;
    loop {
        let mut u_samples: Vec<f64> = Vec::with_capacity(count);
        let mut s_samples: Vec<f64> = Vec::with_capacity(count);

        let span = u_end - u_start;
        for i in 0..count {
            let frac = i as f64 / (count - 1) as f64;
            u_samples.push(u_start + span * frac);
        }

        s_samples.push(0.0);
        for i in 1..count {
            let segment_length = integrate_arc_length(integrand, u_samples[i - 1], u_samples[i], 5);
            let prev = s_samples[i - 1];
            s_samples.push(prev + segment_length);
        }

        let span_full = u_end - u_start;
        let s_refined: f64 = {
            let count_refined = (count - 1) * 2 + 1;
            let mut acc = 0.0;
            for i in 1..count_refined {
                let a = u_start + span_full * ((i - 1) as f64 / (count_refined - 1) as f64);
                let b = u_start + span_full * (i as f64 / (count_refined - 1) as f64);
                acc += integrate_arc_length(integrand, a, b, 5);
            }
            acc
        };

        let residual = (s_samples[count - 1] - s_refined).abs();
        if residual <= tolerance {
            let s_total = *s_samples.last().expect("s_samples is non-empty");
            if s_total <= (MIN_PARAMETRIC_SPEED) {
                return Err(ArcLengthError::DegenerateCurve);
            }
            return Ok(ArcLengthTable::new(s_samples, u_samples));
        }
        if count * 2 > max_samples {
            return Err(ArcLengthError::ToleranceNotMet {
                achieved_residual: residual,
                samples_used: count,
            });
        }
        count *= 2;
    }
}

#[cfg(test)]
mod tests;
