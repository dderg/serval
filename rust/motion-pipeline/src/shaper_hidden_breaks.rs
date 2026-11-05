//! Which of the shaped signal's breakpoints a leader fit seeds.
//!
//! # The jump a kernel image carries
//!
//! The shaped signal is `s = x ∗ k`: `s(t) = ∫ x(t − σ) k(σ) dσ`, with `x`
//! the piecewise-polynomial input and `k` the piecewise-polynomial kernel.
//! Let `x` jump by `Δx⁽ᵖ⁾` in its `p`-th derivative at an input break `b`,
//! and `k` by `Δk⁽ᵠ⁾` in its `q`-th derivative at a kernel cut `c`. Near `b`
//! write `x` as a locally smooth part plus `Σₚ Δx⁽ᵖ⁾ (t − b)ᵖ/p! · H(t − b)`,
//! and `k` likewise around `c`. A smooth part convolved with anything is
//! smooth at `t* = b + c`, so the only part of `s` that breaks there is
//!
//! ```text
//! ∫ Δx⁽ᵖ⁾ (t−σ−b)ᵖ/p! H(t−σ−b) · Δk⁽ᵠ⁾ (σ−c)ᵠ/q! H(σ−c) dσ
//!     = Δx⁽ᵖ⁾ Δk⁽ᵠ⁾ (t − t*)ᵖ⁺ᵠ⁺¹/(p+q+1)! · H(t − t*)
//! ```
//!
//! by the Beta integral. Summed over both series, the right polynomial of `s`
//! at `t*` minus the left one is `D(t) = Σₘ Jₘ (t − t*)ᵐ/m!` with
//! `Jₘ = Σ_{p+q+1=m} Δx⁽ᵖ⁾ Δk⁽ᵠ⁾`, for `m ≥ 1`: the jump of `s⁽ᵐ⁾` at `t*`.
//!
//! A `J₁` is a velocity step of `s`; the rest bend it smoothly from `t*`.
//!
//! # Which images a leader fit seeds
//!
//! A leader fit is proven against the shaped signal's exact polynomials
//! (`shaper_hull`), which follow every break inside a span, so no image
//! needs a seed for a piece to hold the tolerance, and every piece is held
//! to the whole of it. The partition is left to the fit, which halves a span
//! no rung fits. Three kinds of image stay seeds:
//!
//! - one that may carry a velocity step wide enough for a resolution floor
//!   to call it a velocity break, a knot the plan must anchor: a `J₁` past
//!   the acceleration tolerance carried across the narrowest floor, or
//!   jumps that are not known — an image of more than one pair, of a clamp
//!   edge, or of an input the table alone does not describe;
//! - one where the shaped signal enters or leaves an exact cruise, rest
//!   included: the image of a break into an input holding its velocity
//!   under the kernel's last cut, or out of one under its first. A piece
//!   across it could only approximate the cruise, its velocity wandering
//!   within tolerance where the plan holds it, and past the limit where
//!   the plan cruises at it;
//! - the first image at least `minimum_span` past the last seed. Halving a
//!   span across many images pays a ladder per level for rungs that cannot
//!   reach across it, so the seeds cut a target into spans a rung usually
//!   fits outright, and no finer.

use nurbs::algebra::PiecewisePolynomialKernel;
use trajectory::ShapedSignal;

use super::{
    AxisSignalTable, MIN_DEVICE_INTERVAL_S, MOMENT_POWER_CAPACITY, differentiate_in_place,
};
use crate::lowering::FitTol;

/// Holds the derivative series of an input piece and a kernel piece, and
/// their image's, which runs one order past both together.
const SERIES_CAPACITY: usize = 2 * MOMENT_POWER_CAPACITY;

/// The shortest span between seeds, as a share of the kernel's support:
/// the shaped signal bends on the scale of the kernel, so a span this much
/// shorter than it is one a rung fits outright.
const MINIMUM_SPAN_OF_SUPPORT: f64 = 0.25;

/// The share of the acceleration tolerance under which an input piece's
/// acceleration, bounded over the whole piece, holds its velocity: the
/// residue of a source fit's higher terms along a cruise, far below what
/// any fit resolves.
const CRUISE_ACCELERATION_SHARE: f64 = 1.0 / (1_u64 << 30) as f64;

/// The input break and kernel cut whose sum an output break is.
#[derive(Clone, Copy, Debug)]
struct KernelImage {
    input_break: f64,
    cut: usize,
}

/// Derivative values or jumps, order by order from the zeroth.
#[derive(Clone, Copy, Debug)]
pub(crate) struct JumpSeries {
    orders: [f64; SERIES_CAPACITY],
    len: usize,
}

impl JumpSeries {
    fn derivatives_at(coefficients: &[f64], tau: f64) -> Self {
        assert!(
            coefficients.len() <= MOMENT_POWER_CAPACITY,
            "a {}-coefficient piece exceeds the jump series capacity",
            coefficients.len()
        );
        let mut orders = [0.0; SERIES_CAPACITY];
        let mut derivative = [0.0; MOMENT_POWER_CAPACITY];
        let mut length = coefficients.len();
        derivative[..length].copy_from_slice(coefficients);
        for (order, value) in orders[..coefficients.len()].iter_mut().enumerate() {
            if order > 0 {
                length = differentiate_in_place(&mut derivative, length);
            }
            *value = derivative[..length]
                .iter()
                .rev()
                .fold(0.0_f64, |acc, &c| nurbs::fmadd(acc, tau, c));
        }
        Self {
            orders,
            len: coefficients.len(),
        }
    }

    fn minus(self, other: Self) -> Self {
        let len = self.len.max(other.len);
        let mut orders = [0.0; SERIES_CAPACITY];
        for (order, value) in orders[..len].iter_mut().enumerate() {
            *value = self.orders[order] - other.orders[order];
        }
        Self { orders, len }
    }

    /// `Jₘ = Σ_{p+q+1=m} Δx⁽ᵖ⁾ Δk⁽ᵠ⁾`: the jumps of the shaped signal's
    /// derivatives where this input series meets a kernel series.
    fn image_through(self, kernel: Self) -> Self {
        let len = self.len + kernel.len;
        assert!(
            len <= SERIES_CAPACITY,
            "an image of {len} derivative orders exceeds the jump series capacity"
        );
        let mut orders = [0.0; SERIES_CAPACITY];
        for (input_order, &input_jump) in self.orders[..self.len].iter().enumerate() {
            for (kernel_order, &kernel_jump) in kernel.orders[..kernel.len].iter().enumerate() {
                orders[input_order + kernel_order + 1] += input_jump * kernel_jump;
            }
        }
        Self { orders, len }
    }

    /// Whether the velocity step `J₁` is below every velocity break the plan
    /// can resolve: those exceed the acceleration tolerance carried across a
    /// resolution floor, and no floor is narrower than `MIN_DEVICE_INTERVAL_S`.
    fn below_any_velocity_break(&self, fit_tol: FitTol) -> bool {
        self.len < 2 || self.orders[1].abs() <= fit_tol.accel_mm_s2 * MIN_DEVICE_INTERVAL_S
    }
}

/// Every output break of the shaped signal — each input break moved by each
/// kernel cut, as `output_cut_transitions` places it — with the pair it is
/// the image of, where it is the image of exactly one.
pub(crate) struct ShapedBreaks {
    times: Vec<f64>,
    images: Vec<Option<KernelImage>>,
    kernel_jumps: Vec<JumpSeries>,
    minimum_span: f64,
}

impl ShapedBreaks {
    pub(crate) fn new(kernel: &PiecewisePolynomialKernel, input_breaks: &[f64]) -> Self {
        let cuts: Vec<f64> = ShapedSignal::kernel_cut_boundaries(kernel).collect();
        let mut distinct_breaks = input_breaks.to_vec();
        distinct_breaks.sort_by(f64::total_cmp);
        distinct_breaks.dedup();
        let mut located = Vec::with_capacity(distinct_breaks.len() * cuts.len());
        let mut transition = Vec::with_capacity(1);
        for &input_break in &distinct_breaks {
            for (cut, &kernel_break) in cuts.iter().enumerate() {
                transition.clear();
                ShapedSignal::output_cut_transitions(
                    kernel,
                    input_break,
                    kernel_break,
                    &mut transition,
                );
                let [t] = transition[..] else {
                    panic!("an input break and a kernel cut must meet at one output time")
                };
                located.push((t, KernelImage { input_break, cut }));
            }
        }
        located.sort_by(|left, right| left.0.total_cmp(&right.0));
        let mut times: Vec<f64> = Vec::with_capacity(located.len());
        let mut images = Vec::with_capacity(located.len());
        for (t, image) in located {
            if times.last() == Some(&t) {
                *images.last_mut().expect("images track times") = None;
            } else {
                times.push(t);
                images.push(Some(image));
            }
        }
        let (support_start, support_end) = kernel.support();
        Self {
            times,
            images,
            kernel_jumps: cuts
                .iter()
                .map(|&cut| kernel_jumps_at(kernel, cut))
                .collect(),
            minimum_span: MINIMUM_SPAN_OF_SUPPORT * (support_end - support_start),
        }
    }

    pub(crate) fn times(&self) -> &[f64] {
        &self.times
    }

    fn image_at(&self, t: f64) -> Option<KernelImage> {
        let index = self
            .times
            .binary_search_by(|probe| probe.total_cmp(&t))
            .ok()?;
        self.images[index]
    }

    /// Keeps `seeds`' first and last, every interior seed that may carry a
    /// velocity break or bounds an exact cruise, and the first seed at least
    /// `minimum_span` past the last one kept. `input` is `None` where the
    /// convolution's input is not the table alone, and no seed's jumps are
    /// known.
    pub(crate) fn select_seeds(
        &self,
        seeds: Vec<f64>,
        input: Option<&AxisSignalTable>,
        fit_tol: FitTol,
    ) -> Vec<f64> {
        let Some(input) = input else {
            return seeds;
        };
        let (&last, interior) = seeds[1..].split_last().expect("fit seeds span no interval");
        let mut kept = Vec::with_capacity(seeds.len());
        kept.push(seeds[0]);
        for &t in interior {
            let last_kept = *kept.last().expect("kept seeds start with the first");
            let anchors = self.image_at(t).is_none_or(|image| {
                self.bounds_cruise(input, image, fit_tol)
                    || self
                        .image_jumps(input, image)
                        .is_none_or(|jumps| !jumps.below_any_velocity_break(fit_tol))
            });
            if anchors || t - last_kept >= self.minimum_span {
                kept.push(t);
            }
        }
        kept.push(last);
        kept
    }

    fn image_jumps(&self, input: &AxisSignalTable, image: KernelImage) -> Option<JumpSeries> {
        Some(input_jumps_at(input, image.input_break)?.image_through(self.kernel_jumps[image.cut]))
    }

    /// Whether the shaped signal enters or leaves an exact cruise at
    /// `image`: the input holds its velocity past its break and the
    /// kernel's last cut closes the window behind it, or the input holds it
    /// before the break and the first cut opens the window onto it.
    fn bounds_cruise(&self, input: &AxisSignalTable, image: KernelImage, fit_tol: FitTol) -> bool {
        let cruises = |piece: usize| {
            acceleration_bound(input, piece) <= CRUISE_ACCELERATION_SHARE * fit_tol.accel_mm_s2
        };
        let after = input
            .starts
            .partition_point(|&start| start < image.input_break);
        let enters = image.cut == self.kernel_jumps.len() - 1
            && input.starts.get(after) == Some(&image.input_break)
            && cruises(after);
        let leaves = image.cut == 0
            && after > 0
            && input.ends[after - 1] == image.input_break
            && cruises(after - 1);
        enters || leaves
    }
}

/// `Σₖ k(k−1)|cₖ|ρᵏ⁻²` over input piece `piece`, with `ρ` its farther end
/// from the origin it is expanded about: no less than its acceleration
/// anywhere on it.
fn acceleration_bound(input: &AxisSignalTable, piece: usize) -> f64 {
    let origin = input.origins[piece];
    let reach = (input.starts[piece] - origin)
        .abs()
        .max((input.ends[piece] - origin).abs());
    let mut reach_power = 1.0;
    let mut bound = 0.0;
    for (power, &coefficient) in input.coeffs[piece].iter().enumerate().skip(2) {
        bound += (power * (power - 1)) as f64 * coefficient.abs() * reach_power;
        reach_power *= reach;
    }
    bound
}

/// The input's derivative jumps at a seam between two of its pieces; `None`
/// where `at` is no seam the table holds pieces on both sides of.
fn input_jumps_at(input: &AxisSignalTable, at: f64) -> Option<JumpSeries> {
    let right = input.starts.partition_point(|&start| start < at);
    if right == 0
        || right >= input.starts.len()
        || input.starts[right] != at
        || input.ends[right - 1] != at
    {
        return None;
    }
    let left = right - 1;
    Some(
        JumpSeries::derivatives_at(&input.coeffs[right], at - input.origins[right]).minus(
            JumpSeries::derivatives_at(&input.coeffs[left], at - input.origins[left]),
        ),
    )
}

/// The kernel's derivative jumps at one of its cuts, zero outside its
/// support.
fn kernel_jumps_at(kernel: &PiecewisePolynomialKernel, cut: f64) -> JumpSeries {
    let side = |piece: Option<&nurbs::bezier::BezierPiece>, at_end: bool| match piece {
        Some(piece) => JumpSeries::derivatives_at(
            &piece.coeffs,
            if at_end {
                piece.u_end - piece.u_start
            } else {
                0.0
            },
        ),
        None => JumpSeries {
            orders: [0.0; SERIES_CAPACITY],
            len: 0,
        },
    };
    let right = side(
        kernel.pieces.iter().find(|piece| piece.u_start == cut),
        false,
    );
    let left = side(kernel.pieces.iter().find(|piece| piece.u_end == cut), true);
    right.minus(left)
}

#[cfg(test)]
#[path = "shaper_hidden_breaks_tests.rs"]
mod tests;
