//! Which of the shaped signal's breakpoints a leader fit seeds, and what the
//! ones it leaves inside a span cost that span's tolerance.
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
//! # What a hidden image costs a span
//!
//! Let a span `[t₀, t₁]` hold breaks `tᵢ` that are not seeds, each the image
//! of exactly one `(b, c)` pair. On the span, `s = g + Σᵢ rᵢ` with each `rᵢ`
//! either `Dᵢ(t)·H(t − tᵢ)` or `−Dᵢ(t)·H(tᵢ − t)`, chosen per break: both
//! choices leave `g` the one polynomial `P₀ + Σ_{left} Dᵢ` on the whole span
//! (`P₀` the span's leftmost piece), so `g` is exactly what an unhidden span
//! fits. `rᵢ` is nonzero only over the reach `wᵢ = min(tᵢ − t₀, t₁ − tᵢ)`,
//! where
//!
//! ```text
//! |rᵢ|  ≤ Σ_{m≥1} |Jₘ| wᵢᵐ/m!          =: εₚ
//! |rᵢ″| ≤ Σ_{m≥2} |Jₘ| wᵢᵐ⁻²/(m−2)!    =: εₐ
//! ```
//!
//! (A `J₁` is a velocity step of `s`: an impulse in `rᵢ″` that no pointwise
//! acceleration sample, the fit's or the audit's, reads; it costs position
//! only.) Every sub-span of a bisected span holds a subset of its breaks at
//! no greater reach, so the span's bound holds for every piece refined from
//! it.
//!
//! # The tolerance a span with hidden breaks is fitted to
//!
//! A fit `F` accepted against `s` to `tol′` at its probes errs against the
//! polynomial `g` by at most `tol′ + ε` there, since `s − g` is at most `ε`.
//! `F − g` is an error between two polynomials, exactly what an unhidden span
//! hands the probes and their witness, so it is held between the probes as
//! every seed span's is: by at most `tol′ + ε`. Against `s` it then errs by
//! at most `tol′ + 2ε`. Fitting to `tol′ = tol − 2ε` keeps every accepted
//! piece within `tol`, position and acceleration alike.
//!
//! # Which images are hidden
//!
//! Seeds are walked in order, and an image is hidden while the bounds of the
//! images hidden since the last kept seed, each charged at its distance from
//! that seed, stay within a sixteenth of the tolerance. Coalescing then moves
//! seeds by at most a resolution floor, so the span bounds are recomputed on
//! the final partition, and those are what each span's tolerance pays.

use nurbs::algebra::PiecewisePolynomialKernel;
use trajectory::ShapedSignal;

use super::{AxisSignalTable, MOMENT_POWER_CAPACITY, differentiate_into};
use crate::lowering::FitTol;

/// Holds the derivative series of an input piece and a kernel piece, and
/// their image's, which runs one order past both together.
const SERIES_CAPACITY: usize = 2 * MOMENT_POWER_CAPACITY;

/// The share of the fit tolerance the images hidden in one span may bound.
const HIDDEN_SHARE_OF_TOLERANCE: f64 = 1.0 / 16.0;

/// Tolerance a span gives up per unit of its hidden-break bound: one for the
/// probes reading `s` instead of `g`, one for `s` leaving `g` between them.
const HIDDEN_BREAK_TOLERANCE_COST: f64 = 2.0;

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
        for (order, value) in orders[..coefficients.len()].iter_mut().enumerate() {
            let length = differentiate_into(coefficients, order, &mut derivative);
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

    /// `εₚ` and `εₐ` of a break with these jumps at reach `reach`.
    fn bound_over(&self, reach: f64) -> HiddenBreakBound {
        let mut bound = HiddenBreakBound::default();
        let mut power_over_factorial = 1.0;
        for power in 0..self.len {
            if power >= 1 {
                bound.position_mm += self.orders[power].abs() * power_over_factorial;
            }
            if power + 2 < self.len {
                bound.acceleration_mm_s2 += self.orders[power + 2].abs() * power_over_factorial;
            }
            power_over_factorial *= reach / (power + 1) as f64;
        }
        bound
    }
}

/// How far the breaks hidden in a span can move the shaped signal from the
/// polynomial the span is fitted as.
#[derive(Clone, Copy, Default, Debug, PartialEq)]
pub(crate) struct HiddenBreakBound {
    pub position_mm: f64,
    pub acceleration_mm_s2: f64,
}

impl HiddenBreakBound {
    fn plus(self, other: Self) -> Self {
        Self {
            position_mm: self.position_mm + other.position_mm,
            acceleration_mm_s2: self.acceleration_mm_s2 + other.acceleration_mm_s2,
        }
    }

    fn within_share_of(self, fit_tol: FitTol) -> bool {
        self.position_mm <= HIDDEN_SHARE_OF_TOLERANCE * fit_tol.pos_mm
            && self.acceleration_mm_s2 <= HIDDEN_SHARE_OF_TOLERANCE * fit_tol.accel_mm_s2
    }

    /// The tolerance a span hiding these breaks is fitted to, so that what
    /// it accepts still holds `fit_tol` against the shaped signal.
    pub(crate) fn tolerance_left(self, fit_tol: FitTol) -> FitTol {
        let left = FitTol {
            pos_mm: fit_tol.pos_mm - HIDDEN_BREAK_TOLERANCE_COST * self.position_mm,
            accel_mm_s2: fit_tol.accel_mm_s2
                - HIDDEN_BREAK_TOLERANCE_COST * self.acceleration_mm_s2,
        };
        assert!(
            left.pos_mm > 0.0 && left.accel_mm_s2 > 0.0,
            "hidden breaks bounded by {self:?} leave no tolerance of {fit_tol:?}"
        );
        left
    }
}

/// An output break left out of the seeds, with the jumps it carries.
#[derive(Clone, Copy, Debug)]
pub(crate) struct HiddenBreak {
    t: f64,
    jumps: JumpSeries,
}

/// A target's seeds, and the breaks between them it does not seed.
pub(crate) struct LeaderSeeds {
    pub kept: Vec<f64>,
    pub hidden: Vec<HiddenBreak>,
}

/// Every output break of the shaped signal — each input break moved by each
/// kernel cut, as `output_cut_transitions` places it — with the pair it is
/// the image of, where it is the image of exactly one.
pub(crate) struct ShapedBreaks {
    times: Vec<f64>,
    images: Vec<Option<KernelImage>>,
    kernel_jumps: Vec<JumpSeries>,
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
        Self {
            times,
            images,
            kernel_jumps: cuts
                .iter()
                .map(|&cut| kernel_jumps_at(kernel, cut))
                .collect(),
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

    /// Keeps `seeds`' first and last, and every interior seed that is not
    /// the image of one input break with known jumps or whose bound would
    /// take the span it falls in past its share of `fit_tol`. `input` is
    /// `None` where the convolution's input is not the table alone.
    pub(crate) fn select_seeds(
        &self,
        seeds: Vec<f64>,
        input: Option<&AxisSignalTable>,
        fit_tol: FitTol,
    ) -> LeaderSeeds {
        let Some(input) = input else {
            return LeaderSeeds {
                kept: seeds,
                hidden: Vec::new(),
            };
        };
        let (&last, interior) = seeds[1..].split_last().expect("fit seeds span no interval");
        let mut kept = Vec::with_capacity(seeds.len());
        kept.push(seeds[0]);
        let mut hidden = Vec::new();
        let mut spent = HiddenBreakBound::default();
        for &t in interior {
            let last_kept = *kept.last().expect("kept seeds start with the first");
            let charged = self
                .image_at(t)
                .and_then(|image| self.image_jumps(input, image))
                .map(|jumps| (jumps, spent.plus(jumps.bound_over(t - last_kept))))
                .filter(|(_, total)| total.within_share_of(fit_tol));
            match charged {
                Some((jumps, total)) => {
                    spent = total;
                    hidden.push(HiddenBreak { t, jumps });
                }
                None => {
                    kept.push(t);
                    spent = HiddenBreakBound::default();
                }
            }
        }
        kept.push(last);
        LeaderSeeds { kept, hidden }
    }

    fn image_jumps(&self, input: &AxisSignalTable, image: KernelImage) -> Option<JumpSeries> {
        Some(input_jumps_at(input, image.input_break)?.image_through(self.kernel_jumps[image.cut]))
    }
}

/// The bound each span of the final partition `seeds` pays for the breaks
/// `hidden` among them, each charged at its reach in that span.
pub(crate) fn hidden_break_bounds(seeds: &[f64], hidden: &[HiddenBreak]) -> Vec<HiddenBreakBound> {
    let mut next = 0;
    let bounds = seeds
        .windows(2)
        .map(|span| {
            let mut bound = HiddenBreakBound::default();
            while let Some(inside) = hidden.get(next).filter(|inside| inside.t < span[1]) {
                bound = bound.plus(
                    inside
                        .jumps
                        .bound_over((inside.t - span[0]).min(span[1] - inside.t)),
                );
                next += 1;
            }
            bound
        })
        .collect();
    assert_eq!(
        next,
        hidden.len(),
        "hidden breaks fall past the partition's last seed"
    );
    bounds
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
    let left_end = input.ends[left] - input.starts[left];
    Some(
        JumpSeries::derivatives_at(&input.coeffs[right], 0.0)
            .minus(JumpSeries::derivatives_at(&input.coeffs[left], left_end)),
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
