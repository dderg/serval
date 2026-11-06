use crate::AlgebraError;
use crate::bezier::binomial;

const VARIATION_STEPS: usize = 512;
const MOMENT_GRID_STEPS: usize = 1024;
const SHARE_SEARCH_STEPS: u32 = 48;

#[derive(Debug, Clone)]
pub struct PiecewisePolynomialKernel {
    pub pieces: Vec<crate::bezier::BezierPiece>,
}

impl PiecewisePolynomialKernel {
    pub fn single_poly(coeffs: Vec<f64>, support: (f64, f64)) -> Self {
        let piece = crate::bezier::BezierPiece {
            u_start: support.0,
            u_end: support.1,
            coeffs,
        };
        Self {
            pieces: vec![piece],
        }
    }

    #[allow(clippy::needless_pass_by_value)]
    pub fn single_poly_from_absolute(coeffs: Vec<f64>, support: (f64, f64)) -> Self {
        let shifted = absolute_to_pascal_shift(&coeffs, support.0);
        Self::single_poly(shifted, support)
    }

    pub fn support(&self) -> (f64, f64) {
        (
            self.pieces.first().unwrap().u_start,
            self.pieces.last().unwrap().u_end,
        )
    }

    #[must_use]
    pub fn second_moment(&self) -> f64 {
        self.pieces
            .iter()
            .map(|piece| {
                let u0 = piece.u_start;
                let h = piece.u_end - piece.u_start;
                piece
                    .coeffs
                    .iter()
                    .enumerate()
                    .map(|(k, &c)| {
                        let k = k as i32;
                        c * (u0 * u0 * h.powi(k + 1) / f64::from(k + 1)
                            + 2.0 * u0 * h.powi(k + 2) / f64::from(k + 2)
                            + h.powi(k + 3) / f64::from(k + 3))
                    })
                    .sum::<f64>()
            })
            .sum()
    }

    /// `(∫|w|, ∫|u|·|w(u)| du)` over the support, by the midpoint rule on
    /// [`MOMENT_GRID_STEPS`] steps per piece.
    #[must_use]
    pub fn absolute_moments(&self) -> (f64, f64) {
        self.absolute_steps()
            .fold((0.0, 0.0), |(mass, moment), step| {
                (mass + step.weight(), moment + step.u.abs() * step.weight())
            })
    }

    /// At each radius `x`, `(∫_{|u|≤x} |w|, ∫|w(u)|·(x − |u|)⁺ du)`: the
    /// mass of `|w|` within `x` of the origin and its integral over the
    /// radius, by the midpoint rule of [`Self::absolute_moments`].
    #[must_use]
    pub fn absolute_mass_within(&self, radii: &[f64]) -> Vec<(f64, f64)> {
        radii
            .iter()
            .map(|&radius| {
                self.absolute_steps()
                    .filter(|step| step.u.abs() <= radius)
                    .fold((0.0, 0.0), |(mass, integral), step| {
                        (
                            mass + step.weight(),
                            integral + (radius - step.u.abs()) * step.weight(),
                        )
                    })
            })
            .collect()
    }

    /// The least `|w|` within `radius` of the origin: over the midpoint
    /// rule's steps there, the origin and both ends.
    #[must_use]
    pub fn least_absolute_within(&self, radius: f64) -> f64 {
        let beside = |u: f64| {
            self.value_beside(u, true)
                .abs()
                .min(self.value_beside(u, false).abs())
        };
        self.absolute_steps()
            .filter(|step| step.u.abs() <= radius)
            .map(|step| step.absolute)
            .chain([-radius, 0.0, radius].map(beside))
            .fold(f64::INFINITY, f64::min)
    }

    /// What `w` keeps beyond `past ≥ 0` on one side of the origin,
    /// `ahead` (`u > past`) or behind (`u < −past`), `d` the distance
    /// beyond: `∫|w|`, `∫|w|·d` and `∫|w|·d²/2` by the midpoint rule of
    /// [`Self::absolute_moments`], and `|w|` at the near edge plus the
    /// total variation of `w` over the tail, every jump and the drop to 0
    /// past the support included.
    #[must_use]
    pub fn tail(&self, past: f64, ahead: bool) -> KernelTail {
        let beyond = |u: f64| if ahead { u - past } else { -past - u };
        let (mass, first_moment_s, half_second_moment_s2) = self
            .absolute_steps()
            .filter(|step| beyond(step.u) > 0.0)
            .fold((0.0, 0.0, 0.0), |(mass, first, second), step| {
                let d = beyond(step.u);
                (
                    mass + step.weight(),
                    first + d * step.weight(),
                    second + 0.5 * d * d * step.weight(),
                )
            });
        let slope: f64 = self
            .pieces
            .iter()
            .map(|piece| {
                let slope = piece.differentiate();
                let width = (piece.u_end - piece.u_start) / MOMENT_GRID_STEPS as f64;
                (0..MOMENT_GRID_STEPS)
                    .map(|step| piece.u_start + (step as f64 + 0.5) * width)
                    .filter(|&u| beyond(u) > 0.0)
                    .map(|u| slope.evaluate(u).abs() * width)
                    .sum::<f64>()
            })
            .sum();
        let mut breaks: Vec<f64> = self
            .pieces
            .iter()
            .flat_map(|piece| [piece.u_start, piece.u_end])
            .filter(|&u| beyond(u) > 0.0)
            .collect();
        breaks.sort_by(f64::total_cmp);
        breaks.dedup();
        let jumps: f64 = breaks
            .into_iter()
            .map(|u| (self.value_beside(u, true) - self.value_beside(u, false)).abs())
            .sum();
        let edge = if ahead { past } else { -past };
        KernelTail {
            mass,
            first_moment_s,
            half_second_moment_s2,
            variation_per_s: self.value_beside(edge, ahead).abs() + slope + jumps,
        }
    }

    fn absolute_steps(&self) -> impl Iterator<Item = AbsoluteStep> + '_ {
        self.pieces.iter().flat_map(|piece| {
            let width = (piece.u_end - piece.u_start) / MOMENT_GRID_STEPS as f64;
            (0..MOMENT_GRID_STEPS).map(move |step| {
                let u = piece.u_start + (step as f64 + 0.5) * width;
                AbsoluteStep {
                    u,
                    absolute: piece.evaluate(u).abs(),
                    width,
                }
            })
        })
    }

    pub fn from_pieces(pieces: Vec<crate::bezier::BezierPiece>) -> Result<Self, AlgebraError> {
        if pieces.is_empty() {
            return Err(AlgebraError::SupportMismatch);
        }
        for w in pieces.windows(2) {
            if w[0].u_end != w[1].u_start {
                return Err(AlgebraError::SupportMismatch);
            }
        }
        Ok(Self { pieces })
    }

    /// The jerk per unit of input acceleration a set of kernels, one per
    /// axis, lets the shaped signal reach along the worst direction: an
    /// input acceleration `a(u)` in the unit disk leaves a shaped jerk along
    /// the unit `d` of `Σᵢ dᵢ ∫ aᵢ(u)·wᵢ′(t − u) du`, at most
    /// `∫ |(dᵢ·wᵢ′(u))ᵢ| du` with `a` aligned to that vector throughout, a
    /// jump of a kernel counting as an impulse in its slope. For one kernel
    /// it is the kernel's total variation, `2·max w` for a unimodal one; a
    /// multimodal kernel's lobes each take a fresh swing of the input.
    ///
    /// The variation is concave in the squared direction `sᵢ = dᵢ²` over the
    /// simplex, so a golden-section search per share finds its largest.
    /// Slopes are integrated by the midpoint rule on [`VARIATION_STEPS`]
    /// steps between the kernels' breakpoints.
    #[must_use]
    pub fn directional_variation(kernels: &[&Self]) -> f64 {
        assert!(
            !kernels.is_empty(),
            "a directional variation needs a kernel"
        );
        let mut breaks: Vec<f64> = kernels
            .iter()
            .flat_map(|kernel| kernel.pieces.iter().flat_map(|p| [p.u_start, p.u_end]))
            .collect();
        breaks.sort_by(f64::total_cmp);
        breaks.dedup();
        let slopes: Vec<Vec<crate::bezier::BezierPiece>> = kernels
            .iter()
            .map(|kernel| kernel.pieces.iter().map(|p| p.differentiate()).collect())
            .collect();
        let mut weights = Vec::new();
        let mut squares = Vec::new();
        for &at in &breaks {
            weights.push(1.0);
            squares.extend(kernels.iter().map(|kernel| {
                (kernel.value_beside(at, true) - kernel.value_beside(at, false)).powi(2)
            }));
        }
        for span in breaks.windows(2) {
            let h = (span[1] - span[0]) / VARIATION_STEPS as f64;
            for step in 0..VARIATION_STEPS {
                let u = span[0] + (step as f64 + 0.5) * h;
                weights.push(h);
                squares.extend(slopes.iter().map(|pieces| piece_value(pieces, u).powi(2)));
            }
        }
        let variation = |shares: &[f64]| {
            weights
                .iter()
                .zip(squares.chunks_exact(kernels.len()))
                .map(|(weight, square)| {
                    weight
                        * square
                            .iter()
                            .zip(shares)
                            .map(|(q, s)| q * s)
                            .sum::<f64>()
                            .sqrt()
                })
                .sum::<f64>()
        };
        largest_from(&mut vec![0.0; kernels.len()], 0, 1.0, &variation)
    }

    /// The jerk per unit of input acceleration step a set of kernels, one
    /// per axis, lets the shaped signal reach along the worst direction: a
    /// step `Δa` in the unit disk at time `t_s` leaves the shaped jerk
    /// `(Δaᵢ·wᵢ(t − t_s))ᵢ`, whose component along a unit `d` is at most
    /// `|(dᵢ·wᵢ(u))ᵢ|` with the step aligned to that vector, and the largest
    /// of that over the unit `d` is `maxᵢ |wᵢ(u)|`. For one kernel it is
    /// `max |w|`, half its total variation when it is unimodal: the
    /// analogue of [`Self::directional_variation`] for steps the kernels
    /// meet apart.
    ///
    /// Each piece is read at its ends and at [`VARIATION_STEPS`] evenly
    /// spaced instants between them.
    #[must_use]
    pub fn directional_peak(kernels: &[&Self]) -> f64 {
        assert!(!kernels.is_empty(), "a directional peak needs a kernel");
        kernels
            .iter()
            .flat_map(|kernel| kernel.pieces.iter())
            .flat_map(|piece| {
                let width = piece.u_end - piece.u_start;
                (0..=VARIATION_STEPS).map(move |step| {
                    piece
                        .evaluate(piece.u_start + width * step as f64 / VARIATION_STEPS as f64)
                        .abs()
                })
            })
            .fold(0.0, f64::max)
    }

    /// `w` just right (`right`) or left of `u`, 0 outside the support.
    fn value_beside(&self, u: f64, right: bool) -> f64 {
        self.pieces
            .iter()
            .find(|p| {
                if right {
                    p.u_start <= u && u < p.u_end
                } else {
                    p.u_start < u && u <= p.u_end
                }
            })
            .map_or(0.0, |p| p.evaluate(u))
    }
}

/// What a kernel keeps beyond a distance from the origin on one side
/// ([`PiecewisePolynomialKernel::tail`]).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct KernelTail {
    pub mass: f64,
    pub first_moment_s: f64,
    pub half_second_moment_s2: f64,
    pub variation_per_s: f64,
}

/// One step of the midpoint rule over a kernel: its midpoint, `|w|` there
/// and its width.
struct AbsoluteStep {
    u: f64,
    absolute: f64,
    width: f64,
}

impl AbsoluteStep {
    fn weight(&self) -> f64 {
        self.absolute * self.width
    }
}

fn piece_value(pieces: &[crate::bezier::BezierPiece], u: f64) -> f64 {
    pieces
        .iter()
        .find(|p| p.u_start <= u && u < p.u_end)
        .map_or(0.0, |p| p.evaluate(u))
}

/// The largest `f` over the shares from `shares[axis]` on, which sum to
/// `left`, the earlier shares held: `f` is concave there, and so is its
/// largest over the later shares as a function of this one.
fn largest_from(shares: &mut [f64], axis: usize, left: f64, f: &dyn Fn(&[f64]) -> f64) -> f64 {
    if axis + 1 == shares.len() {
        shares[axis] = left;
        return f(shares);
    }
    let mut at = |share: f64| {
        shares[axis] = share;
        largest_from(shares, axis + 1, left - share, f)
    };
    let golden = 0.5 * (5.0_f64.sqrt() - 1.0);
    let mut best = at(0.0).max(at(left));
    let (mut lo, mut hi) = (0.0, left);
    let mut inner = (hi - golden * (hi - lo), lo + golden * (hi - lo));
    let mut values = (at(inner.0), at(inner.1));
    for _ in 0..SHARE_SEARCH_STEPS {
        best = best.max(values.0).max(values.1);
        if values.0 < values.1 {
            lo = inner.0;
            inner = (inner.1, lo + golden * (hi - lo));
            values = (values.1, at(inner.1));
        } else {
            hi = inner.1;
            inner = (hi - golden * (hi - lo), inner.0);
            values = (at(inner.0), values.0);
        }
    }
    best.max(values.0).max(values.1)
}

fn absolute_to_pascal_shift(absolute: &[f64], shift: f64) -> Vec<f64> {
    let d = absolute.len() - 1;
    let mut out = vec![0.0; d + 1];
    let mut shift_pow = vec![1.0; d + 1];
    for k in 1..=d {
        shift_pow[k] = shift_pow[k - 1] * shift;
    }
    for n in 0..=d {
        for k in 0..=n {
            let bin = binomial(n, k) as f64;
            out[k] += absolute[n] * bin * shift_pow[n - k];
        }
    }
    out
}

#[cfg(test)]
#[allow(clippy::float_cmp)]
mod tests;
