//! Rigorous enclosures of the synthetic signals the shaper's tests fit:
//! every fitted piece is proven against one.

use super::Enclosure;
use super::taylor::{self, Ball, TaylorModel, Window};

/// The enclosure over `[lo, hi]` of a signal whose formula on each stretch
/// between its `breaks` is `model` of the window, halved to `accuracy`.
pub(crate) fn modelled(
    (lo, hi): (f64, f64),
    accuracy: [f64; 3],
    breaks: &[f64],
    model: impl Fn(Window) -> [TaylorModel; 3],
) -> Option<Enclosure<'static>> {
    let cuts: Vec<f64> = std::iter::once(lo)
        .chain(breaks.iter().copied().filter(|&t| t > lo && t < hi))
        .chain(std::iter::once(hi))
        .collect();
    let mut out = Vec::new();
    for stretch in cuts.windows(2).filter(|stretch| stretch[0] < stretch[1]) {
        taylor::enclose((stretch[0], stretch[1]), accuracy, &model, &mut out);
    }
    Some(Enclosure::Built(out))
}

/// `A·sin(ω·t)` and its two derivatives on the window, `offset` added to
/// the position.
pub(crate) fn sine(window: Window, offset: f64, amplitude: f64, omega: f64) -> [TaylorModel; 3] {
    let phase = window.time_since(0.0).scale(Ball::exact(omega));
    let (sine, cosine) = phase.sin_cos();
    [
        sine.scale(Ball::exact(amplitude))
            .add_ball(Ball::exact(offset)),
        cosine.scale(Ball::rounded(amplitude * omega, 1)),
        sine.scale(Ball::rounded(-amplitude * omega * omega, 2)),
    ]
}

/// The polynomial `Σ cₖ·τᵏ` in the time `τ` since `origin`, and its two
/// derivatives, on the window.
pub(crate) fn polynomial(window: Window, origin: f64, coefficients: &[f64]) -> [TaylorModel; 3] {
    let tau = window.time_since(origin);
    let derivative = |order: usize| taylor::derivative_of(coefficients, order, &tau);
    [derivative(0), derivative(1), derivative(2)]
}

/// The terms of [`exp`]'s series.
const EXP_TERMS: usize = 12;

/// `eˣ` of the model `x`: its series about the model's centre and the
/// Lagrange remainder past it.
pub(crate) fn exp(x: &TaylorModel) -> TaylorModel {
    let (centre, spread) = x.spread();
    let at_centre = Ball::rounded(libm::exp(centre), 2);
    let mut term = at_centre;
    let coefficients: Vec<Ball> = (0..EXP_TERMS)
        .map(|k| {
            if k > 0 {
                term = term * Ball::rounded(1.0 / k as f64, 1);
            }
            term
        })
        .collect();
    let mut out =
        taylor::polynomial_of_balls(coefficients.into_iter(), &x.add_ball(Ball::exact(-centre)));
    let tail = (at_centre.mid + at_centre.rad)
        * libm::exp(spread)
        * (1..=EXP_TERMS).fold(1.0, |power, k| power * spread / k as f64);
    out.widen(taylor::upper(2.0 * tail));
    out
}
