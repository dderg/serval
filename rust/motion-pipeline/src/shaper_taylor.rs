//! Taylor-model enclosures of a signal as the hull reads them: each window's
//! models as one exact piece, its polynomials' bounds their rounding.

pub(crate) use taylor_model::{
    Ball, TAYLOR_LEN, TaylorModel, Window, derivative_of, polynomial_of_balls, upper,
};

use super::exact::{EXACT_CAPACITY, ExactPiece, LocalPolynomial};

const _: () = assert!(TAYLOR_LEN <= EXACT_CAPACITY);

/// How many times a window whose models miss their accuracy is halved.
pub(crate) const ENCLOSURE_MAX_DEPTH: u32 = 12;

fn local_polynomial(model: &TaylorModel) -> LocalPolynomial {
    let mut coefficients = [0.0; EXACT_CAPACITY];
    coefficients[..model.coefficients().len()].copy_from_slice(model.coefficients());
    LocalPolynomial {
        coefficients,
        len: model.coefficients().len(),
        rounding: model.rad(),
    }
}

fn piece(window: Window, models: &[TaylorModel; 3]) -> ExactPiece {
    ExactPiece {
        lo: window.lo,
        hi: window.hi,
        center: window.center,
        radius: window.radius,
        pva: models.each_ref().map(local_polynomial),
    }
}

/// Encloses a signal over `[lo, hi]`: the window's models, halved until
/// every order's bound is within its `accuracy` or the halving depth runs
/// out, where the closest models stand and the hull decides.
pub(crate) fn enclose(
    (lo, hi): (f64, f64),
    accuracy: [f64; 3],
    model: &impl Fn(Window) -> [TaylorModel; 3],
    out: &mut Vec<ExactPiece>,
) {
    enclose_to_depth((lo, hi), accuracy, model, 0, out);
}

fn enclose_to_depth(
    (lo, hi): (f64, f64),
    accuracy: [f64; 3],
    model: &impl Fn(Window) -> [TaylorModel; 3],
    depth: u32,
    out: &mut Vec<ExactPiece>,
) {
    let window = Window::new(lo, hi);
    let models = model(window);
    let resolved = models
        .iter()
        .zip(accuracy)
        .all(|(model, accuracy)| model.rad() <= accuracy);
    let middle = window.center;
    if resolved || depth == ENCLOSURE_MAX_DEPTH || middle <= lo || middle >= hi {
        out.push(piece(window, &models));
        return;
    }
    enclose_to_depth((lo, middle), accuracy, model, depth + 1, out);
    enclose_to_depth((middle, hi), accuracy, model, depth + 1, out);
}

/// Encloses several signals over `[lo, hi]` whose windows share work:
/// `model(window, axis, shared)` reads signal `axis` on `window`, `shared`
/// holding what the window's first reading built for the rest. Each
/// signal halves on its own, as [`enclose`] does, into `outs[axis]`.
pub(crate) fn enclose_axes<S>(
    (lo, hi): (f64, f64),
    accuracy: [f64; 3],
    model: &impl Fn(Window, usize, &mut Option<S>) -> [TaylorModel; 3],
    outs: &mut [Vec<ExactPiece>],
) {
    let axes: Vec<usize> = (0..outs.len()).collect();
    enclose_axes_to_depth((lo, hi), accuracy, model, &axes, 0, outs);
}

fn enclose_axes_to_depth<S>(
    (lo, hi): (f64, f64),
    accuracy: [f64; 3],
    model: &impl Fn(Window, usize, &mut Option<S>) -> [TaylorModel; 3],
    axes: &[usize],
    depth: u32,
    outs: &mut [Vec<ExactPiece>],
) {
    let window = Window::new(lo, hi);
    let middle = window.center;
    let last = depth == ENCLOSURE_MAX_DEPTH || middle <= lo || middle >= hi;
    let mut shared = None;
    let mut unresolved = Vec::new();
    for &axis in axes {
        let models = model(window, axis, &mut shared);
        let resolved = models
            .iter()
            .zip(accuracy)
            .all(|(model, accuracy)| model.rad() <= accuracy);
        if resolved || last {
            outs[axis].push(piece(window, &models));
        } else {
            unresolved.push(axis);
        }
    }
    if unresolved.is_empty() {
        return;
    }
    enclose_axes_to_depth((lo, middle), accuracy, model, &unresolved, depth + 1, outs);
    enclose_axes_to_depth((middle, hi), accuracy, model, &unresolved, depth + 1, outs);
}

#[cfg(test)]
#[path = "shaper_taylor_tests.rs"]
mod tests;
