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
        unshaped_step: None,
    }
}

/// Relative share of a reading its evaluator's own rounding may move it by,
/// past an enclosure's bound.
const EVALUATOR_NOISE_SHARE: f64 = 1e-9;

/// How many of the steps one ulp of time moves a reading the evaluator's
/// own rounding of its time may move it by, past an enclosure's bound.
const EVALUATOR_TIME_ULPS: f64 = 8.0;

/// Debug builds hold every piece of an enclosure to the signal's own
/// evaluator `truth` at the piece's centre: an enclosure modelling another
/// formula than the signal evaluates would prove fits against the wrong
/// signal, and fails loudly here instead. A piece one ulp wide has no
/// centre inside it to read its interior's formula at, and is skipped.
pub(crate) fn debug_assert_holds(
    pieces: &[ExactPiece],
    truth: impl Fn(f64) -> (f64, f64, f64),
    what: &str,
) {
    if !cfg!(debug_assertions) {
        return;
    }
    for piece in pieces
        .iter()
        .filter(|piece| piece.lo < piece.center && piece.center < piece.hi)
    {
        let (p, v, a) = truth(piece.center);
        let later = piece.center + piece.center.abs() * f64::EPSILON;
        let (p_later, v_later, a_later) = truth(later);
        let readings = [(p, p_later), (v, v_later), (a, a_later)];
        for (order, (value, value_later)) in readings.into_iter().enumerate() {
            let model = &piece.pva[order];
            let miss = (model.eval(0.0) - value).abs();
            let noise = EVALUATOR_NOISE_SHARE * (1.0 + value.abs())
                + EVALUATOR_TIME_ULPS * (value_later - value).abs();
            assert!(
                !(miss > model.rounding + noise),
                "{what}: the enclosure of order {order} on [{}, {}] reads {} at its centre, \
                 the signal {value}, past its bound {}",
                piece.lo,
                piece.hi,
                model.eval(0.0),
                model.rounding
            );
        }
    }
}

/// A model of `√argument`, or the root's range as a constant where its
/// model does not exist or is wider than that range.
pub(crate) fn root_or_range(argument: &TaylorModel) -> TaylorModel {
    let (centre, spread) = argument.spread();
    let lo = (centre - spread).max(0.0).sqrt();
    let hi = upper(upper(centre + spread).max(0.0).sqrt());
    let half_range = upper(0.5 * (hi - lo));
    match argument.sqrt() {
        Some(root) if root.rad() <= half_range => root,
        _ => TaylorModel::constant(Ball {
            mid: 0.5 * (lo + hi),
            rad: half_range,
        }),
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

/// [`enclose`] for a signal whose windows may have no model: halving a
/// window without one, past [`ENCLOSURE_MAX_DEPTH`] only toward the point
/// its models fail at — one half at least modelled — down to
/// [`SINGULAR_MAX_DEPTH`], and giving up — `None` — where the halving
/// depth runs out on one, or on an unbounded model. A window whose bound
/// halving twice running did not shrink by [`STALLED_SHARE`] — a jump or a
/// rounding floor inside it no window resolves — is kept unresolved
/// rather than halved down to the depth limit.
pub(crate) fn try_enclose(
    (lo, hi): (f64, f64),
    accuracy: [f64; 3],
    signal: Signal<'_>,
    out: &mut Vec<ExactPiece>,
) -> Option<()> {
    let window = Window::new(lo, hi);
    try_enclose_to_depth(
        window,
        (signal.model)(window).as_ref(),
        accuracy,
        signal,
        Halving::START,
        out,
    )
}

/// A signal [`try_enclose`] encloses: its models on a window, and its
/// evaluator, which alone holds a window no float lies inside.
#[derive(Clone, Copy)]
pub(crate) struct Signal<'a> {
    pub(crate) model: &'a dyn Fn(Window) -> Option<[TaylorModel; 3]>,
    pub(crate) pva_at: &'a dyn Fn(f64) -> (f64, f64, f64),
}

/// The piece over a window no float lies inside: the signal is read at
/// its two ends alone, so their readings bound it.
fn between_floats(window: Window, pva_at: &dyn Fn(f64) -> (f64, f64, f64)) -> ExactPiece {
    let ((p0, v0, a0), (p1, v1, a1)) = (pva_at(window.lo), pva_at(window.hi));
    let spanning = |low: f64, high: f64| {
        TaylorModel::constant(Ball {
            mid: 0.5 * low + 0.5 * high,
            rad: upper(upper(0.5 * (low - high).abs()) + f64::EPSILON * low.abs().max(high.abs())),
        })
    };
    piece(
        window,
        &[spanning(p0, p1), spanning(v0, v1), spanning(a0, a1)],
    )
}

/// How many times a window without a model is halved toward the point
/// its models fail at: a speed its models divide by vanishing there needs
/// windows as narrow as the speed over its rate.
const SINGULAR_MAX_DEPTH: u32 = 48;

/// The share of its parent's excess over the accuracy a halved window
/// must stay under to count as progress.
const STALLED_SHARE: f64 = 0.75;

/// Where a halving stands: its depth among modelled windows, the excess of
/// the window it halved, how many halvings running made no progress, and
/// how many windows without a model it went through.
#[derive(Clone, Copy)]
struct Halving {
    depth: u32,
    parent_excess: f64,
    stalls: u32,
    singular: u32,
}

impl Halving {
    const START: Self = Self {
        depth: 0,
        parent_excess: f64::INFINITY,
        stalls: 0,
        singular: 0,
    };

    fn child(self, excess: f64) -> Self {
        let stalled = excess > STALLED_SHARE * self.parent_excess;
        Self {
            depth: self.depth + 1,
            parent_excess: excess,
            stalls: if stalled { self.stalls + 1 } else { 0 },
            singular: self.singular,
        }
    }

    fn modelless_child(self) -> Self {
        Self {
            parent_excess: f64::INFINITY,
            stalls: 0,
            singular: self.singular + 1,
            ..self
        }
    }
}

/// How many times `models` miss `accuracy` by, at worst.
fn excess(models: &[TaylorModel; 3], accuracy: [f64; 3]) -> f64 {
    models
        .iter()
        .zip(accuracy)
        .map(|(model, accuracy)| model.rad() / accuracy)
        .fold(0.0, f64::max)
}

fn try_enclose_to_depth(
    window: Window,
    models: Option<&[TaylorModel; 3]>,
    accuracy: [f64; 3],
    signal: Signal<'_>,
    halving: Halving,
    out: &mut Vec<ExactPiece>,
) -> Option<()> {
    let (lo, hi) = (window.lo, window.hi);
    let middle = window.center;
    let unsplittable = middle <= lo || middle >= hi;
    let child = match models {
        Some(models) => {
            let excess = excess(models, accuracy);
            let child = halving.child(excess);
            let last = halving.depth == ENCLOSURE_MAX_DEPTH || unsplittable;
            if excess <= 1.0 || last || child.stalls >= 2 {
                if !models.iter().all(|model| model.rad().is_finite()) {
                    return None;
                }
                out.push(piece(window, models));
                return Some(());
            }
            child
        }
        None if unsplittable => {
            out.push(between_floats(window, signal.pva_at));
            return Some(());
        }
        None if halving.singular == SINGULAR_MAX_DEPTH => return None,
        None => halving.modelless_child(),
    };
    let (left, right) = (Window::new(lo, middle), Window::new(middle, hi));
    let (left_models, right_models) = ((signal.model)(left), (signal.model)(right));
    let modelless_stretch = models.is_none()
        && left_models.is_none()
        && right_models.is_none()
        && halving.singular >= ENCLOSURE_MAX_DEPTH;
    if modelless_stretch {
        return peel_toward_singularity(window, accuracy, signal, child, out);
    }
    try_enclose_to_depth(left, left_models.as_ref(), accuracy, signal, child, out)?;
    try_enclose_to_depth(right, right_models.as_ref(), accuracy, signal, child, out)
}

/// How much of a window without a model, halves and all, is peeled off
/// each end at a time.
const PEEL_SHARE: f64 = 0.125;

/// How many peels a window without a model may take.
const PEEL_MAX: u32 = 256;

/// Encloses a window neither of whose halves has a model either: a speed
/// vanishing at one end as a power of the time to it bounds no polynomial
/// range on a window as wide as its distance from the zero, however far it
/// is halved. Slivers [`PEEL_SHARE`] of the window wide are peeled off
/// whichever end holds a model until the rest does; `None` where neither
/// end does, or the peels run out.
fn peel_toward_singularity(
    window: Window,
    accuracy: [f64; 3],
    signal: Signal<'_>,
    halving: Halving,
    out: &mut Vec<ExactPiece>,
) -> Option<()> {
    let (mut lo, mut hi) = (window.lo, window.hi);
    let mut from_the_end = Vec::new();
    for _ in 0..PEEL_MAX {
        let rest = Window::new(lo, hi);
        if let Some(models) = (signal.model)(rest) {
            try_enclose_to_depth(rest, Some(&models), accuracy, signal, halving, out)?;
            for pieces in from_the_end.into_iter().rev() {
                out.extend::<Vec<ExactPiece>>(pieces);
            }
            return Some(());
        }
        let peel = PEEL_SHARE * (hi - lo);
        let (front, back) = (Window::new(lo, lo + peel), Window::new(hi - peel, hi));
        let (front_models, back_models) = ((signal.model)(front), (signal.model)(back));
        if front_models.is_none() && back_models.is_none() {
            return None;
        }
        if let Some(models) = front_models {
            try_enclose_to_depth(front, Some(&models), accuracy, signal, halving, out)?;
            lo = front.hi;
        }
        if let Some(models) = back_models {
            let mut pieces = Vec::new();
            try_enclose_to_depth(back, Some(&models), accuracy, signal, halving, &mut pieces)?;
            from_the_end.push(pieces);
            hi = back.lo;
        }
        if lo >= hi {
            return None;
        }
    }
    None
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
