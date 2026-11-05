use super::{Ball, Signal, TaylorModel, Window, enclose, try_enclose};

#[test]
fn an_enclosure_halves_its_window_until_each_order_meets_its_accuracy() {
    let mut pieces = Vec::new();
    enclose(
        (-6.0, 6.0),
        [1e-12, f64::INFINITY, 1e-12],
        &|window: Window| {
            let tanh = window.time_since(0.0).tanh();
            [tanh, TaylorModel::UNBOUNDED, tanh]
        },
        &mut pieces,
    );
    assert!(pieces.len() > 1, "a wide tanh needs more than one window");
    assert!(pieces.windows(2).all(|pair| pair[0].hi == pair[1].lo));
    assert!(
        pieces.iter().all(|piece| piece.pva[0].rounding <= 1e-12),
        "every window's tanh met its accuracy"
    );
}

/// A window with a model everywhere but within a few floats of one point,
/// the zero of a speed its models divide by: peeled toward it, the floats
/// left around it are held by the evaluator's readings, float by float.
#[test]
fn the_floats_around_a_point_no_model_reaches_are_held_by_the_evaluator() {
    let zero = 1.0;
    let reach = 2.0 * f64::EPSILON;
    let model = |window: Window| -> Option<[TaylorModel; 3]> {
        (window.lo > zero + reach || window.hi < zero - reach).then(|| {
            let tau = window.time_since(zero);
            [
                tau.mul(&tau),
                tau.scale(Ball::exact(2.0)),
                TaylorModel::constant(Ball::exact(2.0)),
            ]
        })
    };
    let pva_at = |t: f64| {
        let tau = t - zero;
        (tau * tau, 2.0 * tau, 2.0)
    };
    let mut out = Vec::new();
    try_enclose(
        (zero - 1e-3, zero + 1e-3),
        [1e-9, 1e-6, 1e-3],
        Signal {
            model: &model,
            pva_at: &pva_at,
        },
        &mut out,
    )
    .expect("the evaluator holds the floats around the zero");
    assert!(
        out.windows(2).all(|pair| pair[0].hi == pair[1].lo),
        "the pieces tile the window"
    );
    assert_eq!(
        (out[0].lo, out[out.len() - 1].hi),
        (zero - 1e-3, zero + 1e-3)
    );
    assert!(
        out.iter().any(|piece| piece.lo <= zero && zero <= piece.hi),
        "a piece holds the zero"
    );
}
