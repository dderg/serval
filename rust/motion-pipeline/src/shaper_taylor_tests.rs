use super::{TaylorModel, Window, enclose};

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
