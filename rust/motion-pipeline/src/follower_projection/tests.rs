use super::FollowerState;

/// Regression for the bench crash "shaper: axis 3: shaped sample is
/// non-finite": deep into a long stream the odometer's ulp exceeds
/// SPAN_MIN_LEN_MM, so a kernel-creep span's length is float-absorbed
/// (`s0 + len == s0`). The zero-width span's `e_at(s1)` was 0/0 = NaN,
/// poisoning the cumulative `e0` of every span pushed after it.
#[test]
fn float_absorbed_span_is_dropped_instead_of_poisoning_the_table() {
    let odometer = 2.0e5;
    let creep = 2.0e-12;
    assert!(
        creep > super::SPAN_MIN_LEN_MM && odometer + creep == odometer,
        "test premise: len passes the min-length gate but cannot advance \
         the odometer"
    );

    let mut state = FollowerState {
        s_ingested_end: odometer,
        ..FollowerState::default()
    };
    state.push_span(10.0, 0.05, 0.05);
    state.push_span(creep, 0.0, 0.0);
    state.push_span(10.0, 0.05, 0.05);

    assert_eq!(
        state.spans.len(),
        2,
        "the absorbed span must not enter the table"
    );
    for span in &state.spans {
        assert!(
            span.s1 > span.s0 && span.e0.is_finite(),
            "span [{}, {}] e0={} must stay finite and non-degenerate",
            span.s0,
            span.s1,
            span.e0
        );
    }
    let e_total = state.spans_e(state.s_ingested_end);
    assert!(
        (e_total - 2.0 * 10.0 * 0.05).abs() < 1e-9,
        "cumulative extrusion must survive the dropped sliver: {e_total}"
    );
}

/// Below the odometer's resolution threshold the old absolute gate still
/// applies: near zero, a span barely above SPAN_MIN_LEN_MM must be kept.
#[test]
fn representable_tiny_span_near_origin_is_kept() {
    let mut state = FollowerState::default();
    state.push_span(2.0e-12, 0.5, 0.5);
    assert_eq!(state.spans.len(), 1);
    assert!(state.spans[0].e0.is_finite());
    assert!(state.spans[0].s1 > state.spans[0].s0);
}
