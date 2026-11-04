use super::*;

#[test]
fn a_move_may_take_every_back_off_up_to_the_cap() {
    let mut backoffs = 0;
    for _ in 0..BACKOFFS_MAX {
        count_backoff(&mut backoffs, 12, 0.5, (30_000.0, 20_000.0));
    }
    assert_eq!(backoffs, BACKOFFS_MAX);
}

#[test]
#[should_panic(
    expected = "turning guard: line 12 at t=0.500000s still asks 30000 mm/s² of the extruder after \
                64 back-offs (limit 20000 mm/s²)"
)]
fn a_move_still_binding_past_the_back_off_cap_fails_loudly() {
    let mut backoffs = BACKOFFS_MAX;
    count_backoff(&mut backoffs, 12, 0.5, (30_000.0, 20_000.0));
}
