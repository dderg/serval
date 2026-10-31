use super::{Cut, ExecutionCredit, Progress};

fn progress(consumed: u32, retired: u32) -> Progress {
    Progress { consumed, retired }
}

fn cut(source: usize, before: (u32, u32), after: (u32, u32)) -> Cut {
    Cut {
        source,
        before: progress(before.0, before.1),
        after: progress(after.0, after.1),
    }
}

#[test]
fn acceptance_consumption_and_execution_are_distinct() {
    let mut credit = ExecutionCredit::<1>::new();
    assert_eq!(credit.accept(8), 8);
    assert_eq!(credit.accepted_head(5), 13);
    credit.observe(0, progress(5, 2));
    assert_eq!(credit.awaiting_consumption(), 3);
    assert_eq!(credit.outstanding(), 6);
    assert_eq!(credit.accept(2), 10);
    credit.observe(0, progress(10, 5));
    assert_eq!(credit.awaiting_consumption(), 0);
    assert_eq!(credit.outstanding(), 5);
    credit.observe(0, progress(10, 10));
    assert_eq!(credit.outstanding(), 0);
}

#[test]
fn independent_source_odometers_accumulate_without_duplicate_credit() {
    let mut credit = ExecutionCredit::<2>::new();
    credit.accept(12);
    credit.observe(0, progress(4, 2));
    credit.observe(1, progress(6, 3));
    credit.observe(0, progress(4, 2));
    credit.observe(1, progress(6, 3));
    assert_eq!(credit.awaiting_consumption(), 2);
    assert_eq!(credit.outstanding(), 7);
    credit.observe(0, progress(6, 6));
    credit.observe(1, progress(6, 6));
    assert_eq!(credit.awaiting_consumption(), 0);
    assert_eq!(credit.outstanding(), 0);
}

#[test]
fn interruption_reconciles_every_source_before_abandoning_once() {
    let mut credit = ExecutionCredit::<2>::new();
    credit.accept(20);
    credit.observe(0, progress(3, 1));
    credit.observe(1, progress(4, 2));
    assert_eq!(
        credit.interrupt([cut(0, (6, 4), (10, 10)), cut(1, (8, 5), (10, 10))]),
        11
    );
    let snapshot = credit.snapshot();
    assert_eq!(snapshot.consumed, 14);
    assert_eq!(snapshot.retired, 9);
    assert_eq!(snapshot.abandoned_unconsumed, 6);
    assert_eq!(snapshot.abandoned, 11);
    assert_eq!(credit.awaiting_consumption(), 0);
    assert_eq!(credit.outstanding(), 0);
    credit.observe(0, progress(10, 10));
    credit.observe(1, progress(10, 10));
    assert_eq!(credit.snapshot(), snapshot);
    assert_eq!(credit.interrupt([]), 0);
    assert_eq!(credit.snapshot(), snapshot);
}

#[test]
fn resumed_execution_never_includes_abandoned_work() {
    let mut credit = ExecutionCredit::<1>::new();
    credit.accept(10);
    assert_eq!(credit.interrupt([cut(0, (6, 3), (10, 10))]), 7);
    credit.accept(5);
    assert_eq!(credit.awaiting_consumption(), 5);
    assert_eq!(credit.outstanding(), 5);
    credit.observe(0, progress(14, 12));
    assert_eq!(credit.awaiting_consumption(), 1);
    assert_eq!(credit.outstanding(), 3);
    credit.observe(0, progress(15, 15));
    let snapshot = credit.snapshot();
    assert_eq!(snapshot.consumed, 11);
    assert_eq!(snapshot.retired, 8);
    assert_eq!(snapshot.abandoned, 7);
    assert_eq!(credit.awaiting_consumption(), 0);
    assert_eq!(credit.outstanding(), 0);
}

#[test]
fn late_cut_receipt_advances_newer_baselines_instead_of_replacing_them() {
    let mut credit = ExecutionCredit::<1>::new();
    credit.accept(10);
    credit.interrupt([cut(0, (6, 3), (10, 10))]);
    credit.accept(10);
    credit.observe(0, progress(14, 12));
    assert_eq!(credit.interrupt([cut(0, (12, 11), (20, 20))]), 8);
    let snapshot = credit.snapshot();
    assert_eq!(snapshot.consumed, 10);
    assert_eq!(snapshot.retired, 5);
    credit.observe(0, progress(22, 21));
    assert_eq!(credit.snapshot(), snapshot);
    credit.accept(3);
    credit.observe(0, progress(25, 24));
    assert_eq!(credit.snapshot().retired, 8);
    assert_eq!(credit.awaiting_consumption(), 0);
    assert_eq!(credit.outstanding(), 0);
}

#[test]
fn post_cut_staleness_is_independent_for_each_counter_and_source() {
    let mut credit = ExecutionCredit::<2>::new();
    credit.accept(10);
    credit.interrupt([cut(0, (6, 3), (10, 10))]);
    credit.accept(10);
    credit.observe(0, progress(9, 12));
    assert_eq!(credit.snapshot().consumed, 6);
    assert_eq!(credit.snapshot().retired, 5);
    credit.observe(0, progress(14, 11));
    assert_eq!(credit.snapshot().consumed, 10);
    assert_eq!(credit.snapshot().retired, 5);
    credit.observe(0, progress(14, 12));
    credit.observe(1, progress(4, 3));
    credit.observe(1, progress(3, 2));
    assert_eq!(credit.snapshot().consumed, 13);
    assert_eq!(credit.snapshot().retired, 7);
    assert_eq!(credit.awaiting_consumption(), 3);
    assert_eq!(credit.outstanding(), 6);
}

#[test]
fn pre_cut_modular_regressions_remain_visible() {
    let mut credit = ExecutionCredit::<1>::new();
    credit.accept(10);
    credit.observe(0, progress(8, 6));
    credit.observe(0, progress(7, 4));
    assert_eq!(credit.awaiting_consumption(), 3);
    assert_eq!(credit.outstanding(), 6);
    credit.observe(0, progress(u32::MAX, u32::MAX));
    assert_eq!(credit.awaiting_consumption(), 11);
    assert_eq!(credit.outstanding(), 11);
}

#[test]
fn post_cut_half_range_boundary_is_inclusive_only_below_two_to_the_31() {
    let mut credit = ExecutionCredit::<1>::new();
    credit.interrupt([cut(0, (0, 0), (0, 0))]);
    let forward = u32::MAX / 2;
    credit.accept(forward);
    credit.observe(0, progress(forward, forward + 1));
    assert_eq!(credit.awaiting_consumption(), 0);
    assert_eq!(credit.outstanding(), forward);
    credit.observe(0, progress(forward, forward));
    assert_eq!(credit.outstanding(), 0);
}

#[test]
fn acceptance_and_progress_wrap_without_losing_outstanding_work() {
    let mut credit = ExecutionCredit::<1>::default();
    credit.accept(u32::MAX - 2);
    credit.observe(0, progress(u32::MAX - 2, u32::MAX - 2));
    assert_eq!(credit.accepted_head(8), 5);
    assert_eq!(credit.accept(8), 5);
    credit.observe(0, progress(3, 1));
    assert_eq!(credit.awaiting_consumption(), 2);
    assert_eq!(credit.outstanding(), 4);
    credit.observe(0, progress(5, 5));
    assert_eq!(credit.awaiting_consumption(), 0);
    assert_eq!(credit.outstanding(), 0);
}

#[test]
fn cut_jumps_and_resumed_progress_wrap_without_becoming_execution() {
    let mut credit = ExecutionCredit::<1>::new();
    credit.accept(u32::MAX - 2);
    credit.observe(0, progress(u32::MAX - 2, u32::MAX - 2));
    credit.accept(8);
    assert_eq!(
        credit.interrupt([cut(0, (u32::MAX, u32::MAX - 1), (5, 5))]),
        7
    );
    let snapshot = credit.snapshot();
    assert_eq!(snapshot.abandoned_unconsumed, 6);
    assert_eq!(snapshot.abandoned, 7);
    credit.observe(0, progress(u32::MAX, u32::MAX - 1));
    credit.observe(0, progress(5, 5));
    assert_eq!(credit.snapshot(), snapshot);
    credit.accept(4);
    credit.observe(0, progress(9, 9));
    assert_eq!(credit.snapshot().consumed, 3);
    assert_eq!(credit.snapshot().retired, 2);
    assert_eq!(credit.awaiting_consumption(), 0);
    assert_eq!(credit.outstanding(), 0);
}

#[test]
fn repeated_abandonment_wraps_and_empty_cuts_still_close_out_work() {
    let mut credit = ExecutionCredit::<0>::new();
    credit.accept(u32::MAX - 1);
    assert_eq!(credit.interrupt([]), u32::MAX - 1);
    credit.accept(5);
    assert_eq!(credit.interrupt([]), 5);
    assert_eq!(credit.snapshot().abandoned, 3);
    assert_eq!(credit.snapshot().abandoned_unconsumed, 3);
    assert_eq!(credit.awaiting_consumption(), 0);
    assert_eq!(credit.outstanding(), 0);
}

#[test]
fn post_cut_forward_reports_can_wrap_raw_odometers() {
    let mut credit = ExecutionCredit::<1>::new();
    credit.accept(u32::MAX - 2);
    credit.interrupt([cut(0, (0, 0), (u32::MAX - 2, u32::MAX - 2))]);
    credit.accept(5);
    credit.observe(0, progress(2, 1));
    assert_eq!(credit.snapshot().consumed, 5);
    assert_eq!(credit.snapshot().retired, 4);
    assert_eq!(credit.awaiting_consumption(), 0);
    assert_eq!(credit.outstanding(), 1);
    credit.observe(0, progress(2, 2));
    assert_eq!(credit.outstanding(), 0);
}
