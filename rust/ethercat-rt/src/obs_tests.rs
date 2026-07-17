use super::DropReport;

#[test]
fn no_report_while_nothing_dropped() {
    let report = DropReport::new();
    assert_eq!(report.newly_dropped(0), None);
    assert_eq!(report.newly_dropped(0), None);
}

#[test]
fn first_growth_reports_cumulative_count() {
    let report = DropReport::new();
    assert_eq!(report.newly_dropped(7), Some(7));
}

#[test]
fn unchanged_count_after_a_report_stays_quiet() {
    let report = DropReport::new();
    assert_eq!(report.newly_dropped(7), Some(7));
    assert_eq!(report.newly_dropped(7), None);
    assert_eq!(report.newly_dropped(7), None);
}

#[test]
fn each_growth_reports_the_new_cumulative_total() {
    let report = DropReport::new();
    assert_eq!(report.newly_dropped(3), Some(3));
    assert_eq!(report.newly_dropped(3), None);
    assert_eq!(report.newly_dropped(150), Some(150));
    assert_eq!(report.newly_dropped(150), None);
}
