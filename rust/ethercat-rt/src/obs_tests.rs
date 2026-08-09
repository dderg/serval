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

use super::{render_line, LogRecord};
use serde_json::{Map, Value};
use time::OffsetDateTime;

fn record(fields: Map<String, Value>) -> LogRecord {
    LogRecord {
        time: OffsetDateTime::UNIX_EPOCH,
        level: "info",
        target: "obs_tests",
        message: Some("hello".into()),
        fields,
    }
}

#[test]
fn render_line_carries_the_wire_fields() {
    let mut fields = Map::new();
    fields.insert("event".into(), Value::String("unit".into()));
    fields.insert("count".into(), Value::from(3));
    let line = render_line(record(fields));
    let parsed: Value = serde_json::from_str(line.trim_end()).unwrap();
    assert_eq!(parsed["_time"], "1970-01-01T00:00:00.000Z");
    assert_eq!(parsed["_msg"], "hello");
    assert_eq!(parsed["level"], "info");
    assert_eq!(parsed["source"], "host-ec");
    assert_eq!(parsed["subsystem"], "ethercat");
    assert_eq!(parsed["event"], "unit");
    assert_eq!(parsed["count"], 3);
    assert!(line.ends_with('\n'));
}

#[test]
fn render_line_hoists_subsystem_out_of_the_field_map() {
    let mut fields = Map::new();
    fields.insert("subsystem".into(), Value::String("trip-relay".into()));
    let line = render_line(record(fields));
    let parsed: Value = serde_json::from_str(line.trim_end()).unwrap();
    assert_eq!(parsed["subsystem"], "trip-relay");
}

#[test]
fn a_full_channel_drops_instead_of_blocking() {
    let (sender, receiver) = crossbeam_channel::bounded::<LogRecord>(1);
    sender.try_send(record(Map::new())).unwrap();
    let started = std::time::Instant::now();
    let verdict = sender.try_send(record(Map::new()));
    assert!(matches!(
        verdict,
        Err(crossbeam_channel::TrySendError::Full(_))
    ));
    assert!(started.elapsed() < std::time::Duration::from_millis(10));
    drop(receiver);
    let verdict = sender.try_send(record(Map::new()));
    assert!(matches!(
        verdict,
        Err(crossbeam_channel::TrySendError::Disconnected(_))
    ));
}
