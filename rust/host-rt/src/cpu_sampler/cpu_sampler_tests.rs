use super::parse_stat_line;

#[test]
fn parses_plain_comm() {
    let line = "1234 (kalico-dispatch) S 1 1234 1234 0 -1 4194304 100 0 3 0 250 75 0 0 20 0 1 0 100 1000000 500 18446744073709551615 1 1 0 0 0 0 0 0 0 0 0 0 17 0 0 0 0 0 0";
    let (name, ticks) = parse_stat_line(line).unwrap();
    assert_eq!(name, "kalico-dispatch");
    assert_eq!(ticks, 250 + 75);
}

#[test]
fn parses_comm_with_spaces_and_parens() {
    let line = "42 (weird (comm) x) R 1 42 42 0 -1 0 0 0 7 0 10 5 0 0 20 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 17 0 0 0 0 0 0";
    let (name, ticks) = parse_stat_line(line).unwrap();
    assert_eq!(name, "weird (comm) x");
    assert_eq!(ticks, 15);
}

#[test]
fn rejects_truncated_line() {
    assert!(parse_stat_line("99 (short) S 1 99").is_none());
}

#[cfg(target_os = "linux")]
#[test]
fn samples_own_process() {
    let sample = super::sample_process().expect("linux sample");
    assert!(sample.ticks_per_sec > 0);
    assert!(sample.rss_bytes > 0);
    assert!(!sample.threads.is_empty());
}
