use super::parse_stat_comm_and_cpu_ticks;

#[test]
fn parses_comm_and_cpu_ticks_from_thread_stat() {
    let stat = "12345 (kalico-lower) S 1 1 1 0 -1 4194368 21 0 0 0 731 42 0 0 20 0 1 0 100 0 0 18446744073709551615 1 1 0 0 0 0 0 0 0 0 0 0 17 3 0 0 0 0 0";
    let (comm, ticks) = parse_stat_comm_and_cpu_ticks(stat).unwrap();
    assert_eq!(comm, "kalico-lower");
    assert_eq!(ticks, 731 + 42);
}

#[test]
fn comm_with_parens_and_spaces_survives() {
    let stat = "7 (weird (name) x) R 1 1 1 0 -1 0 0 0 0 0 9 1 0 0 20 0 1 0 5 0 0 1 1 1 0";
    let (comm, ticks) = parse_stat_comm_and_cpu_ticks(stat).unwrap();
    assert_eq!(comm, "weird (name) x");
    assert_eq!(ticks, 10);
}

#[test]
fn truncated_stat_is_an_error() {
    assert!(parse_stat_comm_and_cpu_ticks("1 (x) R 1 2 3").is_err());
}
