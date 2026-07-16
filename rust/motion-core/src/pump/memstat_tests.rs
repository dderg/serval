use super::memstat::{parse_status_vm_swap_kb, parse_thread_stat_majflt};

const THREAD_STAT_FIXTURE: &str = "184203 (motion-pump) S 1 184203 184203 0 -1 4194368 \
     2915 0 731 0 512 96 0 0 20 0 14 0 8092065 592805888 21509 18446744073709551615 \
     94903732938752 94903732970936 140721896052048 0 0 0 0 4096 17987 1 0 0 17 3 0 0 0 0 0 \
     94903733089680 94903733091264 94903755599872 140721896059607 140721896059627 \
     140721896059627 140721896062955 0\n";

const STATUS_FIXTURE: &str = "Name:\tmotion-pump\n\
     Umask:\t0022\n\
     State:\tS (sleeping)\n\
     Tgid:\t184203\n\
     Pid:\t184203\n\
     VmPeak:\t  912448 kB\n\
     VmSize:\t  846912 kB\n\
     VmRSS:\t   84136 kB\n\
     VmData:\t  310208 kB\n\
     VmSwap:\t    5324 kB\n\
     Threads:\t14\n\
     voluntary_ctxt_switches:\t2048\n";

#[test]
fn majflt_parsed_from_captured_thread_stat() {
    assert_eq!(parse_thread_stat_majflt(THREAD_STAT_FIXTURE), Ok(731));
}

#[test]
fn majflt_survives_comm_with_spaces_and_parens() {
    let stat = "77 (tokio-runtime (w)) R 1 77 77 0 -1 4194304 10 0 42 0 1 1 0 0 20 0 2 0 100 200 300 400\n";
    assert_eq!(parse_thread_stat_majflt(stat), Ok(42));
}

#[test]
fn majflt_zero_parses() {
    let stat = "5 (idle) S 1 5 5 0 -1 0 0 0 0 0 0 0 0 0 20 0 1 0 1 2 3 4\n";
    assert_eq!(parse_thread_stat_majflt(stat), Ok(0));
}

#[test]
fn stat_without_comm_paren_is_an_error() {
    let err = parse_thread_stat_majflt("garbage without parens").unwrap_err();
    assert!(err.contains("no ')'"), "{err}");
}

#[test]
fn stat_with_too_few_fields_is_an_error() {
    let err = parse_thread_stat_majflt("9 (short) S 1 9 9\n").unwrap_err();
    assert!(err.contains("too few fields"), "{err}");
}

#[test]
fn stat_with_non_numeric_majflt_is_an_error() {
    let stat = "9 (bad) S 1 9 9 0 -1 0 0 0 nope 0 0 0 0 0 20 0 1 0 1 2 3 4\n";
    let err = parse_thread_stat_majflt(stat).unwrap_err();
    assert!(err.contains("not a u64"), "{err}");
}

#[test]
fn vm_swap_parsed_from_captured_status() {
    assert_eq!(parse_status_vm_swap_kb(STATUS_FIXTURE), Ok(5324));
}

#[test]
fn vm_swap_zero_parses() {
    assert_eq!(parse_status_vm_swap_kb("VmSwap:\t       0 kB\n"), Ok(0));
}

#[test]
fn status_without_vm_swap_line_is_an_error() {
    let err = parse_status_vm_swap_kb("Name:\tx\nThreads:\t1\n").unwrap_err();
    assert!(err.contains("no VmSwap line"), "{err}");
}

#[test]
fn status_with_non_numeric_vm_swap_is_an_error() {
    let err = parse_status_vm_swap_kb("VmSwap:\tlots kB\n").unwrap_err();
    assert!(err.contains("not a u64"), "{err}");
}
