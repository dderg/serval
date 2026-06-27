use super::*;

fn args(parts: &[&str]) -> Vec<String> {
    parts.iter().map(|s| (*s).to_string()).collect()
}

#[test]
fn no_slave_flag_falls_back_to_one_drive_at_position_zero() {
    let a = args(&[
        "ethercat-rt",
        "eth0",
        "--counts-per-mm",
        "1000",
        "--rotation-distance",
        "50",
    ]);
    let slaves = parse_slaves(&a).expect("legacy parse");
    assert_eq!(slaves.len(), 1);
    assert_eq!(slaves[0].pos, 0);
    assert_eq!(slaves[0].counts_per_mm, 1000.0);
    assert_eq!(slaves[0].rotation_distance, 50.0);
}

#[test]
fn legacy_defaults_when_no_per_drive_flags() {
    let slaves = parse_slaves(&args(&["ethercat-rt", "eth0"])).expect("defaults");
    assert_eq!(slaves.len(), 1);
    assert_eq!(slaves[0].pos, 0);
    assert_eq!(slaves[0].counts_per_mm, 3276.8);
    assert_eq!(slaves[0].following_error_counts, None);
}

#[test]
fn two_groups_parse_per_drive_params() {
    let a = args(&[
        "ethercat-rt",
        "eth0",
        "--slave",
        "1",
        "--counts-per-mm",
        "3276.8",
        "--rotation-distance",
        "40",
        "--following-error-counts",
        "100000",
        "--slave",
        "2",
        "--counts-per-mm",
        "1638.4",
        "--max-torque-tenth-pct",
        "2000",
    ]);
    let slaves = parse_slaves(&a).expect("two groups");
    assert_eq!(slaves.len(), 2);
    assert_eq!(slaves[0].pos, 1);
    assert_eq!(slaves[0].counts_per_mm, 3276.8);
    assert_eq!(slaves[0].following_error_counts, Some(100_000));
    assert_eq!(slaves[0].max_torque_tenth_pct, None);
    assert_eq!(slaves[1].pos, 2);
    assert_eq!(slaves[1].counts_per_mm, 1638.4);
    assert_eq!(slaves[1].max_torque_tenth_pct, Some(2000));
    assert_eq!(slaves[1].following_error_counts, None);
}

#[test]
fn global_flags_ignored_for_group_form() {
    let a = args(&[
        "ethercat-rt",
        "eth0",
        "--slave",
        "0",
        "--rotation-distance",
        "25",
    ]);
    let slaves = parse_slaves(&a).expect("group");
    assert_eq!(slaves.len(), 1);
    assert_eq!(slaves[0].rotation_distance, 25.0);
}

#[test]
fn duplicate_position_is_rejected() {
    let a = args(&["ethercat-rt", "eth0", "--slave", "1", "--slave", "1"]);
    let err = parse_slaves(&a).expect_err("duplicate must fail");
    assert!(err.contains("duplicate"), "got: {err}");
}

#[test]
fn per_drive_flag_before_any_slave_is_rejected() {
    let a = args(&[
        "ethercat-rt",
        "eth0",
        "--counts-per-mm",
        "10",
        "--slave",
        "1",
    ]);
    let err = parse_slaves(&a).expect_err("orphan flag must fail");
    assert!(err.contains("before any --slave"), "got: {err}");
}

#[test]
fn axis_flag_binds_to_the_current_slave_group() {
    let a = args(&[
        "ethercat-rt",
        "eth0",
        "--slave",
        "0",
        "--axis",
        "0",
        "--slave",
        "1",
        "--axis",
        "2",
    ]);
    let slaves = parse_slaves(&a).expect("parse axes");
    assert_eq!(slaves.len(), 2);
    assert_eq!((slaves[0].pos, slaves[0].axis), (0, 0));
    assert_eq!((slaves[1].pos, slaves[1].axis), (1, 2));
}

#[test]
fn velocity_ff_and_clamp_bind_per_slave_group() {
    let a = args(&[
        "ethercat-rt",
        "eth0",
        "--slave",
        "0",
        "--velocity-ff",
        "--torque-clamp-pct",
        "25",
        "--slave",
        "1",
        "--torque-clamp-pct",
        "60",
    ]);
    let slaves = parse_slaves(&a).expect("parse ff flags");
    assert_eq!(slaves.len(), 2);
    assert_eq!(
        (slaves[0].velocity_ff, slaves[0].torque_clamp_tenths),
        (true, 250)
    );
    assert_eq!(
        (slaves[1].velocity_ff, slaves[1].torque_clamp_tenths),
        (false, 600)
    );
}

#[test]
fn legacy_form_reads_velocity_ff_and_clamp_globally() {
    let a = args(&[
        "ethercat-rt",
        "eth0",
        "--velocity-ff",
        "--torque-clamp-pct",
        "45",
    ]);
    let slaves = parse_slaves(&a).expect("legacy ff");
    assert_eq!(slaves.len(), 1);
    assert!(slaves[0].velocity_ff);
    assert_eq!(slaves[0].torque_clamp_tenths, 450);
}

#[test]
fn clamp_default_is_thirty_percent() {
    let slaves = parse_slaves(&args(&["ethercat-rt", "eth0"])).expect("defaults");
    assert!(!slaves[0].velocity_ff);
    assert_eq!(slaves[0].torque_clamp_tenths, 300);
}

#[test]
fn clamp_out_of_range_is_rejected() {
    let a = args(&[
        "ethercat-rt",
        "eth0",
        "--slave",
        "0",
        "--torque-clamp-pct",
        "500",
    ]);
    let err = parse_slaves(&a).expect_err("over-range clamp must fail");
    assert!(err.contains("outside (0, 400]"), "got: {err}");
}

#[test]
fn invert_binds_per_slave_group() {
    let a = args(&[
        "ethercat-rt",
        "eth0",
        "--slave",
        "0",
        "--invert",
        "--slave",
        "1",
    ]);
    let slaves = parse_slaves(&a).expect("parse invert");
    assert_eq!(slaves.len(), 2);
    assert!(slaves[0].invert);
    assert!(!slaves[1].invert);
}

#[test]
fn legacy_form_reads_invert_globally() {
    let slaves = parse_slaves(&args(&["ethercat-rt", "eth0", "--invert"])).expect("legacy invert");
    assert_eq!(slaves.len(), 1);
    assert!(slaves[0].invert);
}

#[test]
fn invert_defaults_off() {
    let slaves = parse_slaves(&args(&["ethercat-rt", "eth0"])).expect("defaults");
    assert!(!slaves[0].invert);
}

#[test]
fn invert_before_any_slave_is_rejected() {
    let a = args(&["ethercat-rt", "eth0", "--invert", "--slave", "0"]);
    let err = parse_slaves(&a).expect_err("orphan --invert must fail");
    assert!(err.contains("before any --slave"), "got: {err}");
}

#[test]
fn velocity_ff_before_any_slave_is_rejected() {
    let a = args(&["ethercat-rt", "eth0", "--velocity-ff", "--slave", "0"]);
    let err = parse_slaves(&a).expect_err("orphan --velocity-ff must fail");
    assert!(err.contains("before any --slave"), "got: {err}");
}

#[test]
fn axis_before_any_slave_is_rejected() {
    let a = args(&["ethercat-rt", "eth0", "--axis", "1", "--slave", "0"]);
    let err = parse_slaves(&a).expect_err("orphan --axis must fail");
    assert!(err.contains("before any --slave"), "got: {err}");
}

#[test]
fn slave_without_value_is_rejected() {
    let a = args(&["ethercat-rt", "eth0", "--slave"]);
    let err = parse_slaves(&a).expect_err("missing position must fail");
    assert!(err.contains("--slave requires"), "got: {err}");
}

#[test]
fn non_integer_position_is_rejected() {
    let a = args(&["ethercat-rt", "eth0", "--slave", "x"]);
    let err = parse_slaves(&a).expect_err("bad position must fail");
    assert!(err.contains("integer position"), "got: {err}");
}

#[test]
fn too_many_slaves_is_rejected() {
    let mut parts = vec!["ethercat-rt".to_string(), "eth0".to_string()];
    for p in 0..=EC_RT_MAX_SLAVES {
        parts.push("--slave".to_string());
        parts.push(p.to_string());
    }
    let err = parse_slaves(&parts).expect_err("over cap must fail");
    assert!(err.contains("at most"), "got: {err}");
}
