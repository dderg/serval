use super::*;

#[test]
fn origin_maps_to_itself() {
    let m = CountMap::new(3276.8, 14578, 5.0);
    assert_eq!(m.target_counts(5.0), 14578);
}

#[test]
fn positive_delta_rounds_and_adds() {
    let m = CountMap::new(1000.0, 0, 0.0);
    assert_eq!(m.target_counts(1.0004), 1000);
    assert_eq!(m.target_counts(1.0006), 1001);
}

#[test]
fn negative_delta() {
    let m = CountMap::new(1000.0, 5000, 10.0);
    assert_eq!(m.target_counts(9.0), 4000);
}

#[test]
fn origin_no_jump() {
    let counts_per_mm = 3276.8_f64;
    let actual_counts = 14578_i32;
    let origin_mm = 7.5_f64;

    let m = CountMap::new(counts_per_mm, actual_counts, origin_mm);

    assert_eq!(
        m.target_counts(origin_mm),
        actual_counts,
        "origin_mm must map to actual_counts exactly; a mismatch is a startup jump"
    );

    let one_count_fwd = origin_mm + 1.0 / counts_per_mm;
    assert_eq!(
        m.target_counts(one_count_fwd),
        actual_counts + 1,
        "one count forward must be actual_counts + 1"
    );

    let one_count_back = origin_mm - 1.0 / counts_per_mm;
    assert_eq!(
        m.target_counts(one_count_back),
        actual_counts - 1,
        "one count back must be actual_counts - 1"
    );
}

#[test]
fn mm_to_counts_inverts_reporting_scale() {
    let counts_per_mm = 3276.8_f64;
    for &pos_mm in &[0.0_f64, 7.0, 7.5, 123.456, -42.0, 0.0123] {
        let counts = mm_to_counts(pos_mm, counts_per_mm);
        let reported_mm = f64::from(counts) / counts_per_mm;
        let round_trip = mm_to_counts(reported_mm, counts_per_mm);
        assert!(
            (round_trip - counts).abs() <= 1,
            "mm_to_counts must be the inverse of reporting (counts/cpm) within 1 count: \
             pos_mm={pos_mm} counts={counts} reported_mm={reported_mm} round_trip={round_trip}"
        );
    }
}

#[test]
fn velocity_mm_s_counts_per_s_to_mm_per_s() {
    assert!((velocity_mm_s(983_040, 3276.8) - 300.0).abs() < 1e-9);
    assert_eq!(velocity_mm_s(0, 3276.8), 0.0);
    assert!((velocity_mm_s(983_040, -3276.8) - (-300.0)).abs() < 1e-9);
}
