use super::*;

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
