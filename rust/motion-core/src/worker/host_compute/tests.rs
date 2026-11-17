use super::*;

#[test]
fn a_charge_reads_its_scale() {
    assert_eq!(parse(" 2.5 ").map(|charge| charge.scale), Ok(2.5));
}

#[test]
fn a_malformed_charge_is_refused() {
    for spec in ["", "0", "-1", "inf", "nan", "x"] {
        assert!(parse(spec).is_err(), "{spec:?} must be refused");
    }
}
