use super::*;

#[test]
fn rounded_in_range_values_convert() {
    assert_eq!(f64_to_i64(-80.0), Ok(-80));
    assert_eq!(f64_to_u64(1.0e9), Ok(1_000_000_000));
    assert_eq!(f64_to_u32(4_294_967_295.0), Ok(u32::MAX));
    assert_eq!(f64_to_i64(-9_223_372_036_854_775_808.0), Ok(i64::MIN));
}

#[test]
fn nan_and_infinity_are_refused_with_their_value() {
    assert!(f64_to_i64(f64::NAN).unwrap_err().value.is_nan());
    assert_eq!(f64_to_u64(f64::INFINITY).unwrap_err().value, f64::INFINITY);
    assert_eq!(f64_to_u32(f64::NEG_INFINITY).unwrap_err().target, "u32");
}

#[test]
fn out_of_range_values_are_refused_not_saturated() {
    assert!(f64_to_i64(9_223_372_036_854_775_808.0).is_err());
    assert!(f64_to_u64(-1.0).is_err());
    assert!(f64_to_u64(18_446_744_073_709_551_616.0).is_err());
    assert!(f64_to_u32(4_294_967_296.0).is_err());
    assert_eq!(
        f64_to_u32(-3.0).unwrap_err().to_string(),
        "-3 is not representable as u32"
    );
}

#[test]
fn a_clock32_near_its_reference_expands_through_a_wrap() {
    assert_eq!(clock32_delta(0x0000_0F00, 0x1_0000_1000), Ok(-0x100));
    assert_eq!(clock32_delta(0x0000_2000, 0x1_0000_1000), Ok(0x1000));
    assert_eq!(clock32_delta(0x0000_0010, 0x0_FFFF_FFF0), Ok(0x20));
}

#[test]
fn a_clock32_far_from_its_reference_is_refused() {
    let error = clock32_delta(0x8000_0000, 0).unwrap_err();
    assert_eq!(error.clock32, 0x8000_0000);
    assert!(error.to_string().contains("reference is stale or wrong"));
    assert!(clock32_delta(0x4000_0000, 0).is_err());
    assert_eq!(clock32_delta(0x3FFF_FFFF, 0), Ok(0x3FFF_FFFF));
}
