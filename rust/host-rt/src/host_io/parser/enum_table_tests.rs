use super::*;

#[test]
fn from_dict_expands_range() {
    let mut d = IndexMap::new();
    d.insert(
        "PA0".to_string(),
        EnumValue::Range {
            start: 0,
            count: 16,
        },
    );
    let table = EnumTable::from_dict(&d).unwrap();
    assert_eq!(table.by_name.get("PA0"), Some(&0));
    assert_eq!(table.by_name.get("PA15"), Some(&15));
    assert_eq!(table.by_int.get(&15), Some(&"PA15".to_string()));
    assert_eq!(table.by_name.len(), 16);
}

fn range(name: &str, start: i32, count: i32) -> IndexMap<String, EnumValue> {
    let mut d = IndexMap::new();
    d.insert(name.to_string(), EnumValue::Range { start, count });
    d
}

#[test]
fn a_range_name_without_a_numeric_suffix_is_refused_not_started_at_zero() {
    let error = EnumTable::from_dict(&range("PA", 0, 4)).unwrap_err();
    assert!(matches!(error, ParseError::BadEnumRange { ref name } if name == "PA"));
}

#[test]
fn a_range_name_suffix_that_overflows_i32_is_refused() {
    let error = EnumTable::from_dict(&range("PA99999999999", 0, 4)).unwrap_err();
    assert!(matches!(error, ParseError::BadEnumRange { .. }));
}

#[test]
fn a_range_that_runs_past_i32_is_refused() {
    let error = EnumTable::from_dict(&range("PA2147483646", 0, 4)).unwrap_err();
    assert!(matches!(error, ParseError::BadEnumRange { .. }));
}
