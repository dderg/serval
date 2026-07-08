use super::*;

#[test]
fn empty_input_returns_empty_lists() {
    let (axes, pps) = parse("").unwrap();
    assert!(axes.is_empty());
    assert!(pps.is_empty());
}

#[test]
fn blank_lines_and_comments_are_ignored() {
    let (axes, pps) = parse("\n# a comment\n   \n").unwrap();
    assert!(axes.is_empty());
    assert!(pps.is_empty());
}

#[test]
fn parses_one_axis_and_one_post_processor() {
    let (axes, pps) = parse(
        "[axis e]\npost_processors: pa\n\n[post_processor pa]\ntype: linear_pressure_advance\nk: 0.03\n",
    )
    .unwrap();
    assert_eq!(axes.len(), 1);
    assert_eq!(axes[0].name, "e");
    assert_eq!(axes[0].post_processors, vec!["pa".to_string()]);
    assert_eq!(pps.len(), 1);
    assert_eq!(pps[0].name, "pa");
    assert_eq!(pps[0].ty, "linear_pressure_advance");
    assert_eq!(pps[0].params, vec![("k".to_string(), 0.03)]);
}

#[test]
fn comma_list_supports_multiple_post_processors_on_one_axis() {
    let (axes, _) = parse("[axis e]\npost_processors: pa, st\n").unwrap();
    assert_eq!(
        axes[0].post_processors,
        vec!["pa".to_string(), "st".to_string()]
    );
}

#[test]
fn follows_and_motors_lines_are_accepted_and_ignored() {
    let (axes, _) =
        parse("[axis e]\nfollows: x, y, z\nmotors: extruder\npost_processors: pa\n").unwrap();
    assert_eq!(axes[0].name, "e");
    assert!(axes[0].follows.is_empty());
    assert!(axes[0].motors.is_empty());
    assert_eq!(axes[0].post_processors, vec!["pa".to_string()]);
}

#[test]
fn missing_type_line_is_an_error() {
    let err = parse("[post_processor pa]\nk: 0.03\n").unwrap_err();
    assert!(matches!(err, ConfigTextError::MissingType { name } if name == "pa"));
}

#[test]
fn non_numeric_param_value_is_an_error() {
    let err = parse("[post_processor pa]\ntype: linear_pressure_advance\nk: nope\n").unwrap_err();
    assert!(matches!(err, ConfigTextError::BadNumber { key, .. } if key == "k"));
}

#[test]
fn key_value_outside_any_section_is_an_error() {
    let err = parse("k: 0.03\n").unwrap_err();
    assert!(matches!(err, ConfigTextError::NoActiveSection { line: 1 }));
}

#[test]
fn malformed_section_header_is_an_error() {
    let err = parse("[bogus thing]\n").unwrap_err();
    assert!(matches!(err, ConfigTextError::BadLine { line: 1, .. }));
}
