use super::*;

fn collect(text: &str) -> Vec<Result<Token, ParseError>> {
    lex(text).collect()
}

#[test]
fn empty_input_yields_nothing() {
    assert!(collect("").is_empty());
}

#[test]
fn whitespace_only_yields_nothing() {
    assert!(collect("   \n\t  \n").is_empty());
}

#[test]
fn pure_comment_yields_comment_token() {
    let toks = collect("; just a comment\n");
    assert_eq!(toks.len(), 1);
    match &toks[0] {
        Ok(Token::Comment { text, line_no }) => {
            assert_eq!(text.as_ref(), "just a comment");
            assert_eq!(*line_no, 1);
        }
        other => panic!("expected Comment, got {other:?}"),
    }
}

#[test]
fn line_numbers_are_one_indexed() {
    let toks = collect("\n\n; third line\n");
    assert_eq!(toks.len(), 1);
    match &toks[0] {
        Ok(Token::Comment { line_no, .. }) => assert_eq!(*line_no, 3),
        other => panic!("expected Comment, got {other:?}"),
    }
}

#[test]
fn parses_g1_with_xy() {
    let toks = collect("G1 X10 Y-5\n");
    assert_eq!(toks.len(), 1);
    match &toks[0] {
        Ok(Token::Command {
            letter,
            major,
            minor,
            params,
            line_no,
        }) => {
            assert_eq!(*letter, b'G');
            assert_eq!(*major, 1);
            assert_eq!(*minor, None);
            assert_eq!(params.x(), Some(10.0));
            assert_eq!(params.y(), Some(-5.0));
            assert_eq!(params.z(), None);
            assert_eq!(*line_no, 1);
        }
        other => panic!("expected Command, got {other:?}"),
    }
}

#[test]
fn parses_g1_with_decimal_params() {
    let toks = collect("G1 X1.234 Y5.678 E0.123 F1500\n");
    assert_eq!(toks.len(), 1);
    match &toks[0] {
        Ok(Token::Command { params, .. }) => {
            assert_eq!(params.x(), Some(1.234));
            assert_eq!(params.y(), Some(5.678));
            assert_eq!(params.e(), Some(0.123));
            assert_eq!(params.f(), Some(1500.0));
        }
        other => panic!("expected Command, got {other:?}"),
    }
}

#[test]
fn malformed_number_returns_error() {
    let toks = collect("G1 X1.2.3\n");
    assert_eq!(toks.len(), 1);
    match &toks[0] {
        Err(ParseError::MalformedNumber { line_no: 1, .. }) => {}
        other => panic!("expected MalformedNumber, got {other:?}"),
    }
}

#[test]
fn duplicate_param_returns_error() {
    let toks = collect("G1 X1 X2\n");
    assert_eq!(toks.len(), 1);
    match &toks[0] {
        Err(ParseError::DuplicateParam {
            line_no: 1,
            letter: 'X',
        }) => {}
        other => panic!("expected DuplicateParam, got {other:?}"),
    }
}

#[test]
fn inline_comment_is_stripped() {
    let toks = collect("G1 X1.0 Y2.0 ; trailing comment\n");
    assert_eq!(toks.len(), 1);
    match &toks[0] {
        Ok(Token::Command { params, .. }) => {
            assert_eq!(params.x(), Some(1.0));
            assert_eq!(params.y(), Some(2.0));
        }
        other => panic!("expected Command, got {other:?}"),
    }
}

#[test]
fn missing_param_value_returns_error() {
    let toks = collect("G1 X\n");
    assert_eq!(toks.len(), 1);
    match &toks[0] {
        Err(ParseError::MalformedNumber { line_no: 1, .. }) => {}
        other => panic!("expected MalformedNumber for missing value, got {other:?}"),
    }
}

#[test]
fn numeric_token_without_letter_returns_error() {
    let toks = collect("G1 1.0\n");
    assert_eq!(toks.len(), 1);
    match &toks[0] {
        Err(ParseError::MalformedNumber { line_no: 1, .. }) => {}
        other => panic!("expected MalformedNumber for digit-leading param, got {other:?}"),
    }
}

#[test]
fn lowercase_param_letter_is_normalized() {
    let toks = collect("G1 x10\n");
    assert_eq!(toks.len(), 1);
    match &toks[0] {
        Ok(Token::Command { params, .. }) => assert_eq!(params.x(), Some(10.0)),
        other => panic!("expected normalized parameter, got {other:?}"),
    }
}

#[test]
fn multibyte_whitespace_after_head_letter_returns_error() {
    let toks = collect("A\u{85}0\t");
    assert_eq!(toks.len(), 1);
    match &toks[0] {
        Err(ParseError::UnrecognizedHead { line_no: 1, .. }) => {}
        other => panic!("expected UnrecognizedHead, got {other:?}"),
    }
}

#[test]
fn ascii_space_after_head_letter_returns_error() {
    let toks = collect("G 1 X0\n");
    assert_eq!(toks.len(), 1);
    match &toks[0] {
        Err(ParseError::UnrecognizedHead { line_no: 1, .. }) => {}
        other => panic!("expected UnrecognizedHead, got {other:?}"),
    }
}

#[test]
fn head_with_no_number_returns_error() {
    let toks = collect("G\n");
    assert_eq!(toks.len(), 1);
    match &toks[0] {
        Err(ParseError::UnrecognizedHead { line_no: 1, .. }) => {}
        other => panic!("expected UnrecognizedHead for bare head letter, got {other:?}"),
    }
}

#[test]
fn parses_g5_1() {
    let toks = collect("G5.1 X10 Y20 I1 J2\n");
    assert_eq!(toks.len(), 1);
    match &toks[0] {
        Ok(Token::Command {
            letter,
            major,
            minor,
            ..
        }) => {
            assert_eq!(*letter, b'G');
            assert_eq!(*major, 5);
            assert_eq!(*minor, Some(1));
        }
        other => panic!("expected Command, got {other:?}"),
    }
}

#[test]
fn parses_m104() {
    let toks = collect("M104 S210\n");
    assert_eq!(toks.len(), 1);
    match &toks[0] {
        Ok(Token::Command {
            letter,
            major,
            params,
            ..
        }) => {
            assert_eq!(*letter, b'M');
            assert_eq!(*major, 104);
            assert_eq!(params.get(b'S'), Some(210.0));
        }
        other => panic!("expected Command, got {other:?}"),
    }
}

#[test]
fn parses_t0() {
    let toks = collect("T0\n");
    assert_eq!(toks.len(), 1);
    match &toks[0] {
        Ok(Token::Command { letter, major, .. }) => {
            assert_eq!(*letter, b'T');
            assert_eq!(*major, 0);
        }
        other => panic!("expected Command, got {other:?}"),
    }
}

#[test]
fn layer_change_comment_is_marker_token() {
    let toks = collect(";LAYER:5\n");
    assert_eq!(toks.len(), 1);
    match &toks[0] {
        Ok(Token::Marker { kind, line_no }) => {
            assert_eq!(
                *kind,
                crate::marker::MarkerKind::LayerChange { layer: Some(5) }
            );
            assert_eq!(*line_no, 1);
        }
        other => panic!("expected Marker, got {other:?}"),
    }
}

#[test]
fn extended_command_lexes_name_and_args() {
    let toks = collect("SET_VELOCITY_LIMIT ACCEL=3000 VELOCITY=250.5\n");
    assert_eq!(toks.len(), 1);
    match &toks[0] {
        Ok(Token::Extended {
            name,
            args,
            line_no,
        }) => {
            assert_eq!(name.as_ref(), "SET_VELOCITY_LIMIT");
            assert_eq!(*line_no, 1);
            assert_eq!(
                args.iter()
                    .map(|(k, v)| (k.as_ref(), v.as_ref()))
                    .collect::<Vec<_>>(),
                vec![("ACCEL", "3000"), ("VELOCITY", "250.5")]
            );
        }
        other => panic!("expected Extended, got {other:?}"),
    }
}

#[test]
fn extended_command_keeps_non_numeric_arg_values_raw() {
    let toks = collect("EXCLUDE_OBJECT_DEFINE NAME=part1 CENTER=104,100\n");
    match &toks[0] {
        Ok(Token::Extended { name, args, .. }) => {
            assert_eq!(name.as_ref(), "EXCLUDE_OBJECT_DEFINE");
            assert_eq!(args[0].1.as_ref(), "part1");
            assert_eq!(args[1].1.as_ref(), "104,100");
        }
        other => panic!("expected Extended, got {other:?}"),
    }
}

#[test]
fn extended_command_arg_without_equals_is_an_error() {
    let toks = collect("SET_VELOCITY_LIMIT ACCEL\n");
    assert!(matches!(
        &toks[0],
        Err(ParseError::MalformedNumber { line_no: 1, .. })
    ));
}

#[test]
fn lone_uppercase_letter_is_still_an_error() {
    let toks = collect("M\n");
    assert!(matches!(
        &toks[0],
        Err(ParseError::UnrecognizedHead { line_no: 1, .. })
    ));
}

#[test]
fn lowercase_commands_and_parameters_are_normalized() {
    let tokens = collect("g1 x1.25 y-2\n");
    match &tokens[0] {
        Ok(Token::Command { letter, params, .. }) => {
            assert_eq!(*letter, b'G');
            assert_eq!(params.x(), Some(1.25));
            assert_eq!(params.y(), Some(-2.0));
        }
        other => panic!("expected normalized command, got {other:?}"),
    }
}
