use gcode::{Token, lex};
use proptest::prelude::*;
use proptest::test_runner::FileFailurePersistence;

#[derive(Debug)]
enum GeneratedLine {
    Command {
        text: String,
        letter: u8,
        major: u32,
        minor: Option<u32>,
        params: Vec<(u8, String)>,
        line_no: u32,
    },
    Extended {
        text: String,
        name: String,
        args: Vec<(String, String)>,
        line_no: u32,
    },
}

fn decimal_literal(value: i32, precision: u8) -> String {
    if precision == 0 {
        return value.to_string();
    }
    let scale = 10_i64.pow(u32::from(precision));
    let value = i64::from(value);
    let sign = if value < 0 { "-" } else { "" };
    let magnitude = value.abs();
    format!(
        "{sign}{}.{:0width$}",
        magnitude / scale,
        magnitude % scale,
        width = usize::from(precision)
    )
}

fn whitespace() -> impl Strategy<Value = String> {
    prop::collection::vec(prop_oneof![Just(' '), Just('\t')], 1..5)
        .prop_map(|chars| chars.into_iter().collect())
}

fn command_line() -> impl Strategy<Value = GeneratedLine> {
    (
        prop_oneof![Just(b'G'), Just(b'M')],
        0_u32..10_000,
        prop::option::of(0_u32..1000),
        prop::collection::vec(prop::option::of((-2_000_000_i32..=2_000_000, 0_u8..=6)), 6),
        any::<bool>(),
        whitespace(),
        prop::option::of("[ -~]{0,24}"),
        0_u8..4,
        any::<bool>(),
    )
        .prop_map(
            |(letter, major, minor, values, lowercase, ws, comment, blank_lines, crlf)| {
                let rendered_letter = if lowercase {
                    letter.to_ascii_lowercase()
                } else {
                    letter
                };
                let mut command = format!("{}{}", rendered_letter as char, major);
                if let Some(minor) = minor {
                    command.push_str(&format!(".{minor}"));
                }
                let mut params = Vec::new();
                for (axis, value) in [b'X', b'Y', b'Z', b'E', b'F', b'S'].into_iter().zip(values) {
                    if let Some((raw, precision)) = value {
                        let literal = decimal_literal(raw, precision);
                        let rendered_axis = if lowercase {
                            axis.to_ascii_lowercase()
                        } else {
                            axis
                        };
                        command.push_str(&ws);
                        command.push(rendered_axis as char);
                        command.push_str(&literal);
                        params.push((axis, literal));
                    }
                }
                if let Some(comment) = comment {
                    command.push_str(&ws);
                    command.push(';');
                    command.push_str(&comment.replace(['\r', '\n'], ""));
                }
                let ending = if crlf { "\r\n" } else { "\n" };
                let text = format!("{}{}{}", ending.repeat(blank_lines.into()), command, ending);
                GeneratedLine::Command {
                    text,
                    letter,
                    major,
                    minor,
                    params,
                    line_no: u32::from(blank_lines) + 1,
                }
            },
        )
}

fn extended_line() -> impl Strategy<Value = GeneratedLine> {
    (
        prop::collection::vec(
            (
                "[A-Z][A-Z0-9_]{0,7}",
                -2_000_000_i32..=2_000_000,
                0_u8..=6,
                any::<bool>(),
            ),
            0..7,
        ),
        any::<bool>(),
        whitespace(),
        prop::option::of("[ -~]{0,24}"),
        0_u8..4,
        any::<bool>(),
    )
        .prop_map(|(raw_args, lowercase, ws, comment, blank_lines, crlf)| {
            let canonical_name = "SET_VALUE".to_owned();
            let name = if lowercase {
                canonical_name.to_ascii_lowercase()
            } else {
                canonical_name.clone()
            };
            let mut line = name;
            let mut args = Vec::new();
            for (key, raw, precision, quoted) in raw_args {
                let literal = decimal_literal(raw, precision);
                let value = if quoted {
                    format!("\"{literal}\"")
                } else {
                    literal
                };
                let rendered_key = if lowercase {
                    key.to_ascii_lowercase()
                } else {
                    key.clone()
                };
                line.push_str(&ws);
                line.push_str(&rendered_key);
                line.push('=');
                line.push_str(&value);
                args.push((rendered_key, value));
            }
            if let Some(comment) = comment {
                line.push_str(&ws);
                line.push(';');
                line.push_str(&comment.replace(['\r', '\n'], ""));
            }
            let ending = if crlf { "\r\n" } else { "\n" };
            let text = format!("{}{}{}", ending.repeat(blank_lines.into()), line, ending);
            GeneratedLine::Extended {
                text,
                name: canonical_name,
                args,
                line_no: u32::from(blank_lines) + 1,
            }
        })
}

fn generated_line() -> impl Strategy<Value = GeneratedLine> {
    prop_oneof![command_line(), extended_line()]
}

fn serialize(token: &Token) -> String {
    match token {
        Token::Command {
            letter,
            major,
            minor,
            params,
            ..
        } => {
            let mut line = format!("{}{}", *letter as char, major);
            if let Some(minor) = minor {
                line.push_str(&format!(".{minor}"));
            }
            for letter in b'A'..=b'Z' {
                if let Some(value) = params.get(letter) {
                    line.push_str(&format!(" {}{value}", letter as char));
                }
            }
            line
        }
        Token::Extended { name, args, .. } => {
            let mut line = name.to_string();
            for (key, value) in args {
                line.push_str(&format!(" {key}={value}"));
            }
            line
        }
        other => panic!("generated executable line produced {other:?}"),
    }
}

proptest! {
    #![proptest_config(ProptestConfig {
        cases: 512,
        failure_persistence: Some(Box::new(FileFailurePersistence::Direct(
            "proptest-regressions/lex_roundtrip.txt",
        ))),
        ..ProptestConfig::default()
    })]

    #[test]
    fn structured_lines_lex_exactly_and_relex_to_a_fixed_point(line in generated_line()) {
        let mut tokens = lex(match &line { GeneratedLine::Command { text, .. } | GeneratedLine::Extended { text, .. } => text });
        let token = tokens.next().expect("one generated line").expect("well-formed generated line");
        prop_assert!(tokens.next().is_none(), "inline comment must not become a token");

        match (&line, &token) {
            (GeneratedLine::Command { letter, major, minor, params: expected, line_no, .. }, Token::Command { letter: actual_letter, major: actual_major, minor: actual_minor, params, line_no: actual_line }) => {
                prop_assert_eq!(actual_letter, letter);
                prop_assert_eq!(actual_major, major);
                prop_assert_eq!(actual_minor, minor);
                prop_assert_eq!(actual_line, line_no);
                for axis in [b'X', b'Y', b'Z', b'E', b'F', b'S'] {
                    let expected_value = expected.iter().find(|(key, _)| *key == axis).map(|(_, literal)| literal.parse::<f64>().expect("generated decimal"));
                    prop_assert_eq!(params.get(axis), expected_value);
                }
            }
            (GeneratedLine::Extended { name, args: expected, line_no, .. }, Token::Extended { name: actual_name, args, line_no: actual_line }) => {
                prop_assert_eq!(actual_name.as_ref(), name);
                prop_assert_eq!(actual_line, line_no);
                prop_assert_eq!(args.len(), expected.len());
                for ((actual_key, actual_value), (expected_key, expected_value)) in args.iter().zip(expected) {
                    prop_assert_eq!(actual_key.as_ref(), expected_key);
                    prop_assert_eq!(actual_value.as_ref(), expected_value);
                    let actual_float = actual_value.trim_matches('"').parse::<f64>().expect("generated decimal value");
                    let expected_float = expected_value.trim_matches('"').parse::<f64>().expect("generated decimal value");
                    prop_assert_eq!(actual_float, expected_float);
                }
            }
            _ => prop_assert!(false, "wrong token variant: {token:?}"),
        }

        let serialized = serialize(&token);
        let reparsed = lex(&serialized).next().expect("serialized token").expect("serialized token is valid");
        prop_assert_eq!(serialize(&reparsed), serialized);
    }
}
