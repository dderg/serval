use super::*;

fn parser_with_one_command() -> MsgProtoParser {
    let mut d = DataDictionary {
        commands: IndexMap::new(),
        responses: IndexMap::new(),
        output: IndexMap::new(),
        enumerations: IndexMap::new(),
        config: serde_json::json!({}),
        version: "v".into(),
        app: "kalico".into(),
        build_versions: None,
        license: None,
    };
    d.commands.insert("ping val=%u".into(), 42);
    MsgProtoParser::from_dictionary(d).unwrap()
}

#[test]
fn string_and_typed_encode_to_same_bytes() {
    let p = parser_with_one_command();
    let bytes_str = p.encode("ping val=100").unwrap();
    let bytes_typed = p
        .encode_typed("ping", &[("val", FieldValue::U32(100))])
        .unwrap();
    assert_eq!(bytes_str, bytes_typed);
}

#[test]
fn encode_rejects_unknown_command() {
    let p = parser_with_one_command();
    match p.encode("unknown_cmd") {
        Err(ParseError::UnknownCommand(_)) => {}
        other => panic!("expected UnknownCommand, got {:?}", other),
    }
}

#[test]
fn encode_rejects_missing_field() {
    let p = parser_with_one_command();
    match p.encode("ping") {
        Err(ParseError::MissingField(_)) => {}
        other => panic!("expected MissingField, got {:?}", other),
    }
}

#[test]
fn enum_encode_rejects_unknown_name() {
    let mut d = DataDictionary {
        commands: IndexMap::new(),
        responses: IndexMap::new(),
        output: IndexMap::new(),
        enumerations: IndexMap::new(),
        config: serde_json::json!({}),
        version: "v".into(),
        app: "kalico".into(),
        build_versions: None,
        license: None,
    };
    d.commands.insert("config_pin pin=%c".into(), 1);
    let mut pin_table = IndexMap::new();
    pin_table.insert("PA0".to_string(), EnumValue::Single(0));
    d.enumerations.insert("pin".to_string(), pin_table);

    let p = MsgProtoParser::from_dictionary(d).unwrap();
    match p.encode("config_pin pin=PZZZ") {
        Err(ParseError::UnknownEnumValue { value, .. }) => assert_eq!(value, "PZZZ"),
        other => panic!("expected UnknownEnumValue, got {:?}", other),
    }
}

fn dict() -> DataDictionary {
    DataDictionary {
        commands: IndexMap::new(),
        responses: IndexMap::new(),
        output: IndexMap::new(),
        enumerations: IndexMap::new(),
        config: serde_json::json!({}),
        version: "v".into(),
        app: "kalico".into(),
        build_versions: None,
        license: None,
    }
}

fn args_parity_parser() -> MsgProtoParser {
    let mut d = dict();
    d.commands
        .insert("mix a=%c b=%hu c=%hi d=%u e=%i name=%s buf=%*s".into(), 10);
    d.commands.insert("config_pin pin=%c".into(), 11);
    let mut pin_table = IndexMap::new();
    pin_table.insert("PA0".to_string(), EnumValue::Single(5));
    d.enumerations.insert("pin".to_string(), pin_table);
    MsgProtoParser::from_dictionary(d).unwrap()
}

fn int_args(pairs: &[(&str, i64)]) -> Vec<(String, ArgValue)> {
    pairs
        .iter()
        .map(|(k, v)| ((*k).to_string(), ArgValue::Int(*v)))
        .collect()
}

#[test]
fn args_encode_matches_string_encode_across_field_types() {
    let p = args_parity_parser();
    let by_str = p
        .encode("mix a=-5 b=65535 c=-32768 d=4294967295 e=-7 name=hello buf=00ff10")
        .unwrap();
    let mut args = int_args(&[
        ("a", -5),
        ("b", 65535),
        ("c", -32768),
        ("d", 4294967295),
        ("e", -7),
    ]);
    args.push(("name".to_string(), ArgValue::Str("hello".into())));
    args.push(("buf".to_string(), ArgValue::Bytes(vec![0x00, 0xff, 0x10])));
    let by_args = p.encode_args("mix", &args).unwrap();
    assert_eq!(by_str, by_args);
}

#[test]
fn args_encode_wraps_negative_u32_like_string_path() {
    let p = args_parity_parser();
    let by_str = p.encode("mix a=0 b=0 c=0 d=-1 e=0 name=x buf=").unwrap();
    let mut args = int_args(&[("a", 0), ("b", 0), ("c", 0), ("d", -1), ("e", 0)]);
    args.push(("name".to_string(), ArgValue::Str("x".into())));
    args.push(("buf".to_string(), ArgValue::Bytes(vec![])));
    let by_args = p.encode_args("mix", &args).unwrap();
    assert_eq!(by_str, by_args);
}

#[test]
fn args_encode_resolves_enum_names_like_string_path() {
    let p = args_parity_parser();
    let by_str = p.encode("config_pin pin=PA0").unwrap();
    let by_args = p
        .encode_args(
            "config_pin",
            &[("pin".to_string(), ArgValue::Str("PA0".into()))],
        )
        .unwrap();
    assert_eq!(by_str, by_args);
}

#[test]
fn args_encode_rejects_int_for_enum_like_string_path() {
    let p = args_parity_parser();
    match p.encode_args("config_pin", &[("pin".to_string(), ArgValue::Int(5))]) {
        Err(ParseError::UnknownEnumValue { value, .. }) => assert_eq!(value, "5"),
        other => panic!("expected UnknownEnumValue, got {:?}", other),
    }
}

#[test]
fn args_encode_range_checks_like_string_path() {
    let p = args_parity_parser();
    let args = int_args(&[("a", 256), ("b", 0), ("c", 0), ("d", 0), ("e", 0)]);
    match p.encode_args("mix", &args) {
        Err(ParseError::OutOfRange { value, .. }) => assert_eq!(value, 256),
        other => panic!("expected OutOfRange, got {:?}", other),
    }
}

#[test]
fn args_encode_rejects_missing_field_and_type_mismatch() {
    let p = args_parity_parser();
    match p.encode_args("mix", &[]) {
        Err(ParseError::MissingField(_)) => {}
        other => panic!("expected MissingField, got {:?}", other),
    }
    let mut args = int_args(&[("a", 0), ("b", 0), ("c", 0), ("d", 0), ("e", 0)]);
    args.push(("name".to_string(), ArgValue::Str("x".into())));
    args.push(("buf".to_string(), ArgValue::Int(3)));
    match p.encode_args("mix", &args) {
        Err(ParseError::ArgTypeMismatch { field, got }) => {
            assert_eq!(field, "buf");
            assert_eq!(got, "int");
        }
        other => panic!("expected ArgTypeMismatch, got {:?}", other),
    }
}
