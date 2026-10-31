use super::*;

#[test]
fn encodes_string_length_prefixed() {
    let mut buf = Vec::new();
    encode_field_str(
        &mut buf,
        &WrappedField::Plain(FieldType::String),
        "hi",
        &IndexMap::new(),
    )
    .unwrap();
    assert_eq!(buf, vec![2, b'h', b'i']);
}

#[test]
fn encodes_byte_via_vlq() {
    let mut buf = Vec::new();
    encode_field_num(&mut buf, FieldType::Byte, 0xFF).unwrap();
    assert_eq!(buf, vec![0x81, 0x7F]);
}

#[test]
fn byte_field_accepts_signed_negative() {
    use indexmap::IndexMap;
    let enums: IndexMap<String, EnumTable> = IndexMap::new();
    for v in &["-1", "-128", "0", "127", "255"] {
        let mut buf = Vec::new();
        encode_field_str(&mut buf, &WrappedField::Plain(FieldType::Byte), v, &enums)
            .unwrap_or_else(|e| panic!("Byte should accept {v:?}: {e:?}"));
        assert_eq!(decode_vlq(&buf).unwrap(), (v.parse().unwrap(), buf.len()));
    }
}

#[test]
fn byte_field_still_rejects_truly_out_of_range() {
    use indexmap::IndexMap;
    let enums: IndexMap<String, EnumTable> = IndexMap::new();
    for v in &["-129", "256", "1000", "-1000"] {
        let mut buf = Vec::new();
        let r = encode_field_str(&mut buf, &WrappedField::Plain(FieldType::Byte), v, &enums);
        assert!(
            matches!(r, Err(ParseError::OutOfRange { .. })),
            "Byte should reject {v:?}, got {r:?}"
        );
    }
}

#[test]
fn parse_hex_buffer_round_trips() {
    assert_eq!(
        parse_hex_buffer("0123abcd").unwrap(),
        vec![0x01, 0x23, 0xAB, 0xCD]
    );
    assert_eq!(parse_hex_buffer("").unwrap(), Vec::<u8>::new());
    assert!(matches!(parse_hex_buffer("0z"), Err(ParseError::BadHex(_))));
    assert!(matches!(parse_hex_buffer("1"), Err(ParseError::BadHex(_))));
}
