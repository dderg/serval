use sha2::{Digest, Sha256};

include!("../schema_def.rs");

fn sha256(s: &str) -> [u8; 32] {
    let mut h = Sha256::new();
    h.update(s.as_bytes());
    h.finalize().into()
}

fn first_with_fields() -> usize {
    SCHEMA_MESSAGES
        .iter()
        .position(|m| !m.fields.is_empty())
        .expect("schema must contain a message with fields")
}

#[test]
fn schema_hash_is_deterministic_and_matches_published_constant() {
    let text = canonicalize_schema(SCHEMA_MESSAGES);
    let h1 = sha256(&text);
    let h2 = sha256(&text);
    assert_eq!(h1, h2, "SHA-256 must be deterministic");
    assert_eq!(
        h1,
        mcu_protocol::SCHEMA_HASH,
        "test-side hash must match build.rs-emitted SCHEMA_HASH"
    );
    assert_eq!(text, mcu_protocol::SCHEMA_CANONICAL);
}

#[test]
fn schema_hash_changes_when_a_field_type_changes() {
    let f = SCHEMA_MESSAGES[first_with_fields()].fields[0];
    let text = canonicalize_schema(SCHEMA_MESSAGES);
    let mutated = text.replacen(
        &format!("[{}:{}", f.name, f.ty),
        &format!("[{}:u64", f.name),
        1,
    );
    assert_ne!(mutated, text, "the mutation must have landed");
    assert_ne!(
        sha256(&mutated),
        mcu_protocol::SCHEMA_HASH,
        "a field-type change must produce a different schema_hash"
    );
}

#[test]
fn schema_hash_changes_when_a_field_is_added() {
    let i = first_with_fields();
    let mut lines: Vec<String> = canonicalize_schema(SCHEMA_MESSAGES)
        .lines()
        .map(str::to_owned)
        .collect();
    lines[i] = format!(
        "{},new_field:u32]",
        lines[i]
            .strip_suffix(']')
            .expect("canonical line ends with ]")
    );
    let mutated = lines.join("\n") + "\n";
    assert_ne!(sha256(&mutated), mcu_protocol::SCHEMA_HASH);
}

#[test]
fn schema_hash_changes_when_a_version_bumps() {
    let mut messages: Vec<SchemaMessage> = SCHEMA_MESSAGES.to_vec();
    messages[0].version = SCHEMA_MESSAGES[0].version + 1;
    let mutated_hash = sha256(&canonicalize_schema(&messages));
    assert_ne!(mutated_hash, mcu_protocol::SCHEMA_HASH);
}
