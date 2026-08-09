use crate::Document;

fn doc(data: &str) -> Document {
    Document::parse(data, "/nonexistent-dir/printer.cfg").expect("parse ok")
}

#[test]
fn same_section_reference() {
    let d = doc("[a]\nbase: 10\nk: ${base}mm\n");
    assert_eq!(d.get("a", "k").unwrap().0, "10mm");
}

#[test]
fn cross_section_dot_and_colon() {
    let d = doc("[vars]\nspeed: 300\n[a]\nx: ${vars.speed}\ny: ${vars:speed}\n");
    assert_eq!(d.get("a", "x").unwrap().0, "300");
    assert_eq!(d.get("a", "y").unwrap().0, "300");
}

#[test]
fn option_part_may_contain_dots() {
    // KEYCRE: section ends at the FIRST '.' — the rest is the option name.
    let d = doc("[vars]\nmy.opt: 7\n[a]\nx: ${vars.my.opt}\n");
    assert_eq!(d.get("a", "x").unwrap().0, "7");
}

#[test]
fn referenced_values_resolve_recursively() {
    let d = doc("[v]\na: 1\nb: ${a}2\n[x]\nk: ${v.b}3\n");
    assert_eq!(d.get("x", "k").unwrap().0, "123");
}

#[test]
fn missing_reference_errors() {
    let d = doc("[a]\nk: ${nope}\n");
    assert!(d.get("a", "k").is_err());
    let d = doc("[a]\nk: ${other.opt}\n");
    assert!(d.get("a", "k").is_err());
}

#[test]
fn cycle_errors_instead_of_overflowing() {
    let d = doc("[a]\nx: ${y}\ny: ${x}\n");
    assert!(d.get("a", "x").is_err());
}

#[test]
fn escape_yields_literal() {
    let d = doc("[a]\nbase: 1\nk: \\${base}\n");
    assert_eq!(d.get("a", "k").unwrap().0, "${base}");
}

#[test]
fn substitution_budget_leaves_remainder_literal() {
    let refs: String = (0..12).map(|_| "${b}".to_string()).collect();
    let d = doc(&format!("[a]\nb: v\nk: {refs}\n"));
    let out = d.get("a", "k").unwrap().0;
    assert_eq!(out, format!("{}{}", "v".repeat(10), "${b}".repeat(2)));
}

#[test]
fn refs_recorded_once_with_interpolated_values() {
    let d = doc("[v]\na: 1\nb: ${a}2\n[x]\nk: ${v.b}-${v.b}-${v.a}\n");
    let (value, refs) = d.get("x", "k").unwrap();
    assert_eq!(value, "12-12-1");
    let as_tuples: Vec<(&str, &str, &str)> = refs
        .iter()
        .map(|r| (r.section.as_str(), r.option.as_str(), r.value.as_str()))
        .collect();
    // Nested ref (v, a) is consulted while resolving (v, b), so it appears
    // first; each pair recorded once (setdefault semantics).
    assert_eq!(as_tuples, vec![("v", "a", "1"), ("v", "b", "12")]);
}

#[test]
fn implicit_section_recorded_with_current_section_name() {
    let d = doc("[a]\nbase: 5\nk: ${base}\n");
    let (_, refs) = d.get("a", "k").unwrap();
    assert_eq!(refs.len(), 1);
    assert_eq!(refs[0].section, "a");
    assert_eq!(refs[0].option, "base");
}

#[test]
fn get_raw_is_uninterpolated() {
    let d = doc("[a]\nbase: 1\nk: ${base}\n");
    assert_eq!(d.get_raw("a", "k").unwrap(), "${base}");
}

#[test]
fn invalid_reference_shapes_stay_literal() {
    let d = doc("[a]\nk: ${} and ${a$b} and $ {x}\n");
    assert_eq!(d.get("a", "k").unwrap().0, "${} and ${a$b} and $ {x}");
}

#[test]
fn dotted_edge_references_resolve_as_whole_option_names() {
    // KEYCRE backtracking: '${.host}' and '${a.}' are options '.host' and
    // 'a.' of the current section, not malformed references.
    let d = doc("[s]\n.host: 1.2.3.4\na.: x\nu: ${.host}\nv: ${a.}\n");
    assert_eq!(d.get("s", "u").unwrap().0, "1.2.3.4");
    assert_eq!(d.get("s", "v").unwrap().0, "x");
    let d = doc("[s]\nk: ${.typo}\n");
    assert!(d.get("s", "k").is_err());
}

#[test]
fn escape_applies_without_a_complete_reference() {
    let d = doc("[a]\nk: pre \\${x post\n");
    assert_eq!(d.get("a", "k").unwrap().0, "pre ${x post");
}
