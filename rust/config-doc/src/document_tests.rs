use crate::Document;

#[test]
fn mutation_for_autosave() {
    let mut d = Document::default();
    d.add_section("bed_mesh default").unwrap();
    assert!(d.add_section("bed_mesh default").is_err());
    d.set("bed_mesh default", "Points", "1.0, 2.0").unwrap();
    assert!(d.has_option("bed_mesh default", "points"));
    assert_eq!(d.get("bed_mesh default", "points").unwrap().0, "1.0, 2.0");
    assert!(d.remove_section("bed_mesh default"));
    assert!(!d.remove_section("bed_mesh default"));
    assert!(!d.has_section("bed_mesh default"));
}

#[test]
fn set_on_missing_section_errors() {
    let mut d = Document::default();
    assert!(d.set("nope", "k", "v").is_err());
}

#[test]
fn queries_lowercase_option_names_only() {
    let d = Document::parse("[Sec]\nKey: v\n", "x.cfg").unwrap();
    assert!(d.has_option("Sec", "KEY"));
    assert!(!d.has_option("sec", "key"));
    assert!(d.get_raw("Sec", "Key").is_ok());
    assert!(d.options("sec").is_err());
}
