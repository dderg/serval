use crate::Document;

#[test]
fn configparser_write_format() {
    let mut d = Document::default();
    d.add_section("a").unwrap();
    d.set("a", "x", "1").unwrap();
    d.set("a", "empty", "").unwrap();
    d.add_section("b c").unwrap();
    d.set("b c", "multi", "\nG28\nG1 X0").unwrap();
    assert_eq!(
        d.write_string(),
        "[a]\nx = 1\nempty = \n\n[b c]\nmulti = \n\tG28\n\tG1 X0\n\n"
    );
}

#[test]
fn write_reparse_roundtrip() {
    let src = "[gcode_macro m]\ngcode:\n  G28\n  G1 X0\n[a]\nx: 5\n";
    let d = Document::parse(src, "x.cfg").unwrap();
    let re = Document::parse(&d.write_string(), "y.cfg").unwrap();
    assert_eq!(
        re.get("gcode_macro m", "gcode").unwrap().0,
        d.get("gcode_macro m", "gcode").unwrap().0
    );
    assert_eq!(re.get("a", "x").unwrap().0, "5");
}
