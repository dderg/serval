use crate::Document;

fn parse(data: &str) -> Document {
    Document::parse(data, "/nonexistent-dir/printer.cfg").expect("parse ok")
}

fn get(doc: &Document, section: &str, option: &str) -> String {
    doc.get(section, option).expect("option present").0
}

#[test]
fn sections_and_both_delimiters() {
    let doc = parse("[printer]\nmax_velocity: 300\nmax_accel = 5000\n");
    assert_eq!(get(&doc, "printer", "max_velocity"), "300");
    assert_eq!(get(&doc, "printer", "max_accel"), "5000");
}

#[test]
fn option_names_lowercase_section_names_verbatim() {
    let doc = parse("[Axis X]\nMax_Travel: 10\n");
    assert!(doc.has_section("Axis X"));
    assert!(!doc.has_section("axis x"));
    assert!(doc.has_option("Axis X", "MAX_TRAVEL"));
    assert_eq!(doc.options("Axis X").unwrap(), vec!["max_travel"]);
}

#[test]
fn first_delimiter_wins() {
    let doc = parse("[a]\nkey: b:c=d\n");
    assert_eq!(get(&doc, "a", "key"), "b:c=d");
}

#[test]
fn multiline_value_keeps_leading_newline_and_rstrips() {
    let doc = parse("[gcode_macro m]\ngcode:\n    G28\n    G1 X0\n\n\n");
    assert_eq!(get(&doc, "gcode_macro m", "gcode"), "\nG28\nG1 X0");
}

#[test]
fn value_on_key_line_with_continuation() {
    let doc = parse("[a]\nk: first\n  second\n");
    assert_eq!(get(&doc, "a", "k"), "first\nsecond");
}

#[test]
fn blank_line_inside_value_is_preserved() {
    let doc = parse("[a]\nk:\n  one\n\n  two\n");
    assert_eq!(get(&doc, "a", "k"), "\none\n\ntwo");
}

#[test]
fn hash_comment_line_inside_value_becomes_blank_line() {
    // klipper pre-strips '#' before the INI parse, so configparser sees a
    // whitespace-only line with no comment and appends an empty value line.
    let doc = parse("[a]\nk:\n  one\n  # note\n  two\n");
    assert_eq!(get(&doc, "a", "k"), "\none\n\ntwo");
}

#[test]
fn semicolon_comment_line_inside_value_is_dropped() {
    let doc = parse("[a]\nk:\n  one\n  ; note\n  two\n");
    assert_eq!(get(&doc, "a", "k"), "\none\ntwo");
}

#[test]
fn hash_comment_strips_anywhere() {
    let doc = parse("[a]\nk: value # trailing\n");
    assert_eq!(get(&doc, "a", "k"), "value");
    let doc = parse("[a]\nk: val#ue\n");
    assert_eq!(get(&doc, "a", "k"), "val");
}

#[test]
fn semicolon_inline_comment_requires_whitespace() {
    let doc = parse("[a]\nk: value ; trailing\n");
    assert_eq!(get(&doc, "a", "k"), "value");
    let doc = parse("[a]\nk: val;ue\n");
    assert_eq!(get(&doc, "a", "k"), "val;ue");
}

#[test]
fn duplicate_sections_merge_duplicate_options_last_wins() {
    let doc = parse("[a]\nx: 1\ny: 2\n[b]\nz: 3\n[a]\nx: 9\n");
    assert_eq!(get(&doc, "a", "x"), "9");
    assert_eq!(get(&doc, "a", "y"), "2");
    // Merged section keeps its first position; replaced option keeps its slot.
    assert_eq!(doc.section_names().collect::<Vec<_>>(), vec!["a", "b"]);
    assert_eq!(doc.options("a").unwrap(), vec!["x", "y"]);
}

#[test]
fn section_header_trailing_junk_ignored() {
    let doc = parse("[a] junk after\nk: 1\n");
    assert_eq!(get(&doc, "a", "k"), "1");
}

#[test]
fn option_less_section_exists() {
    // Enable-only sections ([exclude_object], [respond]) activate modules
    // by presence; configparser materializes them at the header line.
    let doc = parse("[exclude_object]\n[a]\nx: 1\n");
    assert!(doc.has_section("exclude_object"));
    assert_eq!(doc.options("exclude_object").unwrap(), Vec::<String>::new());
    assert_eq!(
        doc.section_names().collect::<Vec<_>>(),
        vec!["exclude_object", "a"]
    );
}

#[test]
fn empty_value_allowed() {
    let doc = parse("[a]\nk:\n");
    assert_eq!(get(&doc, "a", "k"), "");
}

#[test]
fn option_before_section_errors() {
    assert!(Document::parse("k: 1\n[a]\n", "x.cfg").is_err());
}

#[test]
fn junk_line_errors() {
    assert!(Document::parse("[a]\nno delimiter here\n", "x.cfg").is_err());
}

#[test]
fn empty_option_name_errors() {
    assert!(Document::parse("[a]\n= 5\n", "x.cfg").is_err());
    assert!(Document::parse("[a]\n: y\n", "x.cfg").is_err());
}

#[test]
fn unicode_whitespace_counts_as_one_indent_char() {
    // configparser counts indent in characters; U+00A0 must not read as
    // more indent than an ASCII space just because it is multibyte.
    let doc = parse("[s]\n  x: 1\n\u{00a0} y: 2\n");
    assert_eq!(get(&doc, "s", "x"), "1");
    assert_eq!(get(&doc, "s", "y"), "2");
}

#[test]
fn semicolon_after_unicode_whitespace_is_a_comment() {
    let doc = parse("[s]\nx: foo\u{00a0}; secret\n");
    assert_eq!(get(&doc, "s", "x"), "foo");
}

#[test]
fn default_section_rejected() {
    assert!(Document::parse("[DEFAULT]\nk: 1\n", "x.cfg").is_err());
}

#[test]
fn indented_section_header_swallowed_by_value() {
    // An indented [header] while a value is open is a continuation line.
    let doc = parse("[a]\nk:\n  [not a section]\n  more\n");
    assert_eq!(get(&doc, "a", "k"), "\n[not a section]\nmore");
}

mod include_tests {
    use crate::Document;

    fn write(dir: &std::path::Path, name: &str, data: &str) -> String {
        let path = dir.join(name);
        if let Some(parent) = path.parent() {
            std::fs::create_dir_all(parent).unwrap();
        }
        std::fs::write(&path, data).unwrap();
        path.to_string_lossy().into_owned()
    }

    #[test]
    fn include_literal_and_glob() {
        let dir = tempfile::tempdir().unwrap();
        write(dir.path(), "one.cfg", "[a]\nx: 1\n");
        write(dir.path(), "conf.d/m1.cfg", "[b]\ny: 2\n");
        write(dir.path(), "conf.d/m2.cfg", "[b]\ny: 3\nz: 4\n");
        let main = write(
            dir.path(),
            "printer.cfg",
            "[include one.cfg]\n[include conf.d/*.cfg]\n",
        );
        let doc = Document::parse(&std::fs::read_to_string(&main).unwrap(), &main).unwrap();
        assert_eq!(doc.get("a", "x").unwrap().0, "1");
        // Glob applies in sorted order: m2 overrides m1.
        assert_eq!(doc.get("b", "y").unwrap().0, "3");
        assert_eq!(doc.get("b", "z").unwrap().0, "4");
    }

    #[test]
    fn include_missing_literal_errors_missing_glob_ok() {
        let dir = tempfile::tempdir().unwrap();
        let main = write(dir.path(), "printer.cfg", "[include nope.cfg]\n");
        assert!(Document::parse(&std::fs::read_to_string(&main).unwrap(), &main).is_err());
        let main = write(
            dir.path(),
            "printer2.cfg",
            "[include nope/*.cfg]\n[a]\nx: 1\n",
        );
        let doc = Document::parse(&std::fs::read_to_string(&main).unwrap(), &main).unwrap();
        assert_eq!(doc.get("a", "x").unwrap().0, "1");
    }

    #[test]
    fn include_override_order_is_linear() {
        let dir = tempfile::tempdir().unwrap();
        write(dir.path(), "inc.cfg", "[a]\nx: from_include\n");
        let main = write(
            dir.path(),
            "printer.cfg",
            "[a]\nx: before\n[include inc.cfg]\n[a]\nx: after\n",
        );
        let doc = Document::parse(&std::fs::read_to_string(&main).unwrap(), &main).unwrap();
        assert_eq!(doc.get("a", "x").unwrap().0, "after");

        let main2 = write(
            dir.path(),
            "printer2.cfg",
            "[a]\nx: before\n[include inc.cfg]\n",
        );
        let doc = Document::parse(&std::fs::read_to_string(&main2).unwrap(), &main2).unwrap();
        assert_eq!(doc.get("a", "x").unwrap().0, "from_include");
    }

    #[test]
    fn recursive_include_errors() {
        let dir = tempfile::tempdir().unwrap();
        write(dir.path(), "a.cfg", "[include b.cfg]\n");
        write(dir.path(), "b.cfg", "[include a.cfg]\n");
        let main = write(dir.path(), "printer.cfg", "[include a.cfg]\n");
        assert!(Document::parse(&std::fs::read_to_string(&main).unwrap(), &main).is_err());
    }

    #[test]
    fn recursive_include_through_dotdot_errors() {
        let dir = tempfile::tempdir().unwrap();
        write(dir.path(), "a.cfg", "[include sub/b.cfg]\n");
        write(dir.path(), "sub/b.cfg", "[include ../a.cfg]\n");
        let main = write(dir.path(), "printer.cfg", "[include a.cfg]\n");
        let err = Document::parse(&std::fs::read_to_string(&main).unwrap(), &main)
            .expect_err("cycle must be detected");
        assert!(err.to_string().contains("Recursive include"), "{err}");
    }

    #[test]
    fn parser_state_resets_at_include_boundary() {
        // klipper parses the buffered lines around an include as separate
        // configparser reads: an option after the include has no section.
        let dir = tempfile::tempdir().unwrap();
        write(dir.path(), "inc.cfg", "[b]\ny: 2\n");
        let main = write(
            dir.path(),
            "printer.cfg",
            "[a]\nx: 1\n[include inc.cfg]\norphan: 3\n",
        );
        assert!(Document::parse(&std::fs::read_to_string(&main).unwrap(), &main).is_err());
    }

    #[test]
    fn indented_include_is_not_an_include() {
        let dir = tempfile::tempdir().unwrap();
        let main = write(dir.path(), "printer.cfg", "[a]\nk:\n  [include nope.cfg]\n");
        let doc = Document::parse(&std::fs::read_to_string(&main).unwrap(), &main).unwrap();
        assert_eq!(doc.get("a", "k").unwrap().0, "\n[include nope.cfg]");
    }

    #[test]
    fn bang_include_rewritten_to_absolute() {
        let dir = tempfile::tempdir().unwrap();
        let script = write(dir.path(), "macros/hello.gcode", "M117 hi\n");
        let main = write(
            dir.path(),
            "printer.cfg",
            "[gcode_macro m]\ngcode: !!include macros/hello.gcode\n",
        );
        let doc = Document::parse(&std::fs::read_to_string(&main).unwrap(), &main).unwrap();
        let value = doc.get("gcode_macro m", "gcode").unwrap().0;
        let expected = format!(
            "!!include {}",
            std::path::absolute(&script).unwrap().display()
        );
        assert_eq!(value, expected);
    }

    #[test]
    fn bang_include_missing_errors() {
        let dir = tempfile::tempdir().unwrap();
        let main = write(
            dir.path(),
            "printer.cfg",
            "[gcode_macro m]\ngcode: !!include macros/missing.gcode\n",
        );
        assert!(Document::parse(&std::fs::read_to_string(&main).unwrap(), &main).is_err());
    }

    #[test]
    fn crlf_in_included_file_normalized() {
        let dir = tempfile::tempdir().unwrap();
        write(dir.path(), "inc.cfg", "[b]\r\ny: 2\r\n");
        let main = write(dir.path(), "printer.cfg", "[include inc.cfg]\n");
        let doc = Document::parse(&std::fs::read_to_string(&main).unwrap(), &main).unwrap();
        assert_eq!(doc.get("b", "y").unwrap().0, "2");
    }
}
