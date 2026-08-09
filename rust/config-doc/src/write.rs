//! Serialization in `configparser.RawConfigParser.write()` format, which
//! SAVE_CONFIG round-trips through the autosave block: `key = value` with
//! continuation lines tab-indented, one blank line after every section.

use crate::Document;

pub(crate) fn write_string(doc: &Document) -> String {
    let mut out = String::new();
    for name in doc.section_names() {
        let section = doc.section(name).expect("iterating existing sections");
        out.push('[');
        out.push_str(name);
        out.push_str("]\n");
        for key in section.option_names() {
            let value = section.get(key).expect("iterating existing options");
            out.push_str(key);
            out.push_str(" = ");
            out.push_str(&value.replace('\n', "\n\t"));
            out.push('\n');
        }
        out.push('\n');
    }
    out
}
