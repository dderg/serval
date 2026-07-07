//! Enforces that every `tracing::warn!`/`tracing::error!` call site carries an
//! `event = "..."` field. The structured-log store is queried by
//! `stats by (event)`; an unkeyed warn/error is invisible to that analysis
//! and can fire silently at very high volume (one unkeyed warn was observed
//! firing 277k times in a single day before this lint existed).

use std::fs;
use std::path::{Path, PathBuf};

/// Files that are known, by manual inspection, to contain the byte sequence
/// `error!(` or `warn!(` without being a real `tracing::warn!`/`error!` call
/// site. Each entry must carry a short justification. The scanner's
/// word-boundary check already excludes `compile_error!(`, so this allowlist
/// is expected to stay empty; it exists so a future exception has an
/// explicit, reviewed home instead of a silent regex tweak.
const NON_EMIT_FILE_ALLOWLIST: &[(&str, &str)] = &[
    // (path suffix, justification) - intentionally empty after Part 1.
];

#[derive(Debug, PartialEq, Eq)]
struct Violation {
    line: usize,
    excerpt: String,
}

/// Scans `src` (the contents of a single Rust source file) for
/// `warn!(`/`error!(`/`tracing::warn!(`/`tracing::error!(` invocations that
/// are missing an `event =`/`event=` field, and returns one `Violation` per
/// offending call site.
fn scan_source(src: &str) -> Vec<Violation> {
    let bytes = src.as_bytes();
    let macro_rules_spans = find_macro_rules_spans(bytes);

    let mut violations = Vec::new();
    let needles = ["warn!(", "error!("];

    for needle in needles {
        let mut search_from = 0usize;
        while let Some(rel) = src[search_from..].find(needle) {
            let match_start = search_from + rel;
            search_from = match_start + needle.len();

            if preceded_by_ident_char(bytes, match_start) {
                continue;
            }
            if within_any_span(&macro_rules_spans, match_start) {
                continue;
            }

            let open_paren = match_start + needle.len() - 1;
            let Some(close_paren) = find_matching_paren(bytes, open_paren) else {
                continue;
            };
            let args = &src[open_paren + 1..close_paren];

            if args.contains("event =") || args.contains("event=") {
                continue;
            }

            let line_no = 1 + src[..match_start].matches('\n').count();
            if has_opt_out(src, line_no) {
                continue;
            }

            let line_start = src[..match_start].rfind('\n').map(|i| i + 1).unwrap_or(0);
            let line_end = src[match_start..]
                .find('\n')
                .map(|i| match_start + i)
                .unwrap_or(src.len());
            let excerpt = src[line_start..line_end].trim().to_string();

            violations.push(Violation {
                line: line_no,
                excerpt,
            });
        }
    }

    violations.sort_by_key(|v| v.line);
    violations.dedup();
    violations
}

fn preceded_by_ident_char(bytes: &[u8], idx: usize) -> bool {
    idx > 0 && matches!(bytes[idx - 1], b'a'..=b'z' | b'A'..=b'Z' | b'0'..=b'9' | b'_')
}

fn has_opt_out(src: &str, line_no: usize) -> bool {
    let lines: Vec<&str> = src.lines().collect();
    let this_line = lines.get(line_no.saturating_sub(1)).copied().unwrap_or("");
    let prev_line = if line_no >= 2 {
        lines.get(line_no - 2).copied().unwrap_or("")
    } else {
        ""
    };
    this_line.contains("// unkeyed-log:") || prev_line.contains("// unkeyed-log:")
}

/// Given the byte index of an opening `(`, returns the byte index of its
/// matching closing `)`, skipping over the contents of `"..."` string
/// literals (with `\"` escape handling) so that parens inside format-string
/// literals don't confuse the depth count.
fn find_matching_paren(bytes: &[u8], open_idx: usize) -> Option<usize> {
    debug_assert_eq!(bytes.get(open_idx), Some(&b'('));
    let mut depth: i32 = 0;
    let mut i = open_idx;
    while i < bytes.len() {
        match bytes[i] {
            b'(' => depth += 1,
            b')' => {
                depth -= 1;
                if depth == 0 {
                    return Some(i);
                }
            }
            b'"' => {
                i += 1;
                while i < bytes.len() && bytes[i] != b'"' {
                    if bytes[i] == b'\\' {
                        i += 1;
                    }
                    i += 1;
                }
            }
            _ => {}
        }
        i += 1;
    }
    None
}

/// Finds the byte-offset spans `[start, end)` of every `macro_rules! name {
/// ... }` block, so call sites inside macro *definitions* (which are not
/// live emit sites) can be excluded from the lint.
fn find_macro_rules_spans(bytes: &[u8]) -> Vec<(usize, usize)> {
    let src = std::str::from_utf8(bytes).unwrap_or("");
    let mut spans = Vec::new();
    let mut search_from = 0usize;
    while let Some(rel) = src[search_from..].find("macro_rules!") {
        let start = search_from + rel;
        let Some(brace_rel) = src[start..].find('{') else {
            break;
        };
        let open_brace = start + brace_rel;
        if let Some(close_brace) = find_matching_brace(bytes, open_brace) {
            spans.push((start, close_brace + 1));
            search_from = close_brace + 1;
        } else {
            break;
        }
    }
    spans
}

fn find_matching_brace(bytes: &[u8], open_idx: usize) -> Option<usize> {
    debug_assert_eq!(bytes.get(open_idx), Some(&b'{'));
    let mut depth: i32 = 0;
    let mut i = open_idx;
    while i < bytes.len() {
        match bytes[i] {
            b'{' => depth += 1,
            b'}' => {
                depth -= 1;
                if depth == 0 {
                    return Some(i);
                }
            }
            b'"' => {
                i += 1;
                while i < bytes.len() && bytes[i] != b'"' {
                    if bytes[i] == b'\\' {
                        i += 1;
                    }
                    i += 1;
                }
            }
            _ => {}
        }
        i += 1;
    }
    None
}

fn within_any_span(spans: &[(usize, usize)], idx: usize) -> bool {
    spans.iter().any(|&(s, e)| idx >= s && idx < e)
}

/// Walks `<workspace_root>/rust/*/src/**/*.rs`, skipping `target/` build
/// output and any test files (`.../tests/...` or `*tests.rs`).
fn collect_source_files(rust_dir: &Path) -> Vec<PathBuf> {
    let mut out = Vec::new();
    let Ok(crates) = fs::read_dir(rust_dir) else {
        return out;
    };
    for crate_entry in crates.flatten() {
        let src_dir = crate_entry.path().join("src");
        if src_dir.is_dir() {
            walk_rs_files(&src_dir, &mut out);
        }
    }
    out
}

fn walk_rs_files(dir: &Path, out: &mut Vec<PathBuf>) {
    let Ok(entries) = fs::read_dir(dir) else {
        return;
    };
    for entry in entries.flatten() {
        let path = entry.path();
        let path_str = path.to_string_lossy();
        if path_str.contains("/target/") {
            continue;
        }
        if path.is_dir() {
            walk_rs_files(&path, out);
            continue;
        }
        if path.extension().and_then(|e| e.to_str()) != Some("rs") {
            continue;
        }
        if path_str.contains("/tests/") || path_str.ends_with("tests.rs") {
            continue;
        }
        out.push(path);
    }
}

fn is_allowlisted(path: &Path) -> Option<&'static str> {
    let path_str = path.to_string_lossy();
    NON_EMIT_FILE_ALLOWLIST
        .iter()
        .find(|(suffix, _)| path_str.ends_with(suffix))
        .map(|(_, reason)| *reason)
}

#[test]
fn all_warn_and_error_emit_sites_carry_an_event_field() {
    let manifest_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let workspace_root = manifest_dir
        .join("..")
        .join("..")
        .canonicalize()
        .expect("workspace root must exist");
    let rust_dir = workspace_root.join("rust");
    assert!(
        rust_dir.is_dir(),
        "expected {rust_dir:?} to be the `rust/` workspace directory"
    );

    let files = collect_source_files(&rust_dir);
    assert!(
        !files.is_empty(),
        "expected to find at least one source file under {rust_dir:?}"
    );

    let mut failures = Vec::new();
    for file in files {
        if is_allowlisted(&file).is_some() {
            continue;
        }
        let Ok(contents) = fs::read_to_string(&file) else {
            continue;
        };
        for violation in scan_source(&contents) {
            failures.push(format!(
                "{}:{}: {}",
                file.display(),
                violation.line,
                violation.excerpt
            ));
        }
    }

    assert!(
        failures.is_empty(),
        "found {} warn!/error! call site(s) missing an `event =` field:\n{}",
        failures.len(),
        failures.join("\n")
    );
}

#[test]
fn scanner_flags_a_missing_event_field() {
    let src = r#"
fn f() {
    tracing::warn!("something went wrong: {e}");
}
"#;
    let violations = scan_source(src);
    assert_eq!(violations.len(), 1);
    assert_eq!(violations[0].line, 3);
}

#[test]
fn scanner_accepts_a_keyed_call_site() {
    let src = r#"
fn f() {
    tracing::warn!(event = "something_went_wrong", "something went wrong: {e}");
}
"#;
    assert!(scan_source(src).is_empty());
}

#[test]
fn scanner_ignores_compile_error_macro() {
    let src = r#"compile_error!("features are mutually exclusive");"#;
    assert!(scan_source(src).is_empty());
}

#[test]
fn scanner_ignores_macro_rules_definitions() {
    let src = r"
macro_rules! my_warn {
    ($msg:expr) => {
        tracing::warn!($msg);
    };
}
";
    assert!(scan_source(src).is_empty());
}

#[test]
fn scanner_respects_inline_opt_out() {
    let src = r#"
fn f() {
    tracing::warn!("no event field here"); // unkeyed-log: legacy, tracked in TICKET-1
}
"#;
    assert!(scan_source(src).is_empty());
}
