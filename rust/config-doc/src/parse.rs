//! The klipper config dialect, reproduced from `klippy/configfile.py`
//! `_parse_config` + `configparser.RawConfigParser(strict=False,
//! inline_comment_prefixes=(";", "#"))`:
//!
//! - `#` starts a comment anywhere on a line (klipper pre-strips it before
//!   the INI parse, so no preceding whitespace is required and `#` never
//!   reaches values); `;` comments only at line start or after whitespace.
//! - A whitespace-only line inside an option value contributes an empty
//!   value line — unless the blankness came from a `;` comment (configparser
//!   sees `;` and suppresses the append; it never sees `#`).
//! - `[include <spec>]` flushes and RESETS parser state (klipper parses the
//!   buffered lines around each include as separate configparser reads), so
//!   an option line after an include needs a fresh section header.
//! - `!!include <path>` anywhere in a buffered line is rewritten to an
//!   absolute path at parse time and must exist.

use std::path::{Path, PathBuf};

use crate::{ConfigError, Document, Result, err};

pub(crate) fn parse_into(
    doc: &mut Document,
    data: &str,
    filename: &str,
    visited: &mut Vec<PathBuf>,
) -> Result<()> {
    let abs = normalized_absolute(Path::new(filename))
        .map_err(|e| err(format!("Cannot resolve config path '{filename}': {e}")))?;
    if visited.contains(&abs) {
        return Err(err(format!(
            "Recursive include of config file '{filename}'"
        )));
    }
    visited.push(abs);

    let mut state = ChunkState::default();
    for (lineno, raw_line) in data.split('\n').enumerate() {
        let lineno = lineno + 1;
        let line = strip_hash_comment(raw_line);

        if let Some(spec) = include_directive(line) {
            state.commit_pending(doc);
            state.reset();
            resolve_include(doc, filename, spec, visited)?;
            continue;
        }

        let line = rewrite_bang_includes(line, filename)?;
        state.feed(doc, &line, filename, lineno)?;
    }
    state.commit_pending(doc);

    visited.pop();
    Ok(())
}

fn strip_hash_comment(line: &str) -> &str {
    match line.find('#') {
        Some(pos) => &line[..pos],
        None => line,
    }
}

/// `[include <spec>]` at column 0 (leading whitespace defeats the match,
/// exactly like the `SECTCRE.match` in klipper's `_parse_config`).
fn include_directive(line: &str) -> Option<&str> {
    let header = section_header(line)?;
    header.strip_prefix("include ").map(str::trim)
}

/// configparser SECTCRE anchored at the start of `text`: `[` then one or
/// more non-`]` characters then `]`; anything after the `]` is ignored.
fn section_header(text: &str) -> Option<&str> {
    let rest = text.strip_prefix('[')?;
    let end = rest.find(']')?;
    (end > 0).then(|| &rest[..end])
}

/// `os.path.abspath` equivalent: absolute + LEXICAL normalization
/// (collapse `.` and `..` without touching the filesystem), so an include
/// cycle written through `../` paths maps to one canonical visited entry.
fn normalized_absolute(path: &Path) -> std::io::Result<PathBuf> {
    use std::path::Component;
    let abs = std::path::absolute(path)?;
    let mut out = PathBuf::new();
    for component in abs.components() {
        match component {
            Component::CurDir => {}
            Component::ParentDir => {
                out.pop();
            }
            other => out.push(other),
        }
    }
    Ok(out)
}

fn rewrite_bang_includes(line: &str, source_file: &str) -> Result<String> {
    const MARKER: &str = "!!include ";
    let Some(pos) = line.find(MARKER) else {
        return Ok(line.to_owned());
    };
    let file = &line[pos + MARKER.len()..];
    let new_path = Path::new(source_file)
        .parent()
        .unwrap_or_else(|| Path::new(""))
        .join(file);
    let new_path = std::path::absolute(&new_path)
        .map_err(|e| err(format!("Cannot resolve include path '{file}': {e}")))?;
    if !new_path.is_file() {
        return Err(err(format!(
            "Attempted to include non-existent file {}",
            new_path.display()
        )));
    }
    Ok(format!("{}{MARKER}{}", &line[..pos], new_path.display()))
}

fn has_glob_magic(spec: &str) -> bool {
    spec.contains(['*', '?', '['])
}

fn resolve_include(
    doc: &mut Document,
    source_filename: &str,
    include_spec: &str,
    visited: &mut Vec<PathBuf>,
) -> Result<()> {
    let dirname = Path::new(source_filename)
        .parent()
        .unwrap_or_else(|| Path::new(""));
    let include_glob = dirname.join(include_spec);
    let pattern = include_glob.to_string_lossy();

    let options = glob::MatchOptions {
        case_sensitive: true,
        require_literal_separator: false,
        // Python's glob does not match dotfiles with wildcards.
        require_literal_leading_dot: true,
    };
    let mut names = Vec::new();
    let entries = glob::glob_with(&pattern, options)
        .map_err(|e| err(format!("Bad include pattern '{pattern}': {e}")))?;
    for entry in entries {
        names
            .push(entry.map_err(|e| err(format!("Include pattern '{pattern}': cannot read {e}")))?);
    }
    if names.is_empty() && !has_glob_magic(&pattern) {
        // An empty set is OK for a wildcard but not a direct file reference.
        return Err(err(format!("Include file '{pattern}' does not exist")));
    }
    names.sort();

    for name in names {
        let name_str = name.to_string_lossy().into_owned();
        let data = read_config_file(&name_str)?;
        parse_into(doc, &data, &name_str, visited)?;
    }
    Ok(())
}

pub(crate) fn read_config_file(filename: &str) -> Result<String> {
    let data = std::fs::read_to_string(filename)
        .map_err(|_| err(format!("Unable to open config file {filename}")))?;
    Ok(data.replace("\r\n", "\n"))
}

/// One configparser `read_file()` pass: section/option/continuation state
/// that klipper resets at every `[include]` boundary.
#[derive(Default)]
struct ChunkState {
    cursect: Option<String>,
    curopt: Option<String>,
    value_lines: Vec<String>,
    indent_level: usize,
}

impl ChunkState {
    fn reset(&mut self) {
        *self = Self::default();
    }

    fn commit_pending(&mut self, doc: &mut Document) {
        let (Some(sect), Some(opt)) = (&self.cursect, self.curopt.take()) else {
            return;
        };
        let joined = self.value_lines.join("\n");
        let value = joined.trim_end().to_owned();
        doc.section_mut_or_insert(sect).set(&opt, value);
        self.value_lines.clear();
    }

    fn feed(
        &mut self,
        doc: &mut Document,
        line: &str,
        filename: &str,
        lineno: usize,
    ) -> Result<()> {
        let (value_portion, had_semicolon_comment) = split_semicolon_comment(line);
        let value = value_portion.trim();

        if value.is_empty() {
            let in_value = self.cursect.is_some() && self.curopt.is_some();
            if !had_semicolon_comment && in_value {
                self.value_lines.push(String::new());
            }
            return Ok(());
        }

        let cur_indent = line.chars().take_while(|c| c.is_whitespace()).count();
        let continuing =
            self.cursect.is_some() && self.curopt.is_some() && cur_indent > self.indent_level;
        if continuing {
            self.value_lines.push(value.to_owned());
            return Ok(());
        }

        self.indent_level = cur_indent;
        self.commit_pending(doc);

        if let Some(header) = section_header(value) {
            if header == "DEFAULT" {
                return Err(parse_err(
                    filename,
                    lineno,
                    "the [DEFAULT] section is not supported",
                ));
            }
            // Materialize at the header like configparser: an option-less
            // section still exists (enable-only sections activate modules).
            doc.section_mut_or_insert(header);
            self.cursect = Some(header.to_owned());
            return Ok(());
        }

        if self.cursect.is_none() {
            return Err(parse_err(
                filename,
                lineno,
                &format!("no section header before '{value}'"),
            ));
        }

        let Some(delim) = value.find(['=', ':']) else {
            return Err(parse_err(
                filename,
                lineno,
                &format!("invalid line '{value}'"),
            ));
        };
        let optname = value[..delim].trim_end().to_lowercase();
        if optname.is_empty() {
            return Err(parse_err(
                filename,
                lineno,
                &format!("option line without a name: '{value}'"),
            ));
        }
        let optval = value[delim + 1..].trim().to_owned();
        self.curopt = Some(optname);
        self.value_lines = vec![optval];
        Ok(())
    }
}

/// configparser inline-comment rule for `;`: a comment at column 0, or
/// anywhere preceded by whitespace. Returns the value portion and whether
/// a `;` comment was present (which suppresses the blank-line-in-value
/// append).
fn split_semicolon_comment(line: &str) -> (&str, bool) {
    let mut prev: Option<char> = None;
    for (i, c) in line.char_indices() {
        if c == ';' && prev.is_none_or(char::is_whitespace) {
            return (&line[..i], true);
        }
        prev = Some(c);
    }
    (line, false)
}

fn parse_err(filename: &str, lineno: usize, msg: &str) -> ConfigError {
    err(format!(
        "Config parse error in file '{filename}', line {lineno}: {msg}"
    ))
}
