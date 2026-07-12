//! `${option}` / `${section.option}` / `${section:option}` value
//! interpolation, matching `SectionInterpolation` in klippy/configfile.py:
//! resolved at read time, referenced values themselves interpolated, at
//! most [`MAX_SUBSTITUTIONS`] references substituted per value (the rest
//! stay literal, as configparser's depth loop leaves them), every
//! consulted reference reported for access tracking.
//!
//! Deliberate divergences from the Python implementation (see lib.rs):
//! `\${` is a working escape, substituted text is not re-scanned, and a
//! reference cycle is an error rather than a `RecursionError` crash.

use crate::{Document, InterpolationRef, Result, err};

const MAX_SUBSTITUTIONS: usize = 10;
const MAX_DEPTH: usize = 25;

pub(crate) fn resolve(
    doc: &Document,
    current_section: &str,
    raw: &str,
    refs: &mut Vec<InterpolationRef>,
    depth: usize,
) -> Result<String> {
    if depth > MAX_DEPTH {
        return Err(err(format!(
            "Interpolation depth exceeded in section '{current_section}' \
             (reference cycle?)"
        )));
    }
    if !raw.contains("${") {
        return Ok(raw.to_owned());
    }

    let mut out = String::with_capacity(raw.len());
    let mut rest = raw;
    let mut budget = MAX_SUBSTITUTIONS;
    while !rest.is_empty() {
        let Some(pos) = rest.find("${") else {
            out.push_str(rest);
            break;
        };
        if pos > 0 && rest.as_bytes()[pos - 1] == b'\\' {
            out.push_str(&rest[..pos - 1]);
            out.push_str("${");
            rest = &rest[pos + 2..];
            continue;
        }
        match parse_reference(&rest[pos..]) {
            Some((section, option, consumed)) if budget > 0 => {
                budget -= 1;
                out.push_str(&rest[..pos]);
                let section = section.unwrap_or(current_section);
                let ref_raw = doc.get_raw(section, option)?.to_owned();
                let value = resolve(doc, section, &ref_raw, refs, depth + 1)?;
                record_ref(refs, section, option, &value);
                out.push_str(&value);
                rest = &rest[pos + consumed..];
            }
            _ => {
                out.push_str(&rest[..pos + 2]);
                rest = &rest[pos + 2..];
            }
        }
    }
    Ok(out)
}

/// Parse a `${...}` reference at the start of `text` per the Python
/// KEYCRE grammar: an optional section part (no `.`, `:`, `$`, `{`, `}`)
/// ending at the first `.` or `:`, then a non-empty option part (no `$`,
/// `{`, `}`). When the section split leaves either side empty the regex
/// backtracks and the WHOLE body is the option name (`${.opt}` → option
/// ".opt" in the current section). Returns (section, option, bytes
/// consumed incl. braces).
fn parse_reference(text: &str) -> Option<(Option<&str>, &str, usize)> {
    let body_start = 2;
    let close = text[body_start..].find('}')? + body_start;
    let body = &text[body_start..close];
    if body.is_empty() || body.contains(['$', '{']) {
        return None;
    }
    let (section, option) = match body.find(['.', ':']) {
        Some(split) if split > 0 && split + 1 < body.len() => {
            (Some(&body[..split]), &body[split + 1..])
        }
        _ => (None, body),
    };
    Some((section, option, close + 1))
}

/// Mirror of `access_tracking.setdefault((sect, opt), const)`: first
/// consultation wins, option name recorded exactly as written.
fn record_ref(refs: &mut Vec<InterpolationRef>, section: &str, option: &str, value: &str) {
    if refs
        .iter()
        .any(|r| r.section == section && r.option == option)
    {
        return;
    }
    refs.push(InterpolationRef {
        section: section.to_owned(),
        option: option.to_owned(),
        value: value.to_owned(),
    });
}
