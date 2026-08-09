//! Klipper-compatible config document: the exact parsing dialect of
//! `klippy/configfile.py` (configparser `strict=False` semantics, `#`
//! pre-stripped comments, `[include]` resolution with linear override
//! order, `!!include` path rewriting) plus `${section.option}`
//! interpolation resolved at read time.
//!
//! Behavior is bug-for-bug compatible with the Python implementation it
//! replaces, with deliberate divergences, each loud or strictly more
//! permissive:
//! - `\${` escapes interpolation (the Python regex's optional lookbehind
//!   never suppressed a match, so the documented escape could not work),
//!   and substituted text is not re-scanned for further references.
//! - Interpolation cycles raise an error instead of crashing with
//!   `RecursionError`.
//! - `[DEFAULT]` (configparser's per-section fallback store) is rejected
//!   with a parse error instead of supported.
//! - A `**` glob that is not a whole path component (`[include a**b.cfg]`)
//!   is a parse error; Python's glob quietly degraded it to `*`.

mod interpolate;
mod parse;
mod write;

#[cfg(test)]
mod document_tests;
#[cfg(test)]
mod interpolate_tests;
#[cfg(test)]
mod parse_tests;
#[cfg(test)]
mod write_tests;

use thiserror::Error;

#[derive(Debug, Error)]
#[error("{0}")]
pub struct ConfigError(pub String);

pub type Result<T> = std::result::Result<T, ConfigError>;

pub(crate) fn err(msg: impl Into<String>) -> ConfigError {
    ConfigError(msg.into())
}

fn no_section_err(name: &str) -> ConfigError {
    err(format!("No section: '{name}'"))
}

/// One `[section]`: name kept verbatim (case- and whitespace-sensitive,
/// matching configparser), options keyed by lowercased name in insertion
/// order, duplicate assignment replacing the value in place.
#[derive(Debug, Default, Clone)]
pub struct Section {
    pub name: String,
    options: Vec<(String, String)>,
}

impl Section {
    pub fn get(&self, option: &str) -> Option<&str> {
        let key = option.to_lowercase();
        self.options
            .iter()
            .find(|(k, _)| *k == key)
            .map(|(_, v)| v.as_str())
    }

    pub fn set(&mut self, option: &str, value: String) {
        let key = option.to_lowercase();
        match self.options.iter_mut().find(|(k, _)| *k == key) {
            Some(slot) => slot.1 = value,
            None => self.options.push((key, value)),
        }
    }

    pub fn option_names(&self) -> impl Iterator<Item = &str> {
        self.options.iter().map(|(k, _)| k.as_str())
    }
}

/// An ordered set of sections, mirroring `configparser.RawConfigParser`
/// with `strict=False`: duplicate sections merge, later options win.
#[derive(Debug, Default, Clone)]
pub struct Document {
    sections: Vec<Section>,
}

/// A `${...}` reference resolved during interpolation, reported so the
/// caller can mirror Python's `access_tracking.setdefault((sect, opt), v)`
/// — names exactly as written in the template, value fully interpolated.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct InterpolationRef {
    pub section: String,
    pub option: String,
    pub value: String,
}

impl Document {
    /// Parse config text. `filename` anchors relative `[include]` and
    /// `!!include` paths and appears in error messages.
    pub fn parse(data: &str, filename: &str) -> Result<Self> {
        let mut doc = Self::default();
        let mut visited = Vec::new();
        parse::parse_into(&mut doc, data, filename, &mut visited)?;
        Ok(doc)
    }

    pub fn section_names(&self) -> impl Iterator<Item = &str> {
        self.sections.iter().map(|s| s.name.as_str())
    }

    pub fn section(&self, name: &str) -> Option<&Section> {
        self.sections.iter().find(|s| s.name == name)
    }

    pub fn has_section(&self, name: &str) -> bool {
        self.section(name).is_some()
    }

    pub fn has_option(&self, section: &str, option: &str) -> bool {
        self.section(section)
            .is_some_and(|s| s.get(option).is_some())
    }

    fn section_or_err(&self, name: &str) -> Result<&Section> {
        self.section(name).ok_or_else(|| no_section_err(name))
    }

    /// The stored (raw, uninterpolated) value.
    pub fn get_raw(&self, section: &str, option: &str) -> Result<&str> {
        self.section_or_err(section)?
            .get(option)
            .ok_or_else(|| err(format!("No option '{option}' in section: '{section}'")))
    }

    /// The value with `${...}` references resolved, plus every reference
    /// consulted along the way.
    pub fn get(&self, section: &str, option: &str) -> Result<(String, Vec<InterpolationRef>)> {
        let raw = self.get_raw(section, option)?;
        let mut refs = Vec::new();
        let value = interpolate::resolve(self, section, raw, &mut refs, 0)?;
        Ok((value, refs))
    }

    pub fn options(&self, section: &str) -> Result<Vec<String>> {
        Ok(self
            .section_or_err(section)?
            .option_names()
            .map(str::to_owned)
            .collect())
    }

    /// Create a section; error if it already exists (matching
    /// `configparser.add_section`, which raises even with `strict=False`).
    pub fn add_section(&mut self, name: &str) -> Result<()> {
        if self.has_section(name) {
            return Err(err(format!("Section '{name}' already exists")));
        }
        self.sections.push(Section {
            name: name.to_owned(),
            options: Vec::new(),
        });
        Ok(())
    }

    pub fn set(&mut self, section: &str, option: &str, value: &str) -> Result<()> {
        let sect = self
            .sections
            .iter_mut()
            .find(|s| s.name == section)
            .ok_or_else(|| no_section_err(section))?;
        sect.set(option, value.to_owned());
        Ok(())
    }

    pub fn remove_section(&mut self, name: &str) -> bool {
        let before = self.sections.len();
        self.sections.retain(|s| s.name != name);
        self.sections.len() != before
    }

    /// Serialize in `configparser.write()` format (`key = value`,
    /// multiline values continued with tab indents, blank line after each
    /// section).
    pub fn write_string(&self) -> String {
        write::write_string(self)
    }

    pub(crate) fn section_mut_or_insert(&mut self, name: &str) -> &mut Section {
        if let Some(idx) = self.sections.iter().position(|s| s.name == name) {
            return &mut self.sections[idx];
        }
        self.sections.push(Section {
            name: name.to_owned(),
            options: Vec::new(),
        });
        self.sections.last_mut().expect("just pushed")
    }
}
