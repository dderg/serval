//! `klippy._config_doc`: the config parsing core behind
//! `klippy/configfile.py`. Parsing, `[include]` resolution, and `${}`
//! interpolation live in the pure `config-doc` crate; this module is the
//! thin Python surface. Type coercion (int/float/bool), access tracking,
//! and the SAVE_CONFIG autosave-block text handling stay in Python.

use pyo3::exceptions::PyException;
use pyo3::prelude::*;

pyo3::create_exception!(
    _config_doc,
    ConfigError,
    PyException,
    "Config file parse or lookup error."
);

fn config_err(e: config_doc::ConfigError) -> PyErr {
    ConfigError::new_err(e.to_string())
}

/// An interpolation reference consulted while resolving a value:
/// `(section, option, interpolated_value)`, names as written in `${...}`.
type RefTuple = (String, String, String);

/// Bumped on any change to the Python-visible surface; klippy's loader
/// refuses a module whose API_VERSION does not match, so a stale build
/// fails loud instead of running with skewed semantics.
const API_VERSION: u32 = 1;

#[pyclass(name = "ConfigDocument")]
struct PyConfigDocument {
    inner: config_doc::Document,
}

#[pymethods]
impl PyConfigDocument {
    #[staticmethod]
    fn parse(data: &str, filename: &str) -> PyResult<Self> {
        Ok(Self {
            inner: config_doc::Document::parse(data, filename).map_err(config_err)?,
        })
    }

    fn sections(&self) -> Vec<String> {
        self.inner.section_names().map(str::to_owned).collect()
    }

    fn has_section(&self, section: &str) -> bool {
        self.inner.has_section(section)
    }

    fn options(&self, section: &str) -> PyResult<Vec<String>> {
        self.inner.options(section).map_err(config_err)
    }

    fn has_option(&self, section: &str, option: &str) -> bool {
        self.inner.has_option(section, option)
    }

    fn get(&self, section: &str, option: &str) -> PyResult<String> {
        Ok(self.inner.get(section, option).map_err(config_err)?.0)
    }

    /// The interpolated value plus every `${}` reference consulted, for
    /// mirroring into access_tracking.
    fn get_with_refs(&self, section: &str, option: &str) -> PyResult<(String, Vec<RefTuple>)> {
        let (value, refs) = self.inner.get(section, option).map_err(config_err)?;
        let refs = refs
            .into_iter()
            .map(|r| (r.section, r.option, r.value))
            .collect();
        Ok((value, refs))
    }

    fn add_section(&mut self, section: &str) -> PyResult<()> {
        self.inner.add_section(section).map_err(config_err)
    }

    fn set(&mut self, section: &str, option: &str, value: &str) -> PyResult<()> {
        self.inner.set(section, option, value).map_err(config_err)
    }

    fn remove_section(&mut self, section: &str) -> bool {
        self.inner.remove_section(section)
    }

    fn write_string(&self) -> String {
        self.inner.write_string()
    }
}

#[pymodule]
fn _config_doc(py: Python<'_>, m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<PyConfigDocument>()?;
    m.add("ConfigError", py.get_type::<ConfigError>())?;
    m.add("API_VERSION", API_VERSION)?;
    Ok(())
}
