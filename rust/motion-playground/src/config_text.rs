use pipeline_snapshot::{AxisDecl, PostProcessorDecl};

#[cfg(test)]
mod tests;

#[derive(Debug, thiserror::Error, PartialEq)]
pub enum ConfigTextError {
    #[error(
        "line {line}: expected '[axis <name>]', '[post_processor <name>]', or 'key: value', got '{text}'"
    )]
    BadLine { line: usize, text: String },
    #[error("line {line}: '{key}' is not a valid number: '{value}'")]
    BadNumber {
        line: usize,
        key: String,
        value: String,
    },
    #[error("line {line}: 'key: value' outside any [axis]/[post_processor] section")]
    NoActiveSection { line: usize },
    #[error("[post_processor {name}] is missing a required 'type:' line")]
    MissingType { name: String },
}

enum Section {
    Axis(AxisDecl),
    PostProcessor {
        name: String,
        ty: Option<String>,
        params: Vec<(String, f64)>,
    },
}

/// Parses `[axis <name>]`/`[post_processor <name>]` sections in the same
/// grammar `klippy/motion_setup.py` reads from `printer.cfg`:
/// `post_processors: a, b` and `follows: x, y, z` comma lists on `[axis]`,
/// `type: <name>` plus one `key: value` float per parameter on
/// `[post_processor]`. An explicit `[axis]` section replaces the default
/// topology's declaration wholesale, so an `[axis e]` without `follows:`
/// stops being a follower. `motors:` lines are accepted (so a real
/// printer.cfg's [axis] sections paste in unmodified) but ignored — the
/// playground has no motor lanes.
pub fn parse(text: &str) -> Result<(Vec<AxisDecl>, Vec<PostProcessorDecl>), ConfigTextError> {
    let mut sections: Vec<Section> = Vec::new();

    for (idx, raw_line) in text.lines().enumerate() {
        let line_no = idx + 1;
        let line = raw_line.split('#').next().unwrap_or("").trim();
        if line.is_empty() {
            continue;
        }

        if let Some(rest) = line.strip_prefix('[').and_then(|s| s.strip_suffix(']')) {
            let rest = rest.trim();
            if let Some(name) = rest.strip_prefix("axis ") {
                sections.push(Section::Axis(AxisDecl {
                    name: name.trim().to_string(),
                    follows: Vec::new(),
                    motors: Vec::new(),
                    post_processors: Vec::new(),
                }));
            } else if let Some(name) = rest.strip_prefix("post_processor ") {
                sections.push(Section::PostProcessor {
                    name: name.trim().to_string(),
                    ty: None,
                    params: Vec::new(),
                });
            } else {
                return Err(ConfigTextError::BadLine {
                    line: line_no,
                    text: line.to_string(),
                });
            }
            continue;
        }

        let Some((key, value)) = line.split_once(':') else {
            return Err(ConfigTextError::BadLine {
                line: line_no,
                text: line.to_string(),
            });
        };
        let key = key.trim();
        let value = value.trim();

        match sections.last_mut() {
            None => return Err(ConfigTextError::NoActiveSection { line: line_no }),
            Some(Section::Axis(decl)) => match key {
                "post_processors" => {
                    decl.post_processors = value.split(',').map(|s| s.trim().to_string()).collect();
                }
                "follows" => {
                    decl.follows = value.split(',').map(|s| s.trim().to_string()).collect();
                }
                "motors" => {}
                _ => {
                    return Err(ConfigTextError::BadLine {
                        line: line_no,
                        text: line.to_string(),
                    });
                }
            },
            Some(Section::PostProcessor { ty, params, .. }) => {
                if key == "type" {
                    *ty = Some(value.to_string());
                } else {
                    let parsed: f64 = value.parse().map_err(|_| ConfigTextError::BadNumber {
                        line: line_no,
                        key: key.to_string(),
                        value: value.to_string(),
                    })?;
                    params.push((key.to_string(), parsed));
                }
            }
        }
    }

    let mut axis_decls = Vec::new();
    let mut post_processor_decls = Vec::new();
    for section in sections {
        match section {
            Section::Axis(decl) => axis_decls.push(decl),
            Section::PostProcessor { name, ty, params } => {
                let ty = ty.ok_or_else(|| ConfigTextError::MissingType { name: name.clone() })?;
                post_processor_decls.push(PostProcessorDecl { name, ty, params });
            }
        }
    }
    Ok((axis_decls, post_processor_decls))
}
