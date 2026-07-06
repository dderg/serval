//! G-code text → absolute `(x, y, z, e, feedrate)` waypoints, mirroring
//! `scripts/viz_pipeline.py::parse_gcode` move for move so the playground and
//! the snapshot harness agree on what a pasted file means. E rides as a fifth
//! coordinate so retracts (E-only moves) and extruding moves flow through the
//! pipeline as followers; the engine differences consecutive E to a per-move
//! delta. Extruder mode is M82 (absolute) / M83 (relative), independent of
//! the G90/G91 flag that governs X/Y/Z; under G91 an undeclared extruder
//! rides along as relative, and an E word with no mode declared at all is
//! refused rather than guessed. G92 resets any axis's position (commonly
//! `G92 E0`) without emitting a move.

pub type Waypoint = (f64, f64, f64, f64, f64);

#[derive(Debug, thiserror::Error)]
pub enum WaypointError {
    #[error(transparent)]
    Lex(#[from] gcode::ParseError),
    #[error(
        "line {line}: G{major} arc command is not supported: the motion engine has no native arc ingestion yet, and silently linearizing it here would claim to exercise an arc while feeding the engine straight segments"
    )]
    UnsupportedArc { line: u32, major: u32 },
    #[error(
        "line {line}: E word before any M82/M83 (or G91) — the extruder mode is ambiguous, and guessing absolute turns relative-E slicer output into garbage extrusion ratios. Declare the mode (slicer excerpts printed with relative extrusion need an 'M83' line at the top)."
    )]
    AmbiguousExtruderMode { line: u32 },
}

pub fn parse_gcode(text: &str, max_velocity: f64) -> Result<Vec<Waypoint>, WaypointError> {
    let mut waypoints: Vec<Waypoint> = Vec::new();
    let (mut x, mut y, mut z, mut e) = (0.0_f64, 0.0_f64, 0.0_f64, 0.0_f64);
    let mut feedrate = max_velocity;
    let mut relative = false;
    let mut e_relative: Option<bool> = None;

    for tok in gcode::lex(text) {
        let tok = tok?;
        let gcode::Token::Command {
            letter,
            major,
            minor,
            params,
            line_no,
        } = tok
        else {
            continue;
        };
        if minor.is_some() {
            continue;
        }
        match (letter, major) {
            (b'G', 90) => relative = false,
            (b'G', 91) => relative = true,
            (b'M', 82) => e_relative = Some(false),
            (b'M', 83) => e_relative = Some(true),
            (b'G', 92) => {
                x = params.x().unwrap_or(x);
                y = params.y().unwrap_or(y);
                z = params.z().unwrap_or(z);
                e = params.e().unwrap_or(e);
            }
            (b'G', 2 | 3) => {
                return Err(WaypointError::UnsupportedArc {
                    line: line_no,
                    major,
                });
            }
            (b'G', cmd @ (0 | 1)) => {
                let has_position =
                    params.x().is_some() || params.y().is_some() || params.z().is_some();

                let (nx, ny, nz) = if relative {
                    (
                        x + params.x().unwrap_or(0.0),
                        y + params.y().unwrap_or(0.0),
                        z + params.z().unwrap_or(0.0),
                    )
                } else {
                    (
                        params.x().unwrap_or(x),
                        params.y().unwrap_or(y),
                        params.z().unwrap_or(z),
                    )
                };

                let ne = match params.e() {
                    Some(ev) => {
                        if e_relative.is_none() && !relative {
                            return Err(WaypointError::AmbiguousExtruderMode { line: line_no });
                        }
                        if e_relative.unwrap_or(true) {
                            e + ev
                        } else {
                            ev
                        }
                    }
                    None => e,
                };

                if cmd == 1 {
                    feedrate = params.f().map_or(feedrate, |f| f / 60.0);
                }
                if !(has_position || ne != e) {
                    continue;
                }
                (x, y, z, e) = (nx, ny, nz, ne);
                let e_only_before_any_position = waypoints.is_empty() && !has_position;
                if e_only_before_any_position {
                    continue;
                }
                let move_feedrate = if cmd == 0 { max_velocity } else { feedrate };
                waypoints.push((x, y, z, e, move_feedrate));
            }
            _ => {}
        }
    }
    Ok(waypoints)
}

#[cfg(test)]
mod tests;
