//! Endpoint CLI parsing for the chain layout. Lives in the lib (not the
//! hw-gated binary) so the per-slave group parsing is unit-tested in CI.

/// Mirror of `EC_RT_MAX_SLAVES` in `csrc/libecrt.h` — the upper bound the C
/// backend sizes its per-slave arrays to. Keep the two in sync.
pub const EC_RT_MAX_SLAVES: usize = 8;

/// Per-drive config parsed from one `--slave <pos> ...` CLI group. `axis` is the
/// host's global axis this slave drives; the endpoint routes incoming PushPieces
/// (tagged with the global axis) to this slave's ring through it. It is only
/// meaningful with multiple slaves — the single-drive form leaves it at 0 and
/// routes everything to slot 0.
#[derive(Debug, Clone, PartialEq)]
pub struct SlaveCfg {
    pub pos: i32,
    pub axis: u8,
    pub counts_per_mm: f64,
    pub rotation_distance: f64,
    pub following_error_counts: Option<u32>,
    pub max_torque_tenth_pct: Option<u16>,
}

fn default_cfg(pos: i32) -> SlaveCfg {
    SlaveCfg {
        pos,
        axis: 0,
        counts_per_mm: 3276.8,
        rotation_distance: 40.0,
        following_error_counts: None,
        max_torque_tenth_pct: None,
    }
}

/// Parse the chain layout from repeated `--slave <pos>` groups. Each group's
/// per-drive flags (`--counts-per-mm`, `--rotation-distance`,
/// `--following-error-counts`, `--max-torque-tenth-pct`) apply to the most
/// recent `--slave`. With no `--slave` at all, falls back to a single drive at
/// position 0 reading those flags globally (the legacy single-drive form).
/// Fails loudly on a missing value, a per-drive flag before any `--slave`, a
/// duplicate position, or more than `EC_RT_MAX_SLAVES` drives.
pub fn parse_slaves(args: &[String]) -> Result<Vec<SlaveCfg>, String> {
    if !args.iter().any(|a| a == "--slave") {
        let mut cfg = default_cfg(0);
        if let Some(v) = arg_val(args, "--counts-per-mm").and_then(|s| s.parse().ok()) {
            cfg.counts_per_mm = v;
        }
        if let Some(v) = arg_val(args, "--rotation-distance").and_then(|s| s.parse().ok()) {
            cfg.rotation_distance = v;
        }
        cfg.following_error_counts =
            arg_val(args, "--following-error-counts").and_then(|s| s.parse().ok());
        cfg.max_torque_tenth_pct =
            arg_val(args, "--max-torque-tenth-pct").and_then(|s| s.parse().ok());
        return Ok(vec![cfg]);
    }

    let mut slaves: Vec<SlaveCfg> = Vec::new();
    let mut i = 0;
    while i < args.len() {
        match args[i].as_str() {
            "--slave" => {
                let v = args
                    .get(i + 1)
                    .ok_or_else(|| "--slave requires a position".to_string())?;
                let pos: i32 = v
                    .parse()
                    .map_err(|_| "--slave value must be an integer position".to_string())?;
                slaves.push(default_cfg(pos));
                i += 2;
            }
            f @ ("--axis"
            | "--counts-per-mm"
            | "--rotation-distance"
            | "--following-error-counts"
            | "--max-torque-tenth-pct") => {
                let v = args
                    .get(i + 1)
                    .ok_or_else(|| format!("{f} requires a value"))?;
                let cur = slaves
                    .last_mut()
                    .ok_or_else(|| format!("{f} appeared before any --slave group"))?;
                match f {
                    "--axis" => {
                        cur.axis = v.parse().map_err(|_| "--axis not a u8".to_string())?;
                    }
                    "--counts-per-mm" => {
                        cur.counts_per_mm = v
                            .parse()
                            .map_err(|_| "--counts-per-mm not a number".to_string())?;
                    }
                    "--rotation-distance" => {
                        cur.rotation_distance = v
                            .parse()
                            .map_err(|_| "--rotation-distance not a number".to_string())?;
                    }
                    "--following-error-counts" => {
                        cur.following_error_counts =
                            Some(v.parse().map_err(|_| {
                                "--following-error-counts not a number".to_string()
                            })?);
                    }
                    _ => {
                        cur.max_torque_tenth_pct = Some(
                            v.parse()
                                .map_err(|_| "--max-torque-tenth-pct not a number".to_string())?,
                        );
                    }
                }
                i += 2;
            }
            _ => {
                i += 1;
            }
        }
    }

    if slaves.len() > EC_RT_MAX_SLAVES {
        return Err(format!(
            "{} slaves configured, endpoint supports at most {EC_RT_MAX_SLAVES}",
            slaves.len()
        ));
    }
    let mut seen: Vec<i32> = Vec::new();
    for s in &slaves {
        if seen.contains(&s.pos) {
            return Err(format!("duplicate --slave position {}", s.pos));
        }
        seen.push(s.pos);
    }
    Ok(slaves)
}

fn arg_val(args: &[String], key: &str) -> Option<String> {
    args.iter()
        .position(|a| a == key)
        .and_then(|i| args.get(i + 1).cloned())
}

#[cfg(test)]
mod tests;
