/// Mirror of `EC_RT_MAX_SLAVES` in `csrc/libecrt.h` — the upper bound the C
/// backend sizes its per-slave arrays to. Keep the two in sync.
pub const EC_RT_MAX_SLAVES: usize = 8;

#[derive(Debug, Clone, PartialEq)]
pub struct SlaveCfg {
    pub pos: i32,
    pub axis: u8,
    pub counts_per_mm: f64,
    pub rotation_distance: f64,
    pub following_error_counts: Option<u32>,
    pub max_torque_tenth_pct: Option<u16>,
    pub velocity_ff: bool,
    pub torque_clamp_tenths: i16,
    pub invert: bool,
    pub dynamics_profile: Option<String>,
}

fn default_cfg(pos: i32) -> SlaveCfg {
    SlaveCfg {
        pos,
        axis: 0,
        counts_per_mm: 3276.8,
        rotation_distance: 40.0,
        following_error_counts: None,
        max_torque_tenth_pct: None,
        velocity_ff: false,
        torque_clamp_tenths: 300,
        invert: false,
        dynamics_profile: None,
    }
}

fn parse_clamp_tenths(v: &str) -> Result<i16, String> {
    let pct: f64 = v
        .parse()
        .map_err(|_| "--torque-clamp-pct not a number".to_string())?;
    if !(pct > 0.0 && pct <= 400.0) {
        return Err(format!("--torque-clamp-pct {pct} outside (0, 400]"));
    }
    Ok((pct * 10.0) as i16)
}

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
        cfg.velocity_ff = args.iter().any(|a| a == "--velocity-ff");
        cfg.invert = args.iter().any(|a| a == "--invert");
        if let Some(v) = arg_val(args, "--torque-clamp-pct") {
            cfg.torque_clamp_tenths = parse_clamp_tenths(&v)?;
        }
        cfg.dynamics_profile = arg_val(args, "--slave-dynamics-profile");
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
            "--velocity-ff" => {
                let cur = slaves
                    .last_mut()
                    .ok_or_else(|| "--velocity-ff appeared before any --slave group".to_string())?;
                cur.velocity_ff = true;
                i += 1;
            }
            "--invert" => {
                let cur = slaves
                    .last_mut()
                    .ok_or_else(|| "--invert appeared before any --slave group".to_string())?;
                cur.invert = true;
                i += 1;
            }
            f @ ("--axis"
            | "--counts-per-mm"
            | "--rotation-distance"
            | "--following-error-counts"
            | "--max-torque-tenth-pct"
            | "--torque-clamp-pct"
            | "--slave-dynamics-profile") => {
                let v = args
                    .get(i + 1)
                    .ok_or_else(|| format!("{f} requires a value"))?;
                let cur = slaves
                    .last_mut()
                    .ok_or_else(|| format!("{f} appeared before any --slave group"))?;
                match f {
                    "--slave-dynamics-profile" => {
                        cur.dynamics_profile = Some(v.clone());
                    }
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
                    "--max-torque-tenth-pct" => {
                        cur.max_torque_tenth_pct = Some(
                            v.parse()
                                .map_err(|_| "--max-torque-tenth-pct not a number".to_string())?,
                        );
                    }
                    _ => {
                        cur.torque_clamp_tenths = parse_clamp_tenths(v)?;
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
