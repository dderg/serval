use serde::Deserialize;

pub const ERR_DYNAMICS_BAD_DIM: i32 = -861;
pub const ERR_DYNAMICS_REJECTED: i32 = -862;

#[derive(Debug, Deserialize)]
struct ProfileFile {
    version: u32,
    axes: Vec<String>,
    modes: Vec<String>,
    frame: Vec<Vec<f64>>,
    mass: Vec<f64>,
    viscous: Vec<f64>,
    coulomb: Vec<f64>,
    #[serde(default)]
    #[allow(dead_code)]
    fit_rms_residual: Vec<f64>,
    #[serde(default)]
    pair: Vec<toml::Value>,
}

#[derive(Debug)]
pub enum ProfileError {
    Parse(String),
    Version(u32),
    PairTablesRemoved,
    Dim(&'static str),
    NotFinite(&'static str),
    NonPositive(&'static str),
    ZeroFrameRow(usize),
    FrameRankDeficient,
}

#[derive(Debug)]
pub struct DynamicsModel {
    pub n_slots: usize,
    pub n_modes: usize,
    pub axes: Vec<String>,
    pub modes: Vec<String>,
    frame: Vec<f32>,
    mass: Vec<f32>,
    viscous: Vec<f32>,
    coulomb: Vec<f32>,
}

impl DynamicsModel {
    pub fn from_toml_str(s: &str) -> Result<Self, ProfileError> {
        let f: ProfileFile = toml::from_str(s).map_err(|e| ProfileError::Parse(e.to_string()))?;
        if f.version != 6 {
            return Err(ProfileError::Version(f.version));
        }
        if !f.pair.is_empty() {
            return Err(ProfileError::PairTablesRemoved);
        }
        let n_slots = f.axes.len();
        let n_modes = f.modes.len();
        if f.frame.len() != n_modes {
            return Err(ProfileError::Dim("frame row count must equal modes"));
        }
        if f.frame.iter().any(|row| row.len() != n_slots) {
            return Err(ProfileError::Dim("frame row width must equal slots"));
        }
        let frame: Vec<f64> = f.frame.iter().flatten().copied().collect();
        Self::validated(
            n_slots, n_modes, f.axes, f.modes, frame, f.mass, f.viscous, f.coulomb,
        )
    }

    pub fn from_parts(
        n_slots: usize,
        n_modes: usize,
        frame: &[f32],
        mass: &[f32],
        viscous: &[f32],
        coulomb: &[f32],
    ) -> Result<Self, ProfileError> {
        let axes = (0..n_slots).map(|i| format!("slot{i}")).collect();
        let modes = (0..n_modes).map(|k| format!("mode{k}")).collect();
        let widen = |v: &[f32]| v.iter().map(|&x| f64::from(x)).collect::<Vec<f64>>();
        Self::validated(
            n_slots,
            n_modes,
            axes,
            modes,
            widen(frame),
            widen(mass),
            widen(viscous),
            widen(coulomb),
        )
    }

    #[allow(clippy::too_many_arguments)]
    fn validated(
        n_slots: usize,
        n_modes: usize,
        axes: Vec<String>,
        modes: Vec<String>,
        frame: Vec<f64>,
        mass: Vec<f64>,
        viscous: Vec<f64>,
        coulomb: Vec<f64>,
    ) -> Result<Self, ProfileError> {
        if axes.len() != n_slots {
            return Err(ProfileError::Dim("axes length must equal slots"));
        }
        if n_modes < 1 {
            return Err(ProfileError::Dim("at least one mode required"));
        }
        if modes.len() != n_modes {
            return Err(ProfileError::Dim("modes length must equal modes"));
        }
        if n_modes > n_slots {
            return Err(ProfileError::Dim("modes must not exceed slots"));
        }
        if frame.len() != n_modes * n_slots {
            return Err(ProfileError::Dim("frame must be modes x slots"));
        }
        if mass.len() != n_modes {
            return Err(ProfileError::Dim("mass length must equal modes"));
        }
        if viscous.len() != n_modes {
            return Err(ProfileError::Dim("viscous length must equal modes"));
        }
        if coulomb.len() != n_modes {
            return Err(ProfileError::Dim("coulomb length must equal modes"));
        }
        let all_finite = frame
            .iter()
            .chain(&mass)
            .chain(&viscous)
            .chain(&coulomb)
            .all(|v| v.is_finite());
        if !all_finite {
            return Err(ProfileError::NotFinite("profile contains non-finite value"));
        }
        for &m in &mass {
            if m <= 0.0 {
                return Err(ProfileError::NonPositive("mass must be positive"));
            }
        }
        for &b in &viscous {
            if b < 0.0 {
                return Err(ProfileError::NonPositive("viscous must be non-negative"));
            }
        }
        for &c in &coulomb {
            if c < 0.0 {
                return Err(ProfileError::NonPositive("coulomb must be non-negative"));
            }
        }
        for k in 0..n_modes {
            let row = &frame[k * n_slots..][..n_slots];
            if row.iter().all(|&x| x == 0.0) {
                return Err(ProfileError::ZeroFrameRow(k));
            }
        }
        if !frame_rows_independent(&frame, n_modes, n_slots) {
            return Err(ProfileError::FrameRankDeficient);
        }
        Ok(Self {
            n_slots,
            n_modes,
            axes,
            modes,
            frame: frame.iter().map(|&v| v as f32).collect(),
            mass: mass.iter().map(|&v| v as f32).collect(),
            viscous: viscous.iter().map(|&v| v as f32).collect(),
            coulomb: coulomb.iter().map(|&v| v as f32).collect(),
        })
    }

    pub fn torque_ff(&self, slot: usize, acc_mm_s2: &[f32], vel_mm_s: &[f32]) -> f32 {
        self.eval(slot, acc_mm_s2, vel_mm_s, true)
    }

    /// Buzzed cycles use this variant: a buzz flips sign(v_mode) every half
    /// period, which would turn the Coulomb term into a square-wave torque at
    /// the buzz frequency — far larger than the micrometre-scale excitation it
    /// rides on. Coulomb is a mode-space quantity, so a buzz on any slot
    /// contaminates it and the whole node must drop Coulomb for the cycle.
    pub fn torque_ff_without_coulomb(
        &self,
        slot: usize,
        acc_mm_s2: &[f32],
        vel_mm_s: &[f32],
    ) -> f32 {
        self.eval(slot, acc_mm_s2, vel_mm_s, false)
    }

    fn eval(&self, slot: usize, acc_mm_s2: &[f32], vel_mm_s: &[f32], with_coulomb: bool) -> f32 {
        assert_eq!(acc_mm_s2.len(), self.n_slots);
        assert_eq!(vel_mm_s.len(), self.n_slots);
        assert!(slot < self.n_slots);
        let mut tau = 0.0f32;
        for k in 0..self.n_modes {
            let row = &self.frame[k * self.n_slots..][..self.n_slots];
            let a_mode: f32 = row.iter().zip(acc_mm_s2).map(|(f, a)| f * a).sum();
            let v_mode: f32 = row.iter().zip(vel_mm_s).map(|(f, v)| f * v).sum();
            let mut mode_force = self.mass[k] * a_mode + self.viscous[k] * v_mode;
            if with_coulomb {
                mode_force += self.coulomb[k] * strict_sign(v_mode);
            }
            tau += row[slot] * mode_force;
        }
        tau
    }

    /// Stack independent per-servo profiles into one node model. Slots and
    /// modes concatenate, the frame becomes block-diagonal, and the per-mode
    /// vectors concatenate. `parts` must be in slot order.
    pub fn block_diagonal(parts: Vec<DynamicsModel>) -> Result<Self, ProfileError> {
        if parts.is_empty() {
            return Err(ProfileError::Dim(
                "block_diagonal needs at least one profile",
            ));
        }
        let n_slots: usize = parts.iter().map(|p| p.n_slots).sum();
        let n_modes: usize = parts.iter().map(|p| p.n_modes).sum();
        let mut frame = vec![0.0f32; n_modes * n_slots];
        let mut mass = Vec::with_capacity(n_modes);
        let mut viscous = Vec::with_capacity(n_modes);
        let mut coulomb = Vec::with_capacity(n_modes);
        let mut axes = Vec::with_capacity(n_slots);
        let mut modes = Vec::with_capacity(n_modes);
        let mut slot_base = 0usize;
        let mut mode_base = 0usize;
        for p in &parts {
            for k in 0..p.n_modes {
                for s in 0..p.n_slots {
                    frame[(mode_base + k) * n_slots + (slot_base + s)] = p.frame[k * p.n_slots + s];
                }
                mass.push(p.mass[k]);
                viscous.push(p.viscous[k]);
                coulomb.push(p.coulomb[k]);
                modes.push(p.modes[k].clone());
            }
            for s in 0..p.n_slots {
                axes.push(p.axes[s].clone());
            }
            slot_base += p.n_slots;
            mode_base += p.n_modes;
        }
        Ok(Self {
            n_slots,
            n_modes,
            axes,
            modes,
            frame,
            mass,
            viscous,
            coulomb,
        })
    }
}

fn strict_sign(v: f32) -> f32 {
    if v > 0.0 {
        1.0
    } else if v < 0.0 {
        -1.0
    } else {
        0.0
    }
}

fn frame_rows_independent(frame: &[f64], n_modes: usize, n_slots: usize) -> bool {
    let mut gram = vec![0.0f64; n_modes * n_modes];
    for i in 0..n_modes {
        let ri = &frame[i * n_slots..][..n_slots];
        for j in 0..n_modes {
            let rj = &frame[j * n_slots..][..n_slots];
            gram[i * n_modes + j] = ri.iter().zip(rj).map(|(a, b)| a * b).sum();
        }
    }
    let scale = (0..n_modes)
        .map(|i| gram[i * n_modes + i])
        .fold(0.0, f64::max);
    if scale <= 0.0 {
        return false;
    }
    cholesky_is_pd(&gram, n_modes, scale * 1e-9)
}

/// Rank check: a rank-deficient Gram matrix leaves a final pivot at zero in
/// exact arithmetic, so floating-point rounding can push it either side of
/// zero. `pivot_floor` keeps that noise from reading as positive-definite.
fn cholesky_is_pd(m: &[f64], n: usize, pivot_floor: f64) -> bool {
    let mut l = m.to_vec();
    for k in 0..n {
        for j in 0..k {
            l[k * n + k] -= l[k * n + j] * l[k * n + j];
        }
        if l[k * n + k] <= pivot_floor {
            return false;
        }
        l[k * n + k] = l[k * n + k].sqrt();
        for i in (k + 1)..n {
            for j in 0..k {
                l[i * n + k] -= l[i * n + j] * l[k * n + j];
            }
            l[i * n + k] /= l[k * n + k];
        }
    }
    true
}

pub fn clamp_torque(raw_tenths_pct: f32, limit_tenths_pct: i16, saturation_count: &mut u32) -> i16 {
    assert!(raw_tenths_pct.is_finite(), "non-finite torque FF");
    assert!(limit_tenths_pct > 0, "torque clamp limit must be positive");
    let lim = f32::from(limit_tenths_pct);
    if raw_tenths_pct > lim {
        *saturation_count += 1;
        limit_tenths_pct
    } else if raw_tenths_pct < -lim {
        *saturation_count += 1;
        -limit_tenths_pct
    } else {
        raw_tenths_pct as i16
    }
}

#[cfg(test)]
mod tests;
