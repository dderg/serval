use serde::Deserialize;

pub const ERR_DYNAMICS_BAD_DIM: i32 = -861;
pub const ERR_DYNAMICS_REJECTED: i32 = -862;

#[derive(Debug, Deserialize)]
struct PairTable {
    slots: Vec<String>,
    belt_position_split: [f64; 2],
}

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
    pair: Vec<PairTable>,
}

/// A pair load-share spec as it arrives from the profile or the wire: two slot
/// indices and the shared differential split `w0 + w1·p_belt` applied to the
/// total belt force. `λ` is derived from the frame during validation, never
/// supplied.
#[derive(Debug, Clone, Copy)]
pub struct PairSpec {
    pub first: usize,
    pub second: usize,
    pub w: [f32; 2],
}

#[derive(Debug, Clone, Copy)]
struct Pair {
    first: usize,
    second: usize,
    lambda: f32,
    w: [f32; 2],
}

#[derive(Debug)]
pub enum ProfileError {
    Parse(String),
    Version(u32),
    Dim(&'static str),
    NotFinite(&'static str),
    NonPositive(&'static str),
    ZeroFrameRow(usize),
    FrameRankDeficient,
    PairSlot(&'static str),
    PairNotParallel(usize),
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
    pairs: Vec<Pair>,
    drive_signs: Option<Vec<f32>>,
}

impl DynamicsModel {
    pub fn from_toml_str(s: &str) -> Result<Self, ProfileError> {
        let f: ProfileFile = toml::from_str(s).map_err(|e| ProfileError::Parse(e.to_string()))?;
        if f.version != 4 {
            return Err(ProfileError::Version(f.version));
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
        let mut pairs = Vec::with_capacity(f.pair.len());
        for p in &f.pair {
            if p.slots.len() != 2 {
                return Err(ProfileError::PairSlot(
                    "pair slots must name exactly two axes",
                ));
            }
            let resolve = |name: &str| f.axes.iter().position(|a| a == name);
            let first = resolve(&p.slots[0])
                .ok_or(ProfileError::PairSlot("pair slot not found in axes"))?;
            let second = resolve(&p.slots[1])
                .ok_or(ProfileError::PairSlot("pair slot not found in axes"))?;
            pairs.push(PairSpec {
                first,
                second,
                w: [
                    p.belt_position_split[0] as f32,
                    p.belt_position_split[1] as f32,
                ],
            });
        }
        Self::validated(
            n_slots, n_modes, f.axes, f.modes, frame, f.mass, f.viscous, f.coulomb, &pairs,
        )
    }

    pub fn from_parts(
        n_slots: usize,
        n_modes: usize,
        frame: &[f32],
        mass: &[f32],
        viscous: &[f32],
        coulomb: &[f32],
        pairs: &[PairSpec],
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
            pairs,
        )
    }

    /// Bind the per-slot drive direction signs (`±1`, from
    /// `cmd_counts_per_mm.signum()`). Profiles are machine-config independent,
    /// so the pair load-share differential can only be evaluated once the
    /// endpoint hands the model its own drive signs. A model carrying pairs
    /// panics on evaluation until this is called.
    pub fn bind_drive_signs(&mut self, signs: &[f32]) {
        assert_eq!(
            signs.len(),
            self.n_slots,
            "drive signs must be one per slot"
        );
        assert!(
            signs.iter().all(|&s| s == 1.0 || s == -1.0),
            "drive signs must each be +1 or -1"
        );
        self.drive_signs = Some(signs.to_vec());
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
        pairs: &[PairSpec],
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
        let frame32: Vec<f32> = frame.iter().map(|&v| v as f32).collect();
        let resolved = resolve_pairs(&frame32, n_slots, n_modes, pairs)?;
        Ok(Self {
            n_slots,
            n_modes,
            axes,
            modes,
            frame: frame32,
            mass: mass.iter().map(|&v| v as f32).collect(),
            viscous: viscous.iter().map(|&v| v as f32).collect(),
            coulomb: coulomb.iter().map(|&v| v as f32).collect(),
            pairs: resolved,
            drive_signs: None,
        })
    }

    pub fn torque_ff(
        &self,
        slot: usize,
        acc_mm_s2: &[f32],
        vel_mm_s: &[f32],
        pos_mm: &[f32],
    ) -> f32 {
        self.eval(slot, acc_mm_s2, vel_mm_s, pos_mm, true)
    }

    /// Buzzed cycles use this variant: a buzz flips sign(v_mode) every half
    /// period, which would turn the Coulomb term into a square-wave torque at
    /// the buzz frequency — far larger than the micrometre-scale excitation it
    /// rides on. Coulomb is a mode-space quantity, so a buzz on any slot
    /// contaminates it and the whole node must drop Coulomb for the cycle. The
    /// pair load-share differential is built from those same mode forces, so it
    /// is dropped in lockstep for the whole node.
    pub fn torque_ff_without_coulomb(
        &self,
        slot: usize,
        acc_mm_s2: &[f32],
        vel_mm_s: &[f32],
        pos_mm: &[f32],
    ) -> f32 {
        self.eval(slot, acc_mm_s2, vel_mm_s, pos_mm, false)
    }

    fn eval(
        &self,
        slot: usize,
        acc_mm_s2: &[f32],
        vel_mm_s: &[f32],
        pos_mm: &[f32],
        with_coulomb: bool,
    ) -> f32 {
        assert_eq!(acc_mm_s2.len(), self.n_slots);
        assert_eq!(vel_mm_s.len(), self.n_slots);
        assert_eq!(pos_mm.len(), self.n_slots);
        assert!(slot < self.n_slots);
        let pair = if with_coulomb {
            self.pairs
                .iter()
                .find(|p| p.first == slot || p.second == slot)
        } else {
            None
        };
        let mut tau = 0.0f32;
        let mut belt = 0.0f32;
        for k in 0..self.n_modes {
            let row = &self.frame[k * self.n_slots..][..self.n_slots];
            let a_mode: f32 = row.iter().zip(acc_mm_s2).map(|(f, a)| f * a).sum();
            let v_mode: f32 = row.iter().zip(vel_mm_s).map(|(f, v)| f * v).sum();
            let f_inertial = self.mass[k] * a_mode;
            let f_viscous = self.viscous[k] * v_mode;
            let f_coulomb = self.coulomb[k] * strict_sign(v_mode);
            let mut mode_force = f_inertial + f_viscous;
            if with_coulomb {
                mode_force += f_coulomb;
            }
            tau += row[slot] * mode_force;
            if let Some(p) = pair {
                belt += row[p.first] * (f_inertial + f_viscous + f_coulomb);
            }
        }
        if let Some(p) = pair {
            tau += self.pair_differential(p, slot, belt, pos_mm);
        }
        tau
    }

    fn pair_differential(&self, p: &Pair, slot: usize, belt_share: f32, pos_mm: &[f32]) -> f32 {
        let signs = self
            .drive_signs
            .as_ref()
            .expect("dynamics model with pairs evaluated before drive signs were bound");
        let s_first = signs[p.first];
        let s_second = signs[p.second];
        let belt_sign = s_first + p.lambda * s_second;
        let p_belt = s_first * pos_mm[p.first];
        let differential = (p.w[0] + p.w[1] * p_belt) * belt_sign * belt_share;
        if slot == p.first {
            s_first * differential * 0.5
        } else {
            -s_second * differential * 0.5
        }
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
        let mut pairs = Vec::new();
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
            for pr in &p.pairs {
                pairs.push(Pair {
                    first: pr.first + slot_base,
                    second: pr.second + slot_base,
                    lambda: pr.lambda,
                    w: pr.w,
                });
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
            pairs,
            drive_signs: None,
        })
    }
}

fn resolve_pairs(
    frame: &[f32],
    n_slots: usize,
    n_modes: usize,
    pairs: &[PairSpec],
) -> Result<Vec<Pair>, ProfileError> {
    let mut used = vec![false; n_slots];
    let mut resolved = Vec::with_capacity(pairs.len());
    for (idx, p) in pairs.iter().enumerate() {
        if p.first >= n_slots || p.second >= n_slots {
            return Err(ProfileError::PairSlot("pair slot index out of range"));
        }
        if p.first == p.second {
            return Err(ProfileError::PairSlot("pair slots must be distinct"));
        }
        if !p.w.iter().all(|v| v.is_finite()) {
            return Err(ProfileError::NotFinite("pair split weight not finite"));
        }
        for &slot in &[p.first, p.second] {
            if used[slot] {
                return Err(ProfileError::PairSlot("slot appears in more than one pair"));
            }
            used[slot] = true;
        }
        let lambda = derive_lambda(frame, n_slots, n_modes, p.first, p.second)
            .ok_or(ProfileError::PairNotParallel(idx))?;
        resolved.push(Pair {
            first: p.first,
            second: p.second,
            lambda,
            w: p.w,
        });
    }
    Ok(resolved)
}

/// A pair's two frame columns must be parallel with `λ = ±1` exactly. Returns
/// the derived `λ`, or `None` when neither sign reproduces the second column
/// from the first within a small relative tolerance.
fn derive_lambda(
    frame: &[f32],
    n_slots: usize,
    n_modes: usize,
    first: usize,
    second: usize,
) -> Option<f32> {
    let matches = |lambda: f32| {
        (0..n_modes).all(|k| {
            let a = frame[k * n_slots + first];
            let b = frame[k * n_slots + second];
            let scale = a.abs().max(b.abs());
            (b - lambda * a).abs() <= 1e-9 * scale
        })
    };
    if matches(1.0) {
        Some(1.0)
    } else if matches(-1.0) {
        Some(-1.0)
    } else {
        None
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
