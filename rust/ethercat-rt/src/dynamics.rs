use serde::Deserialize;

pub const ERR_DYNAMICS_BAD_DIM: i32 = -861;
pub const ERR_DYNAMICS_REJECTED: i32 = -862;

#[derive(Debug, Deserialize)]
struct ProfileFile {
    version: u32,
    axes: Vec<String>,
    mass: Vec<Vec<f64>>,
    viscous: Vec<f64>,
    coulomb_fwd: Vec<f64>,
    coulomb_rev: Vec<f64>,
    coulomb_deadband_mm_s: f64,
}

#[derive(Debug)]
pub enum ProfileError {
    Parse(String),
    Version(u32),
    Dim(&'static str),
    NotFinite(&'static str),
    NotSymmetric,
    NotPositiveDefinite,
}

#[derive(Debug)]
pub struct DynamicsModel {
    pub n: usize,
    pub axes: Vec<String>,
    mass: Vec<f32>,
    viscous: Vec<f32>,
    coulomb_fwd: Vec<f32>,
    coulomb_rev: Vec<f32>,
    deadband: f32,
}

impl DynamicsModel {
    pub fn from_toml_str(s: &str) -> Result<Self, ProfileError> {
        let f: ProfileFile = toml::from_str(s).map_err(|e| ProfileError::Parse(e.to_string()))?;
        if f.version != 1 {
            return Err(ProfileError::Version(f.version));
        }
        let n = f.axes.len();
        if f.mass.len() != n || f.mass.iter().any(|row| row.len() != n) {
            return Err(ProfileError::Dim("mass must be n x n"));
        }
        let mass: Vec<f64> = f.mass.iter().flatten().copied().collect();
        Self::validated(
            n,
            f.axes,
            mass,
            f.viscous,
            f.coulomb_fwd,
            f.coulomb_rev,
            f.coulomb_deadband_mm_s,
        )
    }

    pub fn from_parts(
        n: usize,
        mass: &[f32],
        viscous: &[f32],
        coulomb_fwd: &[f32],
        coulomb_rev: &[f32],
        deadband_mm_s: f32,
    ) -> Result<Self, ProfileError> {
        let axes = (0..n).map(|i| format!("slot{i}")).collect();
        let widen = |v: &[f32]| v.iter().map(|&x| f64::from(x)).collect::<Vec<f64>>();
        Self::validated(
            n,
            axes,
            widen(mass),
            widen(viscous),
            widen(coulomb_fwd),
            widen(coulomb_rev),
            f64::from(deadband_mm_s),
        )
    }

    fn validated(
        n: usize,
        axes: Vec<String>,
        mass: Vec<f64>,
        viscous: Vec<f64>,
        coulomb_fwd: Vec<f64>,
        coulomb_rev: Vec<f64>,
        deadband_mm_s: f64,
    ) -> Result<Self, ProfileError> {
        if n == 0 {
            return Err(ProfileError::Dim("axes is empty"));
        }
        if mass.len() != n * n {
            return Err(ProfileError::Dim("mass must be n x n"));
        }
        if viscous.len() != n {
            return Err(ProfileError::Dim("viscous length"));
        }
        if coulomb_fwd.len() != n {
            return Err(ProfileError::Dim("coulomb_fwd length"));
        }
        if coulomb_rev.len() != n {
            return Err(ProfileError::Dim("coulomb_rev length"));
        }
        let all_finite = mass
            .iter()
            .chain(&viscous)
            .chain(&coulomb_fwd)
            .chain(&coulomb_rev)
            .chain(std::iter::once(&deadband_mm_s))
            .all(|v| v.is_finite());
        if !all_finite {
            return Err(ProfileError::NotFinite("profile contains non-finite value"));
        }
        for i in 0..n {
            for j in (i + 1)..n {
                let (a, b) = (mass[i * n + j], mass[j * n + i]);
                if (a - b).abs() > 1e-9 * a.abs().max(b.abs()).max(1e-12) {
                    return Err(ProfileError::NotSymmetric);
                }
            }
        }
        if !cholesky_is_pd(&mass, n) {
            return Err(ProfileError::NotPositiveDefinite);
        }
        Ok(Self {
            n,
            axes,
            mass: mass.iter().map(|&v| v as f32).collect(),
            viscous: viscous.iter().map(|&v| v as f32).collect(),
            coulomb_fwd: coulomb_fwd.iter().map(|&v| v as f32).collect(),
            coulomb_rev: coulomb_rev.iter().map(|&v| v as f32).collect(),
            deadband: deadband_mm_s as f32,
        })
    }

    pub fn torque_ff(&self, axis: usize, acc_mm_s2: &[f32], vel_mm_s: &[f32]) -> f32 {
        let v = vel_mm_s[axis];
        let coulomb = if v > self.deadband {
            self.coulomb_fwd[axis]
        } else if v < -self.deadband {
            self.coulomb_rev[axis]
        } else {
            0.0
        };
        self.torque_ff_without_coulomb(axis, acc_mm_s2, vel_mm_s) + coulomb
    }

    /// Buzzed slots use this variant: a buzz flips sign(v) every half period,
    /// which would turn the Coulomb term into a square-wave torque at the buzz
    /// frequency — far larger than the micrometre-scale excitation it rides on.
    pub fn torque_ff_without_coulomb(
        &self,
        axis: usize,
        acc_mm_s2: &[f32],
        vel_mm_s: &[f32],
    ) -> f32 {
        assert_eq!(acc_mm_s2.len(), self.n);
        assert_eq!(vel_mm_s.len(), self.n);
        assert!(axis < self.n);
        let row = &self.mass[axis * self.n..][..self.n];
        let inertial: f32 = row.iter().zip(acc_mm_s2.iter()).map(|(m, a)| m * a).sum();
        inertial + self.viscous[axis] * vel_mm_s[axis]
    }

    /// Stack independent per-servo profiles into one node model whose mass
    /// matrix is block-diagonal — the cartesian case, where no axis's torque
    /// depends on another's acceleration. `parts` must be in slot order.
    pub fn block_diagonal(parts: Vec<DynamicsModel>) -> Result<Self, ProfileError> {
        if parts.is_empty() {
            return Err(ProfileError::Dim(
                "block_diagonal needs at least one profile",
            ));
        }
        let n: usize = parts.iter().map(|p| p.n).sum();
        let deadband = parts[0].deadband;
        if parts.iter().any(|p| (p.deadband - deadband).abs() > 1e-6) {
            return Err(ProfileError::Dim(
                "per-servo profiles disagree on coulomb deadband",
            ));
        }
        let mut mass = vec![0.0f32; n * n];
        let mut viscous = Vec::with_capacity(n);
        let mut coulomb_fwd = Vec::with_capacity(n);
        let mut coulomb_rev = Vec::with_capacity(n);
        let mut axes = Vec::with_capacity(n);
        let mut base = 0usize;
        for p in &parts {
            for i in 0..p.n {
                for j in 0..p.n {
                    mass[(base + i) * n + (base + j)] = p.mass[i * p.n + j];
                }
                viscous.push(p.viscous[i]);
                coulomb_fwd.push(p.coulomb_fwd[i]);
                coulomb_rev.push(p.coulomb_rev[i]);
                axes.push(p.axes[i].clone());
            }
            base += p.n;
        }
        Ok(Self {
            n,
            axes,
            mass,
            viscous,
            coulomb_fwd,
            coulomb_rev,
            deadband,
        })
    }
}

fn cholesky_is_pd(m: &[f64], n: usize) -> bool {
    let mut l = m.to_vec();
    for k in 0..n {
        for j in 0..k {
            l[k * n + k] -= l[k * n + j] * l[k * n + j];
        }
        if l[k * n + k] <= 0.0 {
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
