pub const COULOMB_DEADBAND_MM_S: f64 = 0.5;

/// Cartesian mode-space feedforward model. `frame` is `n_modes × n_slots`,
/// mapping drive-frame commanded slot velocities/accels to mode quantities:
/// `v_mode[k] = Σ_s frame[k][s] · v_slot[s]`. Per-mode scalars (mass,
/// viscous, coulomb) then produce torque per slot via the same frame:
/// `tau_slot[s] = Σ_k frame[k][s] · (mass[k]·a_mode[k] + viscous[k]·v_mode[k]
/// + coulomb[k]·sgn(v_mode[k]))`.
#[derive(Debug, Clone, PartialEq)]
pub struct Structure {
    pub frame: Vec<Vec<f64>>,
}

/// One entry per Cartesian mode.
#[derive(Debug, PartialEq)]
pub struct PhysicalParams {
    pub mass: Vec<f64>,
    pub viscous: Vec<f64>,
    pub coulomb: Vec<f64>,
}

pub fn coulomb_sign(v: f64) -> f64 {
    if v > COULOMB_DEADBAND_MM_S {
        1.0
    } else if v < -COULOMB_DEADBAND_MM_S {
        -1.0
    } else {
        0.0
    }
}

impl Structure {
    pub fn new(frame: Vec<Vec<f64>>) -> Self {
        assert!(!frame.is_empty(), "frame needs at least one mode");
        let slots = frame[0].len();
        assert!(slots > 0, "frame needs at least one slot");
        assert!(
            frame.iter().all(|r| r.len() == slots),
            "every frame row must have the same slot count"
        );
        assert!(
            frame.len() <= slots,
            "n_modes ({}) must not exceed n_slots ({slots})",
            frame.len()
        );
        Self { frame }
    }

    pub fn axis_count(&self) -> usize {
        self.frame[0].len()
    }

    pub fn mode_count(&self) -> usize {
        self.frame.len()
    }

    /// theta grouped per mode: `[mass_0, viscous_0, coulomb_0, mass_1, ...]`.
    pub fn param_count(&self) -> usize {
        3 * self.mode_count()
    }

    /// Regression row for slot `motor` such that
    /// `tau_slot[motor] = row(...) · theta`. `acc_mode`/`vel_mode`/`cs_mode`
    /// are the per-mode channels at one sample; `cs_mode` is the coulomb sign
    /// column (`coulomb_sign` of the raw mode velocity, then band-filtered).
    pub fn row(
        &self,
        motor: usize,
        acc_mode: &[f64],
        vel_mode: &[f64],
        cs_mode: &[f64],
    ) -> Vec<f64> {
        let n_modes = self.mode_count();
        assert!(motor < self.axis_count(), "motor out of range");
        assert!(
            acc_mode.len() == n_modes && vel_mode.len() == n_modes && cs_mode.len() == n_modes,
            "row needs one value per mode in every channel"
        );
        let mut r = vec![0.0; 3 * n_modes];
        for k in 0..n_modes {
            let f = self.frame[k][motor];
            r[3 * k] = f * acc_mode[k];
            r[3 * k + 1] = f * vel_mode[k];
            r[3 * k + 2] = f * cs_mode[k];
        }
        r
    }

    pub fn pack(&self, p: &PhysicalParams) -> Vec<f64> {
        let n_modes = self.mode_count();
        assert!(
            p.mass.len() == n_modes && p.viscous.len() == n_modes && p.coulomb.len() == n_modes,
            "params must have one entry per mode"
        );
        let mut theta = Vec::with_capacity(3 * n_modes);
        for k in 0..n_modes {
            theta.push(p.mass[k]);
            theta.push(p.viscous[k]);
            theta.push(p.coulomb[k]);
        }
        theta
    }

    pub fn unpack(&self, theta: &[f64]) -> PhysicalParams {
        assert_eq!(theta.len(), self.param_count());
        let n_modes = self.mode_count();
        let mut mass = Vec::with_capacity(n_modes);
        let mut viscous = Vec::with_capacity(n_modes);
        let mut coulomb = Vec::with_capacity(n_modes);
        for k in 0..n_modes {
            mass.push(theta[3 * k]);
            viscous.push(theta[3 * k + 1]);
            coulomb.push(theta[3 * k + 2]);
        }
        PhysicalParams {
            mass,
            viscous,
            coulomb,
        }
    }
}
