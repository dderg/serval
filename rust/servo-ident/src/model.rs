pub const COULOMB_DEADBAND_MM_S: f64 = 0.5;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Structure {
    CartesianScalar,
    CoreXY,
}

#[derive(Debug, PartialEq)]
pub struct PhysicalParams {
    pub mass: Vec<Vec<f64>>,
    pub viscous: Vec<f64>,
    pub coulomb_fwd: Vec<f64>,
    pub coulomb_rev: Vec<f64>,
}

pub fn coulomb_cols(v: f64) -> (f64, f64) {
    if v > COULOMB_DEADBAND_MM_S {
        (1.0, 0.0)
    } else if v < -COULOMB_DEADBAND_MM_S {
        (0.0, 1.0)
    } else {
        (0.0, 0.0)
    }
}

impl Structure {
    pub fn axis_count(self) -> usize {
        match self {
            Structure::CartesianScalar => 1,
            Structure::CoreXY => 2,
        }
    }

    /// Scalar: theta = [m, b, c_fwd, c_rev].
    /// CoreXY: theta = [m_diag, m_off, b_a, cf_a, cr_a, b_b, cf_b, cr_b].
    pub fn param_count(self) -> usize {
        match self {
            Structure::CartesianScalar => 4,
            Structure::CoreXY => 8,
        }
    }

    /// Regression row such that tau_motor = row(motor, acc, vel, cf, cr) · theta.
    /// `cf`/`cr` are the per-motor coulomb regressor columns — computed from
    /// raw velocity via `coulomb_cols`, then filtered identically to every
    /// other channel so the regression stays consistent under band-limiting.
    pub fn row(self, motor: usize, acc: &[f64], vel: &[f64], cf: &[f64], cr: &[f64]) -> Vec<f64> {
        match self {
            Structure::CartesianScalar => {
                assert_eq!(motor, 0);
                assert!(
                    !acc.is_empty() && !vel.is_empty() && !cf.is_empty() && !cr.is_empty(),
                    "scalar row needs 1 sample of each channel"
                );
                vec![acc[0], vel[0], cf[0], cr[0]]
            }
            Structure::CoreXY => {
                assert!(motor < 2);
                assert!(
                    acc.len() >= 2 && vel.len() >= 2 && cf.len() >= 2 && cr.len() >= 2,
                    "corexy row needs 2 samples of each channel"
                );
                let other = 1 - motor;
                #[allow(clippy::indexing_slicing)]
                let mut r = vec![acc[motor], acc[other], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
                let base = 2 + 3 * motor;
                #[allow(clippy::indexing_slicing)]
                {
                    r[base] = vel[motor];
                    r[base + 1] = cf[motor];
                    r[base + 2] = cr[motor];
                }
                r
            }
        }
    }

    pub fn pack(self, p: &PhysicalParams) -> Vec<f64> {
        match self {
            Structure::CartesianScalar => vec![
                p.mass[0][0],
                p.viscous[0],
                p.coulomb_fwd[0],
                p.coulomb_rev[0],
            ],
            Structure::CoreXY => vec![
                p.mass[0][0],
                p.mass[0][1],
                p.viscous[0],
                p.coulomb_fwd[0],
                p.coulomb_rev[0],
                p.viscous[1],
                p.coulomb_fwd[1],
                p.coulomb_rev[1],
            ],
        }
    }

    pub fn unpack(self, theta: &[f64]) -> PhysicalParams {
        assert_eq!(theta.len(), self.param_count());
        match self {
            Structure::CartesianScalar => PhysicalParams {
                mass: vec![vec![theta[0]]],
                viscous: vec![theta[1]],
                coulomb_fwd: vec![theta[2]],
                coulomb_rev: vec![theta[3]],
            },
            Structure::CoreXY => PhysicalParams {
                mass: vec![vec![theta[0], theta[1]], vec![theta[1], theta[0]]],
                viscous: vec![theta[2], theta[5]],
                coulomb_fwd: vec![theta[3], theta[6]],
                coulomb_rev: vec![theta[4], theta[7]],
            },
        }
    }
}
