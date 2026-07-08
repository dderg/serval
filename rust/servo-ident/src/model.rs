pub const COULOMB_DEADBAND_MM_S: f64 = 0.5;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Structure {
    CartesianScalar,
    CoreXY,
    /// CoreXY with two drives per belt, motor order [a0, a1, b0, b1].
    /// Drives of a pair share a belt, so they are commanded identical
    /// kinematics and each carries the fitted per-drive share of the load;
    /// m_diag/m_off are per-drive masses (roughly half the single-drive
    /// CoreXY values), friction is per drive.
    CoreXYAwd,
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
            Structure::CoreXYAwd => 4,
        }
    }

    /// Scalar: theta = [m, b, c_fwd, c_rev].
    /// CoreXY: theta = [m_diag, m_off, b_a, cf_a, cr_a, b_b, cf_b, cr_b].
    /// CoreXYAwd: theta = [m_diag, m_off, then (b, cf, cr) per drive x4].
    pub fn param_count(self) -> usize {
        match self {
            Structure::CartesianScalar => 4,
            Structure::CoreXY => 8,
            Structure::CoreXYAwd => 14,
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
            Structure::CoreXYAwd => {
                assert!(motor < 4);
                assert!(
                    acc.len() >= 4 && vel.len() >= 4 && cf.len() >= 4 && cr.len() >= 4,
                    "corexy-awd row needs 4 samples of each channel"
                );
                let other_belt = motor ^ 2;
                let mut r = vec![0.0; 14];
                #[allow(clippy::indexing_slicing)]
                {
                    r[0] = acc[motor];
                    r[1] = acc[other_belt];
                    let base = 2 + 3 * motor;
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
            Structure::CoreXYAwd => {
                let mut theta = vec![p.mass[0][0], p.mass[0][2] + p.mass[0][3]];
                for m in 0..4 {
                    theta.push(p.viscous[m]);
                    theta.push(p.coulomb_fwd[m]);
                    theta.push(p.coulomb_rev[m]);
                }
                theta
            }
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
            // The endpoint evaluates the mass matrix per SLOT against every
            // slot's commanded acceleration, and a belt pair's slots always
            // carry identical kinematics: torque = m_diag * acc_self +
            // (m_off/2 + m_off/2) * acc_other_belt. Splitting m_off across
            // the pair columns keeps the matrix symmetric, and it stays
            // positive definite exactly when |m_off| < m_diag — the same
            // physicality condition as the single-drive CoreXY profile.
            Structure::CoreXYAwd => {
                let (md, mo) = (theta[0], theta[1]);
                let half = mo / 2.0;
                let mass = (0..4)
                    .map(|i| {
                        (0..4)
                            .map(|j| {
                                if i == j {
                                    md
                                } else if (i ^ 2 == j) || (i ^ 3 == j) {
                                    half
                                } else {
                                    0.0
                                }
                            })
                            .collect()
                    })
                    .collect();
                let per_drive = |off: usize| (0..4).map(|m| theta[2 + 3 * m + off]).collect();
                PhysicalParams {
                    mass,
                    viscous: per_drive(0),
                    coulomb_fwd: per_drive(1),
                    coulomb_rev: per_drive(2),
                }
            }
        }
    }
}
