//! JSON writer for `servo-cal fit --response ferr` — the machine-readable
//! counterpart of `profile_out::render_profile`'s TOML for the torque
//! response. There is no profile to render: the ferr fit exists to drive a
//! secant search on the command-path parameters until its coefficients are
//! statistically zero, so the tuning loop needs the coefficients and their
//! standard errors, not a profile.
//!
//! Sign convention (also printed to stderr per mode): a positive
//! `coef.mass[k]` means mode `k`'s following error GROWS with commanded
//! accel — the torque feedforward under-feeds during acceleration, i.e. the
//! command-path mass is too low. Positive `coef.viscous[k]`/`coef.coulomb[k]`
//! read the same way against commanded velocity and its sign. The tuning
//! macro steps each command-path parameter in the direction that drives its
//! coefficient toward zero.

use serde::Serialize;

use crate::fit::FerrFitResult;
use crate::model::Structure;

#[derive(Debug, Serialize)]
struct FerrCoefficients {
    mass: Vec<f64>,
    viscous: Vec<f64>,
    coulomb: Vec<f64>,
}

#[derive(Debug, Serialize)]
struct FerrFitJson {
    version: u32,
    modes: Vec<String>,
    coef: FerrCoefficients,
    stderr: FerrCoefficients,
    ferr_rms: Vec<f64>,
    samples: usize,
}

pub fn render_ferr_json(structure: &Structure, modes: &[&str], r: &FerrFitResult) -> String {
    assert_eq!(
        modes.len(),
        structure.mode_count(),
        "one mode name per structure row"
    );
    let n = modes.len();
    assert_eq!(r.param_stderr.len(), 3 * n, "one stderr triple per mode");
    let stderr = FerrCoefficients {
        mass: (0..n).map(|k| r.param_stderr[3 * k]).collect(),
        viscous: (0..n).map(|k| r.param_stderr[3 * k + 1]).collect(),
        coulomb: (0..n).map(|k| r.param_stderr[3 * k + 2]).collect(),
    };
    let json = FerrFitJson {
        version: 1,
        modes: modes.iter().map(|m| (*m).to_string()).collect(),
        coef: FerrCoefficients {
            mass: r.params.mass.clone(),
            viscous: r.params.viscous.clone(),
            coulomb: r.params.coulomb.clone(),
        },
        stderr,
        ferr_rms: r.ferr_rms.clone(),
        samples: r.samples,
    };
    serde_json::to_string_pretty(&json).expect("ferr fit result serializes to JSON")
}
