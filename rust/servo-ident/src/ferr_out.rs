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
use crate::prep::TransientRms;

#[derive(Debug, Serialize)]
struct FerrCoefficients {
    mass: Vec<f64>,
    viscous: Vec<f64>,
    coulomb: Vec<f64>,
}

/// Per-term transient following-error statistics, one array entry per mode.
/// `rms`/`sigma` serialize to `null` where the term had too few windows to
/// score (see `TransientRms`).
#[derive(Debug, Serialize)]
struct FerrTransientTerm {
    rms: Vec<Option<f64>>,
    sigma: Vec<Option<f64>>,
    windows: Vec<usize>,
}

/// Transient-scoped RMS objective: the RAW following error scored only over
/// the short windows each command-path term actually controls. The tuning
/// loop's acceptance test reads these, not the whole-capture rms. `lead`
/// scores the decel-to-stop windows where command-path timing error
/// integrates into a direction-locked overshoot lobe — bench captures show
/// the corner-exit deviation lives there, not in the shared accel onsets.
#[derive(Debug, Serialize)]
struct FerrRmsFf {
    mass: FerrTransientTerm,
    viscous: FerrTransientTerm,
    coulomb: FerrTransientTerm,
    lead: FerrTransientTerm,
}

#[derive(Debug, Serialize)]
struct FerrFitJson {
    version: u32,
    modes: Vec<String>,
    coef: FerrCoefficients,
    stderr: FerrCoefficients,
    /// Per-mode jerk nuisance coefficients (absorbed, never applied);
    /// `-jerk[k] / coef.mass[k]` is the implied command→ferr timing skew
    /// in seconds. Empty when the jerk column was disabled.
    jerk: Vec<f64>,
    jerk_stderr: Vec<f64>,
    /// Per-mode RMS of the in-band residual after subtracting the fitted
    /// model, over the fit's masked samples (mm).
    ferr_rms: Vec<f64>,
    /// Per-mode RMS of the RAW following error over the whole capture —
    /// unfiltered, unmasked (mm). This is the tuning loop's objective:
    /// the number the operator actually experiences as tracking error.
    ferr_rms_raw: Vec<f64>,
    /// Transient-scoped RAW ferr rms per command-path term (the acceptance
    /// objective); see `FerrRmsFf`.
    ferr_rms_ff: FerrRmsFf,
    /// Per-mode mean of `sign(accel)·ferr` over short windows right after
    /// each commanded accel transition (mm), raw channels — the operator's
    /// manual heuristic: only the FIRST excursion when torque is applied
    /// carries clean command-path sign, before the drive's own
    /// compensation reacts. Positive = under-fed (mass too low), negative
    /// = over-fed. The tuner's direction hint for the mass search.
    onset_bias: Vec<f64>,
    /// Accel-transition windows the onset bias was scored over.
    onset_windows: usize,
    samples: usize,
}

pub fn render_ferr_json(
    structure: &Structure,
    modes: &[&str],
    r: &FerrFitResult,
    ferr_rms_raw: &[f64],
    onset_bias: &[f64],
    onset_windows: usize,
    mass_rms: &[TransientRms],
    viscous_rms: &[TransientRms],
    coulomb_rms: &[TransientRms],
    lead_rms: &[TransientRms],
) -> String {
    assert_eq!(
        modes.len(),
        structure.mode_count(),
        "one mode name per structure row"
    );
    let n = modes.len();
    assert_eq!(r.param_stderr.len(), 3 * n, "one stderr triple per mode");
    assert_eq!(ferr_rms_raw.len(), n, "one raw rms per mode");
    assert_eq!(onset_bias.len(), n, "one onset bias per mode");
    let transient_term = |results: &[TransientRms]| -> FerrTransientTerm {
        assert_eq!(results.len(), n, "one transient-rms entry per mode");
        FerrTransientTerm {
            rms: results.iter().map(|t| t.rms).collect(),
            sigma: results.iter().map(|t| t.sigma).collect(),
            windows: results.iter().map(|t| t.windows).collect(),
        }
    };
    let stderr = FerrCoefficients {
        mass: (0..n).map(|k| r.param_stderr[3 * k]).collect(),
        viscous: (0..n).map(|k| r.param_stderr[3 * k + 1]).collect(),
        coulomb: (0..n).map(|k| r.param_stderr[3 * k + 2]).collect(),
    };
    let json = FerrFitJson {
        version: 3,
        modes: modes.iter().map(|m| (*m).to_string()).collect(),
        coef: FerrCoefficients {
            mass: r.params.mass.clone(),
            viscous: r.params.viscous.clone(),
            coulomb: r.params.coulomb.clone(),
        },
        stderr,
        jerk: r.jerk.clone(),
        jerk_stderr: r.jerk_stderr.clone(),
        ferr_rms: r.ferr_rms.clone(),
        ferr_rms_raw: ferr_rms_raw.to_vec(),
        onset_bias: onset_bias.to_vec(),
        ferr_rms_ff: FerrRmsFf {
            mass: transient_term(mass_rms),
            viscous: transient_term(viscous_rms),
            coulomb: transient_term(coulomb_rms),
            lead: transient_term(lead_rms),
        },
        onset_windows,
        samples: r.samples,
    };
    serde_json::to_string_pretty(&json).expect("ferr fit result serializes to JSON")
}
