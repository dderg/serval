/// A curvature (or boundary) speed limit this close to the flat ceiling is
/// the fitter's own blend sizing — it solves the corner radius so the apex
/// speed lands *at* the feedrate, to float tolerance. Taking the raw `min`
/// would notch the cap by ~1e-6 mm/s at every blend, and the jerk-limited
/// pass would dutifully dip into each notch with a nanosecond full-rail
/// bang whose phase joints then ring through the lowering as absurd
/// acceleration slivers. Snap such limits up to the ceiling instead.
pub(super) const CAP_NOTCH_REL: f64 = 1e-6;

pub(super) fn notch_free_min(flat_ceiling: f64, limit: f64) -> f64 {
    if limit >= flat_ceiling * (1.0 - CAP_NOTCH_REL) {
        flat_ceiling
    } else {
        limit
    }
}
pub(super) const VELOCITY_FLOOR: f64 = 1e-9;
const KAPPA_EPS: f64 = 1e-9;

pub(super) struct Kinematics {
    pub length: f64,
    pub accel: f64,
    pub jerk: f64,
    pub kappa0: f64,
    pub sigma: f64,
    pub flat_ceiling: f64,
}

impl Kinematics {
    pub(super) fn reversed(&self) -> Kinematics {
        Kinematics {
            length: self.length,
            accel: self.accel,
            jerk: self.jerk,
            kappa0: self.kappa0 + self.sigma * self.length,
            sigma: -self.sigma,
            flat_ceiling: self.flat_ceiling,
        }
    }

    pub(super) fn kappa_abs(&self, s: f64) -> f64 {
        (self.kappa0 + self.sigma * s).abs()
    }

    pub(super) fn is_straight(&self) -> bool {
        self.kappa0.abs() <= KAPPA_EPS && self.sigma.abs() <= KAPPA_EPS
    }
}

pub(super) fn limit_speed(kappa_abs: f64, accel: f64) -> f64 {
    if kappa_abs > 0.0 {
        (accel / kappa_abs).sqrt()
    } else {
        f64::INFINITY
    }
}

/// Forward disk reach through the member: the exact rail law integrated over
/// the arc, capped by the feed ceiling. One integrator serves the seam plan
/// and the member profiles, so their reaches agree to the float.
pub(super) fn disk_reach_v(kin: &Kinematics, v_in: f64, s: f64, _tol: f64) -> Option<f64> {
    let v_in = v_in.min(kin.flat_ceiling);
    if kin.is_straight() {
        let w = v_in * v_in + 2.0 * kin.accel * s;
        return Some(w.max(0.0).sqrt().min(kin.flat_ceiling));
    }
    let law = super::law::ScalarLaw::DiskRail {
        accel: kin.accel,
        kappa0: kin.kappa0,
        sigma: kin.sigma,
        brake: false,
    };
    let end_cap = limit_speed(kin.kappa_abs(s), kin.accel);
    Some(
        super::law::LawSegment::reach_over(law, v_in, s)?
            .min(kin.flat_ceiling)
            .min(end_cap),
    )
}

pub(super) fn disk_reach_v_rev(kin: &Kinematics, v_in: f64, s: f64, tol: f64) -> Option<f64> {
    disk_reach_v(&kin.reversed(), v_in, s, tol)
}

pub(super) struct RunMember<'a> {
    pub kin: &'a Kinematics,
    pub exit_v: f64,
}

#[cfg(test)]
mod tests;
