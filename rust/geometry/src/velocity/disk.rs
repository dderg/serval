use super::law::{LawSegment, ScalarLaw};

pub(super) const VELOCITY_FLOOR: f64 = 1e-9;
const KAPPA_EPS: f64 = 1e-9;

pub(super) struct Kinematics {
    pub length: f64,
    pub accel: f64,
    pub kappa0: f64,
    pub sigma: f64,
    pub flat_ceiling: f64,
}

impl Kinematics {
    pub(super) fn reversed(&self) -> Kinematics {
        Kinematics {
            length: self.length,
            accel: self.accel,
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

/// A member's disk reach and, when the member is curved, the rail it was read
/// off: the member profile reuses that rail instead of integrating it again.
pub(super) struct DiskReach {
    pub v: f64,
    pub rail: Option<LawSegment>,
}

/// Forward disk reach through the member: the exact rail law integrated over
/// the arc, capped by the feed ceiling. One integrator serves the seam plan
/// and the member profiles, so their reaches agree to the float.
pub(super) fn disk_reach_v(kin: &Kinematics, v_in: f64, s: f64, _tol: f64) -> Option<DiskReach> {
    let v_in = v_in.min(kin.flat_ceiling);
    if kin.is_straight() {
        let w = v_in * v_in + 2.0 * kin.accel * s;
        return Some(DiskReach {
            v: w.max(0.0).sqrt().min(kin.flat_ceiling),
            rail: None,
        });
    }
    let law = ScalarLaw::DiskRail {
        accel: kin.accel,
        kappa0: kin.kappa0,
        sigma: kin.sigma,
        brake: false,
    };
    let end_cap = limit_speed(kin.kappa_abs(s), kin.accel);
    let rail = LawSegment::until_arc(0.0, 0.0, v_in, law, s)?;
    Some(DiskReach {
        v: rail.rail_end_speed().min(kin.flat_ceiling).min(end_cap),
        rail: Some(rail),
    })
}

pub(super) fn disk_reach_v_rev(kin: &Kinematics, v_in: f64, s: f64, tol: f64) -> Option<DiskReach> {
    disk_reach_v(&kin.reversed(), v_in, s, tol)
}

/// A member, with the seam plan's forward rail from its entry and reversed
/// rail from its exit for the profile to reuse when it integrates from the
/// same speeds.
pub(super) struct RunMember<'a> {
    pub kin: &'a Kinematics,
    pub forward_rail: Option<&'a LawSegment>,
    pub backward_rail: Option<&'a LawSegment>,
}

#[cfg(test)]
mod tests;
