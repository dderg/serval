use temporal::BindingConstraint;

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BindingLabel {
    pub limit: String,
    pub derivative: &'static str,
    pub via_pa: bool,
}

pub fn label_binding(c: BindingConstraint, names: &[String]) -> Option<BindingLabel> {
    let (set, derivative, via_pa) = match c {
        BindingConstraint::Velocity { set } => (set, "velocity", false),
        BindingConstraint::AccelNorm { set } => (set, "accel", false),
        BindingConstraint::JerkNorm { set } => (set, "jerk", false),
        BindingConstraint::PaVelocity { set } => (set, "velocity", true),
        BindingConstraint::PaAccel { set } => (set, "accel", true),
        BindingConstraint::PaJerk { set } => (set, "jerk", true),
        BindingConstraint::None | BindingConstraint::Boundary => return None,
        _ => {
            debug_assert!(false, "unhandled BindingConstraint in label_binding");
            return None;
        }
    };
    let limit = names
        .get(set)
        .cloned()
        .unwrap_or_else(|| "runtime_caps".to_string());
    Some(BindingLabel {
        limit,
        derivative,
        via_pa,
    })
}

#[cfg(test)]
mod tests;
