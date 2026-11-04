//! The extruder excesses known engine bugs leave on a case, pinned per
//! class by count and worst value. A case fails on any excess outside a
//! pinned class and whenever a pinned class grows, worsens or shrinks, so
//! the list tightens as fixes land; the failure prints the numbers to pin.
//!
//! TODO(ratio-ramp-end-accel, #676): a ratio ramp's ends ask the extruder
//! for more than `[axis e] max_accel`. `Crossing::across` in
//! motion-pipeline's `follower_projection/ratio_ramp.rs` reads the leaders
//! only from the seam on, never over the half of the ramp before it, and
//! where the leaders brake hard the width envelope lays the ramp narrower
//! than the ramp time it derived.
//!
//! TODO(shaped-turning, #685): the planner holds a kernel-free advance's
//! demand along the path, `r·(a_t + k·j_t)`, within `[axis e] max_accel`,
//! but not the shaped path's turning term `k·r·a⊥²/v`, which slow sharp
//! corners raise far past it. Every [`ExcessClass::Turning`] pin is one the
//! shaped-speed predictor #685 designs removes.

use super::extruder::{AccelCause, ExtruderAudit, ExtruderExcess, ToolheadPhase};

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum ExcessClass {
    /// An acceleration excess while the toolhead moves that the path's
    /// turning does not explain.
    RampEndAccel,
    /// An acceleration excess while the toolhead moves that the shaped
    /// path's turning asks of a kernel-free advance, see
    /// [`AccelCause::PathTurning`].
    Turning,
}

impl ExcessClass {
    fn of(excess: &ExtruderExcess) -> Option<(Self, f64)> {
        match *excess {
            ExtruderExcess::Acceleration {
                value,
                phase: ToolheadPhase::Moving,
                cause,
                ..
            } => Some((
                match cause {
                    AccelCause::PathTurning => Self::Turning,
                    AccelCause::Unattributed => Self::RampEndAccel,
                },
                value,
            )),
            ExtruderExcess::Acceleration { .. }
            | ExtruderExcess::Velocity { .. }
            | ExtruderExcess::VelocityStep { .. } => None,
        }
    }
}

/// How many excesses of `class` the case named `case` (`<gcode>/<config>`
/// for the sim prints) leaves, and the worst of them in mm/s², as printed
/// to one decimal.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct KnownExcesses {
    pub case: &'static str,
    pub class: ExcessClass,
    pub count: usize,
    pub worst: f64,
}

pub const KNOWN: &[KnownExcesses] = &[];

fn printed(worst: f64) -> f64 {
    (worst * 10.0).round() / 10.0
}

/// Where the audit of the case named `case` departs from what [`KNOWN`]
/// pins for it, as one failure line.
pub fn extruder_excess(case: &str, audit: &ExtruderAudit) -> Option<String> {
    const SHOWN: usize = 3;
    let mut observed: Vec<(ExcessClass, usize, f64)> = Vec::new();
    let mut unpinned: Vec<String> = Vec::new();
    for excess in &audit.excesses {
        let Some((class, value)) = ExcessClass::of(excess) else {
            unpinned.push(excess.to_string());
            continue;
        };
        match observed.iter_mut().find(|(seen, ..)| *seen == class) {
            Some((_, count, worst)) => {
                *count += 1;
                *worst = worst.max(value);
            }
            None => observed.push((class, 1, value)),
        }
    }
    let pinned: Vec<&KnownExcesses> = KNOWN.iter().filter(|known| known.case == case).collect();
    let mut departures: Vec<String> = Vec::new();
    for &(class, count, worst) in &observed {
        let matches = pinned.iter().any(|known| {
            known.class == class && known.count == count && known.worst == printed(worst)
        });
        if !matches {
            departures.push(format!(
                "pin {class:?}: count {count}, worst {}",
                printed(worst)
            ));
        }
    }
    for known in &pinned {
        if !observed.iter().any(|(class, ..)| *class == known.class) {
            departures.push(format!("unpin {:?}: none left", known.class));
        }
    }
    (!departures.is_empty() || !unpinned.is_empty()).then(|| {
        let shown: Vec<&str> = unpinned.iter().take(SHOWN).map(String::as_str).collect();
        format!(
            "extruder excesses depart from the known ones ({}); {} outside any class: {}",
            departures.join("; "),
            unpinned.len(),
            shown.join("; ")
        )
    })
}
