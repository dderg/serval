//! The extruder excesses known engine bugs leave on a profile, pinned per
//! class by count and worst value. A case fails on any excess outside a
//! pinned class and whenever a pinned class grows, worsens or shrinks, so
//! the list tightens as fixes land; the failure prints the numbers to pin.
//!
//! TODO(ratio-ramp-end-accel): a ratio ramp's ends ask the extruder for
//! more than `[axis e] max_accel`. `Crossing::across` in
//! motion-pipeline's `follower_projection/ratio_ramp.rs` reads the leaders
//! only from the seam on, never over the half of the ramp before it, and
//! where the leaders brake hard the width envelope lays the ramp narrower
//! than the ramp time it derived.
//!
//! TODO(leader-knot-velocity-step): a kernel-free advance turns the
//! acceleration steps a shaped leader's fit takes at its knots (up to twice
//! `max_accel_deviation`) into extruder velocity steps of `k·r` times
//! them, which `motion_pipeline::follower_velocity_step_allowance` grants
//! no allowance.

use std::path::Path;

use super::extruder::{ExtruderAudit, ExtruderExcess, ToolheadPhase};

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum ExcessClass {
    /// An acceleration excess while the toolhead moves.
    RampEndAccel,
    /// A velocity step beyond its allowance.
    LeaderKnotVelocityStep,
}

impl ExcessClass {
    fn of(excess: &ExtruderExcess) -> Option<(Self, f64)> {
        match *excess {
            ExtruderExcess::Acceleration {
                value,
                phase: ToolheadPhase::Moving,
                ..
            } => Some((Self::RampEndAccel, value)),
            ExtruderExcess::VelocityStep { step, .. } => Some((Self::LeaderKnotVelocityStep, step)),
            ExtruderExcess::Acceleration { .. } | ExtruderExcess::Velocity { .. } => None,
        }
    }
}

/// How many excesses of `class` the case on `config` leaves, and the worst
/// of them: mm/s² for an acceleration, mm/s for a velocity step, as
/// printed to one decimal and three decimals.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct KnownExcesses {
    pub config: &'static str,
    pub class: ExcessClass,
    pub count: usize,
    pub worst: f64,
}

pub const KNOWN: &[KnownExcesses] = &[KnownExcesses {
    config: "tools/bench/configs/trident_ramp.cfg",
    class: ExcessClass::RampEndAccel,
    count: 6,
    worst: 5889.2,
}];

fn printed(class: ExcessClass, worst: f64) -> f64 {
    let scale = match class {
        ExcessClass::RampEndAccel => 10.0,
        ExcessClass::LeaderKnotVelocityStep => 1000.0,
    };
    (worst * scale).round() / scale
}

/// Where the audit of the case on `config` departs from what [`KNOWN`]
/// pins for it, as one failure line.
pub fn extruder_excess(config: &Path, audit: &ExtruderAudit) -> Option<String> {
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
    let pinned: Vec<&KnownExcesses> = KNOWN
        .iter()
        .filter(|known| config.ends_with(known.config))
        .collect();
    let mut departures: Vec<String> = Vec::new();
    for &(class, count, worst) in &observed {
        let matches = pinned.iter().any(|known| {
            known.class == class && known.count == count && known.worst == printed(class, worst)
        });
        if !matches {
            departures.push(format!(
                "pin {class:?}: count {count}, worst {}",
                printed(class, worst)
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
