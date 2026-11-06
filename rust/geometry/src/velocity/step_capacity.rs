use crate::Move;
use crate::path::Segment;
use crate::step_capacity::{SPATIAL_LANES, StepCapacity, StepCapacityOverload};

use super::extruder_caps;
use super::extrusion_reach::ReachableSpeed;
use super::rate_cap::LinearRate;

const MCU_CAPACITY_SHARE: f64 = 1.0;

/// What one move asks of the motors the mcus step: per kinematic lane, the
/// largest `|w·t̂|` its path reaches, so that lane's motor runs at most
/// that times the path speed; and the motor velocity of the follower it
/// extrudes on, advance included.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct MotorDemand {
    pub(super) lane_leans: [f64; SPATIAL_LANES],
    pub(super) extrusion: Option<(usize, LinearRate)>,
}

impl MotorDemand {
    /// A curved segment's tangent sweeps a range of headings; the lean is
    /// the exact maximum of `|w·t̂|` over that range, so the motor speed it
    /// bounds holds at every point of the segment.
    pub(super) fn of(
        m: &Move,
        segment: Option<&Segment>,
        ratio_in_reach: f64,
        capacity: &StepCapacity,
    ) -> Self {
        let lane_leans = segment.map_or([0.0; SPATIAL_LANES], |segment| {
            capacity
                .lane_weights
                .map(|weights| segment.heading_lean_along(weights))
        });
        let extrusion = match (m.limits.extruder, segment) {
            (Some(caps), Some(_)) => Some((
                caps.axis_index,
                extruder_caps::velocity_rate(&caps, ratio_in_reach),
            )),
            (Some(caps), None) => {
                let ratio = own_ratio_magnitude(m, caps.axis_index);
                Some((
                    caps.axis_index,
                    LinearRate {
                        per_velocity: ratio,
                        per_accel: ratio * caps.advance_slope_s,
                    },
                ))
            }
            (None, _) => {
                require_no_unbounded_extrusion(m, capacity);
                None
            }
        };
        Self {
            lane_leans,
            extrusion,
        }
    }
}

fn own_ratio_magnitude(m: &Move, axis_index: usize) -> f64 {
    m.segment
        .followers
        .iter()
        .filter(|follower| follower.axis_index == axis_index)
        .map(|follower| follower.ratio.abs().max(follower.ratio_end.abs()))
        .fold(0.0, f64::max)
}

fn require_no_unbounded_extrusion(m: &Move, capacity: &StepCapacity) {
    for follower in &m.segment.followers {
        let loaded = capacity
            .mcus
            .iter()
            .any(|mcu| mcu.follower_s_per_mm(follower.axis_index) > 0.0);
        assert!(
            !loaded || (follower.ratio == 0.0 && follower.ratio_end == 0.0),
            "line {}: the move drives follower axis {} on a step-capacity-capped mcu but \
             carries no extruder caps, so its advance cannot be bounded",
            m.source.start_line,
            follower.axis_index,
        );
    }
}

/// The speed ceiling and acceleration budget under which no mcu steps more
/// than its capacity for the move: every mcu's dispatch is linear in the
/// two, so each over capacity cuts them in turn as the extruder's velocity
/// cap does, and the cuts only lower what the mcus before it asked.
pub(super) fn step_capacity_capped(
    capacity: &StepCapacity,
    demand: MotorDemand,
    reachable: &ReachableSpeed,
    limits: (f64, f64),
) -> ((f64, f64), Option<StepCapacityOverload>) {
    let mut capped = limits;
    let mut overload: Option<StepCapacityOverload> = None;
    for (mcu_index, rate) in mcu_rates(capacity, demand).enumerate() {
        let demand_over_capacity = rate.at(limits.0, limits.1);
        if demand_over_capacity <= MCU_CAPACITY_SHARE {
            continue;
        }
        if overload.is_none_or(|worst| demand_over_capacity > worst.demand_over_capacity) {
            overload = Some(StepCapacityOverload {
                mcu_index,
                demand_over_capacity,
            });
        }
        if rate.at(capped.0, capped.1) > MCU_CAPACITY_SHARE {
            capped = rate.capped(MCU_CAPACITY_SHARE, reachable, capped);
        }
    }
    (capped, overload)
}

/// Each mcu's share of its capacity the move takes per unit of speed and
/// of acceleration budget.
pub(super) fn mcu_rates(
    capacity: &StepCapacity,
    demand: MotorDemand,
) -> impl Iterator<Item = LinearRate> + '_ {
    capacity.mcus.iter().map(move |mcu| {
        let spatial: f64 = mcu
            .lane_s_per_mm
            .iter()
            .zip(demand.lane_leans)
            .map(|(share, lean)| share * lean)
            .sum();
        let (extrusion_velocity, extrusion_accel) =
            demand.extrusion.map_or((0.0, 0.0), |(axis_index, rate)| {
                let share = mcu.follower_s_per_mm(axis_index);
                (share * rate.per_velocity, share * rate.per_accel)
            });
        LinearRate {
            per_velocity: spatial + extrusion_velocity,
            per_accel: extrusion_accel,
        }
    })
}

#[cfg(test)]
mod tests;
