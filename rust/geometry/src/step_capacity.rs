pub const SPATIAL_LANES: usize = 3;

/// How much step dispatch each mcu can take, as the share of its capacity
/// one mm/s of each of its motors' travel costs. The planner holds every
/// move to `Σ share·motor speed ≤ 1` on every mcu.
#[derive(Debug, Clone, PartialEq)]
pub struct StepCapacity {
    /// Per kinematic lane, its motor's position per unit of x, y and z.
    pub lane_weights: [[f64; 3]; SPATIAL_LANES],
    pub mcus: Vec<McuStepLoad>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct McuStepLoad {
    pub mcu: String,
    /// Per kinematic lane: the share of this mcu's capacity one mm/s of the
    /// lane's motor travel costs, summed over the lane's motors it steps.
    pub lane_s_per_mm: [f64; SPATIAL_LANES],
    pub followers: Vec<FollowerStepLoad>,
}

/// The share of an mcu's capacity one mm/s of a follower axis's motor
/// travel costs, summed over the axis's motors that mcu steps.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FollowerStepLoad {
    pub axis_index: usize,
    pub s_per_mm: f64,
}

impl McuStepLoad {
    #[must_use]
    pub fn follower_s_per_mm(&self, axis_index: usize) -> f64 {
        self.followers
            .iter()
            .filter(|load| load.axis_index == axis_index)
            .map(|load| load.s_per_mm)
            .sum()
    }
}

/// The step dispatch a move would have asked of its busiest mcu under the
/// limits it had before the step capacity cut them, as a multiple of that
/// mcu's capacity; above 1 wherever the cap bound.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StepCapacityOverload {
    pub mcu_index: usize,
    pub demand_over_capacity: f64,
}
