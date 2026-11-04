//! How much step dispatch each mcu can take, from the cost one step measured
//! at connect, and what one mm/s of each motor it steps costs it. The live
//! planner and every offline plan derive their cap here.

use geometry::step_capacity::SPATIAL_LANES;
use geometry::{ExtruderCaps, FollowerStepLoad, McuStepLoad, StepCapacity};
use planner_config::PlannerConfig;

/// The share of each mcu's clock the planner lets step dispatch take. On the
/// F401 (tools/bench/README.md, "MCU step-rate capacity") p99 step lateness
/// holds at 4.1-5.1 us up to 225 k steps/s, rises to 7-8 us between 225 and
/// 275 k and ~10 us beyond, and the MCU shuts down ("Stepper too far in
/// past") at ~300 k: the knee sits at 75 % of the ceiling. The
/// self-benchmark reads 0.93 of that ceiling, so 0.75 of the benchmarked
/// capacity keeps every move below the knee with margin.
pub const STEP_CAPACITY_FRACTION: f64 = 0.75;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StepMode {
    Pulsed,
    BothEdge,
}

/// The clock ticks one step costs an mcu's dispatch, per step mode, as its
/// connect-time benchmark measured them.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StepBenchmark {
    pub pulsed_ticks_per_step: u32,
    pub both_edge_ticks_per_step: u32,
}

impl StepBenchmark {
    #[must_use]
    pub fn ticks_per_step(self, mode: StepMode) -> u32 {
        match mode {
            StepMode::Pulsed => self.pulsed_ticks_per_step,
            StepMode::BothEdge => self.both_edge_ticks_per_step,
        }
    }
}

/// What an mcu reported about its step dispatch. A simulated mcu measures
/// nothing (`benchmark: None`) and is never capped.
#[derive(Debug, Clone, PartialEq)]
pub struct McuStepBenchmark {
    pub mcu: String,
    pub clock_freq_hz: f64,
    pub benchmark: Option<StepBenchmark>,
}

/// The clock ticks per second an mcu's steps may take, and what one step of
/// each of its motors costs.
#[derive(Debug, Clone, PartialEq)]
pub struct McuStepBudget {
    pub ticks_per_s: f64,
    pub ticks_per_step: Vec<u32>,
}

impl McuStepBenchmark {
    /// The budget of an mcu stepping motors in `modes`; `None` where nothing
    /// is capped: a simulated mcu, or one stepping no motor.
    pub fn budget(&self, modes: &[StepMode]) -> Result<Option<McuStepBudget>, String> {
        let Some(benchmark) = self.benchmark else {
            return Ok(None);
        };
        if modes.is_empty() {
            return Ok(None);
        }
        if !(self.clock_freq_hz.is_finite() && self.clock_freq_hz > 0.0) {
            return Err(format!(
                "mcu '{}': CLOCK_FREQ must be a finite positive frequency, got {}",
                self.mcu, self.clock_freq_hz
            ));
        }
        if benchmark.pulsed_ticks_per_step == 0 || benchmark.both_edge_ticks_per_step == 0 {
            return Err(format!(
                "mcu '{}' reported an empty step benchmark: {benchmark:?}",
                self.mcu
            ));
        }
        Ok(Some(McuStepBudget {
            ticks_per_s: STEP_CAPACITY_FRACTION * self.clock_freq_hz,
            ticks_per_step: modes
                .iter()
                .map(|&mode| benchmark.ticks_per_step(mode))
                .collect(),
        }))
    }
}

/// What a stepped motor moves: a kinematic lane, or a follower axis by name.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum DrivenAxis {
    Lane(usize),
    Follower(String),
}

#[derive(Debug, Clone, PartialEq)]
pub struct SteppedMotor {
    pub drives: DrivenAxis,
    pub step_distance_mm: f64,
    pub mode: StepMode,
}

#[derive(Debug, Clone, PartialEq)]
pub struct McuStepInput {
    pub benchmark: McuStepBenchmark,
    pub motors: Vec<SteppedMotor>,
}

/// Every mcu's benchmark and the motors it steps, with the kinematics'
/// lane weights: each lane's motor position per unit of x, y and z.
#[derive(Debug, Clone, PartialEq)]
pub struct StepCapacityInput {
    pub lane_weights: [[f64; 3]; SPATIAL_LANES],
    pub mcus: Vec<McuStepInput>,
}

/// The planner's view of every mcu that measured its step dispatch: a motor
/// stepping `d` mm per step at `c` ticks a step takes `c / (d·C)` of an
/// mcu's `C` ticks per second for each mm/s it moves. `None` where no mcu is
/// capped. A follower a capped mcu steps needs extruder caps: its advance's
/// velocity gain is only known to the planner through them.
pub fn step_capacity(
    cfg: &PlannerConfig,
    input: &StepCapacityInput,
) -> Result<Option<StepCapacity>, String> {
    let extruder_caps = super::extruder_caps(cfg)?;
    let mut loads = Vec::new();
    for mcu in &input.mcus {
        let modes: Vec<StepMode> = mcu.motors.iter().map(|motor| motor.mode).collect();
        if let Some(budget) = mcu.benchmark.budget(&modes)? {
            loads.push(mcu_load(cfg, &extruder_caps, mcu, &budget)?);
        }
    }
    Ok((!loads.is_empty()).then(|| StepCapacity {
        lane_weights: input.lane_weights,
        mcus: loads,
    }))
}

fn mcu_load(
    cfg: &PlannerConfig,
    extruder_caps: &[ExtruderCaps],
    mcu: &McuStepInput,
    budget: &McuStepBudget,
) -> Result<McuStepLoad, String> {
    let name = &mcu.benchmark.mcu;
    let mut lane_s_per_mm = [0.0; SPATIAL_LANES];
    let mut followers: Vec<FollowerStepLoad> = Vec::new();
    for (motor, &ticks) in mcu.motors.iter().zip(&budget.ticks_per_step) {
        if !(motor.step_distance_mm.is_finite() && motor.step_distance_mm > 0.0) {
            return Err(format!(
                "mcu '{name}' steps a motor of step distance {} mm, not a finite positive one",
                motor.step_distance_mm
            ));
        }
        let share = f64::from(ticks) / (motor.step_distance_mm * budget.ticks_per_s);
        match &motor.drives {
            DrivenAxis::Lane(lane) => {
                *lane_s_per_mm.get_mut(*lane).ok_or_else(|| {
                    format!("mcu '{name}' steps kinematic lane {lane}, past the spatial lanes")
                })? += share;
            }
            DrivenAxis::Follower(axis) => {
                let axis_index = cfg
                    .axis_registry
                    .axis_index(axis)
                    .map_err(|e| e.to_string())?;
                if !extruder_caps
                    .iter()
                    .any(|caps| caps.axis_index == axis_index)
                {
                    return Err(format!(
                        "mcu '{name}' steps axis '{axis}', which has no limits the planner can \
                         bound its step rate by: give [axis {axis}] a max_velocity or bind an \
                         [extruder] to it"
                    ));
                }
                match followers
                    .iter_mut()
                    .find(|load| load.axis_index == axis_index)
                {
                    Some(load) => load.s_per_mm += share,
                    None => followers.push(FollowerStepLoad {
                        axis_index,
                        s_per_mm: share,
                    }),
                }
            }
        }
    }
    Ok(McuStepLoad {
        mcu: name.clone(),
        lane_s_per_mm,
        followers,
    })
}

#[cfg(test)]
#[path = "step_capacity_tests.rs"]
mod tests;
