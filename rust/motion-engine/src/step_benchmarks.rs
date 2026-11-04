//! The mcus' step benchmarks as klippy reports them, resolved against a
//! config document into the step capacity input both the live planner and
//! every snapshot derive their cap from.

use motion_core::kinematics::{KinematicsKind, KinematicsModule};
use motion_pipeline::live_config::step_capacity::{
    DrivenAxis, McuStepBenchmark, McuStepInput, STEP_CAPACITY_FRACTION, StepBenchmark,
    StepCapacityInput, StepMode, SteppedMotor,
};
use planner_config::from_doc::{KinematicsDecl, read_motion_settings};
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;

pub(crate) const MODE_PULSED: &str = "pulsed";
pub(crate) const MODE_BOTH_EDGE: &str = "both-edge";

/// One mcu's step benchmark as klippy records it (`mcu.get_status()`'s
/// `step_capacity`, a bench run's `run.json`): what one step costs it, its
/// clock, the share of that clock the planner gave its steps, and the motors
/// it steps by name, step distance and mode.
#[derive(Debug, Clone, PartialEq, FromPyObject)]
#[pyo3(from_item_all)]
pub(crate) struct PyStepBenchmark {
    pub(crate) mcu: String,
    pub(crate) clock_freq: f64,
    pub(crate) capacity_fraction: f64,
    pub(crate) pulsed_ticks_per_step: u32,
    pub(crate) both_edge_ticks_per_step: u32,
    pub(crate) simulated: bool,
    pub(crate) motors: Vec<PySteppedMotor>,
}

#[derive(Debug, Clone, PartialEq, FromPyObject)]
#[pyo3(from_item_all)]
pub(crate) struct PySteppedMotor {
    pub(crate) name: String,
    pub(crate) step_distance: f64,
    pub(crate) mode: String,
}

fn step_mode(mcu: &str, motor: &PySteppedMotor) -> Result<StepMode, String> {
    match motor.mode.as_str() {
        MODE_PULSED => Ok(StepMode::Pulsed),
        MODE_BOTH_EDGE => Ok(StepMode::BothEdge),
        other => Err(format!(
            "mcu '{mcu}' steps motor '{}' in mode '{other}', neither '{MODE_PULSED}' nor \
             '{MODE_BOTH_EDGE}'",
            motor.name
        )),
    }
}

fn drives(kinematics: &KinematicsDecl, mcu: &str, motor: &str) -> Result<DrivenAxis, String> {
    let names = |motors: &[String]| motors.iter().any(|name| name == motor);
    if let Some(lane) = kinematics.lanes.iter().find(|lane| names(&lane.motors)) {
        return Ok(DrivenAxis::Lane(lane.lane_idx));
    }
    if let Some(follower) = kinematics
        .followers
        .iter()
        .find(|follower| names(&follower.motors))
    {
        return Ok(DrivenAxis::Follower(follower.axis.clone()));
    }
    Err(format!(
        "mcu '{mcu}' steps motor '{motor}', which no [kinematics] role or [axis] motors: list \
         drives"
    ))
}

impl PyStepBenchmark {
    fn benchmark(&self) -> McuStepBenchmark {
        McuStepBenchmark {
            mcu: self.mcu.clone(),
            clock_freq_hz: self.clock_freq,
            capacity_fraction: self.capacity_fraction,
            benchmark: (!self.simulated).then_some(StepBenchmark {
                pulsed_ticks_per_step: self.pulsed_ticks_per_step,
                both_edge_ticks_per_step: self.both_edge_ticks_per_step,
            }),
        }
    }

    fn modes(&self) -> Result<Vec<StepMode>, String> {
        self.motors
            .iter()
            .map(|motor| step_mode(&self.mcu, motor))
            .collect()
    }

    fn resolve(&self, kinematics: &KinematicsDecl) -> Result<McuStepInput, String> {
        let motors = self
            .motors
            .iter()
            .map(|motor| {
                Ok(SteppedMotor {
                    drives: drives(kinematics, &self.mcu, &motor.name)?,
                    step_distance_mm: motor.step_distance,
                    mode: step_mode(&self.mcu, motor)?,
                })
            })
            .collect::<Result<_, String>>()?;
        Ok(McuStepInput {
            benchmark: self.benchmark(),
            motors,
        })
    }
}

fn resolve(
    kinematics: &KinematicsDecl,
    benchmarks: &[PyStepBenchmark],
) -> Result<StepCapacityInput, String> {
    let kind = KinematicsKind::from_config_type(&kinematics.kind).ok_or_else(|| {
        format!(
            "[kinematics] type '{}' has no step capacity model",
            kinematics.kind
        )
    })?;
    let module = KinematicsModule::from_tag(kind as u8).map_err(|e| e.to_string())?;
    Ok(StepCapacityInput {
        lane_weights: std::array::from_fn(|lane| module.lane_weights(lane)),
        mcus: benchmarks
            .iter()
            .map(|benchmark| benchmark.resolve(kinematics))
            .collect::<Result<_, _>>()?,
    })
}

/// The step capacity input of the printer `config_text` describes, whose
/// mcus reported `benchmarks`.
pub(crate) fn step_capacity_input(
    config_text: &str,
    benchmarks: &[PyStepBenchmark],
) -> Result<StepCapacityInput, String> {
    let doc = config_doc::Document::parse(config_text, "<config>").map_err(|e| e.to_string())?;
    let (settings, _consumed) = read_motion_settings(&doc)?;
    let kinematics = settings
        .kinematics
        .ok_or("step benchmarks need the config's [kinematics] section")?;
    resolve(&kinematics, benchmarks)
}

#[cfg(feature = "snapshot")]
pub(crate) fn optional_step_capacity_input(
    config_text: &str,
    benchmarks: Option<&[PyStepBenchmark]>,
) -> PyResult<Option<StepCapacityInput>> {
    benchmarks
        .map(|benchmarks| step_capacity_input(config_text, benchmarks))
        .transpose()
        .map_err(PyValueError::new_err)
}

/// The clock ticks per second the planner lets `benchmark`'s mcu spend on
/// steps and what one step of each of its motors costs, in `motors` order;
/// `None` for a simulated mcu or one stepping nothing, which nothing caps.
#[pyfunction]
pub(crate) fn step_capacity_budget(
    benchmark: PyStepBenchmark,
) -> PyResult<Option<(f64, Vec<u32>)>> {
    let modes = benchmark.modes().map_err(PyValueError::new_err)?;
    Ok(benchmark
        .benchmark()
        .budget(&modes)
        .map_err(PyValueError::new_err)?
        .map(|budget| (budget.ticks_per_s, budget.ticks_per_step)))
}

/// The share of each mcu's clock the live planner gives step dispatch.
#[pyfunction]
pub(crate) fn step_capacity_fraction() -> f64 {
    STEP_CAPACITY_FRACTION
}

#[cfg(test)]
#[path = "step_benchmarks_tests.rs"]
mod tests;
