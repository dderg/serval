//! The live/snapshot parity gate's Rust half: what a pipeline was fed and
//! what it shaped, recorded by the detached live planner and by the snapshot
//! path, compared item by item from Python.

use std::hash::{DefaultHasher, Hash, Hasher};

use motion_pipeline::InputTrace;
use pyo3::exceptions::{PyIndexError, PyValueError};
use pyo3::prelude::*;
use trajectory::ContinuousSegment;

use crate::bridge::PyMotionEngine;
use crate::viz::{PyWaypoint, waypoints_from_py};

#[pyclass]
pub(crate) struct ParityRecord {
    inputs: Vec<InputTrace>,
    segments: Vec<ContinuousSegment>,
}

/// The inputs that shape the trajectory: every move, dwell and reset, and a
/// stop at rest only where moves precede it. A barrier only sequences
/// dispatch, a drain of an already drained pipeline is no stop, and a rest
/// after the last move leaves the trajectory as it is.
fn shaping(inputs: Vec<InputTrace>) -> Vec<InputTrace> {
    let mut shaped = Vec::with_capacity(inputs.len());
    let mut undrained = false;
    for input in inputs {
        match input {
            InputTrace::Control(_) => {}
            InputTrace::Drain if !undrained => {}
            InputTrace::Drain => {
                undrained = false;
                shaped.push(input);
            }
            InputTrace::Move(_) => {
                undrained = true;
                shaped.push(input);
            }
            InputTrace::Dwell { .. } | InputTrace::Reset { .. } => shaped.push(input),
        }
    }
    while matches!(
        shaped.last(),
        Some(InputTrace::Drain | InputTrace::Dwell { .. })
    ) {
        shaped.pop();
    }
    shaped
}

fn digest(item: &impl std::fmt::Debug) -> u64 {
    let mut hasher = DefaultHasher::new();
    format!("{item:?}").hash(&mut hasher);
    hasher.finish()
}

fn debug_at(items: &[impl std::fmt::Debug], index: usize) -> PyResult<String> {
    items
        .get(index)
        .map(|item| format!("{item:#?}"))
        .ok_or_else(|| PyIndexError::new_err(format!("no item {index} of {}", items.len())))
}

#[pymethods]
impl ParityRecord {
    fn input_digests(&self) -> Vec<u64> {
        self.inputs.iter().map(digest).collect()
    }

    fn segment_digests(&self) -> Vec<u64> {
        self.segments.iter().map(digest).collect()
    }

    fn input_debug(&self, index: usize) -> PyResult<String> {
        debug_at(&self.inputs, index)
    }

    fn segment_debug(&self, index: usize) -> PyResult<String> {
        debug_at(&self.segments, index)
    }
}

#[pymethods]
impl PyMotionEngine {
    /// Everything the detached planner was fed and shaped so far.
    fn detached_record(&self) -> PyResult<ParityRecord> {
        let (inputs, segments) = self.take_detached_record()?;
        Ok(ParityRecord {
            inputs: shaping(inputs),
            segments,
        })
    }

    /// `None` while fence `id` is pending, then the stream time it resolved
    /// to.
    fn detached_fence_poll(&self, id: u64) -> PyResult<Option<f64>> {
        self.detached_fence_end(id)
    }
}

/// What the snapshot path feeds its pipeline for `waypoints` under
/// `config_text` and what it shapes.
#[pyfunction]
pub(crate) fn snapshot_parity_record(
    waypoints: Vec<PyWaypoint>,
    config_text: &str,
) -> PyResult<ParityRecord> {
    let planner = planner_config::from_doc::planner_config_from_text(config_text)
        .map_err(PyValueError::new_err)?;
    let record = snapshot_core::stream_record(&waypoints_from_py(waypoints), &planner)
        .map_err(|e| PyValueError::new_err(e.to_string()))?;
    Ok(ParityRecord {
        inputs: shaping(record.inputs),
        segments: record.segments,
    })
}
