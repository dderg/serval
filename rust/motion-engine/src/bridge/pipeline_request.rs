//! Requests the ingress serves behind a drain of everything queued ahead of
//! them. A `*_start` call stages one; `pipeline_request_poll` sends it once
//! the input channel has room and reports when it is served, so the klippy
//! reactor waits on them between polls instead of inside a native call.

use std::sync::{Arc, Mutex};

use crossbeam_channel::{Receiver, TryRecvError};
use motion_core::lock_ext::LockExt;
use motion_core::pump::{BuzzParams, BuzzToken};
use motion_core::worker::{NudgeParams, StreamWorkerError, StreamWorkerHandle};

pub(crate) enum PipelineRequest {
    Dwell(f64),
    HoldStartUntil(f64),
    SwapMesh(MeshSwap),
    Nudge(NudgeParams),
    Buzz(BuzzParams),
}

pub(crate) struct MeshSwap {
    pub(crate) mesh: Option<Arc<geometry::SurfaceTransform>>,
    pub(crate) gcode_z: f64,
}

/// What a served request leaves for the engine to adopt.
pub(crate) enum Served {
    Done,
    Mesh(MeshSwap),
    Buzz(BuzzToken),
}

enum Resolution {
    Served,
    Unit(Receiver<()>),
    Checked(Receiver<Result<(), String>>),
    Arming(Receiver<Result<BuzzToken, String>>),
    Armed(BuzzToken),
}

pub(crate) struct PendingRequest {
    request: PipelineRequest,
    resolution: Option<Resolution>,
}

impl PendingRequest {
    pub(crate) fn unsent(request: PipelineRequest) -> Self {
        Self {
            request,
            resolution: None,
        }
    }

    pub(crate) fn served(request: PipelineRequest) -> Self {
        Self {
            request,
            resolution: Some(Resolution::Served),
        }
    }

    pub(crate) fn into_served(self) -> Served {
        match (self.request, self.resolution) {
            (PipelineRequest::SwapMesh(swap), _) => Served::Mesh(swap),
            (PipelineRequest::Buzz(_), Some(Resolution::Armed(token))) => Served::Buzz(token),
            (PipelineRequest::Buzz(_), _) => unreachable!("a served buzz holds its token"),
            _ => Served::Done,
        }
    }

    pub(crate) fn advance(
        &mut self,
        planner: &Mutex<Option<StreamWorkerHandle>>,
    ) -> Result<bool, String> {
        if self.resolution.is_none() {
            let guard = planner.lock_ok();
            let planner = guard
                .as_ref()
                .ok_or("planner not initialized — call init_planner first")?;
            match self.send(planner) {
                Ok(resolution) => self.resolution = Some(resolution),
                Err(StreamWorkerError::ChannelFull) => return Ok(false),
                Err(error) => return Err(error.to_string()),
            }
        }
        self.resolution
            .as_mut()
            .expect("a sent request has a resolution")
            .poll()
    }

    fn send(&self, planner: &StreamWorkerHandle) -> Result<Resolution, StreamWorkerError> {
        Ok(match &self.request {
            PipelineRequest::Dwell(secs) => Resolution::Unit(planner.dwell_try_start(*secs)?),
            PipelineRequest::HoldStartUntil(host_s) => {
                Resolution::Unit(planner.hold_start_until_try_start(*host_s)?)
            }
            PipelineRequest::SwapMesh(swap) => {
                Resolution::Unit(planner.update_mesh_try_start(swap.mesh.clone(), swap.gcode_z)?)
            }
            PipelineRequest::Nudge(params) => Resolution::Checked(planner.submit_nudge(*params)?),
            PipelineRequest::Buzz(params) => Resolution::Arming(planner.submit_buzz(params.clone())?),
        })
    }
}

impl Resolution {
    fn poll(&mut self) -> Result<bool, String> {
        let received = match self {
            Self::Served | Self::Armed(_) => return Ok(true),
            Self::Unit(done) => done.try_recv().map(Ok),
            Self::Checked(done) => done.try_recv(),
            Self::Arming(verdict) => match verdict.try_recv() {
                Ok(armed) => {
                    *self = Self::Armed(armed?);
                    return Ok(true);
                }
                Err(error) => Err(error),
            },
        };
        match received {
            Ok(result) => result.map(|()| true),
            Err(TryRecvError::Empty) => Ok(false),
            Err(TryRecvError::Disconnected) => {
                Err("the planner dropped the request without serving it".to_string())
            }
        }
    }
}
