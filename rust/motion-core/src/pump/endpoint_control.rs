use host_rt::mcu_call::McuCall;
use host_rt::mcu_serial_conn::McuSerialConn;
use mcu_protocol::MessageKind;
use mcu_protocol::codec::{Decode, Encode};
use mcu_protocol::messages::{
    SampleGridResponse, SetDynamicsModel, SetDynamicsModelResponse, SetFfLead, SetFfLeadResponse,
};
use std::sync::{Arc, Mutex, mpsc::SyncSender};

use crate::axis_transport::{TRANSPORT_PHASE, TRANSPORT_PULSE, transport_name};
use crate::lock_ext::LockExt;
use crate::mcu_config::{McuAxisConfig, sample_seed_counts, stepcompress_seed_counts};
use crate::types::AxisKey;

use super::{BuzzLane, BuzzRoute, SampleEndpoint, StepcompressEndpoint, WireSink};

pub enum EndpointCommand {
    FreezeMotor {
        mcu_id: u32,
        motor: usize,
        count: i64,
    },
    ReseedMotor {
        mcu_id: u32,
        motor: usize,
        count: i64,
    },
    SeedPosition {
        configs: Vec<McuAxisConfig>,
        position: geometry::MachinePos,
    },
    Handover {
        key: AxisKey,
        from: u8,
        to: u8,
        position: SyncSender<Option<i64>>,
    },
    BuzzRoutes {
        specs: Vec<EndpointBuzzSpec>,
        routes: SyncSender<Vec<BuzzRoute>>,
    },
    SetFfLead {
        mcu_id: u32,
        lead: SetFfLead,
    },
    SetDynamicsModel {
        mcu_id: u32,
        models: Box<(SetDynamicsModel, ethercat_setpoint::dynamics::DynamicsModel)>,
    },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EndpointBuzzSpec {
    Ethercat {
        mcu_handle: u32,
        slot_mask: u8,
        sign_mask: u8,
    },
    Stepper {
        axis_mask: u8,
        sign_mask: u8,
    },
}

enum Side {
    Pulse(Arc<Mutex<StepcompressEndpoint>>),
    Phase(Arc<Mutex<SampleEndpoint>>),
}

impl Side {
    fn quiescent(&self) -> Result<bool, String> {
        match self {
            Self::Pulse(e) => Ok(e.lock_ok().transport_quiescent()),
            Self::Phase(e) => e.lock_ok().transport_quiescent().map_err(|e| e.to_string()),
        }
    }

    fn executed_position(&self, axis: u8) -> Result<i64, String> {
        match self {
            Self::Pulse(e) => e.lock_ok().executed_position(axis),
            Self::Phase(e) => e.lock_ok().executed_position(axis),
        }
        .map_err(|e| e.to_string())
    }

    fn adopt_position(&self, axis: u8, position: i64) -> Result<(), String> {
        match self {
            Self::Pulse(e) => e.lock_ok().reset_axis_position(axis, position),
            Self::Phase(e) => e.lock_ok().reset_axis_position(axis, position),
        }
        .map_err(|e| e.to_string())
    }
}

pub fn buzz_axis_bits(axis_mask: u8, keep: impl Fn(u8) -> bool) -> u8 {
    (0u8..8)
        .filter(|&axis| axis_mask & (1 << axis) != 0 && keep(axis))
        .fold(0u8, |bits, axis| bits | (1 << axis))
}

pub fn buzz_lanes(axis_bits: u8, sign_mask: u8) -> Vec<BuzzLane> {
    (0u8..8)
        .filter(|&axis| axis_bits & (1 << axis) != 0)
        .map(|axis| BuzzLane {
            axis,
            sign: if sign_mask & (1 << axis) != 0 {
                -1.0
            } else {
                1.0
            },
        })
        .collect()
}

impl WireSink {
    fn transport_side(&self, key: AxisKey, mode: u8) -> Result<Side, String> {
        let missing = || {
            format!(
                "switch_axis_transport: mcu {} axis {} has no {} endpoint",
                key.mcu_id,
                key.axis,
                transport_name(mode)
            )
        };
        match mode {
            TRANSPORT_PULSE => self
                .stepcompress
                .get(&key.mcu_id)
                .cloned()
                .map(Side::Pulse)
                .ok_or_else(missing),
            TRANSPORT_PHASE => self
                .samples
                .get(&key.mcu_id)
                .cloned()
                .map(Side::Phase)
                .ok_or_else(missing),
            _ => Err(format!("switch_axis_transport: unknown transport {mode}")),
        }
    }

    pub(super) fn handle_endpoint_command(&self, command: EndpointCommand) -> Result<(), String> {
        match command {
            EndpointCommand::FreezeMotor {
                mcu_id,
                motor,
                count,
            } => {
                self.stepcompress
                    .get(&mcu_id)
                    .ok_or_else(|| format!("keyed trip: no pulse endpoint for mcu {mcu_id}"))?
                    .lock_ok()
                    .freeze_motor(motor, count)
                    .map_err(|e| e.to_string())?;
            }
            EndpointCommand::ReseedMotor {
                mcu_id,
                motor,
                count,
            } => {
                let mut endpoint = self
                    .stepcompress
                    .get(&mcu_id)
                    .ok_or_else(|| {
                        format!("stepcompress reconcile: no pulse endpoint for mcu {mcu_id}")
                    })?
                    .lock_ok();
                endpoint.abort_outbound();
                endpoint.reset_motor_position(motor, count)?;
            }
            EndpointCommand::SeedPosition { configs, position } => {
                for cfg in configs.iter().filter(|cfg| !cfg.ethercat) {
                    if cfg.has_pulse_lanes() {
                        let counts = stepcompress_seed_counts(cfg, position)?;
                        self.stepcompress
                            .get(&cfg.mcu_id)
                            .ok_or_else(|| {
                                format!("position seed: no pulse endpoint for mcu {}", cfg.mcu_id)
                            })?
                            .lock_ok()
                            .reset_position(&counts)
                            .map_err(|e| e.to_string())?;
                    }
                    if cfg.has_phase_lanes() {
                        let counts = sample_seed_counts(cfg, position)?;
                        self.samples
                            .get(&cfg.mcu_id)
                            .ok_or_else(|| {
                                format!("position seed: no phase endpoint for mcu {}", cfg.mcu_id)
                            })?
                            .lock_ok()
                            .reset_position(&counts)
                            .map_err(|e| e.to_string())?;
                    }
                }
            }
            EndpointCommand::Handover {
                key,
                from,
                to,
                position,
            } => {
                if !self.transports.supports(key, to) {
                    return Err(format!(
                        "switch_axis_transport: {key:?} has no {} binding",
                        transport_name(to)
                    ));
                }
                if self.transports.mode(key) != from {
                    return Err(format!(
                        "switch_axis_transport: {key:?} outgoing transport changed during handover"
                    ));
                }
                let outgoing = self.transport_side(key, from)?;
                let incoming = self.transport_side(key, to)?;
                let result = if outgoing.quiescent()? {
                    let executed = outgoing.executed_position(key.axis)?;
                    self.transports.adopt(key, to)?;
                    incoming.adopt_position(key.axis, executed)?;
                    Some(executed)
                } else {
                    None
                };
                position
                    .try_send(result)
                    .map_err(|e| format!("handover result delivery failed: {e}"))?;
            }
            EndpointCommand::BuzzRoutes { specs, routes } => {
                let resolved = self.resolve_buzz_routes(&specs)?;
                routes
                    .try_send(resolved)
                    .map_err(|e| format!("buzz routes delivery failed: {e}"))?;
            }
            EndpointCommand::SetFfLead { mcu_id, lead } => {
                let ring = self
                    .ethercat
                    .get(&mcu_id)
                    .ok_or_else(|| format!("set_ff_lead: mcu {mcu_id} has no EtherCAT filler"))?;
                let conn = ring
                    .conn
                    .upgrade()
                    .ok_or_else(|| format!("set_ff_lead: mcu {mcu_id} connection dropped"))?;
                let mut filler = ring.ring.lock_ok();
                refresh_quiescent_grid(&conn, &mut filler, "set_ff_lead")?;
                let response: SetFfLeadResponse = endpoint_call(
                    &conn,
                    MessageKind::SetFfLead,
                    MessageKind::SetFfLeadResponse,
                    lead.encoded_to_vec(),
                )?;
                require_endpoint_ok(response.result, "set_ff_lead")?;
                require_filler_ok(
                    filler.set_ff_lead(lead.slot as usize, lead.lead_ns),
                    "set_ff_lead",
                )?;
            }
            EndpointCommand::SetDynamicsModel { mcu_id, models } => {
                let (model, host_model) = *models;
                let ring = self.ethercat.get(&mcu_id).ok_or_else(|| {
                    format!("set_dynamics_model: mcu {mcu_id} has no EtherCAT filler")
                })?;
                let conn = ring.conn.upgrade().ok_or_else(|| {
                    format!("set_dynamics_model: mcu {mcu_id} connection dropped")
                })?;
                let mut filler = ring.ring.lock_ok();
                refresh_quiescent_grid(&conn, &mut filler, "set_dynamics_model")?;
                if host_model.n_slots != filler.lane_count() {
                    return Err(format!(
                        "set_dynamics_model: the model covers {} slots but the endpoint's filler drives {} lanes",
                        host_model.n_slots,
                        filler.lane_count()
                    ));
                }
                let response: SetDynamicsModelResponse = endpoint_call(
                    &conn,
                    MessageKind::SetDynamicsModel,
                    MessageKind::SetDynamicsModelResponse,
                    model.encoded_to_vec(),
                )?;
                require_endpoint_ok(response.result, "set_dynamics_model")?;
                require_filler_ok(filler.install_dynamics(host_model), "set_dynamics_model")?;
            }
        }
        Ok(())
    }

    fn resolve_buzz_routes(&self, specs: &[EndpointBuzzSpec]) -> Result<Vec<BuzzRoute>, String> {
        let mut routes = Vec::new();
        for spec in specs {
            match *spec {
                EndpointBuzzSpec::Ethercat {
                    mcu_handle,
                    slot_mask,
                    sign_mask,
                } => {
                    if slot_mask == 0 {
                        return Err("resonance_buzz: ethercat route has an empty slot mask".into());
                    }
                    let filler = self.ethercat.get(&mcu_handle)
                        .ok_or_else(|| format!("resonance_buzz: mcu_handle {mcu_handle} has no EtherCAT setpoint filler"))?;
                    routes.push(BuzzRoute::Ethercat {
                        mcu_id: mcu_handle,
                        filler: Arc::clone(&filler.ring),
                        slot_mask,
                        sign_mask,
                    });
                }
                EndpointBuzzSpec::Stepper {
                    axis_mask,
                    sign_mask,
                } => {
                    if axis_mask == 0 {
                        return Err("resonance_buzz: stepper route has an empty axis mask".into());
                    }
                    let selected = routes.len();
                    let mut pulse: Vec<_> = self.stepcompress.iter().collect();
                    pulse.sort_by_key(|(mcu_id, _)| **mcu_id);
                    for (&mcu_id, endpoint) in pulse {
                        let bits = {
                            let ep = endpoint.lock_ok();
                            buzz_axis_bits(axis_mask, |axis| {
                                ep.drives_axis(axis)
                                    && !self.transports.is_phase(AxisKey { mcu_id, axis })
                            })
                        };
                        if bits != 0 {
                            routes.push(BuzzRoute::Pulse {
                                mcu_id,
                                endpoint: Arc::clone(endpoint),
                                axis_mask: bits,
                                sign_mask,
                            });
                        }
                    }
                    let mut phase: Vec<_> = self.samples.iter().collect();
                    phase.sort_by_key(|(mcu_id, _)| **mcu_id);
                    for (&mcu_id, endpoint) in phase {
                        let bits = {
                            let ep = endpoint.lock_ok();
                            buzz_axis_bits(axis_mask, |axis| {
                                ep.drives_axis(axis)
                                    && self.transports.is_phase(AxisKey { mcu_id, axis })
                            })
                        };
                        if bits != 0 {
                            routes.push(BuzzRoute::Phase {
                                mcu_id,
                                endpoint: Arc::clone(endpoint),
                                lanes: buzz_lanes(bits, sign_mask),
                            });
                        }
                    }
                    if routes.len() == selected {
                        return Err(format!(
                            "resonance_buzz: axis mask 0x{axis_mask:02x} selects no pulse or phase endpoint"
                        ));
                    }
                }
            }
        }
        Ok(routes)
    }
}

fn endpoint_call<T: Decode>(
    conn: &McuSerialConn,
    request: MessageKind,
    expected: MessageKind,
    body: Vec<u8>,
) -> Result<T, String> {
    let (kind, body) = conn
        .mcu_call(request, body, std::time::Duration::from_secs(5))
        .map_err(|e| format!("{request:?}: endpoint call failed: {e:?}"))?;
    if kind != expected {
        return Err(format!("{request:?}: expected {expected:?}, got {kind:?}"));
    }
    T::decode(&body).map_err(|e| format!("{expected:?}: decode failed: {e:?}"))
}

fn refresh_quiescent_grid(
    conn: &McuSerialConn,
    filler: &mut ethercat_setpoint_fill::setpoint_fill::ChainFiller,
    what: &str,
) -> Result<(), String> {
    let grid: SampleGridResponse = endpoint_call(
        conn,
        MessageKind::QuerySampleGrid,
        MessageKind::SampleGridResponse,
        Vec::new(),
    )
    .map_err(|e| format!("{what}: the endpoint's sample grid is unreadable: {e}"))?;
    if grid.executor != ethercat_setpoint::setpoint::EXECUTOR_SETPOINT_RING {
        return Err(format!(
            "{what}: endpoint reports unsupported executor {}",
            grid.executor
        ));
    }
    if grid.ring_depth_cycles == 0 {
        return Err(format!(
            "{what}: endpoint reports a setpoint ring of zero cycles"
        ));
    }
    filler
        .observe_grid(grid.grid_index, grid.grid_clock)
        .map_err(|e| format!("{what}: the endpoint's sample grid was refused: {e:?}"))?;
    if !filler.quiescent() {
        return Err(format!(
            "{what}: the endpoint still has setpoints outstanding — changing the feedforward \
             mid-stream would step the velocity and torque feedforward; wait for the motion to finish"
        ));
    }
    Ok(())
}

fn require_endpoint_ok(result: i32, what: &str) -> Result<(), String> {
    if result != 0 {
        return Err(format!("{what}: endpoint result {result}"));
    }
    Ok(())
}

fn require_filler_ok(result: i32, what: &str) -> Result<(), String> {
    if result != 0 {
        return Err(format!("{what}: host filler refused it (result {result})"));
    }
    Ok(())
}
