//! Message structs and codecs. Flat messages (scalar / fixed-array fields
//! only) live in [`generated`], emitted by `build.rs` from `schema_def.rs`;
//! this file keeps the hand-written codecs for the variable-length messages
//! plus the protocol constants that are not part of any one message.

use crate::codec::{
    Cursor, Decode, DecodeError, Encode, get_f32, get_i32, get_str, get_u8, get_u16, get_u32,
    get_u64, put_f32, put_i32, put_str, put_u8, put_u16, put_u32, put_u64,
};

mod generated;
pub use generated::*;

pub const RUNTIME_CAPS_RESPONSE_BODY_LEN: usize = 4;

/// Usable `PushPieces` payload budget — mirrors `MCU_TX_BUF_SIZE` (256) in
/// `src/mcu_transport_dispatch.c` minus the sync/len/channel/CRC framing. A
/// single shared constant compiled into every chip's firmware; keep in sync.
pub const PIECE_FRAME_PAYLOAD_MAX: usize = 250;

/// Bytes of one axis block header (`axis_idx + piece_count + start_slot + new_head`).
pub const AXIS_BLOCK_HEADER_LEN: usize = 8;

/// Bytes of one wire piece-entry header (start_time + duration + motor_mask +
/// coeff_count + reserved) — everything before the coefficient block.
pub const PIECE_WIRE_HEADER_LEN: usize = 16;

/// Largest wire piece entry (header + 8 coefficients) — also the MCU ring-slot
/// size the entry is zero-extended into.
pub const PIECE_WIRE_MAX_LEN: usize = 48;

/// Maximum Chebyshev coefficients per piece (degree ≤ 7).
pub const MAX_PIECE_COEFFS: usize = 8;

/// Largest `piece_count` per axis that still lets an `axis_count`-axis frame fit
/// `PIECE_FRAME_PAYLOAD_MAX`, assuming an even split. Parameterized off the
/// shared budget — never a per-chip number. Returns 0 when even one piece per
/// axis cannot fit (too many axes for the buffer).
pub fn max_pieces_per_axis(axis_count: u8) -> usize {
    let n = axis_count.max(1) as usize;
    let avail = PIECE_FRAME_PAYLOAD_MAX.saturating_sub(1 + n * AXIS_BLOCK_HEADER_LEN);
    (avail / PIECE_WIRE_MAX_LEN) / n
}

/// One axis' pieces within a single-MCU `PushPieces` frame. Byte-identical to
/// the pre-bundling single-axis layout, now repeated under `axis_count`.
#[derive(Debug, Clone, PartialEq)]
pub struct AxisPieces {
    pub axis_idx: u8,
    pub piece_count: u8,
    pub start_slot: u16,
    pub new_head: u32,
    pub pieces_bytes: Vec<u8>,
}

/// All of one MCU's axis blocks delivered in a single transaction. `axis_count`
/// is the leading byte; `axis_count == 1` is the EtherCAT/single-axis case.
#[derive(Debug, Clone, PartialEq)]
pub struct PushPieces {
    pub axes: Vec<AxisPieces>,
}

impl PushPieces {
    /// Build a single-axis frame — the common shape for single-axis MCUs.
    pub fn single(
        axis_idx: u8,
        piece_count: u8,
        start_slot: u16,
        new_head: u32,
        pieces_bytes: Vec<u8>,
    ) -> Self {
        Self {
            axes: vec![AxisPieces {
                axis_idx,
                piece_count,
                start_slot,
                new_head,
                pieces_bytes,
            }],
        }
    }
}

impl Encode for PushPieces {
    fn encode(&self, out: &mut Vec<u8>) {
        put_u8(out, self.axes.len() as u8);
        for a in &self.axes {
            put_u8(out, a.axis_idx);
            put_u8(out, a.piece_count);
            put_u16(out, a.start_slot);
            put_u32(out, a.new_head);
            out.extend_from_slice(&a.pieces_bytes);
        }
    }
}

impl Decode for PushPieces {
    fn decode_from(c: &mut Cursor<'_>) -> Result<Self, DecodeError> {
        let axis_count = get_u8(c)?;
        if axis_count == 0 {
            return Err(DecodeError::EmptyArray {
                field: "PushPieces.axes",
            });
        }
        let mut axes: Vec<AxisPieces> = Vec::with_capacity(axis_count as usize);
        for _ in 0..axis_count {
            let axis_idx = get_u8(c)?;
            let piece_count = get_u8(c)?;
            let start_slot = get_u16(c)?;
            let new_head = get_u32(c)?;
            if axes.iter().any(|a| a.axis_idx == axis_idx) {
                return Err(DecodeError::DuplicateField {
                    field: "PushPieces.axis_idx",
                });
            }
            // Entries are variable-length: 16-byte header + 4·coeff_count
            // coefficient bytes, coeff_count at offset 13 of each entry.
            let mut pieces_bytes: Vec<u8> = Vec::new();
            for _ in 0..piece_count {
                let mut header = [0u8; PIECE_WIRE_HEADER_LEN];
                for b in &mut header {
                    *b = get_u8(c)?;
                }
                let coeff_count = header[13];
                if coeff_count == 0 || coeff_count as usize > MAX_PIECE_COEFFS {
                    return Err(DecodeError::BadCoeffCount { raw: coeff_count });
                }
                pieces_bytes.extend_from_slice(&header);
                for _ in 0..(4 * coeff_count as usize) {
                    pieces_bytes.push(get_u8(c)?);
                }
            }
            axes.push(AxisPieces {
                axis_idx,
                piece_count,
                start_slot,
                new_head,
                pieces_bytes,
            });
        }
        Ok(Self { axes })
    }
}

/// Per-axis diagnostic echo in a `PushPiecesResponse` — the front piece's start
/// time, used only for the host's transit-diag `arrival_lead`, never control.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AxisDiag {
    pub axis_idx: u8,
    pub front_start_time: u64,
}

/// Frame-level response: one `result` verdict for the whole MCU transaction (a
/// partial frame is desync, not partial success), one `arrival_clock` sampled at
/// frame-receive-complete, and a per-axis diagnostic echo.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PushPiecesResponse {
    pub result: i32,
    pub arrival_clock: u64,
    pub axes: Vec<AxisDiag>,
}

impl PushPiecesResponse {
    /// Build a single-axis response — the common shape for single-axis MCUs.
    pub fn single(result: i32, arrival_clock: u64, axis_idx: u8, front_start_time: u64) -> Self {
        Self {
            result,
            arrival_clock,
            axes: vec![AxisDiag {
                axis_idx,
                front_start_time,
            }],
        }
    }
}

impl Encode for PushPiecesResponse {
    fn encode(&self, out: &mut Vec<u8>) {
        put_i32(out, self.result);
        put_u64(out, self.arrival_clock);
        put_u8(out, self.axes.len() as u8);
        for a in &self.axes {
            put_u8(out, a.axis_idx);
            put_u64(out, a.front_start_time);
        }
    }
}

impl Decode for PushPiecesResponse {
    fn decode_from(c: &mut Cursor<'_>) -> Result<Self, DecodeError> {
        let result = get_i32(c)?;
        let arrival_clock = get_u64(c)?;
        let axis_count = get_u8(c)?;
        let mut axes: Vec<AxisDiag> = Vec::with_capacity(axis_count as usize);
        for _ in 0..axis_count {
            axes.push(AxisDiag {
                axis_idx: get_u8(c)?,
                front_start_time: get_u64(c)?,
            });
        }
        Ok(Self {
            result,
            arrival_clock,
            axes,
        })
    }
}

pub const ERR_SDO_UNSUPPORTED_SIZE: i32 = -801;
pub const ERR_SDO_VERIFY_MISMATCH: i32 = -802;
pub const ERR_SDO_TRANSPORT: i32 = -803;
pub const ERR_SDO_VALUE_RANGE: i32 = -804;
pub const SDO_SIZE_PROBE: u8 = 0;

impl StopCaptureResponse {
    pub const NO_OVERFLOW: u64 = u64::MAX;
}

/// One drive the host asks the endpoint to sample into a `.scap` record. The
/// host owns the slot↔motor map; the endpoint samples `slot` and labels the
/// header block with `name`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CaptureDrive {
    pub slot: u8,
    pub name: String,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct StartCapture {
    pub path: String,
    pub started_utc: String,
    pub drives: Vec<CaptureDrive>,
}

impl Encode for StartCapture {
    fn encode(&self, out: &mut Vec<u8>) {
        put_str(out, &self.path);
        put_str(out, &self.started_utc);
        put_u8(out, self.drives.len() as u8);
        for d in &self.drives {
            put_u8(out, d.slot);
            put_str(out, &d.name);
        }
    }
}

impl Decode for StartCapture {
    fn decode_from(c: &mut Cursor<'_>) -> Result<Self, DecodeError> {
        let path = get_str(c)?;
        let started_utc = get_str(c)?;
        let drive_count = get_u8(c)?;
        if drive_count == 0 {
            return Err(DecodeError::EmptyArray {
                field: "StartCapture.drives",
            });
        }
        let mut drives: Vec<CaptureDrive> = Vec::with_capacity(drive_count as usize);
        for _ in 0..drive_count {
            let slot = get_u8(c)?;
            let name = get_str(c)?;
            if drives.iter().any(|d| d.slot == slot) {
                return Err(DecodeError::DuplicateField {
                    field: "StartCapture.slot",
                });
            }
            drives.push(CaptureDrive { slot, name });
        }
        Ok(Self {
            path,
            started_utc,
            drives,
        })
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct StatusHeartbeat {
    pub engine_state: u8,
    pub fault_code: u16,
    pub retired_counts: Vec<u32>,
    pub ff_saturation_count: u32,
}

impl Encode for StatusHeartbeat {
    fn encode(&self, out: &mut Vec<u8>) {
        put_u8(out, self.engine_state);
        put_u16(out, self.fault_code);
        let num_axes = self.retired_counts.len() as u8;
        put_u8(out, num_axes);
        for &count in &self.retired_counts {
            put_u32(out, count);
        }
        put_u32(out, self.ff_saturation_count);
    }
}

fn decode_u32_array(c: &mut Cursor<'_>, count: u8) -> Result<Vec<u32>, DecodeError> {
    let need = (count as usize)
        .checked_mul(4)
        .ok_or(DecodeError::ArrayLengthExceedsBuffer {
            claimed: u32::from(count),
            available: c.remaining(),
        })?;
    if need > c.remaining() {
        return Err(DecodeError::ArrayLengthExceedsBuffer {
            claimed: u32::from(count),
            available: c.remaining(),
        });
    }
    let mut v = Vec::with_capacity(count as usize);
    for _ in 0..count {
        v.push(get_u32(c)?);
    }
    Ok(v)
}

impl Decode for StatusHeartbeat {
    fn decode_from(c: &mut Cursor<'_>) -> Result<Self, DecodeError> {
        let engine_state = get_u8(c)?;
        let fault_code = get_u16(c)?;
        let num_axes = get_u8(c)?;
        let retired_counts = decode_u32_array(c, num_axes)?;
        let ff_saturation_count = get_u32(c)?;
        Ok(Self {
            engine_state,
            fault_code,
            retired_counts,
            ff_saturation_count,
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MotorSample {
    pub slot: u8,
    pub pos_q16: i32,
    pub vel_q16: i32,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MotorStateResponse {
    pub motors: Vec<MotorSample>,
}

impl Encode for MotorStateResponse {
    fn encode(&self, out: &mut Vec<u8>) {
        put_u8(out, self.motors.len() as u8);
        for m in &self.motors {
            put_u8(out, m.slot);
            put_i32(out, m.pos_q16);
            put_i32(out, m.vel_q16);
        }
    }
}

impl Decode for MotorStateResponse {
    fn decode_from(c: &mut Cursor<'_>) -> Result<Self, DecodeError> {
        let count = get_u8(c)?;
        let need =
            (count as usize)
                .checked_mul(9)
                .ok_or(DecodeError::ArrayLengthExceedsBuffer {
                    claimed: u32::from(count),
                    available: c.remaining(),
                })?;
        if need > c.remaining() {
            return Err(DecodeError::ArrayLengthExceedsBuffer {
                claimed: u32::from(count),
                available: c.remaining(),
            });
        }
        let mut motors = Vec::with_capacity(count as usize);
        for _ in 0..count {
            let slot = get_u8(c)?;
            let pos_q16 = get_i32(c)?;
            let vel_q16 = get_i32(c)?;
            motors.push(MotorSample {
                slot,
                pos_q16,
                vel_q16,
            });
        }
        Ok(Self { motors })
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct SetStrainComp {
    pub slot_a: u8,
    pub slot_b: u8,
    pub lane_a: u8,
    pub lane_b: u8,
    pub kinematics: u8,
    pub nx: u16,
    pub ny: u16,
    pub x0: f32,
    pub y0: f32,
    pub dx: f32,
    pub dy: f32,
    pub values_um: Vec<i32>,
}

impl Encode for SetStrainComp {
    fn encode(&self, out: &mut Vec<u8>) {
        put_u8(out, self.slot_a);
        put_u8(out, self.slot_b);
        put_u8(out, self.lane_a);
        put_u8(out, self.lane_b);
        put_u8(out, self.kinematics);
        put_u16(out, self.nx);
        put_u16(out, self.ny);
        put_f32(out, self.x0);
        put_f32(out, self.y0);
        put_f32(out, self.dx);
        put_f32(out, self.dy);
        put_u32(out, self.values_um.len() as u32);
        for v in &self.values_um {
            put_i32(out, *v);
        }
    }
}

impl Decode for SetStrainComp {
    fn decode_from(c: &mut Cursor<'_>) -> Result<Self, DecodeError> {
        let slot_a = get_u8(c)?;
        let slot_b = get_u8(c)?;
        let lane_a = get_u8(c)?;
        let lane_b = get_u8(c)?;
        let kinematics = get_u8(c)?;
        let nx = get_u16(c)?;
        let ny = get_u16(c)?;
        let x0 = get_f32(c)?;
        let y0 = get_f32(c)?;
        let dx = get_f32(c)?;
        let dy = get_f32(c)?;
        let count = get_u32(c)?;
        let need =
            (count as usize)
                .checked_mul(4)
                .ok_or(DecodeError::ArrayLengthExceedsBuffer {
                    claimed: count,
                    available: c.remaining(),
                })?;
        if need > c.remaining() {
            return Err(DecodeError::ArrayLengthExceedsBuffer {
                claimed: count,
                available: c.remaining(),
            });
        }
        let mut values_um = Vec::with_capacity(count as usize);
        for _ in 0..count {
            values_um.push(get_i32(c)?);
        }
        Ok(Self {
            slot_a,
            slot_b,
            lane_a,
            lane_b,
            kinematics,
            nx,
            ny,
            x0,
            y0,
            dx,
            dy,
            values_um,
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DynamicsPair {
    pub first: u8,
    pub second: u8,
    pub direction_split: f32,
}

#[derive(Debug, Clone, PartialEq)]
pub struct SetDynamicsModel {
    pub slots_count: u8,
    pub modes_count: u8,
    pub frame: Vec<f32>,
    pub mass: Vec<f32>,
    pub viscous: Vec<f32>,
    pub coulomb: Vec<f32>,
    pub compliance: Vec<f32>,
    pub pairs: Vec<DynamicsPair>,
}

impl Encode for SetDynamicsModel {
    fn encode(&self, out: &mut Vec<u8>) {
        let slots = self.slots_count as usize;
        let modes = self.modes_count as usize;
        assert_eq!(self.frame.len(), modes * slots);
        assert_eq!(self.mass.len(), modes);
        assert_eq!(self.viscous.len(), modes);
        assert_eq!(self.coulomb.len(), modes);
        assert_eq!(self.compliance.len(), modes);
        put_u8(out, self.slots_count);
        put_u8(out, self.modes_count);
        for vec in [
            &self.frame,
            &self.mass,
            &self.viscous,
            &self.coulomb,
            &self.compliance,
        ] {
            for v in vec {
                put_f32(out, *v);
            }
        }
        assert!(u8::try_from(self.pairs.len()).is_ok());
        put_u8(out, self.pairs.len() as u8);
        for pair in &self.pairs {
            put_u8(out, pair.first);
            put_u8(out, pair.second);
            put_f32(out, pair.direction_split);
        }
    }
}

fn get_f32_vec(c: &mut Cursor<'_>, count: usize) -> Result<Vec<f32>, DecodeError> {
    let need = count
        .checked_mul(4)
        .ok_or(DecodeError::ArrayLengthExceedsBuffer {
            claimed: count as u32,
            available: c.remaining(),
        })?;
    if need > c.remaining() {
        return Err(DecodeError::ArrayLengthExceedsBuffer {
            claimed: count as u32,
            available: c.remaining(),
        });
    }
    let mut values = Vec::with_capacity(count);
    for _ in 0..count {
        values.push(get_f32(c)?);
    }
    Ok(values)
}

impl Decode for SetDynamicsModel {
    fn decode_from(c: &mut Cursor<'_>) -> Result<Self, DecodeError> {
        let slots_count = get_u8(c)?;
        let modes_count = get_u8(c)?;
        let slots = slots_count as usize;
        let modes = modes_count as usize;
        let frame = get_f32_vec(c, modes * slots)?;
        let mass = get_f32_vec(c, modes)?;
        let viscous = get_f32_vec(c, modes)?;
        let coulomb = get_f32_vec(c, modes)?;
        let compliance = get_f32_vec(c, modes)?;
        let pairs_count = get_u8(c)?;
        let mut pairs = Vec::with_capacity(pairs_count as usize);
        for _ in 0..pairs_count {
            pairs.push(DynamicsPair {
                first: get_u8(c)?,
                second: get_u8(c)?,
                direction_split: get_f32(c)?,
            });
        }
        Ok(Self {
            slots_count,
            modes_count,
            frame,
            mass,
            viscous,
            coulomb,
            compliance,
            pairs,
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum SlaveState {
    Ok = 0x00,
    Offline = 0x01,
    Fault = 0x02,
}

impl SlaveState {
    pub fn from_u8(v: u8) -> Option<Self> {
        match v {
            0x00 => Some(Self::Ok),
            0x01 => Some(Self::Offline),
            0x02 => Some(Self::Fault),
            _ => None,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SlaveStatus {
    pub slave_idx: u8,
    pub state: SlaveState,
    pub fault_code: u16,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ClaimHandshakeReply {
    pub slave_statuses: Vec<SlaveStatus>,
}

impl Encode for ClaimHandshakeReply {
    fn encode(&self, out: &mut Vec<u8>) {
        put_u8(out, self.slave_statuses.len() as u8);
        for s in &self.slave_statuses {
            put_u8(out, s.slave_idx);
            put_u8(out, s.state as u8);
            put_u16(out, s.fault_code);
        }
    }
}

impl Decode for ClaimHandshakeReply {
    fn decode_from(c: &mut Cursor<'_>) -> Result<Self, DecodeError> {
        let count = get_u8(c)?;
        if count == 0 {
            return Err(DecodeError::EmptyArray {
                field: "slave_statuses",
            });
        }
        let entries_len =
            (count as usize)
                .checked_mul(4)
                .ok_or(DecodeError::ArrayLengthExceedsBuffer {
                    claimed: u32::from(count),
                    available: c.remaining(),
                })?;
        if entries_len > c.remaining() {
            return Err(DecodeError::ArrayLengthExceedsBuffer {
                claimed: u32::from(count),
                available: c.remaining(),
            });
        }
        let mut statuses = Vec::with_capacity(count as usize);
        for _ in 0..count {
            let slave_idx = get_u8(c)?;
            let state_raw = get_u8(c)?;
            let state = SlaveState::from_u8(state_raw).ok_or(DecodeError::BadDiscriminant {
                field: "SlaveState",
                raw: state_raw as u32,
            })?;
            let fault_code = get_u16(c)?;
            statuses.push(SlaveStatus {
                slave_idx,
                state,
                fault_code,
            });
        }
        Ok(Self {
            slave_statuses: statuses,
        })
    }
}

#[cfg(test)]
pub(super) fn roundtrip<T>(v: &T) -> T
where
    T: Encode + Decode + PartialEq + std::fmt::Debug,
{
    let bytes = v.encoded_to_vec();
    T::decode(&bytes).expect("decode ok")
}

#[cfg(test)]
mod claim_handshake_tests;
#[cfg(test)]
mod mcu_log_tests;
#[cfg(test)]
mod sdo_tests;
#[cfg(test)]
mod tests;
