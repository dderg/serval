use crate::codec::{
    Cursor, Decode, DecodeError, Encode, get_f32, get_i32, get_str, get_u8, get_u16, get_u32,
    get_u64, put_f32, put_i32, put_str, put_u8, put_u16, put_u32, put_u64,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u16)]
pub enum MessageKind {
    Identify = 0x0001,
    IdentifyResponse = 0x0002,
    ConfigureAxes = 0x0030,
    ConfigureAxesResponse = 0x0031,
    QueryRuntimeCaps = 0x0040,
    RuntimeCapsResponse = 0x0041,
    ClaimHandshake = 0x0042,
    ClaimHandshakeReply = 0x0043,
    QueryMotorState = 0x0044,
    MotorStateResponse = 0x0045,
    PushPieces = 0x0060,
    PushPiecesResponse = 0x0061,
    StartCapture = 0x0068,
    StartCaptureResponse = 0x0069,
    StopCapture = 0x006A,
    StopCaptureResponse = 0x006B,
    ResonanceBuzz = 0x006C,
    ResonanceBuzzResponse = 0x006D,
    ArmSensorlessEndstop = 0x006E,
    ArmSensorlessEndstopResponse = 0x006F,
    SetTorque = 0x0070,
    SetTorqueResponse = 0x0071,
    Stop = 0x0072,
    StopResponse = 0x0073,
    SetDriveLimits = 0x0074,
    SetDriveLimitsResponse = 0x0075,
    RestoreDriveLimits = 0x0076,
    RestoreDriveLimitsResponse = 0x0077,
    ResumeStream = 0x0078,
    ResumeStreamResponse = 0x0079,
    SeedServoHome = 0x007A,
    SeedServoHomeResponse = 0x007B,
    SdoRead = 0x007C,
    SdoReadResponse = 0x007D,
    SdoWrite = 0x007E,
    SdoWriteResponse = 0x007F,
    FaultEvent = 0x0082,
    StatusHeartbeat = 0x0083,
    McuLog = 0x0084,
    EndstopTrip = 0x0085,
}

impl MessageKind {
    pub fn from_u16(v: u16) -> Option<Self> {
        Some(match v {
            0x0001 => Self::Identify,
            0x0002 => Self::IdentifyResponse,
            0x0030 => Self::ConfigureAxes,
            0x0031 => Self::ConfigureAxesResponse,
            0x0040 => Self::QueryRuntimeCaps,
            0x0041 => Self::RuntimeCapsResponse,
            0x0042 => Self::ClaimHandshake,
            0x0043 => Self::ClaimHandshakeReply,
            0x0044 => Self::QueryMotorState,
            0x0045 => Self::MotorStateResponse,
            0x0060 => Self::PushPieces,
            0x0061 => Self::PushPiecesResponse,
            0x0068 => Self::StartCapture,
            0x0069 => Self::StartCaptureResponse,
            0x006A => Self::StopCapture,
            0x006B => Self::StopCaptureResponse,
            0x006C => Self::ResonanceBuzz,
            0x006D => Self::ResonanceBuzzResponse,
            0x006E => Self::ArmSensorlessEndstop,
            0x006F => Self::ArmSensorlessEndstopResponse,
            0x0070 => Self::SetTorque,
            0x0071 => Self::SetTorqueResponse,
            0x0072 => Self::Stop,
            0x0073 => Self::StopResponse,
            0x0074 => Self::SetDriveLimits,
            0x0075 => Self::SetDriveLimitsResponse,
            0x0076 => Self::RestoreDriveLimits,
            0x0077 => Self::RestoreDriveLimitsResponse,
            0x0078 => Self::ResumeStream,
            0x0079 => Self::ResumeStreamResponse,
            0x007A => Self::SeedServoHome,
            0x007B => Self::SeedServoHomeResponse,
            0x007C => Self::SdoRead,
            0x007D => Self::SdoReadResponse,
            0x007E => Self::SdoWrite,
            0x007F => Self::SdoWriteResponse,
            0x0082 => Self::FaultEvent,
            0x0083 => Self::StatusHeartbeat,
            0x0084 => Self::McuLog,
            0x0085 => Self::EndstopTrip,
            _ => return None,
        })
    }

    pub fn as_u16(self) -> u16 {
        self as u16
    }

    pub fn is_schema_validated(self) -> bool {
        !matches!(self, Self::Identify | Self::IdentifyResponse)
    }

    pub fn is_event(self) -> bool {
        let tag = self as u16;
        (0x0080..=0x00BF).contains(&tag)
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ConfigureAxes {
    pub kinematics: u8,
    pub present_mask: u8,
    pub awd_mask: u8,
    pub invert_mask: u8,
    pub steps_per_mm: [f32; 4],
}

impl Encode for ConfigureAxes {
    fn encode(&self, out: &mut Vec<u8>) {
        put_u8(out, self.kinematics);
        put_u8(out, self.present_mask);
        put_u8(out, self.awd_mask);
        put_u8(out, self.invert_mask);
        for v in &self.steps_per_mm {
            put_f32(out, *v);
        }
    }
}

impl Decode for ConfigureAxes {
    fn decode_from(c: &mut Cursor<'_>) -> Result<Self, DecodeError> {
        let kinematics = get_u8(c)?;
        let present_mask = get_u8(c)?;
        let awd_mask = get_u8(c)?;
        let invert_mask = get_u8(c)?;
        let steps_per_mm = [get_f32(c)?, get_f32(c)?, get_f32(c)?, get_f32(c)?];
        Ok(Self {
            kinematics,
            present_mask,
            awd_mask,
            invert_mask,
            steps_per_mm,
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ConfigureAxesResponse {
    pub result: i32,
}

impl Encode for ConfigureAxesResponse {
    fn encode(&self, out: &mut Vec<u8>) {
        put_i32(out, self.result);
    }
}

impl Decode for ConfigureAxesResponse {
    fn decode_from(c: &mut Cursor<'_>) -> Result<Self, DecodeError> {
        Ok(Self {
            result: get_i32(c)?,
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RuntimeCapsResponse {
    pub total_piece_memory: u32,
}

pub const RUNTIME_CAPS_RESPONSE_BODY_LEN: usize = 4;

impl Encode for RuntimeCapsResponse {
    fn encode(&self, out: &mut Vec<u8>) {
        put_u32(out, self.total_piece_memory);
    }
}

impl Decode for RuntimeCapsResponse {
    fn decode_from(c: &mut Cursor<'_>) -> Result<Self, DecodeError> {
        Ok(Self {
            total_piece_memory: get_u32(c)?,
        })
    }
}

/// Usable `PushPieces` payload budget — mirrors `MCU_TX_BUF_SIZE` (256) in
/// `src/mcu_transport_dispatch.c` minus the sync/len/channel/CRC framing. A
/// single shared constant compiled into every chip's firmware; keep in sync.
pub const PIECE_FRAME_PAYLOAD_MAX: usize = 250;

/// Bytes of one axis block header (`axis_idx + piece_count + start_slot + new_head`).
pub const AXIS_BLOCK_HEADER_LEN: usize = 8;

/// Bytes of one encoded `PieceEntry`.
pub const PIECE_ENTRY_LEN: usize = 32;

/// Largest `piece_count` per axis that still lets an `axis_count`-axis frame fit
/// `PIECE_FRAME_PAYLOAD_MAX`, assuming an even split. Parameterized off the
/// shared budget — never a per-chip number. Returns 0 when even one piece per
/// axis cannot fit (too many axes for the buffer).
pub fn max_pieces_per_axis(axis_count: u8) -> usize {
    let n = axis_count.max(1) as usize;
    let avail = PIECE_FRAME_PAYLOAD_MAX.saturating_sub(1 + n * AXIS_BLOCK_HEADER_LEN);
    (avail / PIECE_ENTRY_LEN) / n
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
            let pieces_len = (piece_count as usize).checked_mul(PIECE_ENTRY_LEN).ok_or(
                DecodeError::ArrayLengthExceedsBuffer {
                    claimed: u32::from(piece_count),
                    available: c.remaining(),
                },
            )?;
            if pieces_len > c.remaining() {
                return Err(DecodeError::ArrayLengthExceedsBuffer {
                    claimed: u32::from(piece_count),
                    available: c.remaining(),
                });
            }
            if axes.iter().any(|a| a.axis_idx == axis_idx) {
                return Err(DecodeError::DuplicateField {
                    field: "PushPieces.axis_idx",
                });
            }
            let mut pieces_bytes = vec![0u8; pieces_len];
            for b in &mut pieces_bytes {
                *b = get_u8(c)?;
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

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SetTorque {
    pub value: u8,
    pub execute_at_ns: u64,
}

impl Encode for SetTorque {
    fn encode(&self, out: &mut Vec<u8>) {
        put_u8(out, self.value);
        put_u64(out, self.execute_at_ns);
    }
}

impl Decode for SetTorque {
    fn decode_from(c: &mut Cursor<'_>) -> Result<Self, DecodeError> {
        Ok(Self {
            value: get_u8(c)?,
            execute_at_ns: get_u64(c)?,
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SetTorqueResponse {
    pub result: i32,
}

impl Encode for SetTorqueResponse {
    fn encode(&self, out: &mut Vec<u8>) {
        put_i32(out, self.result);
    }
}

impl Decode for SetTorqueResponse {
    fn decode_from(c: &mut Cursor<'_>) -> Result<Self, DecodeError> {
        Ok(Self {
            result: get_i32(c)?,
        })
    }
}

pub const ERR_SDO_UNSUPPORTED_SIZE: i32 = -801;
pub const ERR_SDO_VERIFY_MISMATCH: i32 = -802;
pub const ERR_SDO_TRANSPORT: i32 = -803;
pub const ERR_SDO_VALUE_RANGE: i32 = -804;
pub const SDO_SIZE_PROBE: u8 = 0;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SdoRead {
    pub slot: u8,
    pub index: u16,
    pub subindex: u8,
}

impl Encode for SdoRead {
    fn encode(&self, out: &mut Vec<u8>) {
        put_u8(out, self.slot);
        put_u16(out, self.index);
        put_u8(out, self.subindex);
    }
}

impl Decode for SdoRead {
    fn decode_from(c: &mut Cursor<'_>) -> Result<Self, DecodeError> {
        Ok(Self {
            slot: get_u8(c)?,
            index: get_u16(c)?,
            subindex: get_u8(c)?,
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SdoReadResponse {
    pub result: i32,
    pub size: u8,
    pub data: [u8; 4],
}

impl Encode for SdoReadResponse {
    fn encode(&self, out: &mut Vec<u8>) {
        put_i32(out, self.result);
        put_u8(out, self.size);
        out.extend_from_slice(&self.data);
    }
}

impl Decode for SdoReadResponse {
    fn decode_from(c: &mut Cursor<'_>) -> Result<Self, DecodeError> {
        Ok(Self {
            result: get_i32(c)?,
            size: get_u8(c)?,
            data: [get_u8(c)?, get_u8(c)?, get_u8(c)?, get_u8(c)?],
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SdoWrite {
    pub slot: u8,
    pub index: u16,
    pub subindex: u8,
    pub size: u8,
    pub value: i64,
}

impl Encode for SdoWrite {
    fn encode(&self, out: &mut Vec<u8>) {
        put_u8(out, self.slot);
        put_u16(out, self.index);
        put_u8(out, self.subindex);
        put_u8(out, self.size);
        put_u64(out, self.value as u64);
    }
}

impl Decode for SdoWrite {
    fn decode_from(c: &mut Cursor<'_>) -> Result<Self, DecodeError> {
        Ok(Self {
            slot: get_u8(c)?,
            index: get_u16(c)?,
            subindex: get_u8(c)?,
            size: get_u8(c)?,
            value: get_u64(c)? as i64,
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SdoWriteResponse {
    pub result: i32,
    pub readback_size: u8,
    pub readback_data: [u8; 4],
}

impl Encode for SdoWriteResponse {
    fn encode(&self, out: &mut Vec<u8>) {
        put_i32(out, self.result);
        put_u8(out, self.readback_size);
        out.extend_from_slice(&self.readback_data);
    }
}

impl Decode for SdoWriteResponse {
    fn decode_from(c: &mut Cursor<'_>) -> Result<Self, DecodeError> {
        Ok(Self {
            result: get_i32(c)?,
            readback_size: get_u8(c)?,
            readback_data: [get_u8(c)?, get_u8(c)?, get_u8(c)?, get_u8(c)?],
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Stop;

impl Encode for Stop {
    fn encode(&self, _out: &mut Vec<u8>) {}
}

impl Decode for Stop {
    fn decode_from(_c: &mut Cursor<'_>) -> Result<Self, DecodeError> {
        Ok(Self)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StopResponse {
    pub result: i32,
    pub discard_clock: u64,
}

impl Encode for StopResponse {
    fn encode(&self, out: &mut Vec<u8>) {
        put_i32(out, self.result);
        put_u64(out, self.discard_clock);
    }
}

impl Decode for StopResponse {
    fn decode_from(c: &mut Cursor<'_>) -> Result<Self, DecodeError> {
        Ok(Self {
            result: get_i32(c)?,
            discard_clock: get_u64(c)?,
        })
    }
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

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StartCaptureResponse {
    pub result: i32,
}

impl Encode for StartCaptureResponse {
    fn encode(&self, out: &mut Vec<u8>) {
        put_i32(out, self.result);
    }
}

impl Decode for StartCaptureResponse {
    fn decode_from(c: &mut Cursor<'_>) -> Result<Self, DecodeError> {
        Ok(Self {
            result: get_i32(c)?,
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StopCapture;

impl Encode for StopCapture {
    fn encode(&self, _out: &mut Vec<u8>) {}
}

impl Decode for StopCapture {
    fn decode_from(_c: &mut Cursor<'_>) -> Result<Self, DecodeError> {
        Ok(Self)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StopCaptureResponse {
    pub result: i32,
    pub samples: u64,
    pub overflow_cycle: u64,
}

impl StopCaptureResponse {
    pub const NO_OVERFLOW: u64 = u64::MAX;
}

impl Encode for StopCaptureResponse {
    fn encode(&self, out: &mut Vec<u8>) {
        put_i32(out, self.result);
        put_u64(out, self.samples);
        put_u64(out, self.overflow_cycle);
    }
}

impl Decode for StopCaptureResponse {
    fn decode_from(c: &mut Cursor<'_>) -> Result<Self, DecodeError> {
        Ok(Self {
            result: get_i32(c)?,
            samples: get_u64(c)?,
            overflow_cycle: get_u64(c)?,
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ArmSensorlessEndstop {
    pub slot: u8,
    pub endstop_id: u8,
    pub torque_trip_tenth_pct: u16,
    pub enable: u8,
}

impl Encode for ArmSensorlessEndstop {
    fn encode(&self, out: &mut Vec<u8>) {
        put_u8(out, self.slot);
        put_u8(out, self.endstop_id);
        put_u16(out, self.torque_trip_tenth_pct);
        put_u8(out, self.enable);
    }
}

impl Decode for ArmSensorlessEndstop {
    fn decode_from(c: &mut Cursor<'_>) -> Result<Self, DecodeError> {
        Ok(Self {
            slot: get_u8(c)?,
            endstop_id: get_u8(c)?,
            torque_trip_tenth_pct: get_u16(c)?,
            enable: get_u8(c)?,
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ArmSensorlessEndstopResponse {
    pub result: i32,
}

impl Encode for ArmSensorlessEndstopResponse {
    fn encode(&self, out: &mut Vec<u8>) {
        put_i32(out, self.result);
    }
}

impl Decode for ArmSensorlessEndstopResponse {
    fn decode_from(c: &mut Cursor<'_>) -> Result<Self, DecodeError> {
        Ok(Self {
            result: get_i32(c)?,
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SetDriveLimits {
    pub slot: u8,
    pub following_error_counts: u32,
    pub max_torque_tenth_pct: u16,
}

impl Encode for SetDriveLimits {
    fn encode(&self, out: &mut Vec<u8>) {
        put_u8(out, self.slot);
        put_u32(out, self.following_error_counts);
        put_u16(out, self.max_torque_tenth_pct);
    }
}

impl Decode for SetDriveLimits {
    fn decode_from(c: &mut Cursor<'_>) -> Result<Self, DecodeError> {
        Ok(Self {
            slot: get_u8(c)?,
            following_error_counts: get_u32(c)?,
            max_torque_tenth_pct: get_u16(c)?,
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SetDriveLimitsResponse {
    pub result: i32,
}

impl Encode for SetDriveLimitsResponse {
    fn encode(&self, out: &mut Vec<u8>) {
        put_i32(out, self.result);
    }
}

impl Decode for SetDriveLimitsResponse {
    fn decode_from(c: &mut Cursor<'_>) -> Result<Self, DecodeError> {
        Ok(Self {
            result: get_i32(c)?,
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RestoreDriveLimits {
    pub slot: u8,
}

impl Encode for RestoreDriveLimits {
    fn encode(&self, out: &mut Vec<u8>) {
        put_u8(out, self.slot);
    }
}

impl Decode for RestoreDriveLimits {
    fn decode_from(c: &mut Cursor<'_>) -> Result<Self, DecodeError> {
        Ok(Self { slot: get_u8(c)? })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RestoreDriveLimitsResponse {
    pub result: i32,
}

impl Encode for RestoreDriveLimitsResponse {
    fn encode(&self, out: &mut Vec<u8>) {
        put_i32(out, self.result);
    }
}

impl Decode for RestoreDriveLimitsResponse {
    fn decode_from(c: &mut Cursor<'_>) -> Result<Self, DecodeError> {
        Ok(Self {
            result: get_i32(c)?,
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SeedServoHome {
    pub slot: u8,
    pub home_q16: i32,
}

impl Encode for SeedServoHome {
    fn encode(&self, out: &mut Vec<u8>) {
        put_u8(out, self.slot);
        put_i32(out, self.home_q16);
    }
}

impl Decode for SeedServoHome {
    fn decode_from(c: &mut Cursor<'_>) -> Result<Self, DecodeError> {
        Ok(Self {
            slot: get_u8(c)?,
            home_q16: get_i32(c)?,
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SeedServoHomeResponse {
    pub result: i32,
}

impl Encode for SeedServoHomeResponse {
    fn encode(&self, out: &mut Vec<u8>) {
        put_i32(out, self.result);
    }
}

impl Decode for SeedServoHomeResponse {
    fn decode_from(c: &mut Cursor<'_>) -> Result<Self, DecodeError> {
        Ok(Self {
            result: get_i32(c)?,
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ResonanceBuzz {
    pub axis_mask: u8,
    pub sign_mask: u8,
    pub freq_start_millihz: u32,
    pub freq_end_millihz: u32,
    pub amplitude_nm: u32,
    pub duration_ms: u32,
    pub ramp_ms: u32,
}

impl Encode for ResonanceBuzz {
    fn encode(&self, out: &mut Vec<u8>) {
        put_u8(out, self.axis_mask);
        put_u8(out, self.sign_mask);
        put_u32(out, self.freq_start_millihz);
        put_u32(out, self.freq_end_millihz);
        put_u32(out, self.amplitude_nm);
        put_u32(out, self.duration_ms);
        put_u32(out, self.ramp_ms);
    }
}

impl Decode for ResonanceBuzz {
    fn decode_from(c: &mut Cursor<'_>) -> Result<Self, DecodeError> {
        Ok(Self {
            axis_mask: get_u8(c)?,
            sign_mask: get_u8(c)?,
            freq_start_millihz: get_u32(c)?,
            freq_end_millihz: get_u32(c)?,
            amplitude_nm: get_u32(c)?,
            duration_ms: get_u32(c)?,
            ramp_ms: get_u32(c)?,
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ResonanceBuzzResponse {
    pub result: i32,
}

impl Encode for ResonanceBuzzResponse {
    fn encode(&self, out: &mut Vec<u8>) {
        put_i32(out, self.result);
    }
}

impl Decode for ResonanceBuzzResponse {
    fn decode_from(c: &mut Cursor<'_>) -> Result<Self, DecodeError> {
        Ok(Self {
            result: get_i32(c)?,
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ResumeStream;

impl Encode for ResumeStream {
    fn encode(&self, _out: &mut Vec<u8>) {}
}

impl Decode for ResumeStream {
    fn decode_from(_c: &mut Cursor<'_>) -> Result<Self, DecodeError> {
        Ok(Self)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ResumeStreamResponse {
    pub result: i32,
}

impl Encode for ResumeStreamResponse {
    fn encode(&self, out: &mut Vec<u8>) {
        put_i32(out, self.result);
    }
}

impl Decode for ResumeStreamResponse {
    fn decode_from(c: &mut Cursor<'_>) -> Result<Self, DecodeError> {
        Ok(Self {
            result: get_i32(c)?,
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FaultEvent {
    pub fault_code: u16,
    pub fault_detail: u32,
    pub segment_id: u32,
}

impl Encode for FaultEvent {
    fn encode(&self, out: &mut Vec<u8>) {
        put_u16(out, self.fault_code);
        put_u32(out, self.fault_detail);
        put_u32(out, self.segment_id);
    }
}

impl Decode for FaultEvent {
    fn decode_from(c: &mut Cursor<'_>) -> Result<Self, DecodeError> {
        Ok(Self {
            fault_code: get_u16(c)?,
            fault_detail: get_u32(c)?,
            segment_id: get_u32(c)?,
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
pub struct McuLog {
    pub mcu_tick: u64,
    pub level: u8,
    pub subsystem: u8,
    pub event: u16,
    pub code: u16,
    pub seq: u16,
    pub args: [u32; 2],
}

impl Encode for McuLog {
    fn encode(&self, out: &mut Vec<u8>) {
        put_u64(out, self.mcu_tick);
        put_u8(out, self.level);
        put_u8(out, self.subsystem);
        put_u16(out, self.event);
        put_u16(out, self.code);
        put_u16(out, self.seq);
        put_u32(out, self.args[0]);
        put_u32(out, self.args[1]);
    }
}

impl Decode for McuLog {
    fn decode_from(c: &mut Cursor<'_>) -> Result<Self, DecodeError> {
        Ok(Self {
            mcu_tick: get_u64(c)?,
            level: get_u8(c)?,
            subsystem: get_u8(c)?,
            event: get_u16(c)?,
            code: get_u16(c)?,
            seq: get_u16(c)?,
            args: [get_u32(c)?, get_u32(c)?],
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EndstopTrip {
    pub endstop_id: u8,
    pub trip_clock: u64,
}

impl Encode for EndstopTrip {
    fn encode(&self, out: &mut Vec<u8>) {
        put_u8(out, self.endstop_id);
        put_u64(out, self.trip_clock);
    }
}

impl Decode for EndstopTrip {
    fn decode_from(c: &mut Cursor<'_>) -> Result<Self, DecodeError> {
        Ok(Self {
            endstop_id: get_u8(c)?,
            trip_clock: get_u64(c)?,
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
