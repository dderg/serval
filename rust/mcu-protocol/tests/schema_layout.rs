//! Schema-driven reference decoder — the drift tripwire.
//!
//! Flat-message codecs are generated from `SCHEMA_MESSAGES` by `build.rs`, so
//! they cannot drift from the schema. The six variable-length messages keep
//! hand-written codecs; for those, nothing structural forces the schema entry
//! to stay truthful. These tests close that gap: they decode real encoder
//! output using only the schema's field descriptions and fail when the bytes
//! and the description disagree — a field added, removed, or resized in the
//! codec without the matching `schema_def.rs` edit (and therefore without the
//! `schema_hash` rotation the Identify lockstep check depends on).

use mcu_protocol::Encode;
use mcu_protocol::messages::{
    AxisDiag, AxisPieces, CaptureDrive, ClaimHandshakeReply, ConfigureAxes, McuLog, MotorSample,
    MotorStateResponse, PushPieces, PushPiecesResponse, SdoReadResponse, SdoWrite, SetStrainComp,
    SlaveState, SlaveStatus, StartCapture, StatusHeartbeat,
};

include!("../schema_def.rs");

#[derive(Debug, Clone, Copy, PartialEq)]
enum Scalar {
    U8,
    U16,
    U32,
    U64,
    I32,
    I64,
    F32,
}

impl Scalar {
    fn size(self) -> usize {
        match self {
            Scalar::U8 => 1,
            Scalar::U16 => 2,
            Scalar::U32 | Scalar::I32 | Scalar::F32 => 4,
            Scalar::U64 | Scalar::I64 => 8,
        }
    }

    fn is_int(self) -> bool {
        self != Scalar::F32
    }
}

#[derive(Debug, Clone)]
enum Ty {
    Scalar(Scalar),
    Fixed(Scalar, usize),
    Str,
    Array {
        elem: Elem,
        bounds: Option<(u64, u64)>,
    },
}

#[derive(Debug, Clone)]
enum Elem {
    Scalar(Scalar),
    Struct(Vec<(String, Ty)>),
}

fn parse_scalar(s: &str) -> Scalar {
    match s {
        "u8" => Scalar::U8,
        "u16" => Scalar::U16,
        "u32" => Scalar::U32,
        "u64" => Scalar::U64,
        "i32" => Scalar::I32,
        "i64" => Scalar::I64,
        "f32" => Scalar::F32,
        _ => panic!("unknown scalar type: {s}"),
    }
}

fn parse_ty(s: &str) -> Ty {
    if s == "string" {
        return Ty::Str;
    }
    if let Some(inner) = s.strip_prefix("array<").and_then(|i| i.strip_suffix('>')) {
        return parse_array(inner);
    }
    if let Some((base, rest)) = s.split_once('[') {
        let n = rest
            .strip_suffix(']')
            .and_then(|x| x.parse().ok())
            .unwrap_or_else(|| panic!("malformed fixed-array type: {s}"));
        return Ty::Fixed(parse_scalar(base), n);
    }
    Ty::Scalar(parse_scalar(s))
}

fn parse_array(inner: &str) -> Ty {
    if let Some(brace) = inner.find('{') {
        let body = inner
            .strip_suffix('}')
            .unwrap_or_else(|| panic!("array struct element must end with '}}': {inner}"));
        let fields = split_top_level(&body[brace + 1..], ',')
            .into_iter()
            .map(|f| {
                let (name, ty) = f
                    .split_once(':')
                    .unwrap_or_else(|| panic!("struct field must be name:ty — got {f}"));
                (name.to_string(), parse_ty(ty))
            })
            .collect();
        return Ty::Array {
            elem: Elem::Struct(fields),
            bounds: None,
        };
    }
    if let Some((scalar, bounds)) = inner.split_once(';') {
        let (lo, hi) = bounds
            .split_once("..=")
            .unwrap_or_else(|| panic!("array bounds must be lo..=hi — got {bounds}"));
        return Ty::Array {
            elem: Elem::Scalar(parse_scalar(scalar)),
            bounds: Some((
                lo.parse().unwrap_or_else(|_| panic!("bad bound {lo}")),
                hi.parse().unwrap_or_else(|_| panic!("bad bound {hi}")),
            )),
        };
    }
    Ty::Array {
        elem: Elem::Scalar(parse_scalar(inner)),
        bounds: None,
    }
}

fn split_top_level(s: &str, sep: char) -> Vec<&str> {
    let mut parts = Vec::new();
    let mut depth = 0usize;
    let mut start = 0;
    for (i, ch) in s.char_indices() {
        match ch {
            '<' | '{' => depth += 1,
            '>' | '}' => depth -= 1,
            c if c == sep && depth == 0 => {
                parts.push(&s[start..i]);
                start = i + 1;
            }
            _ => {}
        }
    }
    parts.push(&s[start..]);
    parts
}

struct Reader<'a> {
    bytes: &'a [u8],
    pos: usize,
}

impl<'a> Reader<'a> {
    fn take(&mut self, n: usize) -> Result<&'a [u8], String> {
        if self.bytes.len() - self.pos < n {
            return Err(format!(
                "unexpected EOF: need {n} byte(s) at offset {}, have {}",
                self.pos,
                self.bytes.len() - self.pos
            ));
        }
        let s = &self.bytes[self.pos..self.pos + n];
        self.pos += n;
        Ok(s)
    }
}

fn le_u64(bytes: &[u8]) -> u64 {
    bytes
        .iter()
        .rev()
        .fold(0u64, |acc, b| (acc << 8) | u64::from(*b))
}

/// Variable-array length convention: an `array<...>` field takes its element
/// count from the most recently decoded integer field in the same struct
/// whose name ends with `count` or starts with `num_`; when no such field
/// precedes it, the array is prefixed with a u8 count on the wire.
fn latest_count(ints: &[(String, u64)]) -> Option<u64> {
    ints.iter()
        .rev()
        .find(|(name, _)| name.ends_with("count") || name.starts_with("num_"))
        .map(|(_, v)| *v)
}

fn decode_struct(fields: &[(String, Ty)], r: &mut Reader<'_>) -> Result<(), String> {
    let mut ints: Vec<(String, u64)> = Vec::new();
    for (name, ty) in fields {
        match ty {
            Ty::Scalar(s) => {
                let raw = r.take(s.size())?;
                if s.is_int() {
                    ints.push((name.clone(), le_u64(raw)));
                }
            }
            Ty::Fixed(s, n) => {
                r.take(s.size() * n)?;
            }
            Ty::Str => {
                let len = le_u64(r.take(2)?) as usize;
                r.take(len)?;
            }
            Ty::Array { elem, bounds } => {
                let n = match latest_count(&ints) {
                    Some(v) => v,
                    None => le_u64(r.take(1)?),
                };
                if let Some((lo, hi)) = bounds {
                    if n < *lo || n > *hi {
                        return Err(format!("array {name} count {n} outside {lo}..={hi}"));
                    }
                }
                for _ in 0..n {
                    match elem {
                        Elem::Scalar(s) => {
                            r.take(s.size())?;
                        }
                        Elem::Struct(fs) => decode_struct(fs, r)?,
                    }
                }
            }
        }
    }
    Ok(())
}

fn reference_decode(name: &str, bytes: &[u8]) -> Result<(), String> {
    let m = SCHEMA_MESSAGES
        .iter()
        .find(|m| m.name == name)
        .unwrap_or_else(|| panic!("{name} missing from SCHEMA_MESSAGES"));
    let fields: Vec<(String, Ty)> = m
        .fields
        .iter()
        .map(|f| (f.name.to_string(), parse_ty(f.ty)))
        .collect();
    let mut r = Reader { bytes, pos: 0 };
    decode_struct(&fields, &mut r)?;
    if r.pos != bytes.len() {
        return Err(format!(
            "{} trailing byte(s) the schema does not describe",
            bytes.len() - r.pos
        ));
    }
    Ok(())
}

fn one_piece(fill: u8, coeff_count: u8) -> Vec<u8> {
    let mut v = vec![fill; 16];
    v[13] = coeff_count;
    v.extend(std::iter::repeat_n(fill, 4 * coeff_count as usize));
    v
}

fn sample_heartbeat() -> StatusHeartbeat {
    StatusHeartbeat {
        engine_state: 1,
        fault_code: 2,
        retired_counts: vec![7, 8, 9],
        ff_saturation_count: 5,
    }
}

#[test]
fn every_schema_field_type_parses() {
    for m in SCHEMA_MESSAGES {
        for f in m.fields {
            let _ = parse_ty(f.ty);
        }
    }
    assert!(!canonicalize_schema(SCHEMA_MESSAGES).is_empty());
}

#[test]
fn push_pieces_matches_schema_layout() {
    let msg = PushPieces {
        axes: vec![
            AxisPieces {
                axis_idx: 0,
                piece_count: 2,
                start_slot: 3,
                new_head: 9,
                pieces_bytes: [one_piece(0x11, 4), one_piece(0x22, 8)].concat(),
            },
            AxisPieces {
                axis_idx: 1,
                piece_count: 1,
                start_slot: 0,
                new_head: 5,
                pieces_bytes: one_piece(0x33, 1),
            },
        ],
    };
    reference_decode("PushPieces", &msg.encoded_to_vec()).unwrap();
}

#[test]
fn push_pieces_response_matches_schema_layout() {
    let msg = PushPiecesResponse {
        result: -309,
        arrival_clock: 0x0102_0304_0506_0708,
        axes: vec![
            AxisDiag {
                axis_idx: 0,
                front_start_time: 111,
            },
            AxisDiag {
                axis_idx: 1,
                front_start_time: 222,
            },
        ],
    };
    reference_decode("PushPiecesResponse", &msg.encoded_to_vec()).unwrap();
}

#[test]
fn start_capture_matches_schema_layout() {
    let msg = StartCapture {
        path: "/tmp/run.scap".to_string(),
        started_utc: "2026-07-06T00:00:00Z".to_string(),
        drives: vec![
            CaptureDrive {
                slot: 0,
                name: "x".to_string(),
            },
            CaptureDrive {
                slot: 1,
                name: "y".to_string(),
            },
        ],
    };
    reference_decode("StartCapture", &msg.encoded_to_vec()).unwrap();
}

#[test]
fn status_heartbeat_matches_schema_layout() {
    reference_decode("StatusHeartbeat", &sample_heartbeat().encoded_to_vec()).unwrap();
}

#[test]
fn motor_state_response_matches_schema_layout() {
    let msg = MotorStateResponse {
        motors: vec![
            MotorSample {
                slot: 0,
                pos_q16: -65536,
                vel_q16: 123,
            },
            MotorSample {
                slot: 1,
                pos_q16: 42,
                vel_q16: -7,
            },
        ],
    };
    reference_decode("MotorStateResponse", &msg.encoded_to_vec()).unwrap();
}

#[test]
fn set_strain_comp_matches_schema_layout() {
    let msg = SetStrainComp {
        slot_a: 0,
        slot_b: 1,
        lane_a: 0,
        lane_b: 1,
        kinematics: 0,
        nx: 2,
        ny: 2,
        x0: 30.0,
        y0: 30.0,
        dx: 120.0,
        dy: 120.0,
        values_um: vec![-150, 40, 0, 220],
    };
    reference_decode("SetStrainComp", &msg.encoded_to_vec()).unwrap();
}

#[test]
fn claim_handshake_reply_matches_schema_layout() {
    let msg = ClaimHandshakeReply {
        slave_statuses: vec![
            SlaveStatus {
                slave_idx: 0,
                state: SlaveState::Ok,
                fault_code: 0,
            },
            SlaveStatus {
                slave_idx: 1,
                state: SlaveState::Fault,
                fault_code: 0x8130,
            },
        ],
    };
    reference_decode("ClaimHandshakeReply", &msg.encoded_to_vec()).unwrap();
}

#[test]
fn generated_flat_codecs_match_schema_layout() {
    let configure = ConfigureAxes {
        kinematics: 1,
        present_mask: 3,
        awd_mask: 0,
        invert_mask: 2,
        steps_per_mm: [80.0, 80.0, 400.0, 693.0],
    };
    reference_decode("ConfigureAxes", &configure.encoded_to_vec()).unwrap();

    let sdo_write = SdoWrite {
        slot: 1,
        index: 0x6060,
        subindex: 0,
        size: 4,
        value: -5,
    };
    reference_decode("SdoWrite", &sdo_write.encoded_to_vec()).unwrap();

    let sdo_read_response = SdoReadResponse {
        result: 0,
        size: 4,
        data: [1, 2, 3, 4],
    };
    reference_decode("SdoReadResponse", &sdo_read_response.encoded_to_vec()).unwrap();

    let mcu_log = McuLog {
        mcu_tick: 99,
        level: 1,
        subsystem: 2,
        event: 3,
        code: 4,
        seq: 5,
        args: [6, 7],
    };
    reference_decode("McuLog", &mcu_log.encoded_to_vec()).unwrap();
}

#[test]
fn codec_bytes_the_schema_does_not_describe_are_caught() {
    let mut bytes = sample_heartbeat().encoded_to_vec();
    bytes.push(0);
    let err = reference_decode("StatusHeartbeat", &bytes).unwrap_err();
    assert!(err.contains("trailing"), "got: {err}");
}

#[test]
fn codec_bytes_missing_from_the_schema_layout_are_caught() {
    let mut bytes = sample_heartbeat().encoded_to_vec();
    bytes.pop();
    let err = reference_decode("StatusHeartbeat", &bytes).unwrap_err();
    assert!(err.contains("EOF"), "got: {err}");
}

#[test]
fn schema_array_bounds_are_enforced() {
    let msg = PushPieces::single(0, 1, 0, 0, one_piece(0, 0));
    let err = reference_decode("PushPieces", &msg.encoded_to_vec()).unwrap_err();
    assert!(err.contains("outside"), "got: {err}");
}
