// Shared schema definition included by `build.rs` and the integration test
// `tests/schema_hash.rs`. Pure data + a pure canonicalization function.
// MUST NOT depend on any other module of this crate (it's `include!`'d, not
// imported as a Rust module).
//
// This table is the single source of truth for message layouts. `build.rs`
// derives three artifacts from it:
//   - `SCHEMA_HASH` (host/MCU lockstep check, exchanged during Identify)
//   - the C header `src/mcu_protocol_schema.h` (type tags + hash)
//   - `src/messages/generated.rs` (`MessageKind` + struct + Encode/Decode
//     impls for every flat message)
//
// Field type language:
//   u8 u16 u32 u64 i32 i64 f32   little-endian scalars (i64 rides the wire
//                                as its two's-complement u64 bytes)
//   T[N]                         fixed-size array, packed, no length prefix
//   string                       u16-le byte length + UTF-8 bytes
//   array<...>                   variable-length; the layout written here is
//                                the wire contract (it feeds the hash), but
//                                the codec is hand-written
//
// Variable-array length convention: an `array<...>` field takes its element
// count from the most recent preceding integer field in the same struct whose
// name ends with `count` or starts with `num_`; when no such field precedes
// it, the array carries its own u8 count prefix on the wire.
//
// Codec placement rule: a message whose fields are all scalars or T[N] gets
// its struct and codec generated into `src/messages/generated.rs`; a message
// with any `string` or `array<` field keeps its hand-written struct and codec
// in `src/messages.rs` — and `tests/schema_layout.rs` decodes that codec's
// real output using only the description written here, so a codec edit
// without the matching schema edit fails the suite.

#[derive(Clone, Copy)]
struct SchemaField {
    name: &'static str,
    ty: &'static str,
}

#[derive(Clone, Copy)]
struct SchemaMessage {
    type_tag: u16,
    name: &'static str,
    version: u8,
    channel: &'static str, // "control" | "events" | "pieces"
    fields: &'static [SchemaField],
}

// Bootstrap messages (Identify=0x0001, IdentifyResponse=0x0002) are
// intentionally excluded — see spec §5. Their byte layout is frozen forever
// and decoupled from `schema_hash`. Including them would make `schema_hash`
// itself depend on the bootstrap layout, which breaks the "fixed forever"
// property of the bootstrap.
//
// Message order: ascending type-tag.
const SCHEMA_MESSAGES: &[SchemaMessage] = &[
    SchemaMessage {
        type_tag: 0x0030,
        name: "ConfigureAxes",
        version: 1,
        channel: "control",
        fields: &[
            SchemaField { name: "kinematics", ty: "u8" },
            SchemaField { name: "present_mask", ty: "u8" },
            SchemaField { name: "awd_mask", ty: "u8" },
            SchemaField { name: "invert_mask", ty: "u8" },
            SchemaField { name: "steps_per_mm", ty: "f32[4]" },
        ],
    },
    SchemaMessage {
        type_tag: 0x0031,
        name: "ConfigureAxesResponse",
        version: 1,
        channel: "control",
        fields: &[
            SchemaField { name: "result", ty: "i32" },
        ],
    },
    SchemaMessage {
        type_tag: 0x0040,
        name: "QueryRuntimeCaps",
        version: 1,
        channel: "control",
        fields: &[],
    },
    SchemaMessage {
        type_tag: 0x0041,
        name: "RuntimeCapsResponse",
        version: 2,
        channel: "control",
        fields: &[
            SchemaField { name: "total_piece_memory", ty: "u32" },
        ],
    },
    SchemaMessage {
        type_tag: 0x0042,
        name: "ClaimHandshake",
        version: 1,
        channel: "control",
        fields: &[],
    },
    SchemaMessage {
        type_tag: 0x0043,
        name: "ClaimHandshakeReply",
        version: 1,
        channel: "control",
        fields: &[
            SchemaField {
                name: "slave_statuses",
                ty: "array<slave_status{slave_idx:u8,state:u8,fault_code:u16}>",
            },
        ],
    },
    SchemaMessage {
        type_tag: 0x0044,
        name: "QueryMotorState",
        version: 1,
        channel: "control",
        fields: &[],
    },
    SchemaMessage {
        type_tag: 0x0045,
        name: "MotorStateResponse",
        version: 1,
        channel: "control",
        fields: &[
            SchemaField {
                name: "motors",
                ty: "array<motor_sample{slot:u8,pos_q16:i32,vel_q16:i32}>",
            },
        ],
    },
    SchemaMessage {
        type_tag: 0x0060,
        name: "PushPieces",
        version: 4,
        channel: "pieces",
        fields: &[
            SchemaField { name: "axis_count", ty: "u8" },
            SchemaField {
                name: "axes",
                ty: "array<axis_pieces{axis_idx:u8,piece_count:u8,start_slot:u16,new_head:u32,pieces:array<piece_entry{start_time:u64,duration:f32,motor_mask:u8,coeff_count:u8,reserved:u16,cheb_coeffs:array<f32;1..=8>}>}>",
            },
        ],
    },
    SchemaMessage {
        type_tag: 0x0061,
        name: "PushPiecesResponse",
        version: 3,
        channel: "control",
        fields: &[
            SchemaField { name: "result", ty: "i32" },
            SchemaField { name: "arrival_clock", ty: "u64" },
            SchemaField { name: "axis_count", ty: "u8" },
            SchemaField {
                name: "axes",
                ty: "array<axis_diag{axis_idx:u8,front_start_time:u64}>",
            },
        ],
    },
    SchemaMessage {
        type_tag: 0x0068,
        name: "StartCapture",
        version: 1,
        channel: "control",
        fields: &[
            SchemaField { name: "path", ty: "string" },
            SchemaField { name: "started_utc", ty: "string" },
            SchemaField {
                name: "drives",
                ty: "array<capture_drive{slot:u8,name:string}>",
            },
        ],
    },
    SchemaMessage {
        type_tag: 0x0069,
        name: "StartCaptureResponse",
        version: 1,
        channel: "control",
        fields: &[
            SchemaField { name: "result", ty: "i32" },
        ],
    },
    SchemaMessage {
        type_tag: 0x006A,
        name: "StopCapture",
        version: 1,
        channel: "control",
        fields: &[],
    },
    SchemaMessage {
        type_tag: 0x006B,
        name: "StopCaptureResponse",
        version: 1,
        channel: "control",
        fields: &[
            SchemaField { name: "result", ty: "i32" },
            SchemaField { name: "samples", ty: "u64" },
            SchemaField { name: "overflow_cycle", ty: "u64" },
        ],
    },
    SchemaMessage {
        type_tag: 0x006C,
        name: "ResonanceBuzz",
        version: 1,
        channel: "control",
        fields: &[
            SchemaField { name: "axis_mask", ty: "u8" },
            SchemaField { name: "sign_mask", ty: "u8" },
            SchemaField { name: "freq_start_millihz", ty: "u32" },
            SchemaField { name: "freq_end_millihz", ty: "u32" },
            SchemaField { name: "amplitude_nm", ty: "u32" },
            SchemaField { name: "duration_ms", ty: "u32" },
            SchemaField { name: "ramp_ms", ty: "u32" },
        ],
    },
    SchemaMessage {
        type_tag: 0x006D,
        name: "ResonanceBuzzResponse",
        version: 1,
        channel: "control",
        fields: &[
            SchemaField { name: "result", ty: "i32" },
        ],
    },
    SchemaMessage {
        type_tag: 0x006E,
        name: "ArmSensorlessEndstop",
        version: 1,
        channel: "control",
        fields: &[
            SchemaField { name: "slot", ty: "u8" },
            SchemaField { name: "endstop_id", ty: "u8" },
            SchemaField {
                name: "torque_trip_tenth_pct",
                ty: "u16",
            },
            SchemaField { name: "enable", ty: "u8" },
        ],
    },
    SchemaMessage {
        type_tag: 0x006F,
        name: "ArmSensorlessEndstopResponse",
        version: 1,
        channel: "control",
        fields: &[
            SchemaField { name: "result", ty: "i32" },
        ],
    },
    SchemaMessage {
        type_tag: 0x0070,
        name: "SetTorque",
        version: 1,
        channel: "control",
        fields: &[
            SchemaField { name: "value", ty: "u8" },
            SchemaField { name: "execute_at_ns", ty: "u64" },
        ],
    },
    SchemaMessage {
        type_tag: 0x0071,
        name: "SetTorqueResponse",
        version: 1,
        channel: "control",
        fields: &[
            SchemaField { name: "result", ty: "i32" },
        ],
    },
    SchemaMessage {
        type_tag: 0x0072,
        name: "Stop",
        version: 1,
        channel: "control",
        fields: &[],
    },
    SchemaMessage {
        type_tag: 0x0073,
        name: "StopResponse",
        version: 2,
        channel: "control",
        fields: &[
            SchemaField { name: "result", ty: "i32" },
            SchemaField { name: "discard_clock", ty: "u64" },
        ],
    },
    SchemaMessage {
        type_tag: 0x0074,
        name: "SetDriveLimits",
        version: 1,
        channel: "control",
        fields: &[
            SchemaField { name: "slot", ty: "u8" },
            SchemaField { name: "following_error_counts", ty: "u32" },
            SchemaField { name: "max_torque_tenth_pct", ty: "u16" },
        ],
    },
    SchemaMessage {
        type_tag: 0x0075,
        name: "SetDriveLimitsResponse",
        version: 1,
        channel: "control",
        fields: &[
            SchemaField { name: "result", ty: "i32" },
        ],
    },
    SchemaMessage {
        type_tag: 0x0076,
        name: "RestoreDriveLimits",
        version: 1,
        channel: "control",
        fields: &[
            SchemaField { name: "slot", ty: "u8" },
        ],
    },
    SchemaMessage {
        type_tag: 0x0077,
        name: "RestoreDriveLimitsResponse",
        version: 1,
        channel: "control",
        fields: &[
            SchemaField { name: "result", ty: "i32" },
        ],
    },
    SchemaMessage {
        type_tag: 0x0078,
        name: "ResumeStream",
        version: 1,
        channel: "control",
        fields: &[],
    },
    SchemaMessage {
        type_tag: 0x0079,
        name: "ResumeStreamResponse",
        version: 1,
        channel: "control",
        fields: &[
            SchemaField { name: "result", ty: "i32" },
        ],
    },
    SchemaMessage {
        type_tag: 0x007A,
        name: "SeedServoHome",
        version: 1,
        channel: "control",
        fields: &[
            SchemaField { name: "slot", ty: "u8" },
            SchemaField { name: "home_q16", ty: "i32" },
        ],
    },
    SchemaMessage {
        type_tag: 0x007B,
        name: "SeedServoHomeResponse",
        version: 1,
        channel: "control",
        fields: &[
            SchemaField { name: "result", ty: "i32" },
        ],
    },
    SchemaMessage {
        type_tag: 0x007C,
        name: "SdoRead",
        version: 1,
        channel: "control",
        fields: &[
            SchemaField { name: "slot", ty: "u8" },
            SchemaField { name: "index", ty: "u16" },
            SchemaField { name: "subindex", ty: "u8" },
        ],
    },
    SchemaMessage {
        type_tag: 0x007D,
        name: "SdoReadResponse",
        version: 1,
        channel: "control",
        fields: &[
            SchemaField { name: "result", ty: "i32" },
            SchemaField { name: "size", ty: "u8" },
            SchemaField { name: "data", ty: "u8[4]" },
        ],
    },
    SchemaMessage {
        type_tag: 0x007E,
        name: "SdoWrite",
        version: 1,
        channel: "control",
        fields: &[
            SchemaField { name: "slot", ty: "u8" },
            SchemaField { name: "index", ty: "u16" },
            SchemaField { name: "subindex", ty: "u8" },
            SchemaField { name: "size", ty: "u8" },
            SchemaField { name: "value", ty: "i64" },
        ],
    },
    SchemaMessage {
        type_tag: 0x007F,
        name: "SdoWriteResponse",
        version: 1,
        channel: "control",
        fields: &[
            SchemaField { name: "result", ty: "i32" },
            SchemaField { name: "readback_size", ty: "u8" },
            SchemaField { name: "readback_data", ty: "u8[4]" },
        ],
    },
    SchemaMessage {
        type_tag: 0x0082,
        name: "FaultEvent",
        version: 1,
        channel: "events",
        fields: &[
            SchemaField { name: "fault_code", ty: "u16" },
            SchemaField { name: "fault_detail", ty: "u32" },
            SchemaField { name: "segment_id", ty: "u32" },
        ],
    },
    SchemaMessage {
        type_tag: 0x0083,
        name: "StatusHeartbeat",
        version: 1,
        channel: "events",
        fields: &[
            SchemaField { name: "engine_state", ty: "u8" },
            SchemaField { name: "fault_code", ty: "u16" },
            SchemaField { name: "num_axes", ty: "u8" },
            SchemaField { name: "retired_counts", ty: "array<u32>" },
            SchemaField { name: "ff_saturation_count", ty: "u32" },
        ],
    },
    SchemaMessage {
        type_tag: 0x0084,
        name: "McuLog",
        version: 1,
        channel: "events",
        fields: &[
            SchemaField { name: "mcu_tick", ty: "u64" },
            SchemaField { name: "level", ty: "u8" },
            SchemaField { name: "subsystem", ty: "u8" },
            SchemaField { name: "event", ty: "u16" },
            SchemaField { name: "code", ty: "u16" },
            SchemaField { name: "seq", ty: "u16" },
            SchemaField { name: "args", ty: "u32[2]" },
        ],
    },
    SchemaMessage {
        type_tag: 0x0085,
        name: "EndstopTrip",
        version: 1,
        channel: "events",
        fields: &[
            SchemaField { name: "endstop_id", ty: "u8" },
            SchemaField { name: "trip_clock", ty: "u64" },
        ],
    },
    SchemaMessage {
        type_tag: 0x0086,
        name: "SetStrainComp",
        version: 2,
        channel: "control",
        fields: &[
            SchemaField { name: "slot_a", ty: "u8" },
            SchemaField { name: "slot_b", ty: "u8" },
            SchemaField { name: "lane_a", ty: "u8" },
            SchemaField { name: "lane_b", ty: "u8" },
            SchemaField { name: "kinematics", ty: "u8" },
            SchemaField { name: "nx", ty: "u16" },
            SchemaField { name: "ny", ty: "u16" },
            SchemaField { name: "x0", ty: "f32" },
            SchemaField { name: "y0", ty: "f32" },
            SchemaField { name: "dx", ty: "f32" },
            SchemaField { name: "dy", ty: "f32" },
            SchemaField { name: "value_count", ty: "u32" },
            SchemaField { name: "values_um", ty: "array<i32>" },
        ],
    },
    SchemaMessage {
        type_tag: 0x0087,
        name: "SetStrainCompResponse",
        version: 1,
        channel: "control",
        fields: &[
            SchemaField { name: "result", ty: "i32" },
        ],
    },
    SchemaMessage {
        type_tag: 0x0088,
        name: "SetDiffDamper",
        version: 2,
        channel: "control",
        fields: &[
            SchemaField { name: "slot_a", ty: "u8" },
            SchemaField { name: "slot_b", ty: "u8" },
            SchemaField { name: "gain_milli", ty: "u32" },
            SchemaField { name: "clamp_tenths", ty: "u16" },
            SchemaField { name: "lpf_millihz", ty: "u32" },
            SchemaField { name: "lead_us", ty: "u16" },
        ],
    },
    SchemaMessage {
        type_tag: 0x0089,
        name: "SetDiffDamperResponse",
        version: 1,
        channel: "control",
        fields: &[
            SchemaField { name: "result", ty: "i32" },
        ],
    },
    SchemaMessage {
        type_tag: 0x008A,
        name: "SetDiffTrim",
        version: 1,
        channel: "control",
        fields: &[
            SchemaField { name: "slot_a", ty: "u8" },
            SchemaField { name: "slot_b", ty: "u8" },
            SchemaField { name: "gain_micro", ty: "u32" },
            SchemaField { name: "clamp_um", ty: "u16" },
            SchemaField { name: "lpf_millihz", ty: "u32" },
        ],
    },
    SchemaMessage {
        type_tag: 0x008B,
        name: "SetDiffTrimResponse",
        version: 1,
        channel: "control",
        fields: &[
            SchemaField { name: "result", ty: "i32" },
        ],
    },
    SchemaMessage {
        type_tag: 0x008C,
        name: "SetDynamicsModel",
        version: 3,
        channel: "control",
        fields: &[
            SchemaField { name: "slots_count", ty: "u8" },
            SchemaField { name: "modes_count", ty: "u8" },
            SchemaField { name: "frame", ty: "array<f32;slots_count*modes_count>" },
            SchemaField { name: "mass", ty: "array<f32>" },
            SchemaField { name: "viscous", ty: "array<f32>" },
            SchemaField { name: "coulomb", ty: "array<f32>" },
            SchemaField { name: "pairs_count", ty: "u8" },
            SchemaField { name: "pairs", ty: "array<{first:u8,second:u8,w:f32[6]}>" },
        ],
    },
    SchemaMessage {
        type_tag: 0x008D,
        name: "SetDynamicsModelResponse",
        version: 1,
        channel: "control",
        fields: &[
            SchemaField { name: "result", ty: "i32" },
        ],
    },
];

/// Bootstrap type tags that the C header must define alongside the schema
/// messages. Bootstrap tags are NOT part of `schema_hash`.
#[allow(dead_code)] // used by build.rs; unused by the schema_hash integration test
const BOOTSTRAP_TAGS: &[(u16, &str)] =
    &[(0x0001, "Identify"), (0x0002, "IdentifyResponse")];

/// Canonical text form. One line per message:
///
///     0xTTTT:NAME:vNN:CHAN:[field1:type1,field2:type2,...]\n
///
/// Hex tag is lowercase, zero-padded to 4 hex digits. Version is `v` + decimal
/// (no padding). Bootstrap messages are excluded.
fn canonicalize_schema(messages: &[SchemaMessage]) -> String {
    let mut out = String::new();
    for m in messages {
        out.push_str(&format!("0x{:04x}:{}:v{}:{}:[", m.type_tag, m.name, m.version, m.channel));
        for (i, f) in m.fields.iter().enumerate() {
            if i > 0 {
                out.push(',');
            }
            out.push_str(f.name);
            out.push(':');
            out.push_str(f.ty);
        }
        out.push_str("]\n");
    }
    out
}
