// Klipper wire contract for the sample-stream transport.
//
// These argstrings are the single source of truth for both sides: the host
// endpoint sends them by name, and `src/sample_wire.h` mirrors them so the
// MCU's `DECL_COMMAND` strings cannot drift. A mismatch is a dictionary
// lookup failure at connect, which is exactly when we want to hear about it.
//
// The stream is: one `SAMPLE_ANCHOR` to place the lane in absolute terms, then
// `SAMPLE_RUN` payloads that abut it exactly. `SAMPLE_OVERLAY` carries an
// additive nudge run on the same clock grid, anchored on itself.
//
// `SAMPLE_BARRIER` fences the runs pushed on a lane so far. The mcu latches
// the fence clock at push time and returns `SAMPLE_BARRIER_ACK` once playback
// has consumed past it, which is what lets the host read a lane's position
// back without racing the executor: a re-anchor cut whose samples already
// reached the wire fences, waits for the receipt, then reconciles against
// `SAMPLE_GET_POSITION`.
//
// WIRE-STABLE: the argstrings are the protocol. Extend by adding commands.

pub const SAMPLE_ANCHOR: &str = "sample_anchor oid=%c clock=%u position=%i";
pub const SAMPLE_RUN: &str = "sample_run oid=%c interval=%u count=%c data=%*s";
pub const SAMPLE_OVERLAY: &str = "sample_overlay oid=%c clock=%u interval=%u count=%c data=%*s";
pub const SAMPLE_BARRIER: &str = "sample_barrier oid=%c seq=%u";
pub const SAMPLE_BARRIER_ACK: &str = "sample_barrier_ack oid=%c seq=%u";
pub const SAMPLE_GET_POSITION: &str = "sample_get_position oid=%c";
pub const SAMPLE_POSITION: &str = "sample_position oid=%c clock=%u position=%i";

pub const SAMPLE_ANCHOR_NAME: &str = "sample_anchor";
pub const SAMPLE_RUN_NAME: &str = "sample_run";
pub const SAMPLE_OVERLAY_NAME: &str = "sample_overlay";
pub const SAMPLE_BARRIER_NAME: &str = "sample_barrier";
pub const SAMPLE_BARRIER_ACK_NAME: &str = "sample_barrier_ack";
pub const SAMPLE_GET_POSITION_NAME: &str = "sample_get_position";
pub const SAMPLE_POSITION_NAME: &str = "sample_position";

/// Every argstring the transport declares, in the order the header mirrors
/// them. Host-side dictionary checks walk this.
pub const SAMPLE_COMMANDS: [&str; 7] = [
    SAMPLE_ANCHOR,
    SAMPLE_RUN,
    SAMPLE_OVERLAY,
    SAMPLE_BARRIER,
    SAMPLE_BARRIER_ACK,
    SAMPLE_GET_POSITION,
    SAMPLE_POSITION,
];

/// The leading token of an argstring: its command name.
pub fn command_name(argstring: &str) -> &str {
    match argstring.split_once(' ') {
        Some((name, _)) => name,
        None => argstring,
    }
}

#[cfg(test)]
#[path = "sample_wire_tests.rs"]
mod sample_wire_tests;
