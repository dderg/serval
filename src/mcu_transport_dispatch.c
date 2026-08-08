#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "mcu_transport_dispatch.h"
#include "mcu_demux.h"
#include "mcu_protocol_schema.h"
#include "board/misc.h"
#include "board/irq.h"
#include "sched.h"
#include "autoconf.h"
#include "stepper.h"

#if CONFIG_MOTION_RUNTIME
#include "runtime.h"
extern void *runtime_handle;
#endif

#if CONFIG_CLASSIC_STEPPING
extern uint64_t runtime_widened_host_clock(void);
#endif

extern int kalico_console_write_raw(const uint8_t *buf, uint16_t len);

#define MCU_FRAME_SYNC 0x55

#define IDENTIFY_RESPONSE_BODY_LEN 81

// MCU_TX_BUF_SIZE, PER_MESSAGE_HEADER_LEN, MESSAGE_VERSION_DEFAULT and the
// RUNTIME_ERR_* codes live in mcu_transport_dispatch.h (shared with piece_sink).
static uint8_t tx_buf[MCU_TX_BUF_SIZE];

static uint32_t reset_epoch;

static void handle_query_runtime_caps(uint32_t correlation_id, const uint8_t *body, uint16_t body_len);
static void handle_query_motor_state(uint32_t correlation_id, const uint8_t *body, uint16_t body_len);
static void handle_stop(uint32_t correlation_id);
static void handle_resume_stream(uint32_t correlation_id);
#if CONFIG_MOTION_RUNTIME
static void handle_stepper_suppress(uint32_t correlation_id,
                                     const uint8_t *body, uint16_t body_len);
#endif

#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <unistd.h>
static void
read_random_u32(uint32_t *out)
{
    int fd = open("/dev/urandom", O_RDONLY);
    if (fd < 0) {
        *out = 0;
        return;
    }
    uint32_t v = 0;
    ssize_t n = read(fd, &v, sizeof(v));
    close(fd);
    if (n != (ssize_t)sizeof(v))
        v = 0;
    *out = v;
}
#else
static void
read_random_u32(uint32_t *out)
{
    *out = 0xA5A5A5A5;
}
#endif

void
kalico_reset_epoch_init(void)
{
    uint32_t v = 0;
    int spins = 0;
    do {
        read_random_u32(&v);
        spins++;
    } while (v == 0 && spins < 8);
    if (v == 0)
        v = 1; // reset_epoch must never be zero
    reset_epoch = v;
}
DECL_INIT(kalico_reset_epoch_init);

int
mcu_transport_send_frame(uint8_t channel, const uint8_t *payload,
                            uint16_t payload_len)
{
    // len field covers [len .. crc] inclusive = 2 + 1 + payload + 2.
    uint32_t len_field = 2u + 1u + (uint32_t)payload_len + 2u;
    uint32_t total = 1u + len_field;
    if (total > MCU_TX_BUF_SIZE)
        return -1;
    tx_buf[0] = MCU_FRAME_SYNC;
    tx_buf[1] = (uint8_t)(len_field & 0xFF);
    tx_buf[2] = (uint8_t)((len_field >> 8) & 0xFF);
    tx_buf[3] = channel;
    if (payload_len > 0)
        memcpy(&tx_buf[4], payload, payload_len);
    uint16_t crc = crc16_ccitt(&tx_buf[1], (uint32_t)(2 + 1 + payload_len));
    tx_buf[total - 2] = (uint8_t)(crc & 0xFF);
    tx_buf[total - 1] = (uint8_t)((crc >> 8) & 0xFF);
    return kalico_console_write_raw(tx_buf, (uint16_t)total);
}

void
encode_message_header(uint8_t *out, uint16_t kind, uint8_t version,
                      uint32_t correlation_id)
{
    out[0] = (uint8_t)(kind & 0xFF);
    out[1] = (uint8_t)((kind >> 8) & 0xFF);
    out[2] = version;
    out[3] = (uint8_t)(correlation_id & 0xFF);
    out[4] = (uint8_t)((correlation_id >> 8) & 0xFF);
    out[5] = (uint8_t)((correlation_id >> 16) & 0xFF);
    out[6] = (uint8_t)((correlation_id >> 24) & 0xFF);
}

static void
handle_identify(uint32_t correlation_id, const uint8_t *body, uint16_t body_len)
{
    if (body_len != 1)
        return;
    uint8_t proto_version = body[0];
    if (proto_version != MCU_PROTO_VERSION) {
        return;
    }

    uint8_t payload[PER_MESSAGE_HEADER_LEN + IDENTIFY_RESPONSE_BODY_LEN];
    encode_message_header(payload, MCU_MSG_IDENTIFY_RESPONSE,
                          0x01, correlation_id);
    uint8_t *body_out = &payload[PER_MESSAGE_HEADER_LEN];

    // IdentifyResponse body layout (offsets, must match host decode):
    //   0  proto_version u8 | 1  firmware_ver u32_le | 5  build_hash [u8;20]
    //   25 schema_hash [u8;32] | 57 reset_epoch u32_le | 61 capabilities u64_le
    //   69 mcu_serial [u8;12]
    body_out[0] = MCU_PROTO_VERSION;
    uint32_t fw = 0x00000001;
    body_out[1] = (uint8_t)(fw & 0xFF);
    body_out[2] = (uint8_t)((fw >> 8) & 0xFF);
    body_out[3] = (uint8_t)((fw >> 16) & 0xFF);
    body_out[4] = (uint8_t)((fw >> 24) & 0xFF);
    memset(&body_out[5], 0, 20);
    memcpy(&body_out[25], MCU_SCHEMA_HASH, 32);
    uint32_t epoch = reset_epoch;
    body_out[57] = (uint8_t)(epoch & 0xFF);
    body_out[58] = (uint8_t)((epoch >> 8) & 0xFF);
    body_out[59] = (uint8_t)((epoch >> 16) & 0xFF);
    body_out[60] = (uint8_t)((epoch >> 24) & 0xFF);
    // capabilities bit 0 = PHASE_STEPPING_CAPABLE. A classic-stepping build
    // has no MCU-side motion runtime and advertises no motion capability.
    memset(&body_out[61], 0, 8);
    body_out[61] = CONFIG_MOTION_RUNTIME ? 0x01 : 0x00;
    memset(&body_out[69], 0, 12);

    mcu_transport_send_frame(MCU_CHANNEL_CONTROL,
                                payload, sizeof(payload));
}

void
mcu_transport_dispatch_frame(uint8_t channel, const uint8_t *payload,
                      uint16_t payload_len)
{
    extern void runtime_diag_progress(uint32_t tag, uint32_t stage, uint32_t value);
    (void)channel;
    if (payload_len < PER_MESSAGE_HEADER_LEN) {
        runtime_diag_progress(0xCD, 1, payload_len);
        return;
    }
    uint16_t kind = (uint16_t)payload[0] | ((uint16_t)payload[1] << 8);
    runtime_diag_progress(0xCD, 2 + (uint32_t)kind, (uint32_t)payload_len);
    uint8_t version = payload[2];
    uint32_t correlation_id = (uint32_t)payload[3]
                            | ((uint32_t)payload[4] << 8)
                            | ((uint32_t)payload[5] << 16)
                            | ((uint32_t)payload[6] << 24);
    const uint8_t *body = &payload[PER_MESSAGE_HEADER_LEN];
    uint16_t body_len = payload_len - PER_MESSAGE_HEADER_LEN;
    (void)version;

    switch (kind) {
    case MCU_MSG_IDENTIFY:
        handle_identify(correlation_id, body, body_len);
        return;
    case MCU_MSG_QUERY_RUNTIME_CAPS:
        handle_query_runtime_caps(correlation_id, body, body_len);
        return;
    case MCU_MSG_QUERY_MOTOR_STATE:
        handle_query_motor_state(correlation_id, body, body_len);
        return;
    case MCU_MSG_STOP:
        handle_stop(correlation_id);
        return;
    case MCU_MSG_RESUME_STREAM:
        handle_resume_stream(correlation_id);
        return;
#if CONFIG_MOTION_RUNTIME
    case MCU_MSG_STEPPER_SUPPRESS:
        handle_stepper_suppress(correlation_id, body, body_len);
        return;
#endif
    default:
        return;
    }
}

extern uint32_t stats_send_time;
extern uint32_t stats_send_time_high;
uint32_t timer_read_time(void);

static void
handle_query_runtime_caps(uint32_t correlation_id, const uint8_t *body,
                          uint16_t body_len)
{
    (void)body;
    (void)body_len;
    uint8_t payload[PER_MESSAGE_HEADER_LEN + 4];
    encode_message_header(payload, MCU_MSG_RUNTIME_CAPS_RESPONSE,
                          MESSAGE_VERSION_DEFAULT, correlation_id);
    uint8_t *b = &payload[PER_MESSAGE_HEADER_LEN];
    // u32 total_piece_memory (bytes); host divides by 32 (PieceEntry size)
    // and axis count for per-axis ring depth. Zero on a classic-stepping
    // build: there is no piece ring to fill.
#if CONFIG_MOTION_RUNTIME
    uint32_t total_piece_memory = (uint32_t)CONFIG_RUNTIME_PIECE_RING_SIZE;
#else
    uint32_t total_piece_memory = 0;
#endif
    b[0] = (uint8_t)(total_piece_memory & 0xFF);
    b[1] = (uint8_t)((total_piece_memory >> 8) & 0xFF);
    b[2] = (uint8_t)((total_piece_memory >> 16) & 0xFF);
    b[3] = (uint8_t)((total_piece_memory >> 24) & 0xFF);
    mcu_transport_send_frame(MCU_CHANNEL_CONTROL,
                                payload, sizeof(payload));
}

// MotorStateResponse body (must match Rust decode):
//   count u8 | count * [slot u8 | pos_q16 i32_le | vel_q16 i32_le] (9 bytes).
#define MCU_MOTOR_STATE_MAX_AXES  8u
#define MCU_MOTOR_STATE_ENTRY_LEN 9u
static void
handle_query_motor_state(uint32_t correlation_id, const uint8_t *body,
                         uint16_t body_len)
{
    (void)body;
    (void)body_len;
    uint8_t slots[MCU_MOTOR_STATE_MAX_AXES];
    int32_t pos[MCU_MOTOR_STATE_MAX_AXES];
    int32_t vel[MCU_MOTOR_STATE_MAX_AXES];
    int n = 0;
#if CONFIG_MOTION_RUNTIME
    if (runtime_handle)
        n = runtime_query_motor_state(runtime_handle, slots, pos, vel,
                                             MCU_MOTOR_STATE_MAX_AXES);
#endif
    if (n < 0)
        n = 0;
    uint8_t payload[PER_MESSAGE_HEADER_LEN + 1
                    + MCU_MOTOR_STATE_MAX_AXES * MCU_MOTOR_STATE_ENTRY_LEN];
    encode_message_header(payload, MCU_MSG_MOTOR_STATE_RESPONSE,
                          MESSAGE_VERSION_DEFAULT, correlation_id);
    uint8_t *b = &payload[PER_MESSAGE_HEADER_LEN];
    b[0] = (uint8_t)n;
    uint8_t *p = &b[1];
    for (int i = 0; i < n; i++) {
        *p++ = slots[i];
        *p++ = (uint8_t)(pos[i] & 0xFF);
        *p++ = (uint8_t)((pos[i] >> 8) & 0xFF);
        *p++ = (uint8_t)((pos[i] >> 16) & 0xFF);
        *p++ = (uint8_t)((pos[i] >> 24) & 0xFF);
        *p++ = (uint8_t)(vel[i] & 0xFF);
        *p++ = (uint8_t)((vel[i] >> 8) & 0xFF);
        *p++ = (uint8_t)((vel[i] >> 16) & 0xFF);
        *p++ = (uint8_t)((vel[i] >> 24) & 0xFF);
    }
    uint16_t used = (uint16_t)(PER_MESSAGE_HEADER_LEN + 1
                               + n * MCU_MOTOR_STATE_ENTRY_LEN);
    mcu_transport_send_frame(MCU_CHANNEL_CONTROL, payload, used);
}

// The PushPieces piece_sink (multi-axis streaming parser + frame-level response)
// lives in src/piece_sink.c so it is host-fuzzable in isolation; it builds on
// the transport seam declared in mcu_transport_dispatch.h.

void
send_status_heartbeat(void)
{
#if CONFIG_MOTION_RUNTIME
    if (!runtime_handle)
        return;

    uint8_t st = 0;
    uint16_t fc = 0;
    uint32_t counts[8];
    int32_t n = runtime_get_heartbeat(runtime_handle,
                                             &st, &fc, counts, 8);
    if (n < 0)
        return;
#else
    uint8_t st = 0;
    uint16_t fc = 0;
    uint32_t counts[8];
    int32_t n = 0;
#endif

    uint8_t payload[MCU_TX_BUF_SIZE];
    int off = 0;
    payload[off++] = (uint8_t)(MCU_MSG_STATUS_HEARTBEAT & 0xFF);
    payload[off++] = (uint8_t)((MCU_MSG_STATUS_HEARTBEAT >> 8) & 0xFF);
    payload[off++] = MESSAGE_VERSION_DEFAULT;
    payload[off++] = 0;
    payload[off++] = 0;
    payload[off++] = 0;
    payload[off++] = 0;
    payload[off++] = st;
    payload[off++] = (uint8_t)(fc & 0xFF);
    payload[off++] = (uint8_t)((fc >> 8) & 0xFF);
    payload[off++] = (uint8_t)n;
    for (int i = 0; i < n; i++) {
        uint32_t v = counts[i];
        payload[off++] = (uint8_t)(v & 0xFF);
        payload[off++] = (uint8_t)((v >> 8) & 0xFF);
        payload[off++] = (uint8_t)((v >> 16) & 0xFF);
        payload[off++] = (uint8_t)((v >> 24) & 0xFF);
    }
    uint32_t ff_saturation_count = 0;
    payload[off++] = (uint8_t)(ff_saturation_count & 0xFF);
    payload[off++] = (uint8_t)((ff_saturation_count >> 8) & 0xFF);
    payload[off++] = (uint8_t)((ff_saturation_count >> 16) & 0xFF);
    payload[off++] = (uint8_t)((ff_saturation_count >> 24) & 0xFF);
    mcu_transport_send_frame(MCU_CHANNEL_CONTROL, payload, (uint16_t)off);
}

void
mcu_transport_emit_fault_event(uint16_t fault_code, uint32_t fault_detail,
                               uint32_t segment_id)
{
    uint8_t payload[PER_MESSAGE_HEADER_LEN + 10];
    encode_message_header(payload, MCU_MSG_FAULT_EVENT,
                          MESSAGE_VERSION_DEFAULT, 0);
    uint8_t *b = &payload[PER_MESSAGE_HEADER_LEN];
    b[0] = (uint8_t)(fault_code & 0xFF);
    b[1] = (uint8_t)((fault_code >> 8) & 0xFF);
    b[2] = (uint8_t)(fault_detail & 0xFF);
    b[3] = (uint8_t)((fault_detail >> 8) & 0xFF);
    b[4] = (uint8_t)((fault_detail >> 16) & 0xFF);
    b[5] = (uint8_t)((fault_detail >> 24) & 0xFF);
    b[6] = (uint8_t)(segment_id & 0xFF);
    b[7] = (uint8_t)((segment_id >> 8) & 0xFF);
    b[8] = (uint8_t)((segment_id >> 16) & 0xFF);
    b[9] = (uint8_t)((segment_id >> 24) & 0xFF);
    mcu_transport_send_frame(MCU_CHANNEL_EVENTS, payload, sizeof(payload));
}

void
mcu_transport_emit_endstop_trip(uint8_t endstop_id, uint64_t trip_clock)
{
    uint8_t payload[PER_MESSAGE_HEADER_LEN + 9];
    encode_message_header(payload, MCU_MSG_ENDSTOP_TRIP,
                          MESSAGE_VERSION_DEFAULT, 0);
    uint8_t *b = &payload[PER_MESSAGE_HEADER_LEN];
    b[0] = endstop_id;
    for (int i = 0; i < 8; i++)
        b[1 + i] = (uint8_t)((trip_clock >> (8 * i)) & 0xFF);
    mcu_transport_send_frame(MCU_CHANNEL_EVENTS, payload, sizeof(payload));
}

static void
send_stop_response(uint32_t correlation_id, int32_t result, uint64_t discard_clock)
{
    uint8_t payload[PER_MESSAGE_HEADER_LEN + 12];
    encode_message_header(payload, MCU_MSG_STOP_RESPONSE,
                          MESSAGE_VERSION_DEFAULT, correlation_id);
    uint8_t *b = &payload[PER_MESSAGE_HEADER_LEN];
    b[0] = (uint8_t)(result & 0xFF);
    b[1] = (uint8_t)((result >> 8) & 0xFF);
    b[2] = (uint8_t)((result >> 16) & 0xFF);
    b[3] = (uint8_t)((result >> 24) & 0xFF);
    for (int i = 0; i < 8; i++)
        b[4 + i] = (uint8_t)((discard_clock >> (8 * i)) & 0xFF);
    mcu_transport_send_frame(MCU_CHANNEL_CONTROL, payload, sizeof(payload));
}

// Motion halts at the FIRST gate — the endstop trip task gates locally
// before the host's Stop broadcast arrives — so the discard clock is
// latched then and re-reported for any repeat Stop while still gated.
// Stamping each Stop with a fresh clock would place the halt several ms
// of travel past where the steppers actually stopped, and every probe
// seeds the toolhead frame from the position reconstructed at this clock.
#if CONFIG_MOTION_RUNTIME || CONFIG_CLASSIC_STEPPING
static uint64_t stop_halt_clock;
#endif
#if CONFIG_CLASSIC_STEPPING
static uint8_t stop_gated;
#endif

#if CONFIG_CLASSIC_STEPPING
void
classic_stop_gate_at(uint64_t halt_clock)
{
    irqstatus_t flag = irq_save();
    if (!stop_gated) {
        stop_gated = 1;
        uint32_t stream_end = 0;
        if (stepper_classic_halt_all(&stream_end)) {
            int32_t delta = (int32_t)(stream_end - (uint32_t)halt_clock);
            if (delta < 0)
                halt_clock = halt_clock + (int64_t)delta;
        }
        stop_halt_clock = halt_clock;
    }
    irq_restore(flag);
}
#endif

int32_t
handle_stop_inner(uint64_t *discard_clock)
{
    *discard_clock = 0;
#if CONFIG_MOTION_RUNTIME
    int32_t rc = RUNTIME_ERR_NOT_INIT;
    if (runtime_handle) {
        irqstatus_t flag = irq_save();
        int32_t was_gated = runtime_pieces_gated(runtime_handle);
        rc = runtime_gate_pieces(runtime_handle);
        if (was_gated <= 0)
            stop_halt_clock = runtime_now_ticks(runtime_handle);
        *discard_clock = stop_halt_clock;
        irq_restore(flag);
    }
#elif CONFIG_CLASSIC_STEPPING
    // The host-computed step stream has no MCU-side gate to close: the
    // halt IS discarding every queued move, and SF_NEED_RESET keeps later
    // queue_step frames out until the host re-anchors with reset_step_clock.
    int32_t rc = 0;
    classic_stop_gate_at(runtime_widened_host_clock());
    irqstatus_t flag = irq_save();
    *discard_clock = stop_halt_clock;
    irq_restore(flag);
#else
    int32_t rc = RUNTIME_ERR_MOTION_RUNTIME_ABSENT;
#endif
    return rc;
}

static void
handle_stop(uint32_t correlation_id)
{
    uint64_t discard_clock = 0;
    int32_t rc = handle_stop_inner(&discard_clock);
    send_stop_response(correlation_id, rc, discard_clock);
}

static void
send_resume_stream_response(uint32_t correlation_id, int32_t result)
{
    uint8_t payload[PER_MESSAGE_HEADER_LEN + 4];
    encode_message_header(payload, MCU_MSG_RESUME_STREAM_RESPONSE,
                          MESSAGE_VERSION_DEFAULT, correlation_id);
    uint8_t *b = &payload[PER_MESSAGE_HEADER_LEN];
    b[0] = (uint8_t)(result & 0xFF);
    b[1] = (uint8_t)((result >> 8) & 0xFF);
    b[2] = (uint8_t)((result >> 16) & 0xFF);
    b[3] = (uint8_t)((result >> 24) & 0xFF);
    mcu_transport_send_frame(MCU_CHANNEL_CONTROL, payload, sizeof(payload));
}

#if CONFIG_MOTION_RUNTIME
static void
send_stepper_suppress_response(uint32_t correlation_id, int32_t result)
{
    uint8_t payload[PER_MESSAGE_HEADER_LEN + 4];
    encode_message_header(payload, MCU_MSG_STEPPER_SUPPRESS_RESPONSE,
                          MESSAGE_VERSION_DEFAULT, correlation_id);
    uint8_t *b = &payload[PER_MESSAGE_HEADER_LEN];
    b[0] = (uint8_t)(result & 0xFF);
    b[1] = (uint8_t)((result >> 8) & 0xFF);
    b[2] = (uint8_t)((result >> 16) & 0xFF);
    b[3] = (uint8_t)((result >> 24) & 0xFF);
    mcu_transport_send_frame(MCU_CHANNEL_CONTROL, payload, sizeof(payload));
}

static void
handle_stepper_suppress(uint32_t correlation_id, const uint8_t *body,
                        uint16_t body_len)
{
    if (body_len < 3)
        shutdown("suppress body truncated");
    if (body[0] == 0xFF && body[1] == 0xFF && !body[2])
        stepper_suppress_clear_all();
    else
        stepper_suppress_set(body[0], body[1]);
    send_stepper_suppress_response(correlation_id, 0);
}
#endif

static void
handle_resume_stream(uint32_t correlation_id)
{
    stepper_suppress_clear_all();
#if CONFIG_MOTION_RUNTIME
    int32_t rc = RUNTIME_ERR_NOT_INIT;
    if (runtime_handle) {
        irqstatus_t flag = irq_save();
        rc = runtime_ungate_pieces(runtime_handle);
        irq_restore(flag);
    }
#elif CONFIG_CLASSIC_STEPPING
    int32_t rc = 0;
    irqstatus_t flag = irq_save();
    stop_gated = 0;
    irq_restore(flag);
#else
    int32_t rc = RUNTIME_ERR_MOTION_RUNTIME_ABSENT;
#endif
    send_resume_stream_response(correlation_id, rc);
}
