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

#include "runtime.h"
extern void *runtime_handle;

extern int kalico_console_write_raw(const uint8_t *buf, uint16_t len);

#define MCU_FRAME_SYNC 0x55
#define MESSAGE_VERSION_DEFAULT 0x01

#define RUNTIME_ERR_INVALID_CURVE -2
#define RUNTIME_ERR_NOT_INIT      -7

// type:u16_le | version:u8 | corr_id:u32_le.
#define PER_MESSAGE_HEADER_LEN 7

#define IDENTIFY_RESPONSE_BODY_LEN 81

#define MCU_TX_BUF_SIZE 256
static uint8_t tx_buf[MCU_TX_BUF_SIZE];

static uint32_t reset_epoch;

static void handle_query_runtime_caps(uint32_t correlation_id, const uint8_t *body, uint16_t body_len);
static void handle_query_motor_state(uint32_t correlation_id, const uint8_t *body, uint16_t body_len);
static void handle_stop(uint32_t correlation_id);
static void handle_resume_stream(uint32_t correlation_id);

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

uint32_t
kalico_reset_epoch_get(void)
{
    return reset_epoch;
}

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

static void
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
    // capabilities bit 0 = PHASE_STEPPING_CAPABLE, advertised unconditionally.
    memset(&body_out[61], 0, 8);
    body_out[61] = 0x01;
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
    // and axis count for per-axis ring depth.
    uint32_t total_piece_memory = (uint32_t)CONFIG_RUNTIME_PIECE_RING_SIZE;
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
    if (runtime_handle)
        n = runtime_query_motor_state(runtime_handle, slots, pos, vel,
                                             MCU_MOTOR_STATE_MAX_AXES);
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

// Pieces wire layout streamed through piece_sink_feed (the sink sees only the
// CRC-covered payload; envelope + CRC are the demuxer's). Single-MCU frame:
//   per-message header (7): type u16_le | version u8 | corr_id u32_le
//   axis_count         (1): u8, number of axis blocks (1..MCU_MAX_FRAME_AXES)
//   then axis_count axis blocks, each:
//     axis block header(8): axis_idx u8 | piece_count u8 | start_slot u16_le
//                           | new_head u32_le
//     then piece_count entries of 32 bytes each.
// Each piece lands at (start_slot + index) % ring_depth for its axis; the
// per-axis frontier advances only in commit, after the demuxer validates CRC.
#define AXIS_BLOCK_HEADER_LEN 8u
#define PIECE_ENTRY_LEN       32u
// Upper bound on axis blocks per frame; sizes the per-axis commit/echo arrays.
// A frame declaring more blocks is rejected before any ring write.
#define MCU_MAX_FRAME_AXES    8u
// Bounds the write index against a malformed over-long block; such a frame is
// rejected anyway by the per-block piece-count self-check in piece_sink_commit.
#define PIECE_SINK_MAX_PIECES 0xFFu

// PushPiecesResponse body (must match Rust decode):
//   result i32_le | arrival_clock u64_le | axis_count u8
//   | axis_count * (axis_idx u8 | front_start_time u64_le).
// `axis_idx`/`first_start_time` may be NULL for an early error (axis_count=0).
#define PIECE_RESP_FIXED_LEN 13u
#define PIECE_RESP_PER_AXIS  9u
_Static_assert(PER_MESSAGE_HEADER_LEN + PIECE_RESP_FIXED_LEN
                   + MCU_MAX_FRAME_AXES * PIECE_RESP_PER_AXIS
               <= MCU_TX_BUF_SIZE,
               "PushPiecesResponse can overflow tx_buf");

static void
send_push_pieces_response(uint32_t correlation_id, int32_t result,
                          uint64_t arrival_clock, uint8_t axis_count,
                          const uint8_t *axis_idx,
                          const uint64_t *first_start_time)
{
    uint8_t payload[PER_MESSAGE_HEADER_LEN + PIECE_RESP_FIXED_LEN
                    + MCU_MAX_FRAME_AXES * PIECE_RESP_PER_AXIS];
    encode_message_header(payload, MCU_MSG_PUSH_PIECES_RESPONSE,
                          MESSAGE_VERSION_DEFAULT, correlation_id);
    uint8_t *b = &payload[PER_MESSAGE_HEADER_LEN];
    for (uint32_t j = 0; j < 4; j++)
        b[j] = (uint8_t)(((uint32_t)result >> (8 * j)) & 0xFF);
    for (uint32_t j = 0; j < 8; j++)
        b[4 + j] = (uint8_t)((arrival_clock >> (8 * j)) & 0xFF);
    uint8_t n = (axis_idx && first_start_time) ? axis_count : 0;
    if (n > MCU_MAX_FRAME_AXES)
        n = MCU_MAX_FRAME_AXES;
    b[12] = n;
    uint32_t off = PIECE_RESP_FIXED_LEN;
    for (uint8_t a = 0; a < n; a++) {
        b[off++] = axis_idx[a];
        for (uint32_t j = 0; j < 8; j++)
            b[off++] = (uint8_t)((first_start_time[a] >> (8 * j)) & 0xFF);
    }
    mcu_transport_send_frame(MCU_CHANNEL_CONTROL, payload,
                             (uint16_t)(PER_MESSAGE_HEADER_LEN + off));
}

// Single-threaded foreground (same context as mcu_demux_pump); no locking.
static struct {
    uint8_t  hdr[PER_MESSAGE_HEADER_LEN];
    uint8_t  blk[AXIS_BLOCK_HEADER_LEN];
    uint8_t  scratch[PIECE_ENTRY_LEN];
    uint32_t correlation_id;
    uint8_t  hdr_seen;        // bytes of the message header accumulated
    uint8_t  have_axis_count;
    uint8_t  axis_count;
    uint8_t  cur_axis;        // block index being parsed (0..axis_count)
    uint8_t  in_block_header;
    uint8_t  blk_seen;        // bytes of the current block header accumulated
    uint8_t  cur_axis_idx;
    uint8_t  cur_piece_count;
    uint16_t cur_start_slot;
    uint32_t cur_new_head;
    uint32_t cur_pieces_seen; // pieces of the current block written
    uint8_t  cur_piece_off;   // byte offset within the current piece
    uint8_t  axis_idx[MCU_MAX_FRAME_AXES];
    uint32_t new_head[MCU_MAX_FRAME_AXES];
    uint64_t first_start_time[MCU_MAX_FRAME_AXES];
    uint8_t  blocks_done;     // fully-parsed blocks
    int32_t  write_rc;
    uint8_t  malformed;       // bounds/duplicate violation -> reject in commit
} piece_sink;

void
piece_sink_begin(void)
{
    piece_sink.correlation_id = 0;
    piece_sink.hdr_seen = 0;
    piece_sink.have_axis_count = 0;
    piece_sink.axis_count = 0;
    piece_sink.cur_axis = 0;
    piece_sink.in_block_header = 0;
    piece_sink.blk_seen = 0;
    piece_sink.cur_pieces_seen = 0;
    piece_sink.cur_piece_off = 0;
    piece_sink.blocks_done = 0;
    piece_sink.write_rc = 0;
    piece_sink.malformed = 0;
}

// Advance to the next block, or leave cur_axis == axis_count when the last
// block is done.
static void
piece_sink_finish_block(void)
{
    piece_sink.blocks_done++;
    piece_sink.cur_axis++;
    if (piece_sink.cur_axis < piece_sink.axis_count) {
        piece_sink.in_block_header = 1;
        piece_sink.blk_seen = 0;
    }
}

void
piece_sink_feed(uint8_t b)
{
    if (piece_sink.hdr_seen < PER_MESSAGE_HEADER_LEN) {
        piece_sink.hdr[piece_sink.hdr_seen++] = b;
        if (piece_sink.hdr_seen == PER_MESSAGE_HEADER_LEN) {
            const uint8_t *h = piece_sink.hdr;
            piece_sink.correlation_id = (uint32_t)h[3]
                                      | ((uint32_t)h[4] << 8)
                                      | ((uint32_t)h[5] << 16)
                                      | ((uint32_t)h[6] << 24);
        }
        return;
    }
    if (!piece_sink.have_axis_count) {
        piece_sink.have_axis_count = 1;
        piece_sink.axis_count = b;
        if (b == 0 || b > MCU_MAX_FRAME_AXES) {
            piece_sink.malformed = 1;
            return;
        }
        piece_sink.in_block_header = 1;
        piece_sink.blk_seen = 0;
        return;
    }
    // Swallow trailing/over-long bytes; a malformed frame is rejected in commit.
    if (piece_sink.malformed || piece_sink.cur_axis >= piece_sink.axis_count)
        return;

    if (piece_sink.in_block_header) {
        piece_sink.blk[piece_sink.blk_seen++] = b;
        if (piece_sink.blk_seen < AXIS_BLOCK_HEADER_LEN)
            return;
        const uint8_t *k = piece_sink.blk;
        piece_sink.cur_axis_idx    = k[0];
        piece_sink.cur_piece_count = k[1];
        piece_sink.cur_start_slot  = (uint16_t)k[2] | ((uint16_t)k[3] << 8);
        piece_sink.cur_new_head    = (uint32_t)k[4] | ((uint32_t)k[5] << 8)
                                   | ((uint32_t)k[6] << 16) | ((uint32_t)k[7] << 24);
        for (uint8_t a = 0; a < piece_sink.cur_axis; a++) {
            if (piece_sink.axis_idx[a] == piece_sink.cur_axis_idx) {
                piece_sink.malformed = 1;
                return;
            }
        }
        piece_sink.axis_idx[piece_sink.cur_axis] = piece_sink.cur_axis_idx;
        piece_sink.new_head[piece_sink.cur_axis] = piece_sink.cur_new_head;
        piece_sink.first_start_time[piece_sink.cur_axis] = 0;
        piece_sink.in_block_header = 0;
        piece_sink.cur_pieces_seen = 0;
        piece_sink.cur_piece_off = 0;
        if (piece_sink.cur_piece_count == 0)
            piece_sink_finish_block();
        return;
    }

    piece_sink.scratch[piece_sink.cur_piece_off++] = b;
    if (piece_sink.cur_piece_off < PIECE_ENTRY_LEN)
        return;
    piece_sink.cur_piece_off = 0;
    if (piece_sink.cur_pieces_seen == 0) {
        const uint8_t *s = piece_sink.scratch;
        piece_sink.first_start_time[piece_sink.cur_axis] =
            (uint64_t)s[0] | ((uint64_t)s[1] << 8) | ((uint64_t)s[2] << 16)
            | ((uint64_t)s[3] << 24) | ((uint64_t)s[4] << 32)
            | ((uint64_t)s[5] << 40) | ((uint64_t)s[6] << 48)
            | ((uint64_t)s[7] << 56);
    }
    // Written pre-CRC; the slot stays invisible to the ISR until commit
    // advances this axis' frontier.
    if (runtime_handle && piece_sink.cur_pieces_seen < PIECE_SINK_MAX_PIECES) {
        int32_t r = runtime_write_piece(
            runtime_handle, piece_sink.cur_axis_idx, piece_sink.cur_start_slot,
            (uint8_t)piece_sink.cur_pieces_seen, piece_sink.scratch);
        if (r != 0 && piece_sink.write_rc == 0)
            piece_sink.write_rc = r;
    }
    piece_sink.cur_pieces_seen++;
    if (piece_sink.cur_pieces_seen == piece_sink.cur_piece_count)
        piece_sink_finish_block();
}

void
piece_sink_commit(void)
{
    uint32_t clk_lo = timer_read_time();
    uint32_t clk_hi = stats_send_time_high + (clk_lo < stats_send_time);
    uint64_t arrival_clock = ((uint64_t)clk_hi << 32) | (uint64_t)clk_lo;

    if (!runtime_handle) {
        send_push_pieces_response(piece_sink.correlation_id,
                                  RUNTIME_ERR_NOT_INIT, 0, 0, NULL, NULL);
        return;
    }
    // CRC catches bit-corruption but not a count/length logic mismatch. A
    // malformed axis_count/axis_idx, or a frame that ended before every
    // declared block was fully streamed, refuses to advance any frontier
    // (partial slots stay below the head, ISR-invisible).
    if (piece_sink.malformed || !piece_sink.have_axis_count
        || piece_sink.blocks_done != piece_sink.axis_count) {
        send_push_pieces_response(piece_sink.correlation_id,
                                  RUNTIME_ERR_INVALID_CURVE, arrival_clock,
                                  0, NULL, NULL);
        return;
    }
    int32_t rc = piece_sink.write_rc;
    if (rc == 0) {
        // Commit each axis' frontier. A non-OK commit (ring overflow / logic)
        // stops here and surfaces frame-level; the host treats it as fatal and
        // halts, so no further frame is delivered.
        for (uint8_t a = 0; a < piece_sink.axis_count; a++) {
            int32_t r = runtime_commit_head(
                runtime_handle, piece_sink.axis_idx[a], piece_sink.new_head[a]);
            if (r != 0) {
                rc = r;
                break;
            }
        }
    }
    send_push_pieces_response(piece_sink.correlation_id, rc, arrival_clock,
                              piece_sink.axis_count, piece_sink.axis_idx,
                              piece_sink.first_start_time);
}

void
send_status_heartbeat(void)
{
    if (!runtime_handle)
        return;

    uint8_t st = 0;
    uint16_t fc = 0;
    uint32_t counts[8];
    int32_t n = runtime_get_heartbeat(runtime_handle,
                                             &st, &fc, counts, 8);
    if (n < 0)
        return;

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

int32_t
handle_stop_inner(uint64_t *discard_clock)
{
    int32_t rc = RUNTIME_ERR_NOT_INIT;
    *discard_clock = 0;
    if (runtime_handle) {
        irqstatus_t flag = irq_save();
        rc = runtime_gate_pieces(runtime_handle);
        *discard_clock = runtime_now_ticks(runtime_handle);
        irq_restore(flag);
    }
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

static void
handle_resume_stream(uint32_t correlation_id)
{
    int32_t rc = RUNTIME_ERR_NOT_INIT;
    if (runtime_handle) {
        irqstatus_t flag = irq_save();
        rc = runtime_ungate_pieces(runtime_handle);
        irq_restore(flag);
    }
    send_resume_stream_response(correlation_id, rc);
}
