// PushPieces piece_sink: the streaming single-MCU/multi-axis frame parser and
// its frame-level response. Extracted from mcu_transport_dispatch.c into its own
// translation unit so it can be compiled and fuzzed on the host in isolation —
// its only seam is the handful of externs declared below plus the transport
// helpers in mcu_transport_dispatch.h.
//
// Pieces wire layout streamed through piece_sink_feed (the sink sees only the
// CRC-covered payload; envelope + CRC are the demuxer's). Single-MCU frame:
//   per-message header (7): type u16_le | version u8 | corr_id u32_le
//   axis_count         (1): u8, number of axis blocks (1..MCU_MAX_FRAME_AXES)
//   then axis_count axis blocks, each:
//     axis block header(8): axis_idx u8 | piece_count u8 | start_slot u16_le
//                           | new_head u32_le
//     then piece_count variable-length entries:
//       entry header (16): start_time u64_le | duration f32_le | motor_mask u8
//                          | coeff_count u8 (1..=8) | reserved u16
//       then coeff_count * 4 bytes of f32_le Chebyshev coefficients.
//     Each entry is zero-extended into the MCU_PIECE_SLOT_LEN ring slot; a
//     coeff_count outside 1..=8 marks the frame malformed (rejected at commit,
//     no frontier advance).
// Each piece lands at (start_slot + index) % ring_depth for its axis; the
// per-axis frontier advances only in commit, after the demuxer validates CRC.
#include <stdint.h>
#include "mcu_transport_dispatch.h"
#include "mcu_protocol_schema.h"
#include "runtime.h"

// Seam to the rest of the MCU (stubbed by the host fuzz harness):
extern void *runtime_handle;
extern uint32_t stats_send_time;
extern uint32_t stats_send_time_high;
uint32_t timer_read_time(void);

#define AXIS_BLOCK_HEADER_LEN 8u
#define PIECE_COEFF_COUNT_OFF 13u
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
    uint8_t  scratch[MCU_PIECE_SLOT_LEN];
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
    uint8_t  cur_piece_len;   // wire length of the current piece (16 + 4*count)
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
    piece_sink.cur_piece_len = 0;
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
        piece_sink.cur_piece_len = 0;
        if (piece_sink.cur_piece_count == 0)
            piece_sink_finish_block();
        return;
    }

    piece_sink.scratch[piece_sink.cur_piece_off++] = b;
    if (piece_sink.cur_piece_off == MCU_PIECE_WIRE_HEADER_LEN) {
        uint8_t coeff_count = piece_sink.scratch[PIECE_COEFF_COUNT_OFF];
        if (coeff_count == 0 || coeff_count > MCU_PIECE_MAX_COEFFS) {
            piece_sink.malformed = 1;
            return;
        }
        piece_sink.cur_piece_len =
            (uint8_t)(MCU_PIECE_WIRE_HEADER_LEN + 4u * coeff_count);
        for (uint32_t i = MCU_PIECE_WIRE_HEADER_LEN; i < MCU_PIECE_SLOT_LEN; i++)
            piece_sink.scratch[i] = 0;
    }
    if (piece_sink.cur_piece_off < MCU_PIECE_WIRE_HEADER_LEN
        || piece_sink.cur_piece_off < piece_sink.cur_piece_len)
        return;
    piece_sink.cur_piece_off = 0;
    piece_sink.cur_piece_len = 0;
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
