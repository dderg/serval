// C-side mirror of rust/mcu-transport/src/demux.rs — keep in sync.

#include <stdio.h>
#include <string.h>
#include "mcu_demux.h"
#include "board/misc.h"
#include "command.h"
#include "mcu_transport_dispatch.h"
#include "sched.h"

#define KLIPPER_LEN_MIN          5
#define KLIPPER_LEN_MAX          64
#define KLIPPER_INTERFRAME_SYNC  0x7E
#define MCU_FRAME_SYNC        0x55
#define MCU_FRAME_MIN_LEN_FIELD  5
#define MCU_FRAME_OVERHEAD       6u  /* envelope(sync+len2+channel=4) + crc(2) */

typedef enum {
    DEMUX_S_WAITING,
    DEMUX_S_KLIPPER,
    DEMUX_S_KALICO,
    DEMUX_S_PIECES,
} demux_state_t;

static demux_state_t state;

// Must match the one-shot crc16_ccitt() in src/generic/crc16_ccitt.c (seed
// 0xffff); the streaming pieces path folds byte-by-byte and never has a
// contiguous buffer to pass to the one-shot variant.
static inline uint16_t
crc16_ccitt_update(uint16_t crc, uint8_t b)
{
    uint8_t data = b ^ (crc & 0xff);
    data ^= data << 4;
    return ((((uint16_t)data << 8) | (crc >> 8))
            ^ (uint8_t)(data >> 4) ^ ((uint16_t)data << 3));
}

static uint16_t pieces_payload_remaining;
static uint16_t pieces_crc;
static uint8_t  pieces_crc_byte;
static uint8_t  pieces_crc_lo;

volatile uint32_t mcu_demux_out_mcu_total
                __attribute__((used, externally_visible));
volatile uint32_t mcu_demux_out_error_total
                __attribute__((used, externally_visible));
volatile uint32_t mcu_demux_out_klipper_total
                __attribute__((used, externally_visible));
volatile uint32_t mcu_demux_crc_mismatch_total
                __attribute__((used, externally_visible));

static uint8_t klipper_buf[MCU_DEMUX_KLIPPER_BUF_SIZE];
static uint16_t klipper_pos;
static uint16_t klipper_remaining;

// Holds the whole on-wire frame including the sync byte:
// [sync(1)][len_lo(1)][len_hi(1)][channel(1)][payload..][crc(2)].
#if CONFIG_MACH_STM32H7
__attribute__((section(".axi_bss")))
#endif
static uint8_t kalico_buf[MCU_DEMUX_MCU_BUF_SIZE];
static uint16_t kalico_pos;
static uint16_t transport_total_len; // 0 means header not yet known

void
mcu_demux_init(void)
{
    state = DEMUX_S_WAITING;
    klipper_pos = 0;
    klipper_remaining = 0;
    kalico_pos = 0;
    transport_total_len = 0;
}
DECL_INIT(mcu_demux_init);

static mcu_demux_output_t
finalize_kalico_frame(void)
{
    // CRC covers [len .. crc-start).
    if (kalico_pos < 1 + MCU_FRAME_MIN_LEN_FIELD)
        return MCU_DEMUX_OUT_ERROR;
    uint16_t payload_end = kalico_pos - 2;
    uint16_t crc_expected = (uint16_t)kalico_buf[payload_end]
                          | ((uint16_t)kalico_buf[payload_end + 1] << 8);
    uint16_t crc_actual = crc16_ccitt(&kalico_buf[1], payload_end - 1);
    if (crc_actual != crc_expected) {
#if CONFIG_MACH_LINUX
        fprintf(stderr, "[mcu] crc mismatch: expected 0x%04x, got 0x%04x, kalico_pos=%u\n",
                crc_expected, crc_actual, kalico_pos);
        fflush(stderr);
#endif
        mcu_demux_crc_mismatch_total++;
        return MCU_DEMUX_OUT_ERROR;
    }
    return MCU_DEMUX_OUT_MCU;
}

mcu_demux_output_t
mcu_demux_feed_byte(uint8_t b)
{
    switch (state) {
    case DEMUX_S_WAITING:
        if (b >= KLIPPER_LEN_MIN && b <= KLIPPER_LEN_MAX) {
            klipper_buf[0] = b;
            klipper_pos = 1;
            klipper_remaining = (uint16_t)b - 1;
            state = DEMUX_S_KLIPPER;
            return MCU_DEMUX_OUT_NONE;
        }
        if (b == MCU_FRAME_SYNC) {
            kalico_buf[0] = b;
            kalico_pos = 1;
            transport_total_len = 0;
            state = DEMUX_S_KALICO;
            return MCU_DEMUX_OUT_NONE;
        }
        return MCU_DEMUX_OUT_NONE;

    case DEMUX_S_KLIPPER:
        klipper_buf[klipper_pos++] = b;
        klipper_remaining--;
        if (klipper_remaining == 0) {
            state = DEMUX_S_WAITING;
            return MCU_DEMUX_OUT_KLIPPER;
        }
        return MCU_DEMUX_OUT_NONE;

    case DEMUX_S_KALICO:
        if (kalico_pos >= MCU_DEMUX_MCU_BUF_SIZE) {
            state = DEMUX_S_WAITING;
            return MCU_DEMUX_OUT_ERROR;
        }
        kalico_buf[kalico_pos++] = b;
        if (transport_total_len == 0 && kalico_pos >= 3) {
            uint16_t len_field = (uint16_t)kalico_buf[1]
                               | ((uint16_t)kalico_buf[2] << 8);
            if (len_field < MCU_FRAME_MIN_LEN_FIELD) {
                state = DEMUX_S_WAITING;
                return MCU_DEMUX_OUT_ERROR;
            }
            uint32_t total = 1u + (uint32_t)len_field;
            // Channel is unknown until pos==4, so bound by the largest legal
            // frame of any channel; the staging-buffer bound is applied
            // per-channel below.
            if (total > MCU_FRAME_MAX_LEN) {
                state = DEMUX_S_WAITING;
                return MCU_DEMUX_OUT_ERROR;
            }
            transport_total_len = (uint16_t)total;
        }
        if (kalico_pos == 4 && kalico_buf[3] == MCU_CHANNEL_PIECES
            && transport_total_len > 0) {
            pieces_payload_remaining =
                (uint16_t)(transport_total_len - MCU_FRAME_OVERHEAD);
            pieces_crc = 0xffff;
            pieces_crc = crc16_ccitt_update(pieces_crc, kalico_buf[1]);
            pieces_crc = crc16_ccitt_update(pieces_crc, kalico_buf[2]);
            pieces_crc = crc16_ccitt_update(pieces_crc, kalico_buf[3]);
            pieces_crc_byte = 0;
            piece_sink_begin();
            state = DEMUX_S_PIECES;
            return MCU_DEMUX_OUT_NONE;
        }
        if (kalico_pos == 4 && kalico_buf[3] != MCU_CHANNEL_PIECES
            && transport_total_len > MCU_DEMUX_MCU_BUF_SIZE) {
            state = DEMUX_S_WAITING;
            return MCU_DEMUX_OUT_ERROR;
        }
        if (transport_total_len > 0 && kalico_pos == transport_total_len) {
            mcu_demux_output_t out = finalize_kalico_frame();
            state = DEMUX_S_WAITING;
            return out;
        }
        return MCU_DEMUX_OUT_NONE;

    case DEMUX_S_PIECES:
        if (pieces_payload_remaining > 0) {
            pieces_crc = crc16_ccitt_update(pieces_crc, b);
            piece_sink_feed(b);
            pieces_payload_remaining--;
            return MCU_DEMUX_OUT_NONE;
        }
        // Trailing CRC, little-endian (low byte first).
        if (pieces_crc_byte == 0) {
            pieces_crc_lo = b;
            pieces_crc_byte = 1;
            return MCU_DEMUX_OUT_NONE;
        }
        {
            uint16_t crc_expected = (uint16_t)pieces_crc_lo
                                  | ((uint16_t)b << 8);
            // The pieces path commits inline and returns OUT_NONE, bypassing
            // mcu_demux_consume(); this is the only reset of kalico_pos /
            // transport_total_len for a committed pieces frame.
            state = DEMUX_S_WAITING;
            kalico_pos = 0;
            transport_total_len = 0;
            if (crc_expected == pieces_crc) {
                piece_sink_commit();
                return MCU_DEMUX_OUT_NONE;
            }
            mcu_demux_crc_mismatch_total++;
            return MCU_DEMUX_OUT_ERROR;
        }
    }
    state = DEMUX_S_WAITING;
    return MCU_DEMUX_OUT_ERROR;
}

void
mcu_demux_consume(void)
{
    klipper_pos = 0;
    klipper_remaining = 0;
    kalico_pos = 0;
    transport_total_len = 0;
}

const uint8_t *
mcu_demux_klipper_buf(void)
{
    return klipper_buf;
}

uint8_t
mcu_demux_klipper_len(void)
{
    return (uint8_t)klipper_pos;
}

const uint8_t *
mcu_demux_mcu_payload(void)
{
    return &kalico_buf[4];
}

uint16_t
mcu_demux_mcu_payload_len(void)
{
    if (kalico_pos < 1 + MCU_FRAME_MIN_LEN_FIELD)
        return 0;
    return (uint16_t)(kalico_pos - MCU_FRAME_OVERHEAD);
}

uint8_t
mcu_demux_mcu_channel(void)
{
    return kalico_buf[3];
}

// 100 ms is above any legitimate inter-byte gap inside one host frame on
// USB-CDC FS, and below the host's identify timeout, so a stuck mid-frame
// demuxer self-heals before the host gives up.
static uint32_t last_byte_time;

void
mcu_demux_pump(const uint8_t *buf, uint16_t len)
{
    if (len == 0)
        return;
    uint32_t now = timer_read_time();
    if (state != DEMUX_S_WAITING) {
        uint32_t idle_ticks = now - last_byte_time;
        if (idle_ticks > timer_from_us(100000)) {
            state = DEMUX_S_WAITING;
            klipper_pos = 0;
            klipper_remaining = 0;
            kalico_pos = 0;
            transport_total_len = 0;
        }
    }
    last_byte_time = now;
    extern void diag_note_msg_enter(uint32_t kind, uint32_t head);
    extern void diag_note_msg_exit(void);
    extern void diag_note_demux(uint32_t backlog, uint32_t msgs);
    uint32_t msg_count = 0;
    for (uint16_t i = 0; i < len; i++) {
        mcu_demux_output_t out = mcu_demux_feed_byte(buf[i]);
        switch (out) {
        case MCU_DEMUX_OUT_NONE:
            break;
        case MCU_DEMUX_OUT_KLIPPER: {
            mcu_demux_out_klipper_total++;
#if CONFIG_MACH_LINUX
            {
                const uint8_t *kb = mcu_demux_klipper_buf();
                uint8_t kl = mcu_demux_klipper_len();
                fprintf(stderr, "[mcu-demux] KLIPPER len=%u seq=0x%02x total=%u\n",
                        kl, kl >= 2 ? kb[1] : 0, mcu_demux_out_klipper_total);
                fflush(stderr);
            }
#endif
            const uint8_t *kbuf = mcu_demux_klipper_buf();
            uint8_t klen = mcu_demux_klipper_len();
            if (CONFIG_HAVE_BOOTLOADER_REQUEST && klen == 32
                && !memcmp(kbuf,
                           " \x1c Request Serial Bootloader!! ~", 32))
                bootloader_request();
            uint_fast8_t pop_count;
            uint32_t khead = (klen > 0 ? kbuf[0] : 0u)
                             | (klen > 1 ? (uint32_t)kbuf[1] << 8 : 0u)
                             | (klen > 2 ? (uint32_t)kbuf[2] << 16 : 0u)
                             | (klen > 3 ? (uint32_t)kbuf[3] << 24 : 0u);
            diag_note_msg_enter(0x200u | (klen > 2 ? kbuf[2] : 0u), khead);
            msg_count++;
            command_find_and_dispatch(
                (uint8_t *)kbuf, klen, &pop_count);
            diag_note_msg_exit();
            mcu_demux_consume();
            break;
        }
        case MCU_DEMUX_OUT_MCU: {
            mcu_demux_out_mcu_total++;
            uint8_t channel = mcu_demux_mcu_channel();
            const uint8_t *pl = mcu_demux_mcu_payload();
            uint8_t plen = mcu_demux_mcu_payload_len();
            uint32_t mhead = channel
                             | (plen > 0 ? (uint32_t)pl[0] << 8 : 0u)
                             | (plen > 1 ? (uint32_t)pl[1] << 16 : 0u)
                             | (plen > 2 ? (uint32_t)pl[2] << 24 : 0u);
            diag_note_msg_enter(0x100u | channel, mhead);
            msg_count++;
            mcu_transport_dispatch_frame(
                channel,
                pl,
                plen);
            diag_note_msg_exit();
            mcu_demux_consume();
            break;
        }
        case MCU_DEMUX_OUT_ERROR:
            mcu_demux_out_error_total++;
            mcu_demux_consume();
            break;
        }
    }
    diag_note_demux(len, msg_count);
}
