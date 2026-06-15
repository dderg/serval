// Mirror of rust/mcu-transport/src/demux.rs — keep in sync.

#ifndef __MCU_DEMUX_H
#define __MCU_DEMUX_H

#include <stdint.h>
#include "autoconf.h"
#include "command.h"

#define MCU_DEMUX_KLIPPER_BUF_SIZE MESSAGE_MAX
#define MCU_DEMUX_MCU_BUF_SIZE 512u
_Static_assert(MCU_DEMUX_MCU_BUF_SIZE >= 64u,
               "kalico_buf too small for control frames");

// Largest legal kalico frame of any channel = a full pieces frame:
// envelope(4) + per-msg header(7) + piece header(8) + 255*32 + crc(2).
#define MCU_FRAME_MAX_LEN (4u + 7u + 8u + 255u * 32u + 2u)
_Static_assert(MCU_FRAME_MAX_LEN >= MCU_DEMUX_MCU_BUF_SIZE,
               "global frame bound must cover the staging buffer");

typedef enum {
    MCU_DEMUX_OUT_NONE,
    MCU_DEMUX_OUT_KLIPPER,
    MCU_DEMUX_OUT_MCU,
    MCU_DEMUX_OUT_ERROR,
} mcu_demux_output_t;

void mcu_demux_init(void);

mcu_demux_output_t mcu_demux_feed_byte(uint8_t b);

void mcu_demux_consume(void);

// Bootloader-request sentinel detection runs inside this on the OUT_KLIPPER
// branch (gated on CONFIG_HAVE_BOOTLOADER_REQUEST); callers need not check.
void mcu_demux_pump(const uint8_t *buf, uint16_t len);

const uint8_t *mcu_demux_klipper_buf(void);
uint8_t        mcu_demux_klipper_len(void);

const uint8_t *mcu_demux_mcu_payload(void);
uint16_t       mcu_demux_mcu_payload_len(void);
uint8_t        mcu_demux_mcu_channel(void);

#endif
