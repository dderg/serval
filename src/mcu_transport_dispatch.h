#ifndef __MCU_TRANSPORT_DISPATCH_H
#define __MCU_TRANSPORT_DISPATCH_H

#include <stdint.h>

#define MCU_CHANNEL_CONTROL 0x00
#define MCU_CHANNEL_EVENTS  0x01
#define MCU_CHANNEL_PIECES  0x02

// Transport framing constants shared by the dispatcher and piece_sink.
#define MCU_TX_BUF_SIZE 256
// type:u16_le | version:u8 | corr_id:u32_le.
#define PER_MESSAGE_HEADER_LEN 7
#define MESSAGE_VERSION_DEFAULT 0x01

// Runtime result codes mirrored from rust/runtime/src/error.rs; keep in sync.
#define RUNTIME_ERR_INVALID_CURVE -2
#define RUNTIME_ERR_NOT_INIT      -7

void piece_sink_begin(void);
void piece_sink_feed(uint8_t b);
void piece_sink_commit(void);

void mcu_transport_dispatch_frame(uint8_t channel, const uint8_t *payload,
                           uint16_t payload_len);

int mcu_transport_send_frame(uint8_t channel, const uint8_t *payload,
                                uint16_t payload_len);

void encode_message_header(uint8_t *out, uint16_t kind, uint8_t version,
                           uint32_t correlation_id);

void kalico_reset_epoch_init(void);
uint32_t kalico_reset_epoch_get(void);

void mcu_transport_emit_fault_event(uint16_t fault_code,
                                    uint32_t fault_detail,
                                    uint32_t segment_id);

void mcu_transport_emit_endstop_trip(uint8_t endstop_id, uint64_t trip_clock);

int32_t handle_stop_inner(uint64_t *discard_clock);

void send_status_heartbeat(void);

#endif
