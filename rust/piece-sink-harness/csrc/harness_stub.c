// Fakes the piece_sink seam and records every call so the Rust tests can assert
// what the parser did, plus capture the response frame it emitted.
#include <stdint.h>
#include <string.h>
#include "mcu_transport_dispatch.h"

void *runtime_handle = (void *)1;
uint32_t stats_send_time = 0;
uint32_t stats_send_time_high = 0;
uint32_t timer_read_time(void) { return 0x12345678u; }

#define MAXW 4096
#define MAXC 16
static int s_write_count, s_commit_count;
static uint8_t s_w_axis[MAXW];
static uint16_t s_w_slot[MAXW];
static uint8_t s_w_idx[MAXW];
static uint8_t s_c_axis[MAXC];
static uint32_t s_c_head[MAXC];
static int32_t s_commit_rc;
static uint8_t s_resp[256];
static int s_resp_len;

int32_t
runtime_write_piece(void *rt, uint8_t axis, uint16_t slot, uint8_t idx,
                    const uint8_t *p)
{
    (void)rt;
    (void)p;
    if (s_write_count < MAXW) {
        s_w_axis[s_write_count] = axis;
        s_w_slot[s_write_count] = slot;
        s_w_idx[s_write_count] = idx;
    }
    s_write_count++;
    return 0;
}

int32_t
runtime_commit_head(void *rt, uint8_t axis, uint32_t new_head)
{
    (void)rt;
    if (s_commit_count < MAXC) {
        s_c_axis[s_commit_count] = axis;
        s_c_head[s_commit_count] = new_head;
    }
    s_commit_count++;
    return s_commit_rc;
}

int
mcu_transport_send_frame(uint8_t channel, const uint8_t *payload, uint16_t len)
{
    (void)channel;
    if (len > sizeof s_resp)
        len = sizeof s_resp;
    memcpy(s_resp, payload, len);
    s_resp_len = (int)len;
    return 0;
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

// ---- accessors read by the Rust harness ----
void harness_reset(void)
{
    s_write_count = 0;
    s_commit_count = 0;
    s_resp_len = 0;
    s_commit_rc = 0;
    runtime_handle = (void *)1;
    piece_sink_begin();
}
int harness_write_count(void) { return s_write_count; }
uint8_t harness_write_axis(int i) { return s_w_axis[i]; }
uint16_t harness_write_slot(int i) { return s_w_slot[i]; }
uint8_t harness_write_idx(int i) { return s_w_idx[i]; }
int harness_commit_count(void) { return s_commit_count; }
uint8_t harness_commit_axis(int i) { return s_c_axis[i]; }
uint32_t harness_commit_head(int i) { return s_c_head[i]; }
int harness_resp(uint8_t *out, int maxlen)
{
    int n = s_resp_len;
    if (n > maxlen)
        n = maxlen;
    memcpy(out, s_resp, (size_t)n);
    return n;
}
void harness_set_commit_rc(int32_t rc) { s_commit_rc = rc; }
void harness_set_runtime_null(int is_null)
{
    runtime_handle = is_null ? (void *)0 : (void *)1;
}
