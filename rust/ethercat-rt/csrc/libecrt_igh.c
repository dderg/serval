/*
 * IgH (EtherLab) EtherCAT master backend.
 *
 * Built under `--features hw` (build.rs compiles this file and links
 * -lethercat). Implements the full ec_rt_* contract from libecrt.h against
 * IgH's kernel master.
 *
 * The master is bound to the NIC out-of-band (MASTER0_DEVICE in
 * /etc/ethercat.conf, served by the ec_master/ec_generic kernel modules) and
 * is requested here by index 0, so `ifname` is ignored.
 *
 * PDO mapping is expressed declaratively (ecrt_slave_config_pdos) and applied
 * by the master during its PRE-OP -> SAFE-OP configuration; the byte layout is
 * out_t 18 bytes / in_t 32 bytes. Inputs are read from the domain image with
 * the EC_READ accessors at the offsets the master assigns; outputs are staged
 * in a shadow struct and flushed into the image each cycle (see rt_exchange for
 * why the staging is required).
 */
#define _GNU_SOURCE
#include "libecrt.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#include <sched.h>
#include <sys/mman.h>
#include <ecrt.h>

/* Drive identity (ethercat slaves -v): AS715N_sAxis A6-EC servo. */
#define VENDOR_ID    0x00400000u
#define PRODUCT_CODE 0x00000715u
#define SLAVE_ALIAS  0
#define SLAVE_POS    0 /* ring position, 0-based */

/* DC AssignActivate for SYNC0-only operation: SYNC0 at the cycle period,
 * shifted half a cycle. The A6-EC requires SYNC0 active before SAFE-OP (else
 * AL 0x0030). */
#define DC_ASSIGN_ACTIVATE 0x0300

#define OUT_BYTES 18
#define IN_BYTES  32

static ec_master_t       *g_master;
static ec_domain_t       *g_domain;
static ec_slave_config_t *g_sc;
static ec_reg_request_t  *g_al_req; /* reads AL status register 0x0134 for diagnostics */
static uint8_t           *g_pd;     /* domain process image (the LRW datagram buffer) */

/* Output field offsets in the domain image (SM2 / RxPDO 1600h = out_t). */
static unsigned o_controlword, o_target, o_touch_probe, o_phys_outputs,
    o_velocity_offset, o_torque_offset;
/* Input field offsets (SM3 / TxPDO 1A00h = in_t). */
static unsigned i_error_code, i_statusword, i_position_actual, i_velocity_actual,
    i_torque_actual, i_following_error, i_tp_status, i_tp1_pos, i_tp2_pos,
    i_digital_inputs;

/* Staged outputs. ecrt_master_receive overwrites the domain image's output
 * region with the echo of the previously-sent frame, so a value written
 * directly into the image before the receive would be clobbered. Callers stage
 * here instead; rt_exchange flushes this into the image after receive, just
 * before queue/send. */
static struct {
    uint16_t controlword;
    int32_t  target_position;
    uint16_t touch_probe;
    uint32_t phys_outputs;
    int32_t  velocity_offset;
    int16_t  torque_offset;
} g_tx;

static int64_t g_cycle_ns;
static struct timespec g_ts;
static int g_enabled;
static int g_activated;

#define TIMESPEC2NS(T) ((uint64_t)(T).tv_sec * 1000000000ULL + (uint64_t)(T).tv_nsec)

static ec_pdo_entry_info_t rx_entries[] = {
    {0x6040, 0x00, 16}, {0x607A, 0x00, 32}, {0x60B8, 0x00, 16},
    {0x60FE, 0x01, 32}, {0x60B1, 0x00, 32}, {0x60B2, 0x00, 16},
};
static ec_pdo_entry_info_t tx_entries[] = {
    {0x603F, 0x00, 16}, {0x6041, 0x00, 16}, {0x6064, 0x00, 32},
    {0x606C, 0x00, 32}, {0x6077, 0x00, 16}, {0x60F4, 0x00, 32},
    {0x60B9, 0x00, 16}, {0x60BA, 0x00, 32}, {0x60BC, 0x00, 32},
    {0x60FD, 0x00, 32},
};
static ec_pdo_info_t rx_pdos[] = {{0x1600, 6, rx_entries}};
static ec_pdo_info_t tx_pdos[] = {{0x1A00, 10, tx_entries}};
static ec_sync_info_t syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, rx_pdos, EC_WD_ENABLE}, /* SM watchdog: drop to SAFE-OP if PDO stops */
    {3, EC_DIR_INPUT, 1, tx_pdos, EC_WD_DISABLE},
    {0xFF, EC_DIR_INVALID, 0, NULL, EC_WD_DEFAULT}, /* EC_END sentinel */
};

static void add_ts(struct timespec *ts, int64_t add) {
    int64_t ns  = add % 1000000000LL;
    int64_t sec = (add - ns) / 1000000000LL;
    ts->tv_sec  += sec;
    ts->tv_nsec += ns;
    if (ts->tv_nsec >= 1000000000LL) { ts->tv_nsec -= 1000000000LL; ts->tv_sec++; }
}

static int go_realtime(int cpu, int prio) {
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        fprintf(stderr, "ec_rt: mlockall failed: %s — grant CAP_IPC_LOCK\n",
                strerror(errno));
        return EC_RT_ERR_RT_MLOCK;
    }
    cpu_set_t set; CPU_ZERO(&set); CPU_SET(cpu, &set);
    if (sched_setaffinity(0, sizeof(set), &set) != 0) {
        fprintf(stderr, "ec_rt: pin to CPU %d failed: %s — isolate the core "
                "and grant the endpoint access to it\n", cpu, strerror(errno));
        return EC_RT_ERR_RT_AFFINITY;
    }
    struct sched_param sp; sp.sched_priority = prio;
    if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0) {
        fprintf(stderr, "ec_rt: SCHED_FIFO(prio %d) failed: %s — grant "
                "CAP_SYS_NICE\n", prio, strerror(errno));
        return EC_RT_ERR_RT_SCHED;
    }
    return 0;
}

/* If the deadline has gone stale (a blocking transaction or a wait between
 * exchanges overran the cycle), re-anchor to now rather than letting
 * clock_nanosleep return immediately cycle after cycle: that catch-up burst
 * sends frames far outside the SYNC0 window and reads as sync loss. */
static void reanchor_if_stale(void) {
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    int64_t behind_ns = (now.tv_sec - g_ts.tv_sec) * 1000000000LL
                      + (now.tv_nsec - g_ts.tv_nsec);
    if (behind_ns > g_cycle_ns) g_ts = now;
}

static void flush_outputs(void) {
    EC_WRITE_U16(g_pd + o_controlword, g_tx.controlword);
    EC_WRITE_S32(g_pd + o_target, g_tx.target_position);
    EC_WRITE_U16(g_pd + o_touch_probe, g_tx.touch_probe);
    EC_WRITE_U32(g_pd + o_phys_outputs, g_tx.phys_outputs);
    EC_WRITE_S32(g_pd + o_velocity_offset, g_tx.velocity_offset);
    EC_WRITE_S16(g_pd + o_torque_offset, g_tx.torque_offset);
}

/* One DC exchange at a fixed cycle period. The sleep lets the previous cycle's
 * frame round-trip, so the receive at the top refreshes the input image. That
 * same receive overwrites the image's output region with the echo of the
 * previously-sent frame, so the caller's staged outputs (g_tx) are flushed into
 * the image AFTER receive, then queued and sent. DC drift is compensated by the
 * kernel master: application_time anchors the network clock to the
 * CLOCK_MONOTONIC wake grid and sync_reference_clock/sync_slave_clocks
 * distribute it — so no application-side phase loop is needed and *toff stays
 * 0. */
static int rt_exchange(int64_t *toff) {
    add_ts(&g_ts, g_cycle_ns);
    reanchor_if_stale();
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &g_ts, NULL);

    ecrt_master_application_time(g_master, TIMESPEC2NS(g_ts));
    ecrt_master_receive(g_master);
    ecrt_domain_process(g_domain);

    flush_outputs();
    ecrt_master_sync_reference_clock(g_master);
    ecrt_master_sync_slave_clocks(g_master);
    ecrt_domain_queue(g_domain);
    ecrt_master_send(g_master);

    ec_domain_state_t ds;
    ecrt_domain_state(g_domain, &ds);
    if (toff) *toff = 0;
    return (int)ds.working_counter;
}

static int reg_entry(uint16_t index, uint8_t sub, unsigned *off) {
    unsigned bit = 0;
    int byte = ecrt_slave_config_reg_pdo_entry(g_sc, index, sub, g_domain, &bit);
    if (byte < 0 || bit != 0) {
        fprintf(stderr, "ec_rt: reg_pdo_entry %04X:%02X failed (byte=%d bit=%u)\n",
                index, sub, byte, bit);
        return -1;
    }
    *off = (unsigned)byte;
    return 0;
}

static int register_pdo_entries(void) {
    return reg_entry(0x6040, 0x00, &o_controlword)
        || reg_entry(0x607A, 0x00, &o_target)
        || reg_entry(0x60B8, 0x00, &o_touch_probe)
        || reg_entry(0x60FE, 0x01, &o_phys_outputs)
        || reg_entry(0x60B1, 0x00, &o_velocity_offset)
        || reg_entry(0x60B2, 0x00, &o_torque_offset)
        || reg_entry(0x603F, 0x00, &i_error_code)
        || reg_entry(0x6041, 0x00, &i_statusword)
        || reg_entry(0x6064, 0x00, &i_position_actual)
        || reg_entry(0x606C, 0x00, &i_velocity_actual)
        || reg_entry(0x6077, 0x00, &i_torque_actual)
        || reg_entry(0x60F4, 0x00, &i_following_error)
        || reg_entry(0x60B9, 0x00, &i_tp_status)
        || reg_entry(0x60BA, 0x00, &i_tp1_pos)
        || reg_entry(0x60BC, 0x00, &i_tp2_pos)
        || reg_entry(0x60FD, 0x00, &i_digital_inputs);
}

/* A session that dies mid-OP (an abrupt endpoint exit drops the cyclic frames
 * the moment the kernel releases the master) latches the drive's sync-loss
 * alarm (0x8700 / ErC1.1). The CiA402 fault reset (6040h bit 7 rising edge)
 * clears it over SDO in PRE-OP, where sync monitoring is inactive — the alarm
 * cannot re-raise mid-clear the way it can during the post-OP park loop on a
 * still-settling DC clock. Runs in the master's idle phase (pre-activate),
 * where ecrt_master_sdo_* reach the slave's mailbox at PRE-OP. */
static void clear_latched_alarm_in_preop(void) {
    uint8_t buf[2];
    size_t rs = 0;
    uint32_t abort = 0;
    if (ecrt_master_sdo_upload(g_master, SLAVE_POS, 0x603F, 0x00, buf, 2, &rs, &abort) != 0)
        return;
    uint16_t err = (uint16_t)(buf[0] | (buf[1] << 8));
    if (err == 0) return;
    fprintf(stderr, "ec_rt: clearing latched drive alarm 0x%04x in PRE-OP\n", err);
    uint16_t cw = 0x0000;
    ecrt_master_sdo_download(g_master, SLAVE_POS, 0x6040, 0x00, (uint8_t *)&cw, 2, &abort);
    cw = 0x0080;
    ecrt_master_sdo_download(g_master, SLAVE_POS, 0x6040, 0x00, (uint8_t *)&cw, 2, &abort);
    cw = 0x0000;
    ecrt_master_sdo_download(g_master, SLAVE_POS, 0x6040, 0x00, (uint8_t *)&cw, 2, &abort);
    if (ecrt_master_sdo_upload(g_master, SLAVE_POS, 0x603F, 0x00, buf, 2, &rs, &abort) == 0) {
        err = (uint16_t)(buf[0] | (buf[1] << 8));
        if (err != 0)
            fprintf(stderr, "ec_rt: drive alarm 0x%04x survived PRE-OP fault reset; "
                    "the post-OP park loop will keep pulsing\n", err);
    }
}

/* Phase 1: request the master, configure the slave, declare the PDO map, stage
 * the static drive setup as config SDOs (applied by the master in PRE-OP before
 * OP), and arm DC — but do not activate.
 * The master's idle state machine brings the slave to PRE-OP, where the caller
 * does its session SDO work (drive limits) via ecrt_master_sdo_* before phase 2. */
int ec_rt_bringup_preop(const char *ifname, int64_t cycle_ns, int rt_cpu, int rt_prio) {
    (void)ifname; /* the IgH master is bound to the NIC via /etc/ethercat.conf */
    g_cycle_ns = cycle_ns < 250000 ? 250000 : cycle_ns;
    g_enabled  = 0;
    g_activated = 0;
    memset(&g_tx, 0, sizeof(g_tx));

    int rt_rc = go_realtime(rt_cpu, rt_prio);
    if (rt_rc != 0) return rt_rc;

    g_master = ecrt_request_master(0);
    if (!g_master) return EC_RT_ERR_EC_INIT;

    g_domain = ecrt_master_create_domain(g_master);
    if (!g_domain) return EC_RT_ERR_EC_INIT;

    g_sc = ecrt_master_slave_config(g_master, SLAVE_ALIAS, SLAVE_POS,
                                    VENDOR_ID, PRODUCT_CODE);
    if (!g_sc) return EC_RT_ERR_NO_SLAVES;

    if (ecrt_slave_config_pdos(g_sc, EC_END, syncs) != 0)
        return EC_RT_ERR_PDO_REMAP;
    if (register_pdo_entries() != 0)
        return EC_RT_ERR_PDO_REMAP;

    /* CSP mode and a disabled following-error timeout, then route both
     * feedforward sources (speed 60B1h, torque 60B2h) to "communication"
     * (C01.13/C01.16 -> 5) with 0% additional FF (C01.14/C01.17 -> 0). The
     * master applies them in PRE-OP. A rejected config SDO leaves the slave
     * short of OP -> OP_TIMEOUT. */
    ecrt_slave_config_sdo8(g_sc, 0x6060, 0x00, 8);
    ecrt_slave_config_sdo16(g_sc, 0x6066, 0x00, 0);
    ecrt_slave_config_sdo16(g_sc, 0x2001, 0x14, 5);
    ecrt_slave_config_sdo16(g_sc, 0x2001, 0x15, 0);
    ecrt_slave_config_sdo16(g_sc, 0x2001, 0x17, 5);
    ecrt_slave_config_sdo16(g_sc, 0x2001, 0x18, 0);

    ecrt_slave_config_dc(g_sc, DC_ASSIGN_ACTIVATE, (uint32_t)g_cycle_ns,
                         (int32_t)(g_cycle_ns / 2), 0, 0);

    g_al_req = ecrt_slave_config_create_reg_request(g_sc, 2);

    clear_latched_alarm_in_preop();
    return 0;
}

/* CiA402 fault reset (6040h bit 7) needs a rising edge: hold it low and high on
 * alternating ~10-cycle windows so a latched fault clears within the walk loop. */
static uint16_t fault_reset_pulse(int64_t cycle) {
    return ((cycle / 10) % 2) ? 0x0080 : 0x0000;
}

/* Phase 2: activate the master (it walks the slave PRE-OP -> SAFE-OP -> OP as we
 * cycle, applying the staged config SDOs and DC), stabilize the DC loop, confirm
 * OP, then park at CiA402 Ready-to-Switch-On (no torque). From the first cycle
 * here the caller must never pause process data — every wait goes through the
 * cycle/park helpers, else the SM watchdog drops the drive to SAFE-OP. */
int ec_rt_bringup_finish(void) {
    if (ecrt_master_activate(g_master) != 0) return EC_RT_ERR_EC_INIT;
    g_activated = 1;

    g_pd = ecrt_domain_data(g_domain);
    if (!g_pd) return EC_RT_ERR_PDO_SIZE;
    if (ecrt_domain_size(g_domain) != (size_t)(OUT_BYTES + IN_BYTES)) {
        fprintf(stderr, "ec_rt: domain size %zu, expected %d\n",
                ecrt_domain_size(g_domain), OUT_BYTES + IN_BYTES);
        return EC_RT_ERR_PDO_SIZE;
    }

    memset(&g_tx, 0, sizeof(g_tx));
    int64_t toff = 0;
    clock_gettime(CLOCK_MONOTONIC, &g_ts);

    /* Walk to OP: the master FSM advances the slave PRE-OP -> SAFE-OP -> OP at
     * roughly one datagram per cycle while we hold controlword 0 / target =
     * actual. Phase lock is not yet possible — the DC clock isn't running until
     * OP. The budget is generous (8 s) — the walk applies every config SDO and
     * the DC handshake, and a cold drive right after power-on is slower. */
    ec_slave_config_state_t st;
    int operational = 0;
    for (int64_t i = 0; i < (int64_t)(8.0e9 / g_cycle_ns); i++) {
        g_tx.controlword = 0;
        g_tx.target_position = EC_READ_S32(g_pd + i_position_actual);
        rt_exchange(&toff);
        ecrt_slave_config_state(g_sc, &st);
        if (st.operational) { operational = 1; break; }
    }
    if (!operational) return EC_RT_ERR_OP_TIMEOUT;

    /* DC is running now: settle for 1.5 s with controlword 0 before walking
     * CiA-402, so the SYNC0 alignment is tight and any latched sync error
     * (0x8700) is steady enough for the park loop to reset. */
    for (int64_t i = 0; i < (int64_t)(1.5e9 / g_cycle_ns); i++) {
        g_tx.controlword = 0;
        g_tx.target_position = EC_READ_S32(g_pd + i_position_actual);
        rt_exchange(&toff);
    }
    fprintf(stderr, "ec_rt: OP reached; park entry sw=0x%04x err=0x%04x\n",
            EC_READ_U16(g_pd + i_statusword), EC_READ_U16(g_pd + i_error_code));

    for (int64_t pc = 0; pc < 3000; pc++) {
        uint16_t sw = EC_READ_U16(g_pd + i_statusword);
        g_tx.target_position = EC_READ_S32(g_pd + i_position_actual);
        if (sw & 0x0008) {
            g_tx.controlword = fault_reset_pulse(pc);
        } else if ((sw & 0x006F) == 0x0021) {
            g_tx.controlword = 0x0006;
            rt_exchange(&toff);
            g_enabled = 0;
            return 0;
        } else {
            g_tx.controlword = 0x0006;
        }
        rt_exchange(&toff);
    }
    fprintf(stderr, "ec_rt: CiA402 park timeout sw=0x%04x err=0x%04x\n",
            EC_READ_U16(g_pd + i_statusword), EC_READ_U16(g_pd + i_error_code));
    return EC_RT_ERR_CIA402_TIMEOUT;
}

int ec_rt_enable(void) {
    /*
     * CiA402 enable state machine — masks/values match the CiA402 table:
     *   sw & 0x004F == 0x0040 => Switch-On Disabled: issue 0x0006
     *   sw & 0x006F == 0x0021 => Ready-to-Switch-On: issue 0x0007
     *   sw & 0x006F == 0x0023 => Switched-On:        issue 0x000F
     *   sw & 0x006F == 0x0027 => Operation Enabled:  done
     *   sw & 0x0008           => Fault: pulse fault-reset on bit 7
     */
    int64_t toff = 0;
    for (int64_t pc = 0; pc < 3000; pc++) {
        uint16_t sw = EC_READ_U16(g_pd + i_statusword);
        g_tx.target_position = EC_READ_S32(g_pd + i_position_actual);
        if (sw & 0x0008) {
            g_tx.controlword = fault_reset_pulse(pc);
        } else if ((sw & 0x004F) == 0x0040) {
            g_tx.controlword = 0x0006;
        } else if ((sw & 0x006F) == 0x0021) {
            g_tx.controlword = 0x0007;
        } else if ((sw & 0x006F) == 0x0023) {
            g_tx.controlword = 0x000F;
        } else if ((sw & 0x006F) == 0x0027) {
            g_tx.controlword = 0x000F;
            rt_exchange(&toff);
            g_enabled = 1;
            return 0;
        } else {
            g_tx.controlword = 0x0000;
        }
        rt_exchange(&toff);
    }
    return EC_RT_ERR_CIA402_TIMEOUT;
}

int ec_rt_cycle(int64_t *toff_ns) {
    if (g_enabled) {
        g_tx.controlword = 0x000F;
    } else {
        g_tx.controlword = 0x0006;
        g_tx.target_position = EC_READ_S32(g_pd + i_position_actual);
    }
    return rt_exchange(toff_ns);
}

void ec_rt_set_target_position(int32_t counts) { g_tx.target_position = counts; }
int32_t  ec_rt_get_position_actual(void) { return EC_READ_S32(g_pd + i_position_actual); }
int32_t  ec_rt_get_velocity_actual(void) { return EC_READ_S32(g_pd + i_velocity_actual); }
uint16_t ec_rt_get_statusword(void)      { return EC_READ_U16(g_pd + i_statusword); }
uint16_t ec_rt_get_error_code(void)      { return EC_READ_U16(g_pd + i_error_code); }
int32_t  ec_rt_get_following_error(void) { return EC_READ_S32(g_pd + i_following_error); }
void ec_rt_set_velocity_offset(int32_t counts_per_s) { g_tx.velocity_offset = counts_per_s; }
void ec_rt_set_torque_offset(int16_t tenths_pct)     { g_tx.torque_offset = tenths_pct; }
int16_t  ec_rt_get_torque_actual(void)   { return EC_READ_S16(g_pd + i_torque_actual); }

void ec_rt_get_telemetry(ec_telemetry_t *out) {
    out->error_code      = EC_READ_U16(g_pd + i_error_code);
    out->statusword      = EC_READ_U16(g_pd + i_statusword);
    out->position_actual = EC_READ_S32(g_pd + i_position_actual);
    out->velocity_actual = EC_READ_S32(g_pd + i_velocity_actual);
    out->torque_actual   = EC_READ_S16(g_pd + i_torque_actual);
    out->following_error = EC_READ_S32(g_pd + i_following_error);
    out->target_position = g_tx.target_position;
    out->velocity_offset = g_tx.velocity_offset;
    out->torque_offset   = g_tx.torque_offset;
}

int ec_rt_read_limits(uint32_t *ferr_counts, uint16_t *ferr_timeout_ms,
                      uint16_t *torque_tenth_pct) {
    uint8_t buf[4];
    size_t rs = 0;
    uint32_t abort = 0;
    if (ecrt_master_sdo_upload(g_master, SLAVE_POS, 0x6065, 0x00, buf, 4, &rs, &abort) != 0)
        return -1;
    memcpy(ferr_counts, buf, 4);
    if (ecrt_master_sdo_upload(g_master, SLAVE_POS, 0x6066, 0x00, buf, 2, &rs, &abort) != 0)
        return -2;
    memcpy(ferr_timeout_ms, buf, 2);
    if (ecrt_master_sdo_upload(g_master, SLAVE_POS, 0x6072, 0x00, buf, 2, &rs, &abort) != 0)
        return -3;
    memcpy(torque_tenth_pct, buf, 2);
    return 0;
}

int ec_rt_write_limits(uint32_t ferr_counts, uint16_t torque_tenth_pct) {
    uint32_t abort = 0;
    uint8_t b4[4];
    memcpy(b4, &ferr_counts, 4);
    if (ecrt_master_sdo_download(g_master, SLAVE_POS, 0x6065, 0x00, b4, 4, &abort) != 0)
        return -1;
    uint8_t b2[2];
    memcpy(b2, &torque_tenth_pct, 2);
    if (ecrt_master_sdo_download(g_master, SLAVE_POS, 0x6072, 0x00, b2, 2, &abort) != 0)
        return -2;
    return 0;
}

int ec_rt_sdo_read(uint16_t index, uint8_t sub, uint8_t *buf, int *size,
                   uint32_t *abort_code) {
    size_t rs = 0;
    uint32_t abort = 0;
    if (ecrt_master_sdo_upload(g_master, SLAVE_POS, index, sub, buf,
                               (size_t)*size, &rs, &abort) != 0) {
        *abort_code = abort;
        return -1;
    }
    *size = (int)rs;
    *abort_code = 0;
    return 0;
}

int ec_rt_sdo_write(uint16_t index, uint8_t sub, const uint8_t *buf, int size,
                    uint32_t *abort_code) {
    uint32_t abort = 0;
    if (ecrt_master_sdo_download(g_master, SLAVE_POS, index, sub, buf,
                                 (size_t)size, &abort) != 0) {
        *abort_code = abort;
        return -1;
    }
    *abort_code = 0;
    return 0;
}

/*
 * CiA-402 homing-method-35 ("current position is home") drive-frame, run as a
 * self-contained DC loop the way ec_rt_enable() runs its enable loop: every wait
 * goes through rt_exchange so process data never pauses. Preconditions (staged
 * off-loop by the caller): mode = Homing (6061h reads 6), 6098h = 35,
 * 607Ch = offset, drive operation-enabled. 6040h bit 4 (0x10) rising edge starts
 * homing and stays set; 6041h bit 12 = attained, bit 13 = error.
 */
int ec_rt_run_homing(void) {
    int64_t toff = 0;
    for (int i = 0; i < 8; i++) {
        g_tx.controlword = 0x000F;
        g_tx.target_position = EC_READ_S32(g_pd + i_position_actual);
        rt_exchange(&toff);
    }
    for (int64_t pc = 0; pc < 3000; pc++) {
        uint16_t sw = EC_READ_U16(g_pd + i_statusword);
        g_tx.controlword = 0x001F;
        g_tx.target_position = EC_READ_S32(g_pd + i_position_actual);
        if (sw & 0x2000) {
            g_tx.controlword = 0x000F;
            rt_exchange(&toff);
            return EC_RT_ERR_HOMING_ERROR;
        }
        if (sw & 0x1000) {
            g_tx.controlword = 0x000F;
            rt_exchange(&toff);
            return 0;
        }
        rt_exchange(&toff);
    }
    g_tx.controlword = 0x000F;
    rt_exchange(&toff);
    return EC_RT_ERR_HOMING_ATTAIN;
}

int ec_rt_park_cycle(int64_t *toff_ns) {
    if (!g_pd) {
        fprintf(stderr, "ec_rt: park_cycle before bringup_finish — PDO map not bound\n");
        abort();
    }
    g_tx.controlword = 0;
    g_tx.target_position = EC_READ_S32(g_pd + i_position_actual);
    return rt_exchange(toff_ns);
}

/* AL status register 0x0134 via a register request, pumped on the DC grid until
 * the master FSM completes it. Diagnostic-only; called after the DC loop halts. */
static uint16_t read_al_status_code(void) {
    if (!g_al_req) return 0;
    ecrt_reg_request_read(g_al_req, 0x0134, 2);
    for (int i = 0; i < 50; i++) {
        int64_t t = 0;
        rt_exchange(&t);
        ec_request_state_t rs = ecrt_reg_request_state(g_al_req);
        if (rs == EC_REQUEST_SUCCESS) return EC_READ_U16(ecrt_reg_request_data(g_al_req));
        if (rs == EC_REQUEST_ERROR) return 0;
    }
    return 0;
}

void ec_rt_al_status(uint16_t *state, uint16_t *alstatuscode) {
    ec_slave_config_state_t st;
    ecrt_slave_config_state(g_sc, &st);
    *state = (uint16_t)st.al_state;
    *alstatuscode = read_al_status_code();
}

void ec_rt_disable(void) {
    g_enabled = 0;
    for (int i = 0; i < 100; i++) {
        g_tx.controlword = 0x0006;
        g_tx.target_position = EC_READ_S32(g_pd + i_position_actual);
        g_tx.velocity_offset = 0;
        g_tx.torque_offset = 0;
        int64_t t = 0;
        rt_exchange(&t);
    }
}

void ec_rt_dump_al_state(void) {
    ec_slave_config_state_t st;
    ecrt_slave_config_state(g_sc, &st);
    uint16_t code = read_al_status_code();
    fprintf(stderr,
            "ec_rt: slave al_state=0x%02x online=%u operational=%u al_status=0x%04x\n",
            st.al_state, st.online, st.operational, code);
}

void ec_rt_shutdown(void) {
    if (!g_master) return;
    if (g_activated) {
        ecrt_master_deactivate(g_master);
        g_activated = 0;
    }
    ecrt_release_master(g_master);
    g_master = NULL;
    g_pd = NULL;
}
