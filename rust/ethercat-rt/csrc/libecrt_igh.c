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
 * One chain drives N slaves: one master, one domain, one DC grid. Each slave is
 * a slot (0-based) configured at the topological ring position the caller passes
 * in slave_positions[]. The static PDO map (out_t 18 bytes / in_t 32 bytes) is
 * registered per slave into the single shared domain image; each slot's input
 * offsets read from its partition with the EC_READ accessors, and its staged
 * outputs are flushed into the image each cycle (see rt_exchange for why the
 * staging is required).
 */
#define _GNU_SOURCE
#include "libecrt.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#include <sched.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <ecrt.h>

/* Drive identity (ethercat slaves -v): AS715N_sAxis A6-EC servo. Every slave on
 * the chain must match. */
#define VENDOR_ID    0x00400000u
#define PRODUCT_CODE 0x00000715u
#define SLAVE_ALIAS  0

/* DC AssignActivate from the A6 ESI (Dc/OpMode "DC-Synchron"): SYNC0-only at
 * 1x the cycle period. The A6-EC requires SYNC0 active before SAFE-OP (else
 * AL 0x0030). The ESI ships ShiftTimeSync0=0; we shift SYNC0 half a cycle so
 * the process-data frame (sent at the cycle boundary) arrives mid-window,
 * maximizing margin to the drive's latch instant. */
#define DC_ASSIGN_ACTIVATE 0x0300

/* DC convergence gate: a non-reference slave counts as clock-locked once
 * DC_LOCK_SAMPLES consecutive reads of its ESC System Time Difference register
 * (0x092C) are within DC_LOCK_TOLERANCE_NS of the reference clock. Gate budget
 * DC_CONVERGE_BUDGET_NS; OP walk budget OP_WALK_BUDGET_NS is generous because
 * the kernel master's FSM keeps retrying the OP transition on its own while the
 * clocks converge — tearing down and re-activating would only restart
 * convergence from a fresh random offset. */
#define DC_LOCK_TOLERANCE_NS  2000
#define DC_LOCK_SAMPLES       100
#define DC_CONVERGE_BUDGET_NS 15.0e9
#define OP_WALK_BUDGET_NS     20.0e9

#define OUT_BYTES 18
#define IN_BYTES  32

static ec_master_t *g_master;
static ec_domain_t *g_domain;
static uint8_t     *g_pd;     /* domain process image (the LRW datagram buffer) */

typedef struct {
    ec_slave_config_t *sc;
    ec_reg_request_t  *al_req; /* reads AL status register 0x0134 for diagnostics */
    ec_reg_request_t  *dc_req; /* reads System Time Difference register 0x092C */
    int32_t            dc_diff_ns;     /* last decoded 0x092C sample */
    int                dc_have_diff;   /* at least one 0x092C sample landed */
    unsigned           dc_lock_streak; /* consecutive in-tolerance 0x092C samples */
    int32_t            pos;    /* topological ring position (0-based) */

    /* Output field offsets in the domain image (SM2 / RxPDO 1600h = out_t). */
    unsigned o_controlword, o_target, o_touch_probe, o_phys_outputs,
        o_velocity_offset, o_torque_offset;
    /* Input field offsets (SM3 / TxPDO 1A00h = in_t). */
    unsigned i_error_code, i_statusword, i_position_actual, i_velocity_actual,
        i_torque_actual, i_following_error, i_tp_status, i_tp1_pos, i_tp2_pos,
        i_digital_inputs;

    /* Staged outputs. ecrt_master_receive overwrites the domain image's output
     * region with the echo of the previously-sent frame, so a value written
     * directly into the image before the receive would be clobbered. Callers
     * stage here instead; rt_exchange flushes this into the image after receive,
     * just before queue/send. */
    struct {
        uint16_t controlword;
        int32_t  target_position;
        uint16_t touch_probe;
        uint32_t phys_outputs;
        int32_t  velocity_offset;
        int16_t  torque_offset;
    } tx;

    int enabled;
} slave_t;

static slave_t g_slaves[EC_RT_MAX_SLAVES];
static int     g_num_slaves;

static int64_t g_cycle_ns;
static struct timespec g_ts;
static int g_activated;

#define TIMESPEC2NS(T) ((uint64_t)(T).tv_sec * 1000000000ULL + (uint64_t)(T).tv_nsec)

/* A slave index out of [0, g_num_slaves) is a pure programming bug in the
 * caller, never transient — abort loudly rather than read past the array. */
static void check_idx(int s) {
    if (s < 0 || s >= g_num_slaves) {
        fprintf(stderr, "ec_rt: slave index %d out of range [0,%d)\n", s, g_num_slaves);
        abort();
    }
}

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

/* Holding /dev/cpu_dma_latency at 0 pins every core out of deep cpuidle
 * states for the life of this fd. Without it an idle machine drops into
 * deep C-states and the RT thread pays the exit latency on wakeup — the
 * bench park of 2026-07-21 00:03 measured wake_late_ns=174215 on an
 * otherwise idle system. The fd is intentionally never closed. */
static int g_cpu_dma_latency_fd = -1;

static int hold_cpu_dma_latency(void) {
    g_cpu_dma_latency_fd = open("/dev/cpu_dma_latency", O_RDWR);
    if (g_cpu_dma_latency_fd < 0) {
        fprintf(stderr, "ec_rt: open /dev/cpu_dma_latency failed: %s — "
                "grant the endpoint write access\n", strerror(errno));
        return EC_RT_ERR_RT_QOS;
    }
    int32_t zero = 0;
    if (write(g_cpu_dma_latency_fd, &zero, sizeof(zero)) != sizeof(zero)) {
        fprintf(stderr, "ec_rt: cpu_dma_latency qos write failed: %s\n",
                strerror(errno));
        return EC_RT_ERR_RT_QOS;
    }
    return 0;
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
    int qos = hold_cpu_dma_latency();
    if (qos != 0) return qos;
    return 0;
}

/* If the deadline has gone stale (a blocking transaction or a wait between
 * exchanges overran the cycle), skip forward rather than letting
 * clock_nanosleep return immediately cycle after cycle: that catch-up burst
 * sends frames far outside the SYNC0 window and reads as sync loss.
 *
 * The skip is a whole number of cycles, never "now": the slaves' SYNC0
 * generators fire at offsets programmed from this grid at DC activation, so
 * any fractional-cycle move would re-roll the frame-to-latch margin to a
 * random value for the rest of the session. Staying on the grid keeps the
 * half-cycle margin deterministic across bringup's blocking waits and any
 * mid-session stall. The counter lets the endpoint report each skip. */
static uint32_t g_reanchor_count;
static int64_t g_last_reanchor_behind_ns;

static void reanchor_if_stale(void) {
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    int64_t behind_ns = (now.tv_sec - g_ts.tv_sec) * 1000000000LL
                      + (now.tv_nsec - g_ts.tv_nsec);
    if (behind_ns > g_cycle_ns) {
        int64_t skip = (behind_ns + g_cycle_ns - 1) / g_cycle_ns;
        add_ts(&g_ts, skip * g_cycle_ns);
        g_reanchor_count++;
        g_last_reanchor_behind_ns = behind_ns;
        fprintf(stderr,
                "ec_rt: cycle overran by %lld ns; skipped %lld cycle(s) "
                "forward on the grid (count %u) — SYNC0 phase preserved\n",
                (long long)behind_ns, (long long)skip, g_reanchor_count);
    }
}

uint32_t ec_rt_reanchor_count(void) { return g_reanchor_count; }
int64_t ec_rt_last_reanchor_behind_ns(void) { return g_last_reanchor_behind_ns; }

static void flush_outputs(void) {
    for (int s = 0; s < g_num_slaves; s++) {
        slave_t *sl = &g_slaves[s];
        EC_WRITE_U16(g_pd + sl->o_controlword, sl->tx.controlword);
        EC_WRITE_S32(g_pd + sl->o_target, sl->tx.target_position);
        EC_WRITE_U16(g_pd + sl->o_touch_probe, sl->tx.touch_probe);
        EC_WRITE_U32(g_pd + sl->o_phys_outputs, sl->tx.phys_outputs);
        EC_WRITE_S32(g_pd + sl->o_velocity_offset, sl->tx.velocity_offset);
        EC_WRITE_S16(g_pd + sl->o_torque_offset, sl->tx.torque_offset);
    }
}

/* One DC exchange at a fixed cycle period covering every slave in the shared
 * domain. The sleep lets the previous cycle's frame round-trip, so the receive
 * at the top refreshes the input image. That same receive overwrites the
 * image's output region with the echo of the previously-sent frame, so every
 * slave's staged outputs (slave_t.tx) are flushed into the image AFTER receive,
 * then queued and sent. DC drift is compensated by the kernel master:
 * application_time anchors the network clock to the CLOCK_MONOTONIC wake grid
 * and sync_reference_clock/sync_slave_clocks distribute it — so no
 * application-side phase loop is needed.
 *
 * *toff reports this frame's lateness relative to the nominal SYNC0 latch
 * (wake grid + the half-cycle shift programmed at DC config): negative is
 * margin to spare, positive means the drives latched last cycle's target.
 * The measurement is taken when ecrt_master_send returns; WIRE_FLIGHT_NS
 * covers the propagation to the last slave in the chain (measured 2.52 us
 * on the 4-drive bench, ~0.84 us per hop) so the reported number reflects
 * arrival, not transmit. Overrun skips stay on the grid
 * (reanchor_if_stale), so the half-cycle latch phase holds all run. */
#define WIRE_FLIGHT_NS 3000

/* Per-cycle stage breakdown of the last rt_exchange, for attributing a
 * frame-timing spike: wake_late = how far past the grid tick the thread
 * actually woke (kernel scheduling latency), recv = ecrt_master_receive
 * (the polled macb RX path), process = ecrt_domain_process (host-side
 * datagram-to-image work), send = output flush + DC sync + queue + ecrt
 * send (the TX path). Whole-exchange totals alone cannot separate a late
 * wakeup from a slow bus path. */
static int64_t g_last_wake_late_ns;
static int64_t g_last_recv_ns;
static int64_t g_last_process_ns;
static int64_t g_last_send_ns;

void ec_rt_cycle_stage_ns(int64_t *wake_late, int64_t *recv, int64_t *process,
                          int64_t *send) {
    if (wake_late) *wake_late = g_last_wake_late_ns;
    if (recv) *recv = g_last_recv_ns;
    if (process) *process = g_last_process_ns;
    if (send) *send = g_last_send_ns;
}

static int rt_exchange(int64_t *toff) {
    add_ts(&g_ts, g_cycle_ns);
    reanchor_if_stale();
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &g_ts, NULL);
    struct timespec woke;
    clock_gettime(CLOCK_MONOTONIC, &woke);
    g_last_wake_late_ns = TIMESPEC2NS(woke) - TIMESPEC2NS(g_ts);

    ecrt_master_application_time(g_master, TIMESPEC2NS(g_ts));
    ecrt_master_receive(g_master);
    struct timespec bus_read;
    clock_gettime(CLOCK_MONOTONIC, &bus_read);
    g_last_recv_ns = TIMESPEC2NS(bus_read) - TIMESPEC2NS(woke);

    ecrt_domain_process(g_domain);
    struct timespec received;
    clock_gettime(CLOCK_MONOTONIC, &received);
    g_last_process_ns = TIMESPEC2NS(received) - TIMESPEC2NS(bus_read);

    flush_outputs();
    ecrt_master_sync_reference_clock(g_master);
    ecrt_master_sync_slave_clocks(g_master);
    ecrt_domain_queue(g_domain);
    ecrt_master_send(g_master);

    struct timespec sent;
    clock_gettime(CLOCK_MONOTONIC, &sent);
    g_last_send_ns = TIMESPEC2NS(sent) - TIMESPEC2NS(received);
    int64_t lateness_ns =
        TIMESPEC2NS(sent) + WIRE_FLIGHT_NS - (TIMESPEC2NS(g_ts) + g_cycle_ns / 2);

    ec_domain_state_t ds;
    ecrt_domain_state(g_domain, &ds);
    if (toff) *toff = lateness_ns;
    return (int)ds.working_counter;
}

static int reg_entry(slave_t *sl, uint16_t index, uint8_t sub, unsigned *off) {
    unsigned bit = 0;
    int byte = ecrt_slave_config_reg_pdo_entry(sl->sc, index, sub, g_domain, &bit);
    if (byte < 0 || bit != 0) {
        fprintf(stderr, "ec_rt: reg_pdo_entry %04X:%02X failed (byte=%d bit=%u)\n",
                index, sub, byte, bit);
        return -1;
    }
    *off = (unsigned)byte;
    return 0;
}

static int register_pdo_entries(slave_t *sl) {
    return reg_entry(sl, 0x6040, 0x00, &sl->o_controlword)
        || reg_entry(sl, 0x607A, 0x00, &sl->o_target)
        || reg_entry(sl, 0x60B8, 0x00, &sl->o_touch_probe)
        || reg_entry(sl, 0x60FE, 0x01, &sl->o_phys_outputs)
        || reg_entry(sl, 0x60B1, 0x00, &sl->o_velocity_offset)
        || reg_entry(sl, 0x60B2, 0x00, &sl->o_torque_offset)
        || reg_entry(sl, 0x603F, 0x00, &sl->i_error_code)
        || reg_entry(sl, 0x6041, 0x00, &sl->i_statusword)
        || reg_entry(sl, 0x6064, 0x00, &sl->i_position_actual)
        || reg_entry(sl, 0x606C, 0x00, &sl->i_velocity_actual)
        || reg_entry(sl, 0x6077, 0x00, &sl->i_torque_actual)
        || reg_entry(sl, 0x60F4, 0x00, &sl->i_following_error)
        || reg_entry(sl, 0x60B9, 0x00, &sl->i_tp_status)
        || reg_entry(sl, 0x60BA, 0x00, &sl->i_tp1_pos)
        || reg_entry(sl, 0x60BC, 0x00, &sl->i_tp2_pos)
        || reg_entry(sl, 0x60FD, 0x00, &sl->i_digital_inputs);
}

/* Stage one slave at its current safe state (target tracks actual, no offsets):
 * controlword 0x000F if it is already torque-enabled, else 0x0006. Used to hold
 * the rest of the domain steady while a per-slave control loop walks one slave. */
static void stage_hold(slave_t *sl) {
    sl->tx.controlword     = sl->enabled ? 0x000F : 0x0006;
    sl->tx.target_position = EC_READ_S32(g_pd + sl->i_position_actual);
    sl->tx.velocity_offset = 0;
    sl->tx.torque_offset   = 0;
}

static void stage_hold_others(int except) {
    for (int s = 0; s < g_num_slaves; s++) {
        if (s != except) stage_hold(&g_slaves[s]);
    }
}

/* A session that dies mid-OP (an abrupt endpoint exit drops the cyclic frames
 * the moment the kernel releases the master) latches the drive's sync-loss
 * alarm (0x8700 / ErC1.1). The CiA402 fault reset (6040h bit 7 rising edge)
 * clears it over SDO in PRE-OP, where sync monitoring is inactive — the alarm
 * cannot re-raise mid-clear the way it can during the post-OP park loop on a
 * still-settling DC clock. Runs in the master's idle phase (pre-activate),
 * where ecrt_master_sdo_* reach the slave's mailbox at PRE-OP. */
static void clear_latched_alarm_in_preop(slave_t *sl) {
    uint8_t buf[2];
    size_t rs = 0;
    uint32_t abort = 0;
    if (ecrt_master_sdo_upload(g_master, sl->pos, 0x603F, 0x00, buf, 2, &rs, &abort) != 0)
        return;
    uint16_t err = (uint16_t)(buf[0] | (buf[1] << 8));
    if (err == 0) return;
    fprintf(stderr, "ec_rt: clearing latched drive alarm 0x%04x in PRE-OP (slave %d)\n",
            err, (int)sl->pos);
    uint16_t cw = 0x0000;
    ecrt_master_sdo_download(g_master, sl->pos, 0x6040, 0x00, (uint8_t *)&cw, 2, &abort);
    cw = 0x0080;
    ecrt_master_sdo_download(g_master, sl->pos, 0x6040, 0x00, (uint8_t *)&cw, 2, &abort);
    cw = 0x0000;
    ecrt_master_sdo_download(g_master, sl->pos, 0x6040, 0x00, (uint8_t *)&cw, 2, &abort);
    if (ecrt_master_sdo_upload(g_master, sl->pos, 0x603F, 0x00, buf, 2, &rs, &abort) == 0) {
        err = (uint16_t)(buf[0] | (buf[1] << 8));
        if (err != 0)
            fprintf(stderr, "ec_rt: drive alarm 0x%04x survived PRE-OP fault reset (slave %d); "
                    "the post-OP park loop will keep pulsing\n", err, (int)sl->pos);
    }
}

static int configure_slave(slave_t *sl) {
    sl->sc = ecrt_master_slave_config(g_master, SLAVE_ALIAS, sl->pos,
                                      VENDOR_ID, PRODUCT_CODE);
    if (!sl->sc) {
        fprintf(stderr, "ec_rt: no slave at position %d (vendor 0x%08x product 0x%08x)\n",
                (int)sl->pos, VENDOR_ID, PRODUCT_CODE);
        return EC_RT_ERR_NO_SLAVES;
    }

    if (ecrt_slave_config_pdos(sl->sc, EC_END, syncs) != 0)
        return EC_RT_ERR_PDO_REMAP;
    if (register_pdo_entries(sl) != 0)
        return EC_RT_ERR_PDO_REMAP;

    /* CSP mode and a disabled following-error timeout, then route both
     * feedforward sources (speed 60B1h, torque 60B2h) to "communication"
     * (C01.13/C01.16 -> 5) with 0% additional FF (C01.14/C01.17 -> 0). The
     * master applies them in PRE-OP. A rejected config SDO leaves the slave
     * short of OP -> OP_TIMEOUT. */
    ecrt_slave_config_sdo8(sl->sc, 0x6060, 0x00, 8);
    ecrt_slave_config_sdo16(sl->sc, 0x6066, 0x00, 0);
    ecrt_slave_config_sdo16(sl->sc, 0x2001, 0x14, 5);
    ecrt_slave_config_sdo16(sl->sc, 0x2001, 0x15, 0);
    ecrt_slave_config_sdo16(sl->sc, 0x2001, 0x17, 5);
    ecrt_slave_config_sdo16(sl->sc, 0x2001, 0x18, 0);

    ecrt_slave_config_dc(sl->sc, DC_ASSIGN_ACTIVATE, (uint32_t)g_cycle_ns,
                         (int32_t)(g_cycle_ns / 2), 0, 0);

    sl->al_req = ecrt_slave_config_create_reg_request(sl->sc, 2);
    sl->dc_req = ecrt_slave_config_create_reg_request(sl->sc, 4);
    if (!sl->al_req || !sl->dc_req) {
        fprintf(stderr, "ec_rt: reg request allocation failed (slave %d)\n",
                (int)sl->pos);
        return EC_RT_ERR_EC_INIT;
    }

    clear_latched_alarm_in_preop(sl);
    return 0;
}

/* Phase 1: request the master, configure every slave, declare the PDO maps,
 * stage the static drive setup as config SDOs (applied by the master in PRE-OP
 * before OP), and arm DC — but do not activate. The master's idle state machine
 * brings each slave to PRE-OP, where the caller does its session SDO work (drive
 * limits) via ecrt_master_sdo_* before phase 2. */
int ec_rt_bringup_preop(const char *ifname, int64_t cycle_ns, int rt_cpu, int rt_prio,
                        const int32_t *slave_positions, int num_slaves) {
    (void)ifname; /* the IgH master is bound to the NIC via /etc/ethercat.conf */
    if (num_slaves < 1 || num_slaves > EC_RT_MAX_SLAVES) {
        fprintf(stderr, "ec_rt: num_slaves %d outside [1,%d]\n",
                num_slaves, EC_RT_MAX_SLAVES);
        return EC_RT_ERR_TOO_MANY_SLAVES;
    }
    g_cycle_ns = cycle_ns < 250000 ? 250000 : cycle_ns;
    g_activated = 0;
    g_num_slaves = num_slaves;
    memset(g_slaves, 0, sizeof(g_slaves));
    for (int s = 0; s < num_slaves; s++) g_slaves[s].pos = slave_positions[s];

    int rt_rc = go_realtime(rt_cpu, rt_prio);
    if (rt_rc != 0) return rt_rc;

    g_master = ecrt_request_master(0);
    if (!g_master) return EC_RT_ERR_EC_INIT;

    g_domain = ecrt_master_create_domain(g_master);
    if (!g_domain) return EC_RT_ERR_EC_INIT;

    for (int s = 0; s < num_slaves; s++) {
        int rc = configure_slave(&g_slaves[s]);
        if (rc != 0) {
            fprintf(stderr, "ec_rt: slot %d (position %d) preop config failed rc=%d\n",
                    s, (int)g_slaves[s].pos, rc);
            return rc;
        }
    }

    /* Pin slot 0 as the DC reference clock. Without an explicit selection IgH's
     * auto-pick makes multi-slave clock distribution racy at startup: a
     * non-reference slave's SYNC0 may not lock in time, and the A6-EC gates
     * PRE-OP -> SAFE-OP on SYNC0, so it stalls at PRE-OP and the OP walk times
     * out. Explicit selection lets the reference time distribute deterministically
     * so every slave's SYNC0 is up before the OP transition. */
    if (ecrt_master_select_reference_clock(g_master, g_slaves[0].sc) != 0) {
        fprintf(stderr, "ec_rt: select_reference_clock(slot 0, position %d) failed\n",
                (int)g_slaves[0].pos);
        return EC_RT_ERR_EC_INIT;
    }
    return 0;
}

/* CiA402 fault reset (6040h bit 7) needs a rising edge: hold it low and high on
 * alternating ~10-cycle windows so a latched fault clears within the walk loop. */
static uint16_t fault_reset_pulse(int64_t cycle) {
    return ((cycle / 10) % 2) ? 0x0080 : 0x0000;
}

/* One CiA402 park step for a slave: drive toward Ready-to-Switch-On (0x0021) at
 * controlword 0x0006, pulsing fault-reset if faulted. Returns 1 once parked. */
static int park_step(slave_t *sl, int64_t pc) {
    uint16_t sw = EC_READ_U16(g_pd + sl->i_statusword);
    sl->tx.target_position = EC_READ_S32(g_pd + sl->i_position_actual);
    if (sw & 0x0008) {
        sl->tx.controlword = fault_reset_pulse(pc);
        return 0;
    }
    if ((sw & 0x006F) == 0x0021) {
        sl->tx.controlword = 0x0006;
        sl->enabled = 0;
        return 1;
    }
    sl->tx.controlword = 0x0006;
    return 0;
}

/* ESC System Time Difference (0x092C) is sign-magnitude: bit 31 is the sign,
 * bits 30..0 the offset magnitude in ns. */
static int32_t decode_system_time_diff(uint32_t raw) {
    int32_t mag = (int32_t)(raw & 0x7FFFFFFFu);
    return (raw & 0x80000000u) ? -mag : mag;
}

/* Keep a continuous stream of 0x092C samples flowing for one slave: harvest a
 * completed register read into dc_diff_ns / dc_lock_streak and immediately
 * re-issue. Call once per rt_exchange cycle. */
static void pump_dc_diff(slave_t *sl) {
    switch (ecrt_reg_request_state(sl->dc_req)) {
    case EC_REQUEST_BUSY:
        return;
    case EC_REQUEST_SUCCESS:
        sl->dc_diff_ns = decode_system_time_diff(
            EC_READ_U32(ecrt_reg_request_data(sl->dc_req)));
        sl->dc_have_diff = 1;
        if (sl->dc_diff_ns >= -DC_LOCK_TOLERANCE_NS &&
            sl->dc_diff_ns <= DC_LOCK_TOLERANCE_NS)
            sl->dc_lock_streak++;
        else
            sl->dc_lock_streak = 0;
        break;
    case EC_REQUEST_UNUSED:
    case EC_REQUEST_ERROR:
        break;
    }
    ecrt_reg_request_read(sl->dc_req, 0x092C, 4);
}

static void pump_dc_diff_all(void) {
    for (int s = 0; s < g_num_slaves; s++) pump_dc_diff(&g_slaves[s]);
}

/* Slot 0 is the DC reference clock — its 0x092C measures against the master's
 * CLOCK_MONOTONIC grid, not against the bus reference, so only non-reference
 * slots gate the bring-up. */
static int dc_locked_all(void) {
    for (int s = 1; s < g_num_slaves; s++)
        if (g_slaves[s].dc_lock_streak < DC_LOCK_SAMPLES) return 0;
    return 1;
}

static void log_dc_diffs(const char *tag) {
    for (int s = 0; s < g_num_slaves; s++) {
        slave_t *sl = &g_slaves[s];
        ec_slave_config_state_t st;
        ecrt_slave_config_state(sl->sc, &st);
        fprintf(stderr,
                "ec_rt: %s slot %d (position %d) al_state=0x%02x dc_diff=%s%d ns streak=%u\n",
                tag, s, (int)sl->pos, st.al_state,
                sl->dc_have_diff ? "" : "unsampled ", sl->dc_have_diff ? sl->dc_diff_ns : 0,
                sl->dc_lock_streak);
    }
}

/* Phase 2: activate the master (it walks every slave PRE-OP -> SAFE-OP -> OP as
 * we cycle, applying the staged config SDOs and DC), stabilize the DC loop,
 * confirm all OP, then park each at CiA402 Ready-to-Switch-On (no torque). From
 * the first cycle here the caller must never pause process data — every wait
 * goes through the cycle/park helpers, else the SM watchdog drops a drive to
 * SAFE-OP. */
int ec_rt_bringup_finish(void) {
    /* Seed the application time from the DC-loop clock grid before activating.
     * ecrt_master_activate derives each slave's SYNC0 cyclic start time from the
     * master application time; activating with the default 0 places the
     * non-reference slaves' SYNC0 at a per-boot-random phase they may never lock,
     * so a second drive stalls below OP and the walk times out while the
     * reference slave (SYNC0 off its own clock) always makes OP. */
    clock_gettime(CLOCK_MONOTONIC, &g_ts);
    ecrt_master_application_time(g_master, TIMESPEC2NS(g_ts));
    if (ecrt_master_activate(g_master) != 0) return EC_RT_ERR_EC_INIT;
    g_activated = 1;

    g_pd = ecrt_domain_data(g_domain);
    if (!g_pd) return EC_RT_ERR_PDO_SIZE;
    size_t want = (size_t)(g_num_slaves * (OUT_BYTES + IN_BYTES));
    if (ecrt_domain_size(g_domain) != want) {
        fprintf(stderr, "ec_rt: domain size %zu, expected %zu (%d slaves)\n",
                ecrt_domain_size(g_domain), want, g_num_slaves);
        return EC_RT_ERR_PDO_SIZE;
    }

    int64_t toff = 0;
    const int64_t cycles_per_sec = (int64_t)(1.0e9 / g_cycle_ns);

    /* Walk every slave to OP while continuously sampling each slave's DC offset
     * (0x092C): the master FSM advances each PRE-OP -> SAFE-OP -> OP on its own
     * and keeps retrying a refused OP transition, and sync_slave_clocks pulls a
     * non-reference slave's clock in from any starting offset — so the right
     * move on a slow slave is to keep cycling, never to tear down and re-roll. */
    int all_op = 0;
    for (int64_t i = 0; i < (int64_t)(OP_WALK_BUDGET_NS / g_cycle_ns); i++) {
        for (int s = 0; s < g_num_slaves; s++) {
            g_slaves[s].tx.controlword = 0;
            g_slaves[s].tx.target_position = EC_READ_S32(g_pd + g_slaves[s].i_position_actual);
        }
        rt_exchange(&toff);
        pump_dc_diff_all();
        if (i % cycles_per_sec == cycles_per_sec - 1) log_dc_diffs("op_walk");
        all_op = 1;
        for (int s = 0; s < g_num_slaves; s++) {
            ec_slave_config_state_t st;
            ecrt_slave_config_state(g_slaves[s].sc, &st);
            if (!st.operational) { all_op = 0; break; }
        }
        if (all_op) break;
    }
    if (!all_op) {
        log_dc_diffs("op_timeout");
        return EC_RT_ERR_OP_TIMEOUT;
    }

    /* Hold at controlword 0 until every non-reference slave's clock is measured
     * as locked to the reference (0x092C within tolerance for a sustained
     * streak) before walking CiA-402 — commanding the drive on an unconverged
     * clock is what latches the sync-loss alarms (ErC1.1 / 0x8700). */
    int dc_locked = 0;
    for (int64_t i = 0; i < (int64_t)(DC_CONVERGE_BUDGET_NS / g_cycle_ns); i++) {
        for (int s = 0; s < g_num_slaves; s++) {
            g_slaves[s].tx.controlword = 0;
            g_slaves[s].tx.target_position = EC_READ_S32(g_pd + g_slaves[s].i_position_actual);
        }
        rt_exchange(&toff);
        pump_dc_diff_all();
        if (i % cycles_per_sec == cycles_per_sec - 1) log_dc_diffs("dc_converge");
        if (dc_locked_all()) { dc_locked = 1; break; }
    }
    if (!dc_locked) {
        log_dc_diffs("dc_converge_timeout");
        return EC_RT_ERR_DC_CONVERGE;
    }
    log_dc_diffs("dc_locked");
    for (int s = 0; s < g_num_slaves; s++)
        fprintf(stderr, "ec_rt: OP reached slot %d; park entry sw=0x%04x err=0x%04x\n",
                s, EC_READ_U16(g_pd + g_slaves[s].i_statusword),
                EC_READ_U16(g_pd + g_slaves[s].i_error_code));

    int parked[EC_RT_MAX_SLAVES] = {0};
    for (int64_t pc = 0; pc < 3000; pc++) {
        int all_parked = 1;
        for (int s = 0; s < g_num_slaves; s++) {
            if (parked[s]) {
                g_slaves[s].tx.controlword = 0x0006;
                g_slaves[s].tx.target_position = EC_READ_S32(g_pd + g_slaves[s].i_position_actual);
            } else if (park_step(&g_slaves[s], pc)) {
                parked[s] = 1;
            } else {
                all_parked = 0;
            }
        }
        rt_exchange(&toff);
        if (all_parked) return 0;
    }
    for (int s = 0; s < g_num_slaves; s++)
        if (!parked[s])
            fprintf(stderr, "ec_rt: CiA402 park timeout slot %d sw=0x%04x err=0x%04x\n",
                    s, EC_READ_U16(g_pd + g_slaves[s].i_statusword),
                    EC_READ_U16(g_pd + g_slaves[s].i_error_code));
    return EC_RT_ERR_CIA402_TIMEOUT;
}

/* Cycles to dwell in Switched-On (target tracking actual) before commanding
 * Operation-Enabled. A non-reference DC drive can lose its CSP interpolation
 * base across a warm endpoint restart; without re-baselining, its first enabled
 * cycle reads target-vs-stale-reference as a huge one-cycle jump and trips
 * Er87.1 the instant torque comes on. Holding Switched-On with target=actual
 * for this many cycles lets the drive adopt the present position as its base,
 * which a power cycle would otherwise do for free. */
#define ENABLE_SWITCHED_ON_DWELL 200

static void stage_enable_phase(uint16_t controlword) {
    for (int s = 0; s < g_num_slaves; s++) {
        slave_t *sl = &g_slaves[s];
        sl->tx.controlword = controlword;
        sl->tx.target_position = EC_READ_S32(g_pd + sl->i_position_actual);
        sl->tx.velocity_offset = 0;
        sl->tx.torque_offset = 0;
    }
}

static int cia402_enable_state(uint16_t sw) {
    if (sw & 0x0008) return -1;
    if ((sw & 0x004F) == 0x0040) return 1;
    if ((sw & 0x006F) == 0x0021) return 1;
    if ((sw & 0x006F) == 0x0023) return 1;
    if ((sw & 0x006F) == 0x0027) return 1;
    return 0;
}

static int all_statuswords_are(uint16_t mask, uint16_t value) {
    for (int s = 0; s < g_num_slaves; s++) {
        slave_t *sl = &g_slaves[s];
        uint16_t sw = EC_READ_U16(g_pd + sl->i_statusword);
        if (cia402_enable_state(sw) != 1) {
            fprintf(stderr,
                    "ec_rt: chain enable rejected slot %d statusword=0x%04x err=0x%04x\n",
                    s, sw, EC_READ_U16(g_pd + sl->i_error_code));
            return -1;
        }
    }
    for (int s = 0; s < g_num_slaves; s++)
        if ((EC_READ_U16(g_pd + g_slaves[s].i_statusword) & mask) != value)
            return 0;
    return 1;
}

int ec_rt_enable_all(void) {
    enum {
        ENABLE_SHUTDOWN,
        ENABLE_SWITCH_ON,
        ENABLE_DWELL,
        ENABLE_OPERATION,
    } phase = ENABLE_SHUTDOWN;
    int64_t toff = 0;
    int64_t switched_on_dwell = 0;

    for (int64_t pc = 0; pc < 3000; pc++) {
        switch (phase) {
        case ENABLE_SHUTDOWN:
            stage_enable_phase(0x0006);
            break;
        case ENABLE_SWITCH_ON:
        case ENABLE_DWELL:
            stage_enable_phase(0x0007);
            break;
        case ENABLE_OPERATION:
            stage_enable_phase(0x000F);
            break;
        }
        rt_exchange(&toff);

        if (phase == ENABLE_DWELL) switched_on_dwell++;
        int state;
        switch (phase) {
        case ENABLE_SHUTDOWN:
            state = all_statuswords_are(0x006F, 0x0021);
            if (state < 0) return EC_RT_ERR_CIA402_TIMEOUT;
            if (state) phase = ENABLE_SWITCH_ON;
            break;
        case ENABLE_SWITCH_ON:
            state = all_statuswords_are(0x006F, 0x0023);
            if (state < 0) return EC_RT_ERR_CIA402_TIMEOUT;
            if (state) phase = ENABLE_DWELL;
            break;
        case ENABLE_DWELL:
            state = all_statuswords_are(0x006F, 0x0023);
            if (state < 0) return EC_RT_ERR_CIA402_TIMEOUT;
            if (state) {
                if (switched_on_dwell == ENABLE_SWITCHED_ON_DWELL)
                    phase = ENABLE_OPERATION;
            } else {
                switched_on_dwell = 0;
                phase = ENABLE_SWITCH_ON;
            }
            break;
        case ENABLE_OPERATION:
            state = all_statuswords_are(0x006F, 0x0027);
            if (state < 0) return EC_RT_ERR_CIA402_TIMEOUT;
            if (!state) break;
            for (int s = 0; s < g_num_slaves; s++) {
                slave_t *sl = &g_slaves[s];
                sl->enabled = 1;
                fprintf(stderr,
                        "ec_rt: slot %d operation-enabled (switched-on dwell=%lld, sw=0x%04x err=0x%04x)\n",
                        s, (long long)switched_on_dwell,
                        EC_READ_U16(g_pd + sl->i_statusword),
                        EC_READ_U16(g_pd + sl->i_error_code));
            }
            return 0;
        }
    }
    for (int s = 0; s < g_num_slaves; s++) {
        slave_t *sl = &g_slaves[s];
        fprintf(stderr,
                "ec_rt: chain enable timeout slot %d statusword=0x%04x err=0x%04x\n",
                s, EC_READ_U16(g_pd + sl->i_statusword),
                EC_READ_U16(g_pd + sl->i_error_code));
    }
    return EC_RT_ERR_CIA402_TIMEOUT;
}

int ec_rt_cycle(int64_t *toff_ns) {
    for (int s = 0; s < g_num_slaves; s++) {
        slave_t *sl = &g_slaves[s];
        if (sl->enabled) {
            sl->tx.controlword = 0x000F;
        } else {
            sl->tx.controlword = 0x0006;
            sl->tx.target_position = EC_READ_S32(g_pd + sl->i_position_actual);
        }
    }
    return rt_exchange(toff_ns);
}

uint64_t ec_rt_cycle_time_ns(void) {
    return TIMESPEC2NS(g_ts);
}

void ec_rt_set_target_position(int slave, int32_t counts) {
    check_idx(slave);
    g_slaves[slave].tx.target_position = counts;
}
int32_t ec_rt_get_position_actual(int slave) {
    check_idx(slave);
    return EC_READ_S32(g_pd + g_slaves[slave].i_position_actual);
}
int32_t ec_rt_get_velocity_actual(int slave) {
    check_idx(slave);
    return EC_READ_S32(g_pd + g_slaves[slave].i_velocity_actual);
}
uint16_t ec_rt_get_statusword(int slave) {
    check_idx(slave);
    return EC_READ_U16(g_pd + g_slaves[slave].i_statusword);
}
uint16_t ec_rt_get_error_code(int slave) {
    check_idx(slave);
    return EC_READ_U16(g_pd + g_slaves[slave].i_error_code);
}
int32_t ec_rt_get_following_error(int slave) {
    check_idx(slave);
    return EC_READ_S32(g_pd + g_slaves[slave].i_following_error);
}
void ec_rt_set_velocity_offset(int slave, int32_t counts_per_s) {
    check_idx(slave);
    g_slaves[slave].tx.velocity_offset = counts_per_s;
}
void ec_rt_set_torque_offset(int slave, int16_t tenths_pct) {
    check_idx(slave);
    g_slaves[slave].tx.torque_offset = tenths_pct;
}
int16_t ec_rt_get_torque_actual(int slave) {
    check_idx(slave);
    return EC_READ_S16(g_pd + g_slaves[slave].i_torque_actual);
}

void ec_rt_get_telemetry(int slave, ec_telemetry_t *out) {
    check_idx(slave);
    slave_t *sl = &g_slaves[slave];
    out->error_code      = EC_READ_U16(g_pd + sl->i_error_code);
    out->statusword      = EC_READ_U16(g_pd + sl->i_statusword);
    out->position_actual = EC_READ_S32(g_pd + sl->i_position_actual);
    out->velocity_actual = EC_READ_S32(g_pd + sl->i_velocity_actual);
    out->torque_actual   = EC_READ_S16(g_pd + sl->i_torque_actual);
    out->following_error = EC_READ_S32(g_pd + sl->i_following_error);
    out->target_position = sl->tx.target_position;
    out->velocity_offset = sl->tx.velocity_offset;
    out->torque_offset   = sl->tx.torque_offset;
}

int ec_rt_read_limits(int slave, uint32_t *ferr_counts, uint16_t *ferr_timeout_ms,
                      uint16_t *torque_tenth_pct) {
    check_idx(slave);
    int32_t pos = g_slaves[slave].pos;
    uint8_t buf[4];
    size_t rs = 0;
    uint32_t abort = 0;
    if (ecrt_master_sdo_upload(g_master, pos, 0x6065, 0x00, buf, 4, &rs, &abort) != 0)
        return -1;
    memcpy(ferr_counts, buf, 4);
    if (ecrt_master_sdo_upload(g_master, pos, 0x6066, 0x00, buf, 2, &rs, &abort) != 0)
        return -2;
    memcpy(ferr_timeout_ms, buf, 2);
    if (ecrt_master_sdo_upload(g_master, pos, 0x6072, 0x00, buf, 2, &rs, &abort) != 0)
        return -3;
    memcpy(torque_tenth_pct, buf, 2);
    return 0;
}

int ec_rt_write_limits(int slave, uint32_t ferr_counts, uint16_t torque_tenth_pct) {
    check_idx(slave);
    int32_t pos = g_slaves[slave].pos;
    uint32_t abort = 0;
    uint8_t b4[4];
    memcpy(b4, &ferr_counts, 4);
    if (ecrt_master_sdo_download(g_master, pos, 0x6065, 0x00, b4, 4, &abort) != 0)
        return -1;
    uint8_t b2[2];
    memcpy(b2, &torque_tenth_pct, 2);
    if (ecrt_master_sdo_download(g_master, pos, 0x6072, 0x00, b2, 2, &abort) != 0)
        return -2;
    return 0;
}

int ec_rt_sdo_read(int slave, uint16_t index, uint8_t sub, uint8_t *buf, int *size,
                   uint32_t *abort_code) {
    check_idx(slave);
    size_t rs = 0;
    uint32_t abort = 0;
    if (ecrt_master_sdo_upload(g_master, g_slaves[slave].pos, index, sub, buf,
                               (size_t)*size, &rs, &abort) != 0) {
        *abort_code = abort;
        return -1;
    }
    *size = (int)rs;
    *abort_code = 0;
    return 0;
}

int ec_rt_sdo_write(int slave, uint16_t index, uint8_t sub, const uint8_t *buf, int size,
                    uint32_t *abort_code) {
    check_idx(slave);
    uint32_t abort = 0;
    if (ecrt_master_sdo_download(g_master, g_slaves[slave].pos, index, sub, buf,
                                 (size_t)size, &abort) != 0) {
        *abort_code = abort;
        return -1;
    }
    *abort_code = 0;
    return 0;
}

/*
 * CiA-402 homing-method-35 ("current position is home") drive-frame on one
 * slave, run as a self-contained DC loop while the other slaves hold steady.
 * Preconditions (staged off-loop by the caller): mode = Homing (6061h reads 6),
 * 6098h = 35, 607Ch = offset, drive operation-enabled. 6040h bit 4 (0x10)
 * rising edge starts homing and stays set; 6041h bit 12 = attained, bit 13 = error.
 */
int ec_rt_run_homing(int slave) {
    check_idx(slave);
    slave_t *sl = &g_slaves[slave];
    int64_t toff = 0;
    for (int i = 0; i < 8; i++) {
        stage_hold_others(slave);
        sl->tx.controlword = 0x000F;
        sl->tx.target_position = EC_READ_S32(g_pd + sl->i_position_actual);
        rt_exchange(&toff);
    }
    /* Attain latency is drive- and slot-dependent: on the trident bench slot 0
     * attains in well under a second while slot 1 routinely needs more than
     * the old 3000-cycle window. 12000 cycles (~12 s at 1 kHz) bounds the
     * worst observed case with margin; the loop exits on bit 12 immediately. */
    uint16_t last_sw = 0;
    for (int64_t pc = 0; pc < 12000; pc++) {
        stage_hold_others(slave);
        uint16_t sw = EC_READ_U16(g_pd + sl->i_statusword);
        last_sw = sw;
        sl->tx.controlword = 0x001F;
        sl->tx.target_position = EC_READ_S32(g_pd + sl->i_position_actual);
        if (sw & 0x2000) {
            sl->tx.controlword = 0x000F;
            rt_exchange(&toff);
            return EC_RT_ERR_HOMING_ERROR;
        }
        if (sw & 0x1000) {
            sl->tx.controlword = 0x000F;
            rt_exchange(&toff);
            return 0;
        }
        rt_exchange(&toff);
    }
    sl->tx.controlword = 0x000F;
    rt_exchange(&toff);
    fprintf(stderr,
            "ec_rt: homing attain timeout slave=%d final statusword=0x%04x\n",
            slave, last_sw);
    return EC_RT_ERR_HOMING_ATTAIN;
}

int ec_rt_park_cycle(int64_t *toff_ns) {
    if (!g_pd) {
        fprintf(stderr, "ec_rt: park_cycle before bringup_finish — PDO map not bound\n");
        abort();
    }
    for (int s = 0; s < g_num_slaves; s++) {
        g_slaves[s].tx.controlword = 0;
        g_slaves[s].tx.target_position = EC_READ_S32(g_pd + g_slaves[s].i_position_actual);
    }
    return rt_exchange(toff_ns);
}

/* AL status register 0x0134 via a register request, pumped on the DC grid until
 * the master FSM completes it. Diagnostic-only; called after the DC loop halts. */
static uint16_t read_al_status_code(slave_t *sl) {
    if (!sl->al_req) return 0;
    ecrt_reg_request_read(sl->al_req, 0x0134, 2);
    for (int i = 0; i < 50; i++) {
        int64_t t = 0;
        rt_exchange(&t);
        ec_request_state_t rs = ecrt_reg_request_state(sl->al_req);
        if (rs == EC_REQUEST_SUCCESS) return EC_READ_U16(ecrt_reg_request_data(sl->al_req));
        if (rs == EC_REQUEST_ERROR) return 0;
    }
    return 0;
}

void ec_rt_al_status(int slave, uint16_t *state, uint16_t *alstatuscode) {
    check_idx(slave);
    slave_t *sl = &g_slaves[slave];
    ec_slave_config_state_t st;
    ecrt_slave_config_state(sl->sc, &st);
    *state = (uint16_t)st.al_state;
    *alstatuscode = read_al_status_code(sl);
}

void ec_rt_disable_all(void) {
    for (int i = 0; i < 100; i++) {
        for (int s = 0; s < g_num_slaves; s++) {
            slave_t *sl = &g_slaves[s];
            sl->enabled = 0;
            sl->tx.controlword = 0x0006;
            sl->tx.target_position = EC_READ_S32(g_pd + sl->i_position_actual);
            sl->tx.velocity_offset = 0;
            sl->tx.torque_offset = 0;
        }
        int64_t t = 0;
        rt_exchange(&t);
    }
}

void ec_rt_dump_al_state(void) {
    for (int s = 0; s < g_num_slaves; s++) {
        slave_t *sl = &g_slaves[s];
        ec_slave_config_state_t st;
        ecrt_slave_config_state(sl->sc, &st);
        uint16_t code = read_al_status_code(sl);
        fprintf(stderr,
                "ec_rt: slot %d (position %d) al_state=0x%02x online=%u operational=%u al_status=0x%04x\n",
                s, (int)sl->pos, st.al_state, st.online, st.operational, code);
    }
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
