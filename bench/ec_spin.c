/*
 * ec_spin.c - minimal SOEM CSP bench: spin the A6-EC / AS715N servo.
 *
 * Bring-up-only code (CLAUDE.md "no-throwaway" exception): a deliberately
 * simple, known-good reference we build the real Rust implementation on.
 *
 * The drive supports ONLY DC SYNC0 (1C32:04=0x04) and powers up in SM-sync,
 * so we must (a) tell its firmware to use DC SYNC0 (1C32:01=2) and (b) feed it
 * a disciplined distributed clock, else Er74.1 "no sync signal". Startup order:
 *   enumerate -> CSP (6060=8) + sync-type SDOs -> map default PDO 1701/1B01
 *   -> SAFE-OP -> DC-corrected cyclic loop to STABILIZE the clock & phase-lock
 *   -> activate SYNC0 (with phase shift) -> SETTLE -> OP
 *   -> CiA402 fault-reset if needed -> enable (06->07->0F)
 *   -> ramp target position: +RPM for PHASE_SEC, -RPM for PHASE_SEC, hold, stop.
 *
 * 1:1 gear, 17-bit encoder => 131072 counts/rev. 30 rpm = 65536 counts/s.
 *
 * Run: sudo ./ec_spin eth0 [cycle_us]     (cycle_us default 2000, min 250)
 */
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <inttypes.h>
#include <time.h>
#include <math.h>
#include <signal.h>
#include <sched.h>
#include <sys/mman.h>
#include "ethercat.h"

#define IF_DEFAULT      "eth0"
#define RT_PRIO         80
#define RT_CPU          3              /* pin the cyclic loop to this core */
#define COUNTS_PER_REV  131072.0       /* 17-bit encoder, 1:1 gear */
#define TARGET_RPM      30.0
#define PHASE_SEC       3.0
#define HOLD_SEC        0.5
#define STABILIZE_SEC   2.0            /* lock DC + phase-align before SYNC0 */
#define SETTLE_SEC      0.5            /* let SYNC0 run before requesting OP */
#define WAIT_OP_SEC     2.0
#define DISABLE_SEC     0.2

#pragma pack(push, 1)
typedef struct {                 /* RxPDO 0x1701 (12 bytes) */
    uint16_t controlword;        /* 6040    */
    int32_t  target_position;    /* 607A    */
    uint16_t touch_probe_fn;     /* 60B8    */
    uint32_t phys_outputs;       /* 60FE:01 */
} out_t;
typedef struct {                 /* TxPDO 0x1B01 (28 bytes) */
    uint16_t error_code;         /* 603F */
    uint16_t statusword;         /* 6041 */
    int32_t  position_actual;    /* 6064 */
    int16_t  torque_actual;      /* 6077 */
    int32_t  following_error;    /* 60F4 */
    uint16_t tp_status;          /* 60B9 */
    int32_t  tp1_pos;            /* 60BA */
    int32_t  tp2_pos;            /* 60BC */
    uint32_t digital_inputs;     /* 60FD */
} in_t;
#pragma pack(pop)

static char IOmap[4096];
static volatile sig_atomic_t g_stop = 0;
static void on_sigint(int s) { (void)s; g_stop = 1; }

static void add_timespec(struct timespec *ts, int64_t addtime) {
    int64_t nsec = addtime % 1000000000LL;
    int64_t sec  = (addtime - nsec) / 1000000000LL;
    ts->tv_sec  += sec;
    ts->tv_nsec += nsec;
    if (ts->tv_nsec >= 1000000000LL) { ts->tv_nsec -= 1000000000LL; ts->tv_sec++; }
}

/* PI controller to align the local cycle to the DC reference clock. */
static void dc_sync(int64_t reftime, int64_t cycletime, int64_t *offset) {
    static int64_t integral = 0;
    int64_t delta = reftime % cycletime;
    if (delta > cycletime / 2) delta -= cycletime;
    if (delta > 0) integral++;
    if (delta < 0) integral--;
    *offset = -(delta / 100) - (integral / 20);
}

static void go_realtime(void) {
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) perror("mlockall (continuing)");
    cpu_set_t set; CPU_ZERO(&set); CPU_SET(RT_CPU, &set);
    if (sched_setaffinity(0, sizeof(set), &set) != 0) perror("setaffinity (continuing)");
    struct sched_param sp; sp.sched_priority = RT_PRIO;
    if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0) perror("SCHED_FIFO (continuing)");
    else printf("RT: SCHED_FIFO prio %d, pinned to CPU %d, memory locked\n", RT_PRIO, RT_CPU);
}

int main(int argc, char **argv) {
    const char *ifname = (argc > 1) ? argv[1] : IF_DEFAULT;
    int64_t cycle_ns   = (argc > 2) ? (int64_t)atoll(argv[2]) * 1000 : 2000000;
    if (cycle_ns < 250000) cycle_ns = 250000;
    const int64_t sync0_shift = cycle_ns / 2;
    setvbuf(stdout, NULL, _IOLBF, 0);   /* line-buffered: survive SIGTERM/pipe */
    signal(SIGINT, on_sigint);
    go_realtime();
    printf("cycle=%lld us (%.0f Hz), SYNC0 shift=%lld ns\n",
           (long long)(cycle_ns / 1000), 1e9 / (double)cycle_ns, (long long)sync0_shift);

    if (!ec_init(ifname)) { printf("ec_init failed on %s\n", ifname); return 1; }
    printf("ec_init ok on %s\n", ifname);
    if (ec_config_init(FALSE) <= 0) { printf("no slaves found\n"); ec_close(); return 1; }
    printf("%d slave(s): %s\n", ec_slavecount, ec_slave[1].name);

    int8_t mode = 8;  /* CSP */
    ec_SDOwrite(1, 0x6060, 0x00, FALSE, sizeof(mode), &mode, EC_TIMEOUTRXM);

    /* Arm DC SYNC0 in firmware (default is SM-sync -> "no sync signal"). */
    uint16_t sync_dc = 2;                 /* 2 = DC SYNC0 */
    uint32_t cyc_ns  = (uint32_t)cycle_ns;
    ec_SDOwrite(1, 0x1C32, 0x01, FALSE, sizeof(sync_dc), &sync_dc, EC_TIMEOUTRXM);
    ec_SDOwrite(1, 0x1C33, 0x01, FALSE, sizeof(sync_dc), &sync_dc, EC_TIMEOUTRXM);
    ec_SDOwrite(1, 0x1C32, 0x02, FALSE, sizeof(cyc_ns),  &cyc_ns,  EC_TIMEOUTRXM);
    ec_SDOwrite(1, 0x1C33, 0x02, FALSE, sizeof(cyc_ns),  &cyc_ns,  EC_TIMEOUTRXM);

    /* SYNC0 must be active BEFORE the SAFE-OP transition: the drive validates
     * DC config on entering SAFE-OP, and a declared DC mode with an inactive
     * SYNC0 (cycle reg = 0) trips AL 0x0030 "invalid DC sync configuration".
     * So configure DC + activate SYNC0 here in PRE-OP, then map (-> SAFE-OP). */
    ec_configdc();
    ec_dcsync0(1, TRUE, (uint32_t)cycle_ns, (int32_t)sync0_shift);
    ec_config_map(&IOmap);
    ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
    printf("SAFE-OP reached (SYNC0 active, DC declared). Aligning clock...\n");

    out_t *out = (out_t *) ec_slave[1].outputs;
    in_t  *in  = (in_t  *) ec_slave[1].inputs;
    out->controlword = 0; out->target_position = 0;
    out->touch_probe_fn = 0; out->phys_outputs = 0;

    const int64_t stabilize_cyc = (int64_t)(STABILIZE_SEC * 1e9 / cycle_ns);
    const int64_t settle_cyc    = (int64_t)(SETTLE_SEC    * 1e9 / cycle_ns);
    const int64_t wait_op_cyc   = (int64_t)(WAIT_OP_SEC   * 1e9 / cycle_ns);
    const int64_t disable_cyc   = (int64_t)(DISABLE_SEC   * 1e9 / cycle_ns);
    const double  per_cycle     = (TARGET_RPM / 60.0) * COUNTS_PER_REV * (cycle_ns / 1e9);

    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    int64_t toff = 0;

    enum { STABILIZE, SETTLE, WAIT_OP, ALIGN, FWD, REV, HOLD, DISABLE, DONE } phase = STABILIZE;
    int64_t pc = 0;
    double cmd = 0.0, t_phase = 0.0;
    int announced = 0, prdiv = 0, op_ok = 0;

    while (!g_stop && phase != DONE) {
        add_timespec(&ts, cycle_ns + toff);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
        ec_send_processdata();
        int wkc = ec_receive_processdata(EC_TIMEOUTRET);
        uint16_t sw = in->statusword;
        pc++;

        switch (phase) {
        case STABILIZE:                          /* SYNC0 already active; just phase-align */
            out->controlword = 0; out->target_position = in->position_actual;
            if (pc >= stabilize_cyc) {
                ec_slave[0].state = EC_STATE_OPERATIONAL; ec_writestate(0);
                printf("clock aligned, requested OP...\n"); phase = WAIT_OP; pc = 0;
            }
            break;
        case SETTLE: break;                      /* unused */
        case WAIT_OP:
            out->controlword = 0; out->target_position = in->position_actual;
            if (pc % 20 == 0) ec_readstate();
            if (ec_slave[0].state == EC_STATE_OPERATIONAL) {
                printf("OP reached. Enabling CiA402...\n"); op_ok = 1; phase = ALIGN; pc = 0;
            } else if (pc >= wait_op_cyc) {
                ec_readstate();
                printf("FAILED to reach OP. state=0x%2.2x AL=0x%4.4x (%s)\n",
                       ec_slave[1].state, ec_slave[1].ALstatuscode,
                       ec_ALstatuscode2string(ec_slave[1].ALstatuscode));
                phase = DONE;
            }
            break;
        case ALIGN:
            out->target_position = in->position_actual;
            cmd = (double) in->position_actual;
            if (sw & 0x0008) {
                out->controlword = ((pc / 10) % 2) ? 0x0080 : 0x0000;   /* pulse fault reset */
                if (pc >= 1500) { printf("fault won't clear: sw=0x%04x err=0x%04x\n", sw, in->error_code); phase = DISABLE; pc = 0; }
            } else if ((sw & 0x004F) == 0x0040) out->controlword = 0x0006;
            else if   ((sw & 0x006F) == 0x0021) out->controlword = 0x0007;
            else if   ((sw & 0x006F) == 0x0023) out->controlword = 0x000F;
            else if   ((sw & 0x006F) == 0x0027) {
                out->controlword = 0x000F;
                if (!announced) { printf("operation enabled @ pos=%d\n", in->position_actual);
                    announced = 1; t_phase = 0.0; phase = FWD; pc = 0; }
            } else out->controlword = 0x0000;
            break;
        case FWD:
            if (sw & 0x0008) { printf("FAULT during FWD: err=0x%04x ferr=%d\n", in->error_code, in->following_error); phase = DISABLE; pc = 0; break; }
            out->controlword = 0x000F; cmd += per_cycle; out->target_position = (int32_t) llround(cmd);
            if ((t_phase += cycle_ns / 1e9) >= PHASE_SEC) { t_phase = 0; phase = REV; printf("reverse\n"); }
            break;
        case REV:
            if (sw & 0x0008) { printf("FAULT during REV: err=0x%04x ferr=%d\n", in->error_code, in->following_error); phase = DISABLE; pc = 0; break; }
            out->controlword = 0x000F; cmd -= per_cycle; out->target_position = (int32_t) llround(cmd);
            if ((t_phase += cycle_ns / 1e9) >= PHASE_SEC) { t_phase = 0; phase = HOLD; printf("hold\n"); }
            break;
        case HOLD:
            if (sw & 0x0008) { printf("FAULT during HOLD: err=0x%04x\n", in->error_code); phase = DISABLE; pc = 0; break; }
            out->controlword = 0x000F; out->target_position = (int32_t) llround(cmd);
            if ((t_phase += cycle_ns / 1e9) >= HOLD_SEC) { phase = DISABLE; pc = 0; printf("stopping\n"); }
            break;
        case DISABLE:
            out->target_position = in->position_actual; out->controlword = 0x0006;
            if (pc >= disable_cyc) phase = DONE;
            break;
        default: break;
        }

        dc_sync(ec_DCtime, cycle_ns, &toff);

        if (++prdiv >= (int)(0.5e9 / cycle_ns)) {   /* ~2 prints/sec */
            prdiv = 0;
            ec_readstate();
            printf("phase=%d ecat=0x%02x AL=0x%04x wkc=%d sw=0x%04x err=0x%04x pos=%d cmd=%d ferr=%d trq=%d toff=%lld\n",
                   phase, ec_slave[1].state, ec_slave[1].ALstatuscode, wkc, sw, in->error_code,
                   in->position_actual, out->target_position, in->following_error,
                   in->torque_actual, (long long)toff);
        }
    }

    printf("stopping SYNC0, returning to INIT...\n");
    ec_dcsync0(1, FALSE, 0, 0);
    ec_slave[0].state = EC_STATE_INIT; ec_writestate(0);
    ec_close();
    printf("done. op_ok=%d\n", op_ok);
    return 0;
}
