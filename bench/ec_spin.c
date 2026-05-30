/*
 * ec_spin.c - minimal SOEM CSP bench: drive the A6-EC / AS715N servo with a
 * streamed Cyclic-Synchronous-Position trajectory.
 *
 * Bring-up-only code (CLAUDE.md "no-throwaway" exception): a deliberately
 * simple, known-good reference we build the real Rust implementation on.
 *
 * The drive supports ONLY DC SYNC0 (1C32:04=0x04) and powers up in SM-sync,
 * so we must (a) tell its firmware to use DC SYNC0 (1C32:01=2) and (b) have
 * SYNC0 active BEFORE the SAFE-OP transition, else Er74.1 / AL 0x0030.
 * Startup: enumerate -> CSP (6060=8) + sync-type SDOs -> configdc + dcsync0
 *   (PRE-OP) -> map (-> SAFE-OP) -> align DC -> OP -> fault-reset -> enable
 *   -> stream target position from traj() each cycle -> stop.
 *
 * 1:1 gear, 17-bit encoder => 131072 counts/rev.
 *
 * Run: sudo ./ec_spin eth0 [cycle_us] [sine|ramp]   (defaults: 2000 us, sine)
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
#define RT_CPU          3
#define COUNTS_PER_REV  131072.0       /* 17-bit encoder, 1:1 gear */

/* --- ramp trajectory params --- */
#define RAMP_RPM        30.0
#define RAMP_PHASE_SEC  3.0
#define RAMP_HOLD_SEC   0.5
/* --- sine (1-cos) trajectory params: gentle oscillation --- */
#define SINE_SWING_REV  1.0            /* peak-to-peak travel, revolutions */
#define SINE_PERIOD_SEC 4.0            /* one full there-and-back */
#define SINE_CYCLES     3

#define STABILIZE_SEC   1.5
#define WAIT_OP_SEC     2.0
#define DISABLE_SEC     0.2

typedef enum { TRAJ_SINE, TRAJ_RAMP } traj_mode_t;

/* Position offset from start (counts) at time t (s). Both profiles begin AND
 * end at zero offset with zero velocity, so enable/disable are bump-free. */
static double traj_offset(traj_mode_t m, double t, double *total_dur) {
    if (m == TRAJ_RAMP) {
        const double cps = (RAMP_RPM / 60.0) * COUNTS_PER_REV;
        *total_dur = 2.0 * RAMP_PHASE_SEC + RAMP_HOLD_SEC;
        if (t < RAMP_PHASE_SEC)            return cps * t;                      /* fwd */
        if (t < 2.0 * RAMP_PHASE_SEC)      return cps * (2.0 * RAMP_PHASE_SEC - t); /* rev */
        return 0.0;                                                            /* hold @ start */
    } else { /* TRAJ_SINE: A*(1-cos(wt)) -> starts at 0 with zero velocity */
        const double A = SINE_SWING_REV * COUNTS_PER_REV / 2.0;
        const double w = 2.0 * M_PI / SINE_PERIOD_SEC;
        *total_dur = SINE_PERIOD_SEC * SINE_CYCLES;
        return A * (1.0 - cos(w * t));
    }
}

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
    traj_mode_t mode   = (argc > 3 && strcmp(argv[3], "ramp") == 0) ? TRAJ_RAMP : TRAJ_SINE;
    const int64_t sync0_shift = cycle_ns / 2;

    setvbuf(stdout, NULL, _IOLBF, 0);
    signal(SIGINT, on_sigint);
    go_realtime();
    printf("cycle=%lld us (%.0f Hz), traj=%s, SYNC0 shift=%lld ns\n",
           (long long)(cycle_ns / 1000), 1e9 / (double)cycle_ns,
           mode == TRAJ_RAMP ? "ramp" : "sine", (long long)sync0_shift);

    if (!ec_init(ifname)) { printf("ec_init failed on %s\n", ifname); return 1; }
    if (ec_config_init(FALSE) <= 0) { printf("no slaves found\n"); ec_close(); return 1; }
    printf("%d slave(s): %s\n", ec_slavecount, ec_slave[1].name);

    int8_t opmode = 8;  /* CSP */
    ec_SDOwrite(1, 0x6060, 0x00, FALSE, sizeof(opmode), &opmode, EC_TIMEOUTRXM);
    uint16_t sync_dc = 2;                 /* DC SYNC0 */
    uint32_t cyc_ns  = (uint32_t)cycle_ns;
    ec_SDOwrite(1, 0x1C32, 0x01, FALSE, sizeof(sync_dc), &sync_dc, EC_TIMEOUTRXM);
    ec_SDOwrite(1, 0x1C33, 0x01, FALSE, sizeof(sync_dc), &sync_dc, EC_TIMEOUTRXM);
    ec_SDOwrite(1, 0x1C32, 0x02, FALSE, sizeof(cyc_ns),  &cyc_ns,  EC_TIMEOUTRXM);
    ec_SDOwrite(1, 0x1C33, 0x02, FALSE, sizeof(cyc_ns),  &cyc_ns,  EC_TIMEOUTRXM);

    /* SYNC0 active BEFORE SAFE-OP (else AL 0x0030 invalid DC config). */
    ec_configdc();
    ec_dcsync0(1, TRUE, (uint32_t)cycle_ns, (int32_t)sync0_shift);
    ec_config_map(&IOmap);
    ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
    printf("SAFE-OP reached (SYNC0 active). Aligning clock...\n");

    out_t *out = (out_t *) ec_slave[1].outputs;
    in_t  *in  = (in_t  *) ec_slave[1].inputs;
    out->controlword = 0; out->target_position = 0;
    out->touch_probe_fn = 0; out->phys_outputs = 0;

    const int64_t stabilize_cyc = (int64_t)(STABILIZE_SEC * 1e9 / cycle_ns);
    const int64_t wait_op_cyc   = (int64_t)(WAIT_OP_SEC   * 1e9 / cycle_ns);
    const int64_t disable_cyc   = (int64_t)(DISABLE_SEC   * 1e9 / cycle_ns);
    const double  dt            = cycle_ns / 1e9;

    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    int64_t toff = 0;

    enum { STABILIZE, WAIT_OP, ALIGN, RUN, DISABLE, DONE } phase = STABILIZE;
    int64_t pc = 0;
    int32_t start_pos = 0;
    double  t_run = 0.0, total_dur = 0.0;
    int announced = 0, prdiv = 0, op_ok = 0;

    while (!g_stop && phase != DONE) {
        add_timespec(&ts, cycle_ns + toff);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
        ec_send_processdata();
        int wkc = ec_receive_processdata(EC_TIMEOUTRET);
        uint16_t sw = in->statusword;
        pc++;

        switch (phase) {
        case STABILIZE:
            out->controlword = 0; out->target_position = in->position_actual;
            if (pc >= stabilize_cyc) {
                ec_slave[0].state = EC_STATE_OPERATIONAL; ec_writestate(0);
                printf("clock aligned, requested OP...\n"); phase = WAIT_OP; pc = 0;
            }
            break;
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
            if (sw & 0x0008) {
                out->controlword = ((pc / 10) % 2) ? 0x0080 : 0x0000;   /* pulse fault reset */
                if (pc >= 1500) { printf("fault won't clear: sw=0x%04x err=0x%04x\n", sw, in->error_code); phase = DISABLE; pc = 0; }
            } else if ((sw & 0x004F) == 0x0040) out->controlword = 0x0006;
            else if   ((sw & 0x006F) == 0x0021) out->controlword = 0x0007;
            else if   ((sw & 0x006F) == 0x0023) out->controlword = 0x000F;
            else if   ((sw & 0x006F) == 0x0027) {
                out->controlword = 0x000F;
                if (!announced) {
                    start_pos = in->position_actual; t_run = 0.0;
                    printf("operation enabled @ pos=%d, streaming %s trajectory\n",
                           start_pos, mode == TRAJ_RAMP ? "ramp" : "sine");
                    announced = 1; phase = RUN; pc = 0;
                }
            } else out->controlword = 0x0000;
            break;
        case RUN: {
            if (sw & 0x0008) { printf("FAULT during RUN: err=0x%04x ferr=%d\n", in->error_code, in->following_error); phase = DISABLE; pc = 0; break; }
            double off = traj_offset(mode, t_run, &total_dur);
            out->controlword = 0x000F;
            out->target_position = start_pos + (int32_t) llround(off);
            t_run += dt;
            if (t_run >= total_dur) { phase = DISABLE; pc = 0; printf("trajectory done, stopping\n"); }
            break;
        }
        case DISABLE:
            out->target_position = in->position_actual; out->controlword = 0x0006;
            if (pc >= disable_cyc) phase = DONE;
            break;
        default: break;
        }

        dc_sync(ec_DCtime, cycle_ns, &toff);

        if (++prdiv >= (int)(0.5e9 / cycle_ns)) {
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
