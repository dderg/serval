#ifndef LIBECRT_H
#define LIBECRT_H
#include <stdint.h>

/* Upper bound on drives the endpoint configures on one chain. The bring-up
 * caller passes the exact count; this only sizes the static per-slave arrays. */
#define EC_RT_MAX_SLAVES 8

#define EC_RT_ERR_EC_INIT         (-1)
#define EC_RT_ERR_NO_SLAVES       (-2)
#define EC_RT_ERR_SAFE_OP_TIMEOUT (-3)
#define EC_RT_ERR_OP_TIMEOUT      (-4)
#define EC_RT_ERR_CIA402_TIMEOUT  (-5)
#define EC_RT_ERR_PDO_REMAP       (-6)
#define EC_RT_ERR_PDO_SIZE        (-7)
#define EC_RT_ERR_PREOP_TIMEOUT   (-8)
#define EC_RT_ERR_INIT_TIMEOUT    (-9)
#define EC_RT_ERR_RT_MLOCK        (-10)
#define EC_RT_ERR_RT_AFFINITY     (-11)
#define EC_RT_ERR_RT_SCHED        (-12)
#define EC_RT_ERR_FF_ROUTING      (-13)
#define EC_RT_ERR_HOMING_ATTAIN   (-15)
#define EC_RT_ERR_HOMING_ERROR    (-16)
#define EC_RT_ERR_TOO_MANY_SLAVES (-17)
#define EC_RT_ERR_BAD_SLAVE_IDX   (-18)
#define EC_RT_ERR_DC_CONVERGE     (-19)

/* Two-phase bring-up for N slaves on one chain (one master, one domain, one DC
 * grid). `slave_positions[num_slaves]` are the topological ring positions
 * (0-based) to configure, in slot order — slot i drives slave_positions[i].
 * Phase 1 stops at PRE-OP for every slave (PDO maps, CSP mode, sync types, FF
 * routing written); the caller does its session SDO work there, where the
 * drives expect no process data. Phase 2 starts SYNC0, walks every slave to
 * OPERATIONAL, and parks each at CiA402 Ready-to-Switch-On (no torque);
 * ec_rt_enable() applies torque per slave. From phase 2 on, every wait between
 * exchanges must go through ec_rt_cycle — pausing process data in OP trips the
 * drives' sync-loss monitor (ErC1.1). 0 or an EC_RT_ERR_* above; a missing or
 * mismatched configured position fails loudly naming its slot. */
int  ec_rt_bringup_preop(const char *ifname, int64_t cycle_ns, int rt_cpu, int rt_prio,
                         const int32_t *slave_positions, int num_slaves);
int  ec_rt_bringup_finish(void);

/* Drive slot `slave`'s CiA402 enable state machine to Operation Enabled while
 * holding every other slave at its current safe state. */
int  ec_rt_enable(int slave);

/* Drive-frame via CiA-402 homing method 35 ("current position is home", no
 * motion) on slot `slave`. Self-contained DC loop (like ec_rt_enable): pulses
 * 6040h bit 4 and polls 6041h bit 12/13 for that slave while the rest hold.
 * Preconditions: mode-of-operation already switched to Homing (6060h=6,
 * confirmed via 6061h) with 6098h=35 and 607Ch=offset staged off-loop, and the
 * drive operation-enabled. 0 = homing attained; EC_RT_ERR_HOMING_* on
 * error/timeout. The caller restores CSP afterward. */
int  ec_rt_run_homing(int slave);

/* One steady-state DC cycle covering the whole domain: stage every slave's
 * controlword, sleep to next deadline, send+recv process data, run the DC PI
 * jitter correction. Writes the PI offset to *toff_ns. Returns the working
 * counter (3*num_slaves == healthy). */
int  ec_rt_cycle(int64_t *toff_ns);

/* Nominal CLOCK_MONOTONIC time (ns) of the last exchange's wake deadline —
 * the grid point the DC network is disciplined to via application_time.
 * Targets staged now are flushed one cycle after this value, so evaluating
 * the trajectory there (instead of at a live clock read) keeps loop
 * scheduling jitter out of the commanded positions. */
uint64_t ec_rt_cycle_time_ns(void);

/* Stage slot `slave`'s CSP target for the next cycle's send. */
void ec_rt_set_target_position(int slave, int32_t counts);

int32_t  ec_rt_get_position_actual(int slave);
int32_t  ec_rt_get_velocity_actual(int slave);
uint16_t ec_rt_get_statusword(int slave);
uint16_t ec_rt_get_error_code(int slave);
int32_t  ec_rt_get_following_error(int slave);

/* Stage slot `slave`'s CiA402 offsets for the next cycle's send (zeroed at
 * bring-up and on disable). Velocity in encoder counts/s, torque in 0.1% of
 * rated. */
void ec_rt_set_velocity_offset(int slave, int32_t counts_per_s);
void ec_rt_set_torque_offset(int slave, int16_t tenths_pct);
int16_t ec_rt_get_torque_actual(int slave);

typedef struct {
    uint16_t error_code;
    uint16_t statusword;
    int32_t  position_actual;
    int32_t  velocity_actual;
    int16_t  torque_actual;
    int32_t  following_error;
    int32_t  target_position;
    int32_t  velocity_offset;
    int16_t  torque_offset;
} ec_telemetry_t;

void ec_rt_get_telemetry(int slave, ec_telemetry_t *out);

/* SDO-read 6065h/6066h/6072h from slot `slave`. 0 ok; -1/-2/-3 per failing object. */
int ec_rt_read_limits(int slave, uint32_t *ferr_counts, uint16_t *ferr_timeout_ms,
                      uint16_t *torque_tenth_pct);

/* SDO-write 6065h and 6072h to slot `slave`. 0 ok; -1/-2 per failing object. */
int ec_rt_write_limits(int slave, uint32_t ferr_counts, uint16_t torque_tenth_pct);

/* One parked process-data cycle (controlword 0, target tracks actual), paced
 * to the DC grid. A slave in OP drops to SAFE-OP when cyclic frames stop for
 * longer than its SM watchdog (~100 ms) — call this in any host-side wait
 * between bringup and the DC loop. Returns the cycle's working counter. */
int ec_rt_park_cycle(int64_t *toff_ns);

/* Refresh AL state for slot `slave`: state (EC_STATE_*) and ALstatuscode. */
void ec_rt_al_status(int slave, uint16_t *state, uint16_t *alstatuscode);

/* controlword = 0x0006 (disable voltage path) on slot `slave`, held a few
 * cycles while the rest of the domain keeps cycling. */
void ec_rt_disable(int slave);

void ec_rt_dump_al_state(void);

void ec_rt_shutdown(void);

/* SDO upload from slot `slave`. On entry *size is the buffer capacity; on
 * success it holds the object's byte count. Returns 0 on success, -1 on failure
 * with *abort_code holding the CoE abort code (0 = transport-level failure). */
int ec_rt_sdo_read(int slave, uint16_t index, uint8_t sub, uint8_t *buf, int *size,
                   uint32_t *abort_code);

/* SDO download to slot `slave`. Same return/abort_code convention. */
int ec_rt_sdo_write(int slave, uint16_t index, uint8_t sub, const uint8_t *buf, int size,
                    uint32_t *abort_code);

#endif
