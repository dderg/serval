/*
 * IgH (EtherLab) EtherCAT master backend — SKELETON.
 *
 * Selected by `--features igh` (build.rs compiles this instead of the SOEM
 * libecrt.c and links -lethercat). It defines the full ec_rt_* contract from
 * libecrt.h so the endpoint binary links, but the master logic (domains, PDO
 * registration, DC sync, CiA-402 bring-up) is not ported yet: bring-up/cycle
 * paths return EC_RT_ERR_IGH_UNIMPLEMENTED so the endpoint refuses to run
 * rather than silently driving nothing. Fill these in incrementally; SOEM
 * stays the known-good fallback behind `--features hw`.
 */
#include "libecrt.h"
#include <ecrt.h>
#include <string.h>

static int igh_unimplemented(void) {
    volatile unsigned int magic = ecrt_version_magic();
    (void)magic;
    return EC_RT_ERR_IGH_UNIMPLEMENTED;
}

int ec_rt_bringup_preop(const char *ifname, int64_t cycle_ns, int rt_cpu, int rt_prio) {
    (void)ifname; (void)cycle_ns; (void)rt_cpu; (void)rt_prio;
    return igh_unimplemented(); // TODO: IgH port — request master, configure slave, PDOs, DC, reach PRE-OP
}

int ec_rt_bringup_finish(void) {
    return igh_unimplemented(); // TODO: IgH port — activate master, SYNC0, reach OPERATIONAL, park at Ready-to-Switch-On
}

int ec_rt_enable(void) {
    return igh_unimplemented(); // TODO: IgH port — CiA-402 enable sequence, apply torque
}

int ec_rt_run_homing(void) {
    return igh_unimplemented(); // TODO: IgH port — CiA-402 homing method 35 DC loop
}

int ec_rt_cycle(int64_t *toff_ns) {
    if (toff_ns) *toff_ns = 0;
    return igh_unimplemented(); // TODO: IgH port — one steady-state DC cycle (send/recv, PI jitter correction)
}

int ec_rt_park_cycle(int64_t *toff_ns) {
    if (toff_ns) *toff_ns = 0;
    return igh_unimplemented(); // TODO: IgH port — one parked DC cycle (controlword 0, target tracks actual)
}

void ec_rt_set_target_position(int32_t counts) { (void)counts; } // TODO: IgH port
int32_t  ec_rt_get_position_actual(void)  { return 0; }          // TODO: IgH port
int32_t  ec_rt_get_velocity_actual(void)  { return 0; }          // TODO: IgH port
uint16_t ec_rt_get_statusword(void)       { return 0; }          // TODO: IgH port
uint16_t ec_rt_get_error_code(void)       { return 0; }          // TODO: IgH port
int32_t  ec_rt_get_following_error(void)  { return 0; }          // TODO: IgH port
void ec_rt_set_velocity_offset(int32_t counts_per_s) { (void)counts_per_s; } // TODO: IgH port
void ec_rt_set_torque_offset(int16_t tenths_pct)     { (void)tenths_pct; }   // TODO: IgH port
int16_t  ec_rt_get_torque_actual(void)    { return 0; }          // TODO: IgH port

void ec_rt_get_telemetry(ec_telemetry_t *out) {
    if (out) memset(out, 0, sizeof(*out)); // TODO: IgH port
}

int ec_rt_read_limits(uint32_t *ferr_counts, uint16_t *ferr_timeout_ms,
                      uint16_t *torque_tenth_pct) {
    (void)ferr_counts; (void)ferr_timeout_ms; (void)torque_tenth_pct;
    return igh_unimplemented(); // TODO: IgH port — SDO-read 6065h/6066h/6072h
}

int ec_rt_write_limits(uint32_t ferr_counts, uint16_t torque_tenth_pct) {
    (void)ferr_counts; (void)torque_tenth_pct;
    return igh_unimplemented(); // TODO: IgH port — SDO-write 6065h/6072h
}

void ec_rt_al_status(uint16_t *state, uint16_t *alstatuscode) {
    if (state) *state = 0;
    if (alstatuscode) *alstatuscode = 0; // TODO: IgH port
}

void ec_rt_disable(void) {}        // TODO: IgH port — controlword 0x0006
void ec_rt_dump_al_state(void) {}  // TODO: IgH port
void ec_rt_shutdown(void) {}       // TODO: IgH port — release master

int ec_rt_sdo_read(uint16_t index, uint8_t sub, uint8_t *buf, int *size,
                   uint32_t *abort_code) {
    (void)index; (void)sub; (void)buf; (void)size;
    if (abort_code) *abort_code = 0;
    return igh_unimplemented(); // TODO: IgH port — ecrt_master_sdo_upload
}

int ec_rt_sdo_write(uint16_t index, uint8_t sub, const uint8_t *buf, int size,
                    uint32_t *abort_code) {
    (void)index; (void)sub; (void)buf; (void)size;
    if (abort_code) *abort_code = 0;
    return igh_unimplemented(); // TODO: IgH port — ecrt_master_sdo_download
}
