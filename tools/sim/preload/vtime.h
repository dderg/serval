#ifndef VTIME_H
#define VTIME_H

#include <stdint.h>
#include <stdatomic.h>

#define VTIME_SHM_NAME "/vtime"
#define VTIME_MAX_PACERS 8
#define VTIME_MAX_WAITERS 128

// A pacer is a thread that must observe every period of virtual time
// (an MCU motion-tick thread). While any pacer is active, no participant
// may advance the shared clock past min(floor_ns): each pacer raises its
// floor one period at a time as it ticks, so virtual time can never skip
// over a motion sample.
struct vtime_pacer_slot {
    _Atomic uint32_t active;
    uint32_t reserved;
    _Atomic uint64_t floor_ns;
    // How far virtual time may run past this pacer's floor. Per pacer
    // because the tolerable overshoot is a property of the work being
    // paced: a motion tick only needs its sample period preserved, while a
    // classic step queue shuts down once its own clock reads more than 1 ms
    // past a scheduled step.
    _Atomic uint64_t slack_ns;
};

// The harness (tools/sim/world.py) writes harness_paced and the pace
// anchors at these fixed offsets: 8, 16 and 24.
struct vtime_shm {
    _Atomic uint64_t nanos;
    // While the test script itself runs (between its waits on the printer),
    // virtual time advances no faster than real time from the anchor pair,
    // as a real printer would idle while its operator types; inside a wait
    // it runs free.
    _Atomic uint32_t harness_paced;
    _Atomic uint32_t num_participants;
    _Atomic uint64_t harness_pace_vtime;
    _Atomic uint64_t harness_pace_real;
    _Atomic uint32_t pacer_slots_used;
    uint32_t reserved;
    struct vtime_pacer_slot pacers[VTIME_MAX_PACERS];
    // Virtual time holds still while the host (klippy) has a thread outside
    // a blocking wait, so host computation costs no virtual time and the
    // host can never fall behind the printer it drives.
    _Atomic uint32_t host_present;
    _Atomic int32_t host_running;
    // Every thread blocked in a timed wait publishes the virtual time it
    // waits for (0 = none) and its pid << 32 | tid; whoever advances virtual
    // time onto that target wakes it at once instead of leaving the clock
    // parked until the waiter's real-time sleep slice expires. A host
    // waiter's target is also a floor: virtual time may reach it but not
    // pass it, so the host always wakes on time and closes the gate again.
    _Atomic uint32_t waiter_slots_used;
    uint32_t reserved2;
    _Atomic uint64_t waiter_target[VTIME_MAX_WAITERS];
    _Atomic uint64_t waiter_thread[VTIME_MAX_WAITERS];
    _Atomic uint32_t waiter_kicked[VTIME_MAX_WAITERS];
    _Atomic uint32_t waiter_is_floor[VTIME_MAX_WAITERS];
};

#endif
