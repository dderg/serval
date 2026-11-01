#ifndef VTIME_H
#define VTIME_H

#include <stdint.h>
#include <stdatomic.h>

#define VTIME_SHM_NAME "/vtime"
#define VTIME_MAX_PACERS 8
#define VTIME_MAX_WAITERS 128
#define VTIME_MAX_HOST_THREADS 64
#define VTIME_MAX_LINKS 8
#define VTIME_SHM_BYTES 4096

struct vtime_device_link {
    _Atomic uint32_t pts_plus_one;
    uint32_t reserved;
    _Atomic uint64_t written;
    _Atomic uint64_t consumed;
    _Atomic uint64_t owner_stepping_to_ns;
};

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
    _Atomic uint32_t clock_lock_owner;
    _Atomic uint32_t host_pid;
    _Atomic uint64_t host_wake_epoch;
    _Atomic uint64_t host_idle_epoch;
    _Atomic uint32_t host_tid[VTIME_MAX_HOST_THREADS];
    _Atomic uint32_t link_slots_used;
    _Atomic int32_t devices_handling;
    struct vtime_device_link links[VTIME_MAX_LINKS];
};

_Static_assert(sizeof(struct vtime_shm) <= VTIME_SHM_BYTES,
               "tools/sim/world.py VTIME_SHM_SIZE must cover struct vtime_shm");
_Static_assert(__builtin_offsetof(struct vtime_shm, host_tid) == 3344,
               "tools/sim/world.py _HOST_TID_OFFSET");
_Static_assert(__builtin_offsetof(struct vtime_shm, links) == 3608,
               "tools/sim/world.py _LINKS_OFFSET");
_Static_assert(sizeof(struct vtime_device_link) == 32,
               "tools/sim/world.py _LINK_SLOT_SIZE");

#endif
