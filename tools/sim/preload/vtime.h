#ifndef VTIME_H
#define VTIME_H

#include <stdint.h>
#include <stdatomic.h>

#define VTIME_SHM_NAME "/vtime"
#define VTIME_MAX_PACERS 8

// A pacer is a thread that must observe every period of virtual time
// (an MCU motion-tick thread). While any pacer is active, no participant
// may advance the shared clock past min(floor_ns): each pacer raises its
// floor one period at a time as it ticks, so virtual time can never skip
// over a motion sample.
struct vtime_pacer_slot {
    _Atomic uint32_t active;
    uint32_t reserved;
    _Atomic uint64_t floor_ns;
};

struct vtime_shm {
    _Atomic uint64_t nanos;
    _Atomic uint32_t num_sleepers;
    _Atomic uint32_t num_participants;
    _Atomic uint32_t initialized;
    _Atomic uint32_t pacer_slots_used;
    uint32_t reserved;
    struct vtime_pacer_slot pacers[VTIME_MAX_PACERS];
};

#endif
