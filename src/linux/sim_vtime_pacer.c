// libvtime pacer for simulator builds without CONFIG_MOTION_RUNTIME.
//
// A MOTION_RUNTIME build gets its pacer from the motion tick thread in
// runtime_tick_host.c; without that thread nothing bounds how far the shared
// virtual clock may run ahead of this MCU process. When the host deschedules
// the process, virtual time keeps running at the speed cap and the step queue
// wakes to a clock that already passed its scheduled step -
// stepper_load_next() then (correctly) shuts down on lateness that exists only
// in the simulator.
//
// While this pacer is registered, no participant may raise virtual time past
// the period this thread is about to execute, so a descheduled MCU holds the
// whole simulated world back instead of falling behind it.

#include <dlfcn.h> // dlsym
#include <pthread.h> // pthread_create
#include <stdio.h> // fprintf
#include <stdlib.h> // abort
#include <time.h> // clock_gettime
#include "sched.h" // DECL_INIT
// The classic step queue shuts down once its own clock reads more than 1 ms
// past a scheduled step, so the whole overshoot budget - the period this
// thread is about to execute plus the slack libvtime grants past it - has to
// fit inside that. 100 us + 400 us leaves half the budget for the host
// descheduling the process between the pacer tick and the step timer.
#define PACER_PERIOD_NS 100000ULL
#define PACER_SLACK_NS 400000ULL

static int (*vtime_pacer_register_slack)(uint64_t period_ns,
                                         uint64_t slack_ns);
static void (*vtime_pacer_advance)(int slot, uint64_t target_ns,
                                   uint64_t period_ns);

static void *
sim_vtime_pacer_main(void *arg)
{
    (void)arg;
    int slot = vtime_pacer_register_slack(PACER_PERIOD_NS, PACER_SLACK_NS);
    if (slot < 0) {
        fprintf(stderr, "sim_vtime_pacer: no free libvtime pacer slot\n");
        abort();
    }
    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);
    for (;;) {
        next.tv_nsec += PACER_PERIOD_NS;
        while (next.tv_nsec >= 1000000000L) {
            next.tv_nsec -= 1000000000L;
            next.tv_sec += 1;
        }
        vtime_pacer_advance(slot, (uint64_t)next.tv_sec * 1000000000ULL
                            + (uint64_t)next.tv_nsec, PACER_PERIOD_NS);
    }
    return NULL;
}

void
sim_vtime_pacer_init(void)
{
    vtime_pacer_register_slack = dlsym(RTLD_DEFAULT,
                                       "vtime_pacer_register_slack");
    vtime_pacer_advance = dlsym(RTLD_DEFAULT, "vtime_pacer_advance");
    if (!vtime_pacer_register_slack || !vtime_pacer_advance) {
        fprintf(stderr, "sim_vtime_pacer: libvtime preload missing\n");
        abort();
    }
    pthread_t thread;
    int rc = pthread_create(&thread, NULL, sim_vtime_pacer_main, NULL);
    if (rc != 0) {
        fprintf(stderr, "sim_vtime_pacer: pthread_create failed: %d\n", rc);
        abort();
    }
}
DECL_INIT(sim_vtime_pacer_init);
