#include <dlfcn.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "sched.h"
#include "sim_vtime_pacer.h"

#define PACER_PERIOD_NS 100000ULL
#define PACER_SLACK_NS 400000ULL

static int pacer_slot = -1;
static void (*set_floor)(int, uint64_t);

void
sim_vtime_pacer_init(void)
{
    int (*register_pacer)(uint64_t, uint64_t) =
        dlsym(RTLD_DEFAULT, "vtime_pacer_register_slack");
    set_floor = dlsym(RTLD_DEFAULT, "vtime_pacer_set_floor");
    if (!register_pacer || !set_floor) {
        fprintf(stderr, "sim_vtime_pacer: libvtime preload missing\n");
        abort();
    }
    pacer_slot = register_pacer(PACER_PERIOD_NS, PACER_SLACK_NS);
    if (pacer_slot < 0) {
        fprintf(stderr, "sim_vtime_pacer: no free libvtime pacer slot\n");
        abort();
    }
    set_floor(pacer_slot, UINT64_MAX - PACER_SLACK_NS);
}
DECL_INIT(sim_vtime_pacer_init);

void
sim_vtime_pacer_set_floor(uint64_t ns)
{
    if (pacer_slot >= 0)
        set_floor(pacer_slot, ns);
}
