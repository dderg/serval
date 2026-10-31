#ifndef __LINUX_SIM_VTIME_PACER_H
#define __LINUX_SIM_VTIME_PACER_H

#include <stdint.h>

void sim_vtime_pacer_init(void);
void sim_vtime_pacer_set_floor(uint64_t ns);

#endif // sim_vtime_pacer.h
