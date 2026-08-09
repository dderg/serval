#ifndef RUNTIME_STORAGE_H
#define RUNTIME_STORAGE_H

#include "autoconf.h"
#include <stdint.h>

#ifndef CONFIG_RUNTIME_STORAGE_SIZE
#  error "CONFIG_RUNTIME_STORAGE_SIZE missing — Kconfig broken"
#endif

#define RT_STORAGE_SIZE CONFIG_RUNTIME_STORAGE_SIZE

extern uint8_t rt_storage[RT_STORAGE_SIZE];

#endif
