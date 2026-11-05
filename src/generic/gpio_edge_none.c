// Boards without timed pin edges: every input is polled
//
// Copyright (C) 2026  Serval contributors
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "command.h" // shutdown
#include "generic/gpio_edge.h" // gpio_edge_claim
#include "sched.h" // sched_shutdown

uint_fast8_t
gpio_edge_claim(struct gpio_edge *edge, uint32_t pin)
{
    return 0;
}

void
gpio_edge_arm(struct gpio_edge *edge, uint_fast8_t rising)
{
    shutdown("gpio edge armed on a board without edge interrupts");
}

void
gpio_edge_disarm(struct gpio_edge *edge)
{
    shutdown("gpio edge disarmed on a board without edge interrupts");
}

uint_fast8_t
gpio_edge_pending(struct gpio_edge *edge)
{
    shutdown("gpio edge queried on a board without edge interrupts");
}
