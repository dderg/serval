#ifndef __GENERIC_GPIO_EDGE_H
#define __GENERIC_GPIO_EDGE_H

#include <stdint.h> // uint32_t

struct gpio_edge {
    void (*on_edge_irq)(struct gpio_edge *edge, uint32_t entry_time);
    uint8_t line;
};

uint_fast8_t gpio_edge_claim(struct gpio_edge *edge, uint32_t pin);
void gpio_edge_arm(struct gpio_edge *edge, uint_fast8_t rising);
void gpio_edge_disarm(struct gpio_edge *edge);
uint_fast8_t gpio_edge_pending(struct gpio_edge *edge);

#endif // gpio_edge.h
