#ifndef _PHASE_STEPPING_SPI_H
#define _PHASE_STEPPING_SPI_H

#include <stdint.h>

struct spi_config;

/* Cache an SPI bus config for XDIRECT writes. Call once per bus_id, after
 * spi_setup() and BEFORE any register_motor() naming this bus. Multiple
 * TMC5160s share the cfg but each owns its own CS GPIO. bus_id is a phase
 * slot, not kalico's SPI peripheral index. */
void phase_stepping_register_bus(uint8_t bus_id, struct spi_config cfg);

/* Cache the CS GPIO (idle-high) for one motor and append it to the owning
 * bus's transmit sequence. Call after register_bus(bus_id). motor_idx is the
 * runtime per-motor slot; cs_pin_id is the kalico GPIO encoding (port*16+pin).
 * No-op if the bus is unregistered. */
void phase_stepping_register_motor(uint8_t motor_idx,
                                   uint8_t bus_id,
                                   uint8_t cs_pin_id);

/* Stage one TMC5160 XDIRECT datagram into the motor's slot of its bus's active
 * half-buffer. Staging only — no SPI traffic; phase_stepping_commit_tick()
 * triggers the per-bus DMA batch.
 *
 * Datagram (40-bit, MSB first):
 *   byte 0 = 0xAD              -- write (0x80) | XDIRECT (0x2D)
 *   byte 1 = (coil_b >> 8) & 1 -- coil_B sign bit
 *   byte 2 = coil_b & 0xFF     -- coil_B low 8 bits
 *   byte 3 = (coil_a >> 8) & 1 -- coil_A sign bit
 *   byte 4 = coil_a & 0xFF     -- coil_A low 8 bits */
void phase_stepping_write_xdirect(uint8_t motor_idx,
                                  int16_t coil_a, int16_t coil_b);

/* Per-bus fault kinds packed into the phase_stepping_commit_tick() return
 * word. Mirror of fault_helpers::PHASE_DMA_KIND_* on the Rust side. */
#define PHASE_DMA_KIND_OVERRUN  1u
#define PHASE_DMA_KIND_TEIF     2u
#define PHASE_DMA_KIND_FEIF     3u
#define PHASE_DMA_KIND_UNDERRUN 4u

/* Swap, cache-clean and arm the per-bus DMA batch for the tick just dispatched.
 * Returns 0 when every bus is clean, else (bus_id << 8) | kind for the first
 * faulting bus. Rust calls this once per tick after all axes are dispatched and
 * raises the matching structured fault; the C ISR never calls back into Rust. */
uint32_t phase_stepping_commit_tick(void);

uint32_t phase_spi_get_skip_count(void);
uint32_t phase_spi_get_write_count(void);

void phase_stepping_enable_writes(void);
void phase_stepping_disable_writes(void);

#ifdef CONFIG_MACH_STM32H7
/* Foreground (task-context) SPI arbitration. A foreground TMC register access
 * claims the owning phase bus so a concurrent DMA arm cannot interleave; the
 * phase batch has priority (commit defers one tick when foreground holds the
 * bus). Returns the bus token, or -1 when the SPI peripheral is not a phase
 * bus (the caller then transfers without arbitration). */
int  phase_spi_fg_begin(struct spi_config config);
void phase_spi_fg_end(int bus_token);

void spi_transfer_locked(struct spi_config config, uint8_t receive_data,
                         uint8_t len, uint8_t *data);
#else
#include "gpio.h"
static inline void
spi_transfer_locked(struct spi_config config, uint8_t receive_data,
                    uint8_t len, uint8_t *data)
{
    spi_transfer(config, receive_data, len, data);
}
#endif

#endif
