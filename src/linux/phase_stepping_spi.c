#include "phase_stepping_spi.h"
#include "gpio.h"
#include "board/irq.h"

#define MAX_PHASE_BUSES  4
#define MAX_PHASE_MOTORS 16

static volatile uint8_t  phase_spi_busy = 0;
static volatile uint32_t phase_spi_skip_count = 0;
static volatile uint32_t phase_spi_write_count = 0;
static volatile uint8_t phase_spi_writes_enabled = 0;

__attribute__((used, externally_visible))
uint8_t
phase_spi_try_acquire(void)
{
    irqstatus_t flag = irq_save();
    uint8_t was_busy = phase_spi_busy;
    if (!was_busy)
        phase_spi_busy = 1;
    irq_restore(flag);
    return !was_busy;
}

__attribute__((used, externally_visible))
void
phase_spi_release(void)
{
    phase_spi_busy = 0;
}

__attribute__((used, externally_visible))
uint32_t
phase_spi_get_skip_count(void)
{
    return phase_spi_skip_count;
}

__attribute__((used, externally_visible))
uint32_t
phase_spi_get_write_count(void)
{
    return phase_spi_write_count;
}

struct phase_bus_state {
    struct spi_config cfg;
    uint8_t configured;
};

struct phase_motor_state {
    struct gpio_out cs;
    uint8_t bus_id;
    uint8_t configured;
};

static struct phase_bus_state  phase_buses[MAX_PHASE_BUSES];
static struct phase_motor_state phase_motors[MAX_PHASE_MOTORS];

// used,externally_visible: called only from the Rust runtime via FFI, so
// -fwhole-program LTO would DCE the bodies and the link would fail.
__attribute__((used, externally_visible))
void
phase_stepping_register_bus(uint8_t bus_id, struct spi_config cfg)
{
    if (bus_id >= MAX_PHASE_BUSES)
        return;
    phase_buses[bus_id].cfg = cfg;
    phase_buses[bus_id].configured = 1;
}

__attribute__((used, externally_visible))
void
phase_stepping_register_motor(uint8_t motor_idx, uint8_t bus_id,
                              uint8_t cs_pin_id)
{
    if (motor_idx >= MAX_PHASE_MOTORS || bus_id >= MAX_PHASE_BUSES)
        return;
    phase_motors[motor_idx].cs = gpio_out_setup(cs_pin_id, 1); // idle high
    phase_motors[motor_idx].bus_id = bus_id;
    phase_motors[motor_idx].configured = 1;
}

__attribute__((used, externally_visible))
void
phase_stepping_enable_writes(void)
{
    phase_spi_writes_enabled = 1;
}

__attribute__((used, externally_visible))
void
phase_stepping_disable_writes(void)
{
    phase_spi_writes_enabled = 0;
}

__attribute__((used, externally_visible))
void
phase_stepping_write_xdirect(uint8_t motor_idx,
                             int16_t coil_a, int16_t coil_b)
{
    if (!phase_spi_writes_enabled)
        return;
    if (motor_idx >= MAX_PHASE_MOTORS || !phase_motors[motor_idx].configured)
        return;
    uint8_t bus_id = phase_motors[motor_idx].bus_id;
    if (bus_id >= MAX_PHASE_BUSES || !phase_buses[bus_id].configured)
        return;

    // If Klipper's spi_transfer holds the bus, skip this cycle (one skip = 25 us,
    // inaudible); the skip count is the SPI3-contention canary.
    if (!phase_spi_try_acquire()) {
        phase_spi_skip_count++;
        return;
    }

    // Cast through uint16_t so the sign bit shifts logically (signed >> is
    // implementation-defined).
    uint16_t ua = (uint16_t)coil_a;
    uint16_t ub = (uint16_t)coil_b;

    uint8_t datagram[5] = {
        0xAD,                                // write | XDIRECT (0x2D)
        (uint8_t)((ub >> 8) & 0x01),         // coil_B sign bit
        (uint8_t)(ub & 0xFF),                // coil_B low 8 bits
        (uint8_t)((ua >> 8) & 0x01),         // coil_A sign bit
        (uint8_t)(ua & 0xFF),                // coil_A low 8 bits
    };

    spi_prepare(phase_buses[bus_id].cfg);
    gpio_out_write(phase_motors[motor_idx].cs, 0);
    // Unlocked variant — we already hold phase_spi_busy; the public
    // spi_transfer would deadlock the ISR on its own lock.
    spi_transfer_locked(phase_buses[bus_id].cfg, 0,
                        sizeof(datagram), datagram);
    gpio_out_write(phase_motors[motor_idx].cs, 1);

    phase_spi_write_count++;
    phase_spi_release();
}

#define TMC_WRITE_BIT 0x80

static int
phase_motor_bus(uint8_t motor_idx, uint8_t *bus_out)
{
    if (motor_idx >= MAX_PHASE_MOTORS || !phase_motors[motor_idx].configured)
        return -1;
    uint8_t bus_id = phase_motors[motor_idx].bus_id;
    if (bus_id >= MAX_PHASE_BUSES || !phase_buses[bus_id].configured)
        return -1;
    *bus_out = bus_id;
    return 0;
}

static int
phase_spi_acquire_blocking(void)
{
    for (uint32_t spins = 0; spins < 1000000; spins++) {
        if (phase_spi_try_acquire())
            return 0;
    }
    return -1;
}

static void
phase_reg_xfer(uint8_t bus_id, uint8_t motor_idx, uint8_t *buf, uint8_t capture)
{
    spi_prepare(phase_buses[bus_id].cfg);
    gpio_out_write(phase_motors[motor_idx].cs, 0);
    spi_transfer_locked(phase_buses[bus_id].cfg, capture, 5, buf);
    gpio_out_write(phase_motors[motor_idx].cs, 1);
}

__attribute__((used, externally_visible))
int
phase_spi_write_register(uint8_t motor_idx, uint8_t addr, uint32_t val)
{
    uint8_t bus_id;
    if (phase_motor_bus(motor_idx, &bus_id))
        return -1;
    if (phase_spi_acquire_blocking())
        return -1;
    uint8_t buf[5] = {
        (uint8_t)(TMC_WRITE_BIT | (addr & 0x7F)),
        (uint8_t)(val >> 24), (uint8_t)(val >> 16),
        (uint8_t)(val >> 8), (uint8_t)val,
    };
    phase_reg_xfer(bus_id, motor_idx, buf, 0);
    phase_spi_release();
    return 0;
}

// A TMC read returns the addressed register on the *next* datagram, so the
// first transfer only primes the address and the second captures the value.
__attribute__((used, externally_visible))
int
phase_spi_read_register(uint8_t motor_idx, uint8_t addr, uint32_t *out)
{
    uint8_t bus_id;
    if (phase_motor_bus(motor_idx, &bus_id))
        return -1;
    if (phase_spi_acquire_blocking())
        return -1;
    uint8_t buf[5] = { (uint8_t)(addr & 0x7F), 0, 0, 0, 0 };
    phase_reg_xfer(bus_id, motor_idx, buf, 0);
    buf[0] = (uint8_t)(addr & 0x7F);
    buf[1] = buf[2] = buf[3] = buf[4] = 0;
    phase_reg_xfer(bus_id, motor_idx, buf, 1);
    phase_spi_release();
    *out = ((uint32_t)buf[1] << 24) | ((uint32_t)buf[2] << 16)
         | ((uint32_t)buf[3] << 8) | (uint32_t)buf[4];
    return 0;
}

__attribute__((used, externally_visible))
int
phase_spi_rmw_register(uint8_t motor_idx, uint8_t addr, uint32_t mask,
                       uint32_t set_bits, uint32_t *verified)
{
    uint32_t cur;
    if (phase_spi_read_register(motor_idx, addr, &cur))
        return -1;
    uint32_t next = (cur & ~mask) | (set_bits & mask);
    if (phase_spi_write_register(motor_idx, addr, next))
        return -1;
    return phase_spi_read_register(motor_idx, addr, verified);
}
