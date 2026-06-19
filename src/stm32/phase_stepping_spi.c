#include "autoconf.h"
#include "phase_stepping_spi.h"
#include "gpio.h"
#include "board/irq.h"
#include "board/misc.h"
#include "internal.h"

#define MAX_PHASE_BUSES  4
#define MAX_PHASE_MOTORS 16   // must match Rust state::MAX_STEPPER_OIDS

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
    struct spi_config fast_cfg;
    uint8_t configured;
};

struct phase_motor_state {
    struct gpio_out cs;
    uint8_t bus_id;
    uint8_t configured;
};

static struct phase_bus_state  phase_buses[MAX_PHASE_BUSES];
static struct phase_motor_state phase_motors[MAX_PHASE_MOTORS];

// used,externally_visible: called only from Rust via FFI; without this,
// -fwhole-program LTO DCEs the bodies and the link fails.
__attribute__((used, externally_visible))
void
phase_stepping_register_bus(uint8_t bus_id, struct spi_config cfg)
{
    if (bus_id >= MAX_PHASE_BUSES)
        return;
    phase_buses[bus_id].cfg = cfg;
    // ~1 MHz TMC default = ~40 µs per 5-byte transfer, exceeds the 25 µs tick
    // budget; raise the MBR divisor to ~8 MHz (TMC5160 max SPI rate).
    struct spi_config fast = cfg;
    uint32_t pclk = get_pclock_frequency((uint32_t)(uintptr_t)cfg.spi);
    uint32_t target_rate = 8000000;
    uint32_t div = 0;
    while ((pclk >> (div + 1)) > target_rate && div < 7)
        div++;
    fast.div = div;
    phase_buses[bus_id].fast_cfg = fast;
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

    if (!phase_spi_try_acquire()) {
        phase_spi_skip_count++;
        return;
    }

    // signed >> is implementation-defined; cast through uint16_t for logical shift.
    uint16_t ua = (uint16_t)coil_a;
    uint16_t ub = (uint16_t)coil_b;

    uint8_t datagram[5] = {
        0xAD,                                // write | XDIRECT (0x2D)
        (uint8_t)((ub >> 8) & 0x01),         // coil_B sign bit
        (uint8_t)(ub & 0xFF),                // coil_B low 8 bits
        (uint8_t)((ua >> 8) & 0x01),         // coil_A sign bit
        (uint8_t)(ua & 0xFF),                // coil_A low 8 bits
    };

#if CONFIG_MACH_STM32H7
    struct spi_config fast = phase_buses[bus_id].fast_cfg;
    SPI_TypeDef *spi = fast.spi;

    spi->CFG1 = ((uint32_t)fast.div << SPI_CFG1_MBR_Pos)
              | (7 << SPI_CFG1_DSIZE_Pos);
    spi->CFG2 = ((uint32_t)fast.mode << SPI_CFG2_CPHA_Pos)
              | SPI_CFG2_MASTER | SPI_CFG2_SSM | SPI_CFG2_AFCNTR
              | SPI_CFG2_SSOE;

    gpio_out_write(phase_motors[motor_idx].cs, 0);

    spi->CR2 = 5u << SPI_CR2_TSIZE_Pos;
    spi->CR1 = SPI_CR1_SSI | SPI_CR1_SPE;
    spi->CR1 = SPI_CR1_SSI | SPI_CR1_CSTART | SPI_CR1_SPE;

    for (int i = 0; i < 5; i++) {
        uint32_t deadline = timer_read_time() + timer_from_us(50);
        while (!(spi->SR & SPI_SR_TXP)) {
            if (!timer_is_before(timer_read_time(), deadline))
                goto bail;
        }
        *(volatile uint8_t *)&spi->TXDR = datagram[i];
    }
    for (int i = 0; i < 5; i++) {
        uint32_t deadline = timer_read_time() + timer_from_us(50);
        while (!(spi->SR & SPI_SR_RXP)) {
            if (!timer_is_before(timer_read_time(), deadline))
                goto bail;
        }
        (void)*(volatile uint8_t *)&spi->RXDR;
    }
    {
        uint32_t deadline = timer_read_time() + timer_from_us(100);
        while (!(spi->SR & SPI_SR_EOT)) {
            if (!timer_is_before(timer_read_time(), deadline))
                goto bail;
        }
    }

bail:
    spi->IFCR = 0xFFFFFFFF;
    spi->CR1 = SPI_CR1_SSI;
#else
    spi_prepare(phase_buses[bus_id].fast_cfg);
    gpio_out_write(phase_motors[motor_idx].cs, 0);
    spi_transfer(phase_buses[bus_id].fast_cfg, 0,
                 sizeof(datagram), datagram);
#endif
    gpio_out_write(phase_motors[motor_idx].cs, 1);

    phase_spi_write_count++;
    phase_spi_release();
}

#define TMC_WRITE_BIT 0x80

// Spin until the busy flag is ours or a 1 ms deadline passes. The ISR XDIRECT
// writes are suppressed (phase_stepping_disable_writes) across a handover, so
// real contention is nil; the deadline only guards against a wedged peer.
static int
phase_spi_acquire_blocking(void)
{
    uint32_t deadline = timer_read_time() + timer_from_us(1000);
    while (!phase_spi_try_acquire()) {
        if (!timer_is_before(timer_read_time(), deadline))
            return -1;
    }
    return 0;
}

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

// One 5-byte TMC datagram. Caller MUST already hold phase_spi_busy. With
// `capture`, the received bytes overwrite `buf` (a TMC read returns the
// previously-addressed register on the *next* access, so a register read is
// two of these: prime, then capture). Returns 0 or -1 on a per-byte timeout.
static int
phase_spi_xfer5_locked(uint8_t bus_id, uint8_t motor_idx, uint8_t *buf,
                       uint8_t capture)
{
#if CONFIG_MACH_STM32H7
    struct spi_config fast = phase_buses[bus_id].fast_cfg;
    SPI_TypeDef *spi = fast.spi;

    spi->CFG1 = ((uint32_t)fast.div << SPI_CFG1_MBR_Pos)
              | (7 << SPI_CFG1_DSIZE_Pos);
    spi->CFG2 = ((uint32_t)fast.mode << SPI_CFG2_CPHA_Pos)
              | SPI_CFG2_MASTER | SPI_CFG2_SSM | SPI_CFG2_AFCNTR
              | SPI_CFG2_SSOE;

    gpio_out_write(phase_motors[motor_idx].cs, 0);

    spi->CR2 = 5u << SPI_CR2_TSIZE_Pos;
    spi->CR1 = SPI_CR1_SSI | SPI_CR1_SPE;
    spi->CR1 = SPI_CR1_SSI | SPI_CR1_CSTART | SPI_CR1_SPE;

    int timed_out = 0;
    for (int i = 0; i < 5; i++) {
        uint32_t deadline = timer_read_time() + timer_from_us(100);
        while (!(spi->SR & SPI_SR_TXP)) {
            if (!timer_is_before(timer_read_time(), deadline)) {
                timed_out = 1;
                goto done;
            }
        }
        *(volatile uint8_t *)&spi->TXDR = buf[i];
    }
    for (int i = 0; i < 5; i++) {
        uint32_t deadline = timer_read_time() + timer_from_us(100);
        while (!(spi->SR & SPI_SR_RXP)) {
            if (!timer_is_before(timer_read_time(), deadline)) {
                timed_out = 1;
                goto done;
            }
        }
        uint8_t rx = *(volatile uint8_t *)&spi->RXDR;
        if (capture)
            buf[i] = rx;
    }
    {
        uint32_t deadline = timer_read_time() + timer_from_us(100);
        while (!(spi->SR & SPI_SR_EOT)) {
            if (!timer_is_before(timer_read_time(), deadline)) {
                timed_out = 1;
                goto done;
            }
        }
    }

done:
    spi->IFCR = 0xFFFFFFFF;
    spi->CR1 = SPI_CR1_SSI;
    gpio_out_write(phase_motors[motor_idx].cs, 1);
    return timed_out ? -1 : 0;
#else
    spi_prepare(phase_buses[bus_id].fast_cfg);
    gpio_out_write(phase_motors[motor_idx].cs, 0);
    spi_transfer_locked(phase_buses[bus_id].fast_cfg, capture, 5, buf);
    gpio_out_write(phase_motors[motor_idx].cs, 1);
    return 0;
#endif
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
    int rc = phase_spi_xfer5_locked(bus_id, motor_idx, buf, 0);
    phase_spi_release();
    return rc;
}

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
    int rc = phase_spi_xfer5_locked(bus_id, motor_idx, buf, 0);
    if (!rc) {
        buf[0] = (uint8_t)(addr & 0x7F);
        buf[1] = buf[2] = buf[3] = buf[4] = 0;
        rc = phase_spi_xfer5_locked(bus_id, motor_idx, buf, 1);
    }
    phase_spi_release();
    if (rc)
        return -1;
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
