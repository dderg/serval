#include "autoconf.h"
#include "phase_stepping_spi.h"
#include "gpio.h"
#include "board/irq.h"
#include "board/misc.h"
#include "board/armcm_boot.h"
#include "generic/motion_nvic_prio.h"
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

#if CONFIG_MACH_STM32H7
#define PHASE_DMA_SPI1_TX_REQ 38u
#define PHASE_DMA_SPI1_RX_REQ 37u
#define PHASE_DMA_TX_STREAM   DMA1_Stream0
#define PHASE_DMA_RX_STREAM   DMA1_Stream1
#define PHASE_DMA_CLEAR01 ( \
      DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTEIF0 \
    | DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CFEIF0 \
    | DMA_LIFCR_CTCIF1 | DMA_LIFCR_CHTIF1 | DMA_LIFCR_CTEIF1 \
    | DMA_LIFCR_CDMEIF1 | DMA_LIFCR_CFEIF1)

#define PHASE_Q_SIZE 16
#define PHASE_Q_MASK (PHASE_Q_SIZE - 1)

static uint8_t __attribute__((section(".axi_bss"), aligned(32)))
    phase_dma_txbuf[MAX_PHASE_MOTORS][32];
static uint8_t __attribute__((section(".axi_bss"), aligned(32))) phase_dma_rxbuf[32];
static uint8_t phase_dma_inited;

static volatile uint8_t phase_q[PHASE_Q_SIZE];
static volatile uint8_t phase_q_head;
static volatile uint8_t phase_q_tail;
static volatile uint8_t phase_dma_active;
static struct gpio_out phase_dma_cur_cs;
static SPI_TypeDef *phase_dma_cur_spi;

static void
phase_dma_arm(uint8_t motor)
{
    uint8_t bus_id = phase_motors[motor].bus_id;
    struct spi_config fast = phase_buses[bus_id].fast_cfg;
    SPI_TypeDef *spi = fast.spi;

    spi->CR1 = SPI_CR1_SSI;
    spi->CFG1 = ((uint32_t)fast.div << SPI_CFG1_MBR_Pos)
              | (7 << SPI_CFG1_DSIZE_Pos)
              | SPI_CFG1_TXDMAEN | SPI_CFG1_RXDMAEN;
    spi->CFG2 = ((uint32_t)fast.mode << SPI_CFG2_CPHA_Pos)
              | SPI_CFG2_MASTER | SPI_CFG2_SSM | SPI_CFG2_AFCNTR | SPI_CFG2_SSOE;

    PHASE_DMA_TX_STREAM->CR &= ~DMA_SxCR_EN;
    PHASE_DMA_RX_STREAM->CR &= ~DMA_SxCR_EN;
    while ((PHASE_DMA_TX_STREAM->CR | PHASE_DMA_RX_STREAM->CR) & DMA_SxCR_EN)
        ;
    DMA1->LIFCR = PHASE_DMA_CLEAR01;

    PHASE_DMA_RX_STREAM->FCR = 0;
    PHASE_DMA_RX_STREAM->PAR = (uint32_t)(uintptr_t)&spi->RXDR;
    PHASE_DMA_RX_STREAM->M0AR = (uint32_t)(uintptr_t)phase_dma_rxbuf;
    PHASE_DMA_RX_STREAM->NDTR = 5;
    PHASE_DMA_RX_STREAM->CR = DMA_SxCR_MINC | DMA_SxCR_TCIE;

    PHASE_DMA_TX_STREAM->FCR = 0;
    PHASE_DMA_TX_STREAM->PAR = (uint32_t)(uintptr_t)&spi->TXDR;
    PHASE_DMA_TX_STREAM->M0AR = (uint32_t)(uintptr_t)phase_dma_txbuf[motor];
    PHASE_DMA_TX_STREAM->NDTR = 5;
    PHASE_DMA_TX_STREAM->CR = DMA_SxCR_DIR_0 | DMA_SxCR_MINC;

    gpio_out_write(phase_motors[motor].cs, 0);

    PHASE_DMA_RX_STREAM->CR |= DMA_SxCR_EN;
    PHASE_DMA_TX_STREAM->CR |= DMA_SxCR_EN;

    spi->CR2 = 5u << SPI_CR2_TSIZE_Pos;
    spi->CR1 = SPI_CR1_SSI | SPI_CR1_SPE;
    spi->IFCR = 0xFFFFFFFF;
    spi->CR1 = SPI_CR1_SSI | SPI_CR1_CSTART | SPI_CR1_SPE;

    phase_dma_cur_cs = phase_motors[motor].cs;
    phase_dma_cur_spi = spi;
}

static void
phase_dma_advance(void)
{
    if (phase_q_head == phase_q_tail) {
        phase_dma_active = 0;
        phase_spi_release();
        return;
    }
    uint8_t motor = phase_q[phase_q_head];
    phase_q_head = (phase_q_head + 1) & PHASE_Q_MASK;
    phase_dma_arm(motor);
}

static void
phase_dma_complete_current(void)
{
    SPI_TypeDef *spi = phase_dma_cur_spi;
    gpio_out_write(phase_dma_cur_cs, 1);
    PHASE_DMA_TX_STREAM->CR &= ~DMA_SxCR_EN;
    PHASE_DMA_RX_STREAM->CR &= ~DMA_SxCR_EN;
    DMA1->LIFCR = PHASE_DMA_CLEAR01;
    spi->IFCR = 0xFFFFFFFF;
    spi->CR1 = SPI_CR1_SSI;
    phase_spi_write_count++;
}

void
phase_dma_rx_isr(void)
{
    if (DMA1->LISR & DMA_LISR_TCIF1) {
        DMA1->LIFCR = DMA_LIFCR_CTCIF1;
        phase_dma_complete_current();
        irqstatus_t flag = irq_save();
        phase_dma_advance();
        irq_restore(flag);
    }
}

static void
phase_dma_init_once(void)
{
    if (phase_dma_inited)
        return;
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
    (void)RCC->AHB1ENR;
    DMAMUX1_Channel0->CCR = PHASE_DMA_SPI1_TX_REQ;
    DMAMUX1_Channel1->CCR = PHASE_DMA_SPI1_RX_REQ;
    armcm_enable_irq(phase_dma_rx_isr, DMA1_Stream1_IRQn, MOTION_NVIC_PRIO - 1);
    phase_dma_inited = 1;
}
#endif

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
    phase_dma_init_once();

    for (int i = 0; i < 5; i++)
        phase_dma_txbuf[motor_idx][i] = datagram[i];
    SCB_CleanDCache_by_Addr((uint32_t *)phase_dma_txbuf[motor_idx], 32);

    irqstatus_t flag = irq_save();
    if (phase_dma_active) {
        uint8_t next = (phase_q_tail + 1) & PHASE_Q_MASK;
        if (next == phase_q_head) {
            phase_spi_skip_count++;
        } else {
            phase_q[phase_q_tail] = motor_idx;
            phase_q_tail = next;
        }
    } else if (phase_spi_try_acquire()) {
        phase_q[phase_q_tail] = motor_idx;
        phase_q_tail = (phase_q_tail + 1) & PHASE_Q_MASK;
        phase_dma_active = 1;
        phase_dma_advance();
    } else {
        phase_spi_skip_count++;
    }
    irq_restore(flag);
#else
    if (!phase_spi_try_acquire()) {
        phase_spi_skip_count++;
        return;
    }
    spi_prepare(phase_buses[bus_id].fast_cfg);
    gpio_out_write(phase_motors[motor_idx].cs, 0);
    spi_transfer(phase_buses[bus_id].fast_cfg, 0,
                 sizeof(datagram), datagram);
    gpio_out_write(phase_motors[motor_idx].cs, 1);
    phase_spi_write_count++;
    phase_spi_release();
#endif
}
