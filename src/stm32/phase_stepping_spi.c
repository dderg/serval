#include "autoconf.h"
#include "phase_stepping_spi.h"
#include "gpio.h"
#include "command.h"
#include "board/irq.h"
#include "board/misc.h"
#include "internal.h"
#if CONFIG_MACH_STM32H7
#include "board/armcm_boot.h"
#include "generic/motion_nvic_prio.h"
#endif

#define MAX_PHASE_BUSES   4
#define MAX_PHASE_MOTORS  16   // must match Rust state::MAX_STEPPER_OIDS
#define XDIRECT_LEN       5
#define PHASE_TXBUF_STRIDE 96
#define PHASE_FG_BOUNCE   32   // foreground DMA scratch; one cache line, holds a
                               // 5-byte TMC datagram. Longer transfers fall back
                               // to PIO (never issued during phase stepping).
_Static_assert(PHASE_TXBUF_STRIDE % 32 == 0,
               "half-buffer must be 32-byte (cache-line) aligned: "
               "SCB_CleanDCache_by_Addr granularity is one 32-byte line");
_Static_assert(PHASE_TXBUF_STRIDE >= MAX_PHASE_MOTORS * XDIRECT_LEN,
               "half-buffer too small to hold every motor's datagram");

#define PHASE_FAULT_TEIF 0x01u
#define PHASE_FAULT_FEIF 0x02u
#define PHASE_FAULT_EOT  0x04u

// Packed into the free upper bits [31:16] of the commit status word; Rust
// decodes only [15:0] (bus<<8|kind), so these reach the logged fault_detail.
#define PHASE_DIAG_CURSOR_SHIFT 16          // bits 19:16 = cursor at fault
#define PHASE_DIAG_NDTR_SHIFT   20          // bits 23:20 = DMA bytes still pending
#define PHASE_DIAG_TCIF_BIT     (1u << 24)  // DMA TCIF latched (transfer done)
#define PHASE_DIAG_TCRAN_BIT    (1u << 25)  // TC ISR has executed at least once
#define PHASE_DIAG_FGSTUCK_BIT  (1u << 26)  // overrun source = foreground holder
#define PHASE_DIAG_SPE_BIT      (1u << 27)  // SPI SPE set (peripheral enabled)
#define PHASE_DIAG_TXP_BIT      (1u << 28)  // SPI TXP (tx fifo has space)
#define PHASE_DIAG_DMAEN_BIT    (1u << 29)  // DMA stream still enabled
#define PHASE_DIAG_SUSP_BIT     (1u << 30)  // SPI master suspended (clock stalled)
#define PHASE_DIAG_EOT_BIT      (1u << 31)  // SPI EOT set (transfer ended)

enum { PHASE_OWNER_NONE = 0, PHASE_OWNER_PHASE = 1, PHASE_OWNER_FG = 2 };

struct phase_bus_state {
    struct spi_config cfg;
    struct spi_config fast_cfg;
    uint8_t configured;

    uint8_t seq[MAX_PHASE_MOTORS];
    uint8_t seq_len;

#if CONFIG_MACH_STM32H7
    __attribute__((aligned(32))) uint8_t txbuf[2][PHASE_TXBUF_STRIDE];
    __attribute__((aligned(32))) uint8_t fg_txbuf[PHASE_FG_BOUNCE];
    __attribute__((aligned(32))) uint8_t fg_rxbuf[PHASE_FG_BOUNCE];
    volatile uint8_t busy;
    volatile uint8_t owner;
    volatile uint8_t sticky_fault;
    uint8_t active_half;
    uint8_t commit_half;
    uint8_t cursor;
    uint8_t fg_defer_count;

    DMA_Stream_TypeDef *stream;
    DMAMUX_Channel_TypeDef *mux;
    volatile uint32_t *isr_reg;
    volatile uint32_t *ifcr_reg;
    uint32_t tcif, teif, feif, flag_clear;
    IRQn_Type irqn;

    DMA_Stream_TypeDef *rx_stream;
    DMAMUX_Channel_TypeDef *rx_mux;
    volatile uint32_t *rx_isr_reg;
    volatile uint32_t *rx_ifcr_reg;
    uint32_t rx_tcif, rx_teif, rx_feif, rx_flag_clear;
#endif
};

struct phase_motor_state {
    struct gpio_out cs;
    uint8_t bus_id;
    uint8_t configured;
};

#if CONFIG_MACH_STM32H7
// DMA1/2 cannot reach DTCM (0x20000000); the TX double-buffer must live in
// DMA-reachable AXI SRAM. Budget is asserted here and summed in runtime_storage.c.
__attribute__((section(".axi_bss")))
#endif
static struct phase_bus_state  phase_buses[MAX_PHASE_BUSES];
static struct phase_motor_state phase_motors[MAX_PHASE_MOTORS];

#if CONFIG_MACH_STM32H7
_Static_assert(sizeof(phase_buses) <= 2048,
               "phase_buses exceeds its .axi_bss budget — raise "
               "AXI_BSS_PHASE_BUSES_BYTES in runtime_storage.c to match");
#endif

static volatile uint32_t phase_defer_count = 0;
static volatile uint32_t phase_write_count = 0;
static volatile uint8_t  phase_spi_writes_enabled = 0;

__attribute__((used, externally_visible))
uint32_t
phase_spi_get_skip_count(void)
{
    return phase_defer_count;
}

__attribute__((used, externally_visible))
uint32_t
phase_spi_get_write_count(void)
{
    return phase_write_count;
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

static void
phase_pack_datagram(uint8_t *dst, int16_t coil_a, int16_t coil_b)
{
    uint16_t ua = (uint16_t)coil_a;
    uint16_t ub = (uint16_t)coil_b;
    dst[0] = 0xAD;
    dst[1] = (uint8_t)((ub >> 8) & 0x01);
    dst[2] = (uint8_t)(ub & 0xFF);
    dst[3] = (uint8_t)((ua >> 8) & 0x01);
    dst[4] = (uint8_t)(ua & 0xFF);
}

#if CONFIG_MACH_STM32H7

// DMAMUX1 request-line IDs for each SPI peripheral's TX channel (RM0433 Tbl.
// "DMAMUX1: assignment of multiplexer inputs to resources").
#define SPI1_TX_DMAMUX_REQ 38
#define SPI2_TX_DMAMUX_REQ 40
#define SPI3_TX_DMAMUX_REQ 62
#define SPI4_TX_DMAMUX_REQ 84
#define SPI5_TX_DMAMUX_REQ 86

// Each SPI's RX request line sits one below its TX line in the DMAMUX1 table.
#define SPI1_RX_DMAMUX_REQ 37
#define SPI2_RX_DMAMUX_REQ 39
#define SPI3_RX_DMAMUX_REQ 61
#define SPI4_RX_DMAMUX_REQ 83
#define SPI5_RX_DMAMUX_REQ 85

struct dma_assign {
    DMA_Stream_TypeDef *stream;
    DMAMUX_Channel_TypeDef *mux;
    IRQn_Type irqn;
    uint32_t tcif, teif, feif, clear;
};

static const struct dma_assign dma_table[MAX_PHASE_BUSES] = {
    { DMA1_Stream0, DMAMUX1_Channel0, DMA1_Stream0_IRQn, DMA_LISR_TCIF0,
      DMA_LISR_TEIF0, DMA_LISR_FEIF0, DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0
      | DMA_LIFCR_CTEIF0 | DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CFEIF0 },
    { DMA1_Stream1, DMAMUX1_Channel1, DMA1_Stream1_IRQn, DMA_LISR_TCIF1,
      DMA_LISR_TEIF1, DMA_LISR_FEIF1, DMA_LIFCR_CTCIF1 | DMA_LIFCR_CHTIF1
      | DMA_LIFCR_CTEIF1 | DMA_LIFCR_CDMEIF1 | DMA_LIFCR_CFEIF1 },
    { DMA1_Stream2, DMAMUX1_Channel2, DMA1_Stream2_IRQn, DMA_LISR_TCIF2,
      DMA_LISR_TEIF2, DMA_LISR_FEIF2, DMA_LIFCR_CTCIF2 | DMA_LIFCR_CHTIF2
      | DMA_LIFCR_CTEIF2 | DMA_LIFCR_CDMEIF2 | DMA_LIFCR_CFEIF2 },
    { DMA1_Stream3, DMAMUX1_Channel3, DMA1_Stream3_IRQn, DMA_LISR_TCIF3,
      DMA_LISR_TEIF3, DMA_LISR_FEIF3, DMA_LIFCR_CTCIF3 | DMA_LIFCR_CHTIF3
      | DMA_LIFCR_CTEIF3 | DMA_LIFCR_CDMEIF3 | DMA_LIFCR_CFEIF3 },
};

// RX streams for foreground full-duplex reads. Foreground polls completion, so
// these carry no IRQ wiring. Streams 4-7 report status in the high half (HISR/
// HIFCR); the phase TX streams 0-3 use the low half.
static const struct dma_assign rx_dma_table[MAX_PHASE_BUSES] = {
    { DMA1_Stream4, DMAMUX1_Channel4, DMA1_Stream4_IRQn, DMA_HISR_TCIF4,
      DMA_HISR_TEIF4, DMA_HISR_FEIF4, DMA_HIFCR_CTCIF4 | DMA_HIFCR_CHTIF4
      | DMA_HIFCR_CTEIF4 | DMA_HIFCR_CDMEIF4 | DMA_HIFCR_CFEIF4 },
    { DMA1_Stream5, DMAMUX1_Channel5, DMA1_Stream5_IRQn, DMA_HISR_TCIF5,
      DMA_HISR_TEIF5, DMA_HISR_FEIF5, DMA_HIFCR_CTCIF5 | DMA_HIFCR_CHTIF5
      | DMA_HIFCR_CTEIF5 | DMA_HIFCR_CDMEIF5 | DMA_HIFCR_CFEIF5 },
    { DMA1_Stream6, DMAMUX1_Channel6, DMA1_Stream6_IRQn, DMA_HISR_TCIF6,
      DMA_HISR_TEIF6, DMA_HISR_FEIF6, DMA_HIFCR_CTCIF6 | DMA_HIFCR_CHTIF6
      | DMA_HIFCR_CTEIF6 | DMA_HIFCR_CDMEIF6 | DMA_HIFCR_CFEIF6 },
    { DMA1_Stream7, DMAMUX1_Channel7, DMA1_Stream7_IRQn, DMA_HISR_TCIF7,
      DMA_HISR_TEIF7, DMA_HISR_FEIF7, DMA_HIFCR_CTCIF7 | DMA_HIFCR_CHTIF7
      | DMA_HIFCR_CTEIF7 | DMA_HIFCR_CDMEIF7 | DMA_HIFCR_CFEIF7 },
};

static uint8_t
phase_spi_tx_request(SPI_TypeDef *spi)
{
    if (spi == SPI1)
        return SPI1_TX_DMAMUX_REQ;
    if (spi == SPI2)
        return SPI2_TX_DMAMUX_REQ;
#ifdef SPI3
    if (spi == SPI3)
        return SPI3_TX_DMAMUX_REQ;
#endif
#ifdef SPI4
    if (spi == SPI4)
        return SPI4_TX_DMAMUX_REQ;
#endif
#ifdef SPI5
    if (spi == SPI5)
        return SPI5_TX_DMAMUX_REQ;
#endif
    shutdown("phase DMA: SPI peripheral not on DMAMUX1 (use spi1..5)");
}

static uint8_t
phase_spi_rx_request(SPI_TypeDef *spi)
{
    if (spi == SPI1)
        return SPI1_RX_DMAMUX_REQ;
    if (spi == SPI2)
        return SPI2_RX_DMAMUX_REQ;
#ifdef SPI3
    if (spi == SPI3)
        return SPI3_RX_DMAMUX_REQ;
#endif
#ifdef SPI4
    if (spi == SPI4)
        return SPI4_RX_DMAMUX_REQ;
#endif
#ifdef SPI5
    if (spi == SPI5)
        return SPI5_RX_DMAMUX_REQ;
#endif
    shutdown("phase DMA: SPI peripheral not on DMAMUX1 (use spi1..5)");
}

static volatile uint32_t phase_tc_count;

static void phase_dma_tc_isr(uint8_t bus_id);

void phase_dma_s0_irq(void) { phase_dma_tc_isr(0); }
void phase_dma_s1_irq(void) { phase_dma_tc_isr(1); }
void phase_dma_s2_irq(void) { phase_dma_tc_isr(2); }
void phase_dma_s3_irq(void) { phase_dma_tc_isr(3); }

static void
phase_dma_init_bus(uint8_t bus_id)
{
    struct phase_bus_state *bus = &phase_buses[bus_id];
    const struct dma_assign *a = &dma_table[bus_id];

    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
    (void)RCC->AHB1ENR;

    bus->stream = a->stream;
    bus->mux = a->mux;
    bus->isr_reg = &DMA1->LISR;
    bus->ifcr_reg = &DMA1->LIFCR;
    bus->tcif = a->tcif;
    bus->teif = a->teif;
    bus->feif = a->feif;
    bus->flag_clear = a->clear;
    bus->irqn = a->irqn;

    bus->stream->CR = 0;
    while (bus->stream->CR & DMA_SxCR_EN)
        ;
    *bus->ifcr_reg = bus->flag_clear;
    bus->mux->CCR = phase_spi_tx_request(bus->cfg.spi);

    const struct dma_assign *r = &rx_dma_table[bus_id];
    bus->rx_stream = r->stream;
    bus->rx_mux = r->mux;
    bus->rx_isr_reg = &DMA1->HISR;
    bus->rx_ifcr_reg = &DMA1->HIFCR;
    bus->rx_tcif = r->tcif;
    bus->rx_teif = r->teif;
    bus->rx_feif = r->feif;
    bus->rx_flag_clear = r->clear;

    bus->rx_stream->CR = 0;
    while (bus->rx_stream->CR & DMA_SxCR_EN)
        ;
    *bus->rx_ifcr_reg = bus->rx_flag_clear;
    bus->rx_mux->CCR = phase_spi_rx_request(bus->cfg.spi);

    switch (bus_id) {
    case 0:
        armcm_enable_irq(phase_dma_s0_irq, DMA1_Stream0_IRQn, MOTION_NVIC_PRIO);
        break;
    case 1:
        armcm_enable_irq(phase_dma_s1_irq, DMA1_Stream1_IRQn, MOTION_NVIC_PRIO);
        break;
    case 2:
        armcm_enable_irq(phase_dma_s2_irq, DMA1_Stream2_IRQn, MOTION_NVIC_PRIO);
        break;
    case 3:
        armcm_enable_irq(phase_dma_s3_irq, DMA1_Stream3_IRQn, MOTION_NVIC_PRIO);
        break;
    default:
        break;
    }
}

static void
phase_dma_arm_motor(struct phase_bus_state *bus)
{
    uint8_t midx = bus->seq[bus->cursor];
    struct spi_config fast = bus->fast_cfg;
    SPI_TypeDef *spi = fast.spi;

    spi->CR1 = SPI_CR1_SSI; // CFG1/CFG2 are write-protected while SPE=1
    spi->CFG1 = ((uint32_t)fast.div << SPI_CFG1_MBR_Pos)
              | (7u << SPI_CFG1_DSIZE_Pos)
              | SPI_CFG1_TXDMAEN | SPI_CFG1_RXDMAEN;
    // Full-duplex on every bus access: phase writes and foreground reads run the
    // identical COMM mode, so the peripheral never flips between simplex
    // transmitter and full-duplex — a flip left the master suspended for the
    // next arm to inherit (CSTART onto a leftover suspend moves no bytes). The
    // RX-DMA drains the inbound fifo each frame, which is what the earlier
    // simplex switch was avoiding: undrained RX gated the master clock.
    spi->CFG2 = ((uint32_t)fast.mode << SPI_CFG2_CPHA_Pos)
              | SPI_CFG2_MASTER | SPI_CFG2_SSM | SPI_CFG2_AFCNTR
              | SPI_CFG2_SSOE;

    DMA_Stream_TypeDef *st = bus->stream;
    DMA_Stream_TypeDef *rx = bus->rx_stream;
    st->CR &= ~DMA_SxCR_EN;
    while (st->CR & DMA_SxCR_EN)
        ;
    rx->CR &= ~DMA_SxCR_EN;
    while (rx->CR & DMA_SxCR_EN)
        ;
    st->FCR = 0; // direct mode, no FIFO-error interrupt — not the reset 0x21
    rx->FCR = 0;
    *bus->ifcr_reg = bus->flag_clear;
    *bus->rx_ifcr_reg = bus->rx_flag_clear;

    // RX drains the inbound fifo to one scratch byte (XDIRECT is write-only, its
    // response is discarded); no MINC, no completion IRQ — TX-TC stays the
    // trigger and the EOT gate below times CS-high.
    rx->PAR = (uint32_t)(uintptr_t)&spi->RXDR;
    rx->M0AR = (uint32_t)(uintptr_t)&bus->fg_rxbuf[0];
    rx->NDTR = XDIRECT_LEN;
    rx->CR = DMA_SxCR_EN;

    st->PAR = (uint32_t)(uintptr_t)&spi->TXDR;
    st->M0AR = (uint32_t)(uintptr_t)&bus->txbuf[bus->commit_half][midx * XDIRECT_LEN];
    st->NDTR = XDIRECT_LEN;
    st->CR = DMA_SxCR_DIR_0 | DMA_SxCR_MINC | DMA_SxCR_TCIE | DMA_SxCR_TEIE;
    st->CR |= DMA_SxCR_EN;

    gpio_out_write(phase_motors[midx].cs, 0);

    spi->CR2 = (uint32_t)XDIRECT_LEN << SPI_CR2_TSIZE_Pos;
    spi->CR1 = SPI_CR1_SSI | SPI_CR1_SPE;
    // Clear stale EOT/SUSP only after SPE=1 — while the peripheral is disabled
    // the clear does not take, so a CSTART onto a leftover suspend (from a
    // foreground transfer or a torn-down batch) starts already-ended and raises
    // no TX-DMA request, freezing NDTR at full.
    spi->IFCR = 0xFFFFFFFF;
    spi->CR1 = SPI_CR1_SSI | SPI_CR1_CSTART | SPI_CR1_SPE;
}

static void
phase_dma_release_current(struct phase_bus_state *bus)
{
    bus->stream->CR &= ~DMA_SxCR_EN;
    bus->rx_stream->CR &= ~DMA_SxCR_EN;
    *bus->ifcr_reg = bus->flag_clear;
    *bus->rx_ifcr_reg = bus->rx_flag_clear;
    SPI_TypeDef *spi = bus->fast_cfg.spi;
    spi->IFCR = 0xFFFFFFFF;
    spi->CR1 = SPI_CR1_SSI;
    gpio_out_write(phase_motors[bus->seq[bus->cursor]].cs, 1);
    bus->busy = 0;
    bus->owner = PHASE_OWNER_NONE;
}

static void
phase_dma_tc_isr(uint8_t bus_id)
{
    phase_tc_count++;
    struct phase_bus_state *bus = &phase_buses[bus_id];
    uint32_t isr = *bus->isr_reg;
    *bus->ifcr_reg = bus->flag_clear;

    uint8_t err = 0;
    if (isr & bus->teif) {
        bus->sticky_fault |= PHASE_FAULT_TEIF;
        err = 1;
    }
    if (isr & bus->feif) {
        bus->sticky_fault |= PHASE_FAULT_FEIF;
        err = 1;
    }

    bus->stream->CR &= ~DMA_SxCR_EN;
    bus->rx_stream->CR &= ~DMA_SxCR_EN;
    *bus->rx_ifcr_reg = bus->rx_flag_clear;

    // TC = TX FIFO fed, not last byte shifted out; gate CS-high on EOT.
    SPI_TypeDef *spi = bus->fast_cfg.spi;
    if (!err) {
        uint32_t deadline = timer_read_time() + timer_from_us(50);
        while (!(spi->SR & SPI_SR_EOT)) {
            if (!timer_is_before(timer_read_time(), deadline)) {
                bus->sticky_fault |= PHASE_FAULT_EOT;
                err = 1;
                break;
            }
        }
    }

    spi->IFCR = 0xFFFFFFFF;
    spi->CR1 = SPI_CR1_SSI;
    gpio_out_write(phase_motors[bus->seq[bus->cursor]].cs, 1);

    if (err) {
        // Stop the walk: do not clock more motors onto faulted HW. The sticky
        // bit surfaces a loud fault at the next commit tick.
        bus->busy = 0;
        bus->owner = PHASE_OWNER_NONE;
        return;
    }

    bus->cursor++;
    if (bus->cursor < bus->seq_len) {
        phase_dma_arm_motor(bus);
    } else {
        bus->busy = 0;
        bus->owner = PHASE_OWNER_NONE;
        phase_write_count++;
    }
}

static int
phase_bus_for_spi(SPI_TypeDef *spi)
{
    for (int b = 0; b < MAX_PHASE_BUSES; b++)
        if (phase_buses[b].configured && phase_buses[b].cfg.spi == spi)
            return b;
    return -1;
}

int
phase_spi_fg_begin(struct spi_config config)
{
    int b = phase_bus_for_spi(config.spi);
    if (b < 0)
        return -1;
    struct phase_bus_state *bus = &phase_buses[b];
    uint32_t deadline = timer_read_time() + timer_from_us(500);
    for (;;) {
        while (bus->busy) {
            if (!timer_is_before(timer_read_time(), deadline))
                shutdown("phase bus stuck: foreground SPI could not claim the "
                         "bus (phase DMA batch never drained)");
        }
        irqstatus_t flag = irq_save();
        if (!bus->busy) {
            bus->busy = 1;
            bus->owner = PHASE_OWNER_FG;
            irq_restore(flag);
            return b;
        }
        irq_restore(flag);
    }
}

void
phase_spi_fg_end(int bus_token)
{
    if (bus_token < 0)
        return;
    phase_buses[bus_token].busy = 0;
    phase_buses[bus_token].owner = PHASE_OWNER_NONE;
}

// Foreground TMC register access on a phase-managed bus, driven by full-duplex
// DMA instead of PIO so no programmed-IO transfer ever shares the spi1 bus with
// the phase batch. Synchronous: claims the bus (phase commit defers while held),
// runs one transfer, and blocks on RX-DMA completion — RX-TC fires only once
// every frame is fully shifted in, so it doubles as the true end-of-transfer.
// Returns 0 on success, -1 when the SPI is not a phase bus or the length
// exceeds the bounce (caller then falls back to the PIO path).
int
phase_spi_fg_dma_transfer(struct spi_config config, uint8_t receive_data,
                          uint8_t len, uint8_t *data)
{
    int b = phase_bus_for_spi(config.spi);
    if (b < 0 || len > PHASE_FG_BOUNCE)
        return -1;
    struct phase_bus_state *bus = &phase_buses[b];
    SPI_TypeDef *spi = config.spi;

    uint32_t claim_deadline = timer_read_time() + timer_from_us(500);
    for (;;) {
        while (bus->busy) {
            if (!timer_is_before(timer_read_time(), claim_deadline))
                shutdown("phase bus stuck: foreground DMA could not claim the "
                         "bus (phase batch never drained)");
        }
        irqstatus_t flag = irq_save();
        if (!bus->busy) {
            bus->busy = 1;
            bus->owner = PHASE_OWNER_FG;
            irq_restore(flag);
            break;
        }
        irq_restore(flag);
    }

    for (uint8_t i = 0; i < len; i++)
        bus->fg_txbuf[i] = data[i];
    SCB_CleanDCache_by_Addr((void *)bus->fg_txbuf, (int32_t)PHASE_FG_BOUNCE);
    SCB_InvalidateDCache_by_Addr((void *)bus->fg_rxbuf, (int32_t)PHASE_FG_BOUNCE);

    spi->CR1 = SPI_CR1_SSI;
    spi->CFG1 = ((uint32_t)config.div << SPI_CFG1_MBR_Pos)
              | (7u << SPI_CFG1_DSIZE_Pos)
              | SPI_CFG1_TXDMAEN | SPI_CFG1_RXDMAEN;
    spi->CFG2 = ((uint32_t)config.mode << SPI_CFG2_CPHA_Pos)
              | SPI_CFG2_MASTER | SPI_CFG2_SSM | SPI_CFG2_AFCNTR | SPI_CFG2_SSOE;

    DMA_Stream_TypeDef *tx = bus->stream;
    DMA_Stream_TypeDef *rx = bus->rx_stream;
    tx->CR &= ~DMA_SxCR_EN;
    while (tx->CR & DMA_SxCR_EN)
        ;
    rx->CR &= ~DMA_SxCR_EN;
    while (rx->CR & DMA_SxCR_EN)
        ;
    tx->FCR = 0;
    rx->FCR = 0;
    *bus->ifcr_reg = bus->flag_clear;
    *bus->rx_ifcr_reg = bus->rx_flag_clear;

    // No TCIE/TEIE: the phase TX stream (0-3) has the phase TC ISR wired, so an
    // enabled interrupt here would fire it with owner==FG. Foreground polls.
    rx->PAR = (uint32_t)(uintptr_t)&spi->RXDR;
    rx->M0AR = (uint32_t)(uintptr_t)bus->fg_rxbuf;
    rx->NDTR = len;
    rx->CR = DMA_SxCR_MINC;
    tx->PAR = (uint32_t)(uintptr_t)&spi->TXDR;
    tx->M0AR = (uint32_t)(uintptr_t)bus->fg_txbuf;
    tx->NDTR = len;
    tx->CR = DMA_SxCR_DIR_0 | DMA_SxCR_MINC;
    rx->CR |= DMA_SxCR_EN;
    tx->CR |= DMA_SxCR_EN;

    spi->CR2 = (uint32_t)len << SPI_CR2_TSIZE_Pos;
    spi->CR1 = SPI_CR1_SSI | SPI_CR1_SPE;
    spi->IFCR = 0xFFFFFFFF;
    spi->CR1 = SPI_CR1_SSI | SPI_CR1_CSTART | SPI_CR1_SPE;

    uint32_t deadline = timer_read_time() + timer_from_us(100 * (uint32_t)len + 100);
    for (;;) {
        uint32_t rx_isr = *bus->rx_isr_reg;
        if (rx_isr & bus->rx_tcif)
            break;
        if ((rx_isr & (bus->rx_teif | bus->rx_feif))
            || (*bus->isr_reg & bus->teif)) {
            shutdown("phase foreground DMA: SPI bus transfer error");
        }
        if (!timer_is_before(timer_read_time(), deadline))
            shutdown("phase foreground DMA: SPI rx timeout");
    }

    tx->CR &= ~DMA_SxCR_EN;
    rx->CR &= ~DMA_SxCR_EN;
    *bus->ifcr_reg = bus->flag_clear;
    *bus->rx_ifcr_reg = bus->rx_flag_clear;
    spi->IFCR = 0xFFFFFFFF;
    spi->CR1 = SPI_CR1_SSI;

    SCB_InvalidateDCache_by_Addr((void *)bus->fg_rxbuf, PHASE_FG_BOUNCE);
    if (receive_data)
        for (uint8_t i = 0; i < len; i++)
            data[i] = bus->fg_rxbuf[i];

    bus->busy = 0;
    bus->owner = PHASE_OWNER_NONE;
    return 0;
}

static uint32_t
phase_overrun_diag(struct phase_bus_state *bus, uint32_t isr)
{
    SPI_TypeDef *spi = bus->fast_cfg.spi;
    uint32_t sr = spi->SR;
    DMA_Stream_TypeDef *st = bus->stream;
    return ((uint32_t)(bus->cursor & 0xF) << PHASE_DIAG_CURSOR_SHIFT)
         | ((st->NDTR & 0xF) << PHASE_DIAG_NDTR_SHIFT)
         | ((isr & bus->tcif) ? PHASE_DIAG_TCIF_BIT : 0u)
         | (phase_tc_count ? PHASE_DIAG_TCRAN_BIT : 0u)
         | ((spi->CR1 & SPI_CR1_SPE) ? PHASE_DIAG_SPE_BIT : 0u)
         | ((sr & SPI_SR_TXP) ? PHASE_DIAG_TXP_BIT : 0u)
         | ((st->CR & DMA_SxCR_EN) ? PHASE_DIAG_DMAEN_BIT : 0u)
         | ((sr & SPI_SR_SUSP) ? PHASE_DIAG_SUSP_BIT : 0u)
         | ((sr & SPI_SR_EOT) ? PHASE_DIAG_EOT_BIT : 0u);
}

#endif // CONFIG_MACH_STM32H7

// used,externally_visible: called only from Rust via FFI; without this,
// -fwhole-program LTO DCEs the bodies and the link fails.
__attribute__((used, externally_visible))
void
phase_stepping_register_bus(uint8_t bus_id, struct spi_config cfg)
{
    if (bus_id >= MAX_PHASE_BUSES)
        return;
    phase_buses[bus_id].cfg = cfg;
    struct spi_config fast = cfg;
    uint32_t pclk = get_pclock_frequency((uint32_t)(uintptr_t)cfg.spi);
    uint32_t target_rate = 8000000;
    uint32_t div = 0;
    while ((pclk >> (div + 1)) > target_rate && div < 7)
        div++;
    fast.div = div;
    phase_buses[bus_id].fast_cfg = fast;
    phase_buses[bus_id].seq_len = 0;
    phase_buses[bus_id].configured = 1;
#if CONFIG_MACH_STM32H7
    phase_buses[bus_id].active_half = 0;
    phase_buses[bus_id].busy = 0;
    phase_buses[bus_id].owner = PHASE_OWNER_NONE;
    phase_buses[bus_id].sticky_fault = 0;
    phase_buses[bus_id].fg_defer_count = 0;
    phase_dma_init_bus(bus_id);
#endif
}

__attribute__((used, externally_visible))
void
phase_stepping_register_motor(uint8_t motor_idx, uint8_t bus_id,
                              uint8_t cs_pin_id)
{
    if (motor_idx >= MAX_PHASE_MOTORS || bus_id >= MAX_PHASE_BUSES)
        return;
    if (!phase_buses[bus_id].configured)
        return;
    phase_motors[motor_idx].cs = gpio_out_setup(cs_pin_id, 1); // idle high
    phase_motors[motor_idx].bus_id = bus_id;
    phase_motors[motor_idx].configured = 1;

    struct phase_bus_state *bus = &phase_buses[bus_id];
    for (uint8_t i = 0; i < bus->seq_len; i++)
        if (bus->seq[i] == motor_idx)
            return;
    if (bus->seq_len < MAX_PHASE_MOTORS)
        bus->seq[bus->seq_len++] = motor_idx;
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

#if CONFIG_MACH_STM32H7
    struct phase_bus_state *bus = &phase_buses[bus_id];
    phase_pack_datagram(&bus->txbuf[bus->active_half][motor_idx * XDIRECT_LEN],
                        coil_a, coil_b);
#else
    uint8_t datagram[XDIRECT_LEN];
    phase_pack_datagram(datagram, coil_a, coil_b);
    spi_prepare(phase_buses[bus_id].fast_cfg);
    gpio_out_write(phase_motors[motor_idx].cs, 0);
    spi_transfer(phase_buses[bus_id].fast_cfg, 0, sizeof(datagram), datagram);
    gpio_out_write(phase_motors[motor_idx].cs, 1);
    phase_write_count++;
#endif
}

__attribute__((used, externally_visible))
uint32_t
phase_stepping_commit_tick(void)
{
#if CONFIG_MACH_STM32H7
    uint32_t result = 0;
    for (uint8_t b = 0; b < MAX_PHASE_BUSES; b++) {
        struct phase_bus_state *bus = &phase_buses[b];
        if (!bus->configured || bus->seq_len == 0)
            continue;

        irqstatus_t flag = irq_save();

        if (bus->busy) {
            if (bus->owner == PHASE_OWNER_PHASE) {
                // Equal-priority TC ISR can be queued behind this tick, so a
                // physically-complete batch still reads busy==1. Final-motor +
                // DMA TCIF latched + SPI EOT set => pending-but-done, not an
                // overrun: finalize inline and re-arm.
                SPI_TypeDef *spi = bus->fast_cfg.spi;
                uint32_t isr = *bus->isr_reg;
                uint8_t on_last = (uint8_t)(bus->cursor + 1) >= bus->seq_len;
                uint8_t hw_done = (isr & bus->tcif)
                                  && !(isr & (bus->teif | bus->feif))
                                  && (spi->SR & SPI_SR_EOT);
                if (on_last && hw_done) {
                    phase_dma_release_current(bus);
                    phase_write_count++;
                    NVIC_ClearPendingIRQ(bus->irqn);
                } else {
                    if (result == 0)
                        result = ((uint32_t)b << 8) | PHASE_DMA_KIND_OVERRUN
                            | phase_overrun_diag(bus, isr);
                    irq_restore(flag);
                    continue;
                }
            } else {
                phase_defer_count++;
                if (++bus->fg_defer_count >= 2 && result == 0)
                    result = ((uint32_t)b << 8) | PHASE_DMA_KIND_OVERRUN
                        | PHASE_DIAG_FGSTUCK_BIT
                        | phase_overrun_diag(bus, *bus->isr_reg);
                irq_restore(flag);
                continue;
            }
        }
        bus->fg_defer_count = 0;

        uint8_t sticky = bus->sticky_fault;
        if (sticky) {
            if (result == 0) {
                if (sticky & (PHASE_FAULT_TEIF | PHASE_FAULT_EOT))
                    result = ((uint32_t)b << 8) | PHASE_DMA_KIND_TEIF;
                else if (sticky & PHASE_FAULT_FEIF)
                    result = ((uint32_t)b << 8) | PHASE_DMA_KIND_FEIF;
                bus->sticky_fault = 0;
            }
            irq_restore(flag);
            continue;
        }

        if (!phase_spi_writes_enabled) {
            irq_restore(flag);
            continue;
        }

        bus->busy = 1;
        bus->owner = PHASE_OWNER_PHASE;

        uint8_t half = bus->active_half;
        bus->active_half ^= 1;
        bus->commit_half = half;
        bus->cursor = 0;
        SCB_CleanDCache_by_Addr((void *)bus->txbuf[half],
                                (int32_t)sizeof(bus->txbuf[half]));
        phase_dma_arm_motor(bus);
        irq_restore(flag);
    }
    return result;
#else
    return 0;
#endif
}
