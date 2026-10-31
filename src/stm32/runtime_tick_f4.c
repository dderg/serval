#include "autoconf.h"
#include "generic/armcm_boot.h"
#include "internal.h"
#include "runtime.h"
#include "generic/runtime_tick.h"
#include "generic/motion_nvic_prio.h"

#if CONFIG_MACH_STM32F4

extern const uint32_t runtime_clock_freq;

extern void* runtime_handle;

// Kernel clock accounts for the APB timer-doubler; use this, not
// runtime_clock_freq, for ARR and delta scaling.
static uint32_t
motion_timer_clk(void)
{
    uint32_t pclk = get_pclock_frequency((uint32_t)TIM5);
    uint32_t clkdiv = CONFIG_CLOCK_FREQ / pclk;
    if (clkdiv > 1)
        clkdiv /= 2;
    return CONFIG_CLOCK_FREQ / clkdiv;
}

// used,externally_visible on the Rust-only entry points: survives
// -fwhole-program LTO.
__attribute__((used, externally_visible))
void
runtime_tick_disable(void)
{
    TIM5->CR1 &= ~TIM_CR1_CEN;
    NVIC_DisableIRQ(TIM5_IRQn);
}

// NEVER flash CONFIG_MCU_SIM=y to silicon (IWDG-disabled debug build).
__attribute__((used, externally_visible))
uint32_t
runtime_cyccnt_read(void)
{
#if CONFIG_MCU_SIM
    extern volatile uint32_t runtime_sim_cyccnt;
    return runtime_sim_cyccnt;
#else
    return DWT->CYCCNT;
#endif
}

__attribute__((used, externally_visible))
void
runtime_tick_enable(void)
{
    if (TIM5->CR1 & TIM_CR1_CEN) {
        return;
    }
    TIM5->CR1 &= ~TIM_CR1_CEN;
    TIM5->ARR  = (motion_timer_clk() / CONFIG_MOTION_SAMPLE_RATE_HZ) - 1U;
    TIM5->EGR  = TIM_EGR_UG;
    TIM5->SR   = 0;
    TIM5->SR   = ~TIM_SR_UIF;
    TIM5->CR1 |= TIM_CR1_CEN;
    NVIC_EnableIRQ(TIM5_IRQn);
}

void
runtime_tick_init(void)
{
    NVIC_DisableIRQ(TIM5_IRQn);

    // Clock-on before any TIM5 register access.
    RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
    __DSB();

    TIM5->CR1 &= ~TIM_CR1_CEN;
    TIM5->SR = 0;

    TIM5->PSC = 0;
    TIM5->ARR = (motion_timer_clk() / CONFIG_MOTION_SAMPLE_RATE_HZ) - 1U;

    TIM5->CR1 = TIM_CR1_ARPE;
    TIM5->DIER = TIM_DIER_UIE;

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    NVIC_SetPriority(TIM5_IRQn, MOTION_NVIC_PRIO);

    TIM5->EGR  = TIM_EGR_UG;
    TIM5->SR   = ~TIM_SR_UIF;
    TIM5->CR1 |= TIM_CR1_CEN;
    NVIC_EnableIRQ(TIM5_IRQn);
}

// -311 stacked-PC capture (mirror of H7). NOT static: the naked asm is the sole
// writer, and a file-local static would be GC-stripped.
volatile uint32_t *tim5_exc_frame __attribute__((used, externally_visible));

__attribute__((used))
static void TIM5_IRQHandler_body(uint32_t *frame);

__attribute__((naked))
void
TIM5_IRQHandler(void)
{
    __asm volatile (
        "tst    lr, #4              \n"  // EXC_RETURN bit 2: 0=MSP, 1=PSP
        "ite    eq                  \n"
        "mrseq  r0, msp             \n"
        "mrsne  r0, psp             \n"
        "ldr    r1, =tim5_exc_frame \n"
        "str    r0, [r1]            \n"
        "b      TIM5_IRQHandler_body\n"  // tail-call; LR (EXC_RETURN) preserved
    );
}

__attribute__((used, externally_visible))
uint32_t
runtime_tim5_stacked_pc(void)
{
    if (!tim5_exc_frame)
        return 0;
    return tim5_exc_frame[6];
}

__attribute__((used, externally_visible))
uint32_t
runtime_tim5_stacked_exc(void)
{
    if (!tim5_exc_frame)
        return 0;
    return tim5_exc_frame[7] & 0x1FFu;
}

static void
TIM5_IRQHandler_body(uint32_t *frame)
{
    (void)frame;
    extern void diag_tim5_account(uint32_t enter, uint32_t exit);
    uint32_t diag_enter = DWT->CYCCNT;

    TIM5->SR = ~TIM_SR_UIF;            // entry-time ack

#if CONFIG_MCU_SIM
    extern volatile uint32_t runtime_sim_cyccnt;
    runtime_sim_cyccnt += (runtime_clock_freq / 40000U);
#endif

    uint32_t before = runtime_cyccnt_read();
    if (runtime_handle) {
        runtime_tick_sample(runtime_handle);
    }
    uint32_t after = runtime_cyccnt_read();

    extern void diag_runtime_tick_account(uint32_t cycles);
    diag_runtime_tick_account(after - before);

    diag_tim5_account(diag_enter, DWT->CYCCNT);
}

DECL_ARMCM_IRQ(TIM5_IRQHandler, TIM5_IRQn);

#endif
