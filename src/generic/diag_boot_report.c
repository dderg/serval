#include <stdint.h>
#include <string.h>
#include "autoconf.h"
#include "board/internal.h"
#include "sched.h"
#include "fault_handler_internal.h"

extern volatile uint8_t runtime_liveness_ok;
#if CONFIG_MOTION_RUNTIME
extern void *runtime_handle;
extern uint32_t runtime_handle_tick_counter(void *handle);
extern uint8_t  runtime_handle_status(void *handle);
#endif

static uint32_t preboot_cur_task_func;
static uint32_t preboot_cur_msg_kind;

// A task/msg marker left open across a reset would be closed by the first
// task hook against the fresh clock epoch — a wrapped duration that clamps to
// DIAG_STALL_CAP_CYC and poisons the boot replay's worst slots. Save the
// markers for the replay and clear them before any task hook runs.
static void
discard_preboot_progress_markers(void)
{
    if (live_snap.magic != LIVE_MAGIC)
        return;
    preboot_cur_task_func = live_snap.cur_task_func;
    preboot_cur_msg_kind = live_snap.cur_msg_kind;
    live_snap.cur_task_func = 0;
    live_snap.cur_msg_kind = 0;
    diag_cache_clean();
}

#if CONFIG_MACH_STM32H7
__attribute__((section(".bkp_bss"), used))
#else
__attribute__((section(".persistent_diag"), used))
#endif
volatile struct live_snapshot live_snap;

void
fault_handler_init(void)
{
#if (__CORTEX_M >= 3)
    SCB->SHCSR |= SCB_SHCSR_USGFAULTENA_Msk
                | SCB_SHCSR_BUSFAULTENA_Msk
                | SCB_SHCSR_MEMFAULTENA_Msk;
    SCB->CCR |= SCB_CCR_DIV_0_TRP_Msk;
    // Do not enable UNALIGN_TRP: unaligned half-word/word loads are common here.
#endif
#if CONFIG_MACH_STM32H7
    RCC->AHB4ENR |= RCC_AHB4ENR_BKPRAMEN;
    PWR->CR1 |= PWR_CR1_DBP;
    PWR->CR2 |= PWR_CR2_BREN;
    {
        volatile int spin = 0;
        while (!(PWR->CR2 & PWR_CR2_BRRDY) && spin < 100000) spin++;
    }
#endif
    discard_preboot_progress_markers();
}
DECL_INIT(fault_handler_init);

#include "board/misc.h"

uint32_t boot_first_tick;
uint32_t reset_cause_snapshot;

#if CONFIG_MACH_STM32H7
#define PRIOR_SECTION ".bkp_bss"
#else
#define PRIOR_SECTION ".persistent_diag"
#endif
// The held run's live_snap, taken at boot before the per-run fields are
// zeroed; all "prior run" reporting reads this, never live_snap.
__attribute__((section(PRIOR_SECTION), used))
struct live_snapshot prior_snap;
__attribute__((section(PRIOR_SECTION), used))
struct diag_counters prior_diag;
__attribute__((section(PRIOR_SECTION), used))
struct diag_event    prior_ring[DIAG_RING_LEN];
__attribute__((section(PRIOR_SECTION), used))
volatile struct prior_report_state prior_state;
uint32_t             prior_diag_present;

#if CONFIG_MACH_STM32H7
#include "board/internal.h"
#endif

static uint32_t
read_reset_cause(void)
{
#if CONFIG_MACH_STM32H7
    return RCC->RSR;
#elif CONFIG_MACH_STM32F4
    return RCC->CSR;
#else
    return 0;
#endif
}

static void
clear_reset_cause(void)
{
#if CONFIG_MACH_STM32H7
    RCC->RSR |= RCC_RSR_RMVF;
#elif CONFIG_MACH_STM32F4
    RCC->CSR |= RCC_CSR_RMVF;
#endif
}

static void
fault_handler_report_boot_init(uint32_t now)
{
    boot_first_tick = now;
    boot_tick_initialized = 1;
    reset_cause_snapshot = read_reset_cause();
    uint32_t reset_cause_raw = reset_cause_snapshot;
    clear_reset_cause();
    uint32_t ended_run_present = live_snap.magic == LIVE_MAGIC;
    uint32_t ended_diag_present = diag.magic == DIAG_MAGIC;
    uint32_t ended_boot_count = ended_diag_present ? diag.boot_count : 0;
    uint32_t holding_unreported = prior_state.magic == PRIOR_MAGIC
                                  && !prior_state.reported;
    if (holding_unreported) {
        prior_state.runs_skipped++;
    } else {
        if (ended_run_present) {
            memcpy(&prior_snap, (const void *)&live_snap, sizeof(prior_snap));
            prior_snap.cur_task_func = preboot_cur_task_func;
            prior_snap.cur_msg_kind = preboot_cur_msg_kind;
        } else {
            memset(&prior_snap, 0, sizeof(prior_snap));
        }
        if (ended_diag_present) {
            memcpy(&prior_diag, (const void *)&diag, sizeof(prior_diag));
            memcpy(prior_ring, (const void *)diag_ring, sizeof(prior_ring));
        } else {
            memset(&prior_diag, 0, sizeof(prior_diag));
            memset(prior_ring, 0, sizeof(prior_ring));
        }
        prior_state.magic = PRIOR_MAGIC;
        prior_state.reported = 0;
        prior_state.reset_cause = reset_cause_raw;
        prior_state.runs_skipped = 0;
    }
    reset_cause_snapshot = prior_state.reset_cause;
    prior_diag_present = prior_diag.magic == DIAG_MAGIC;
    uint32_t iwdg_resets = ended_run_present ? live_snap.iwdg_reset_count : 0;
#if CONFIG_MACH_STM32H7
    if (reset_cause_raw & RCC_RSR_IWDG1RSTF)
        iwdg_resets++;
#elif CONFIG_MACH_STM32F4
    if (reset_cause_raw & RCC_CSR_IWDGRSTF)
        iwdg_resets++;
#endif
    memset((void *)&live_snap, 0, sizeof(live_snap));
    live_snap.rearm_min_margin = (uint32_t)INT32_MAX;
    live_snap.iwdg_reset_count = iwdg_resets;

    memset((void *)&diag, 0, sizeof(diag));
    diag.magic = DIAG_MAGIC;
    diag.boot_count = ended_boot_count + 1;
    for (uint32_t i = 0; i < DIAG_RING_LEN; i++) {
        diag_ring[i].tag = DIAG_EV_NONE;
        diag_ring[i].seq = 0;
        diag_ring[i].timestamp = 0;
        diag_ring[i].a = 0;
        diag_ring[i].b = 0;
    }
    diag_cache_clean();
}

static void
fault_handler_report_liveness_update(uint32_t now)
{
    uint32_t live_now = runtime_liveness_ok;
    uint8_t engine_now = 0xFF;
    uint32_t tick_now = 0;
#if CONFIG_MOTION_RUNTIME
    if (runtime_handle) {
        tick_now = runtime_handle_tick_counter(runtime_handle);
        engine_now = runtime_handle_status(runtime_handle);
    }
#endif
    if (live_snap.magic != LIVE_MAGIC)
        live_snap.boot_count = 0;
    live_snap.live = live_now;
    live_snap.engine_status = (uint32_t)engine_now;
    live_snap.tick_counter = tick_now;
    live_snap.sample_time = now;
    live_snap.samples_taken++;
    if (engine_now == 1)
        live_snap.last_engine_running_tick = tick_now;
    live_snap.magic = LIVE_MAGIC;
}

void
fault_handler_report_task(void)
{
    uint32_t now = timer_read_time();
    if (!boot_tick_initialized) {
        fault_handler_report_boot_init(now);
        return;
    }
    fault_handler_report_liveness_update(now);
}
DECL_TASK(fault_handler_report_task);
