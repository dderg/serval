#include <stdint.h>
#include "autoconf.h"
#include "board/internal.h"
#include "fault_handler_internal.h"

#if CONFIG_MACH_STM32H7
__attribute__((section(".bkp_bss"), used))
#else
__attribute__((section(".persistent_diag"), used))
#endif
volatile struct fault_record fault_rec;

void __attribute__((noreturn, used))
fault_capture_and_reset(uint32_t kind, uint32_t *frame, uint32_t exc_return)
{
    fault_rec.r0  = frame[0];
    fault_rec.r1  = frame[1];
    fault_rec.r2  = frame[2];
    fault_rec.r3  = frame[3];
    fault_rec.r12 = frame[4];
    fault_rec.lr  = frame[5];
    fault_rec.pc  = frame[6];
    fault_rec.psr = frame[7];
    fault_rec.exc_return = exc_return;

#if (__CORTEX_M >= 3)
    fault_rec.cfsr  = SCB->CFSR;
    fault_rec.hfsr  = SCB->HFSR;
    fault_rec.dfsr  = SCB->DFSR;
    fault_rec.bfar  = SCB->BFAR;
    fault_rec.mmfar = SCB->MMFAR;
    fault_rec.afsr  = SCB->AFSR;
#else
    fault_rec.cfsr  = 0;
    fault_rec.hfsr  = 0;
    fault_rec.dfsr  = 0;
    fault_rec.bfar  = 0;
    fault_rec.mmfar = 0;
    fault_rec.afsr  = 0;
#endif
    fault_rec.shcsr = SCB->SHCSR;

    fault_rec.exc_kind = kind;
    if (fault_rec.magic != FAULT_MAGIC)
        fault_rec.fault_count = 0;
    fault_rec.fault_count++;
    fault_rec.magic = FAULT_MAGIC;

    __DSB();
    NVIC_SystemReset();

    for (;;);
}

#include "armcm_boot.h"

// ARMv6-M has no IT blocks / conditional MRS and a narrow B can't reach across
// .text, so the M0+ trampoline uses branch-over + BL instead of ite/mrseq.
#if (__CORTEX_M >= 3)
#define FAULT_TRAMPOLINE_SELECT_SP                                      \
            "tst lr, #4\n\t"                                            \
            "ite eq\n\t"                                                \
            "mrseq r1, msp\n\t"                                         \
            "mrsne r1, psp\n\t"
#define FAULT_TRAMPOLINE_TAIL "b fault_capture_and_reset\n\t"
#else
#define FAULT_TRAMPOLINE_SELECT_SP                                      \
            "movs r1, #4\n\t"                                          \
            "mov  r2, lr\n\t"                                          \
            "tst  r2, r1\n\t"                                          \
            "beq  1f\n\t"                                              \
            "mrs  r1, psp\n\t"                                         \
            "b    2f\n\t"                                              \
            "1:\n\t"                                                  \
            "mrs  r1, msp\n\t"                                        \
            "2:\n\t"
#define FAULT_TRAMPOLINE_TAIL "bl fault_capture_and_reset\n\t"
#endif

#define FAULT_TRAMPOLINE(NAME, KIND, IRQ_NUM)                           \
    void __attribute__((naked, used)) NAME(void)                        \
    {                                                                   \
        asm volatile (                                                  \
            FAULT_TRAMPOLINE_SELECT_SP                                  \
            "mov r0, %0\n\t"                                            \
            "mov r2, lr\n\t"                                            \
            FAULT_TRAMPOLINE_TAIL                                       \
            : : "i"(KIND) : "r0", "r1", "r2"                            \
        );                                                              \
    }                                                                   \
    DECL_ARMCM_IRQ(NAME, IRQ_NUM)

FAULT_TRAMPOLINE(HardFault_Handler, 1, -13);
#if (__CORTEX_M >= 3)
FAULT_TRAMPOLINE(BusFault_Handler, 2, -11);
FAULT_TRAMPOLINE(UsageFault_Handler, 3, -10);
FAULT_TRAMPOLINE(MemManage_Handler, 4, -12);
#endif
