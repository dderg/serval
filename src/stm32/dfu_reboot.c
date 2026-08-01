// Reboot into stm32 ROM dfu bootloader
//
// Copyright (C) 2019-2022  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "internal.h" // NVIC_SystemReset
#include "board/irq.h" // irq_disable

// Many stm32 chips have a USB capable "DFU bootloader" in their ROM.
// In order to invoke that bootloader it is necessary to reset the
// chip and jump to a chip specific hardware address.
//
// To reset the chip, the dfu_reboot() code sets a flag in memory (at
// an arbitrary position that is unlikely to be overwritten during a
// chip reset), and resets the chip.  If dfu_reboot_check() sees that
// flag on the next boot it will perform a code jump to the ROM
// address.

// On H7 the flag must NOT be a hardcoded AXI SRAM address: .axi_bss packs
// large live statics (rt_storage spans ~72-120 KB) from 0x24000000 up, so any
// fixed 0x2400xxxx address lands inside one of them — runtime writes could
// forge the magic and divert a routine restart into ROM DFU. A linker-reserved
// .axi_bss static has the needed semantics (NOLOAD, never memset at boot,
// survives NVIC_SystemReset); aligned(32) gives it its own cache line for the
// clean in dfu_reboot().
//
// `volatile` is load-bearing, not decoration: dfu_reboot_check() inlines into
// the reset handler, and a plain static is provably still zero-initialised
// that early, so LTO folds the magic comparison to false and deletes the ROM
// jump outright. The value survives across a reset the compiler cannot see.
#if CONFIG_MACH_STM32H7
static volatile uint64_t usb_boot_flag
    __attribute__((section(".axi_bss"), aligned(32), used));
#define USB_BOOT_FLAG_ADDR ((uint32_t)&usb_boot_flag)
#else
#define USB_BOOT_FLAG_ADDR (CONFIG_RAM_START + CONFIG_RAM_SIZE - 1024)
#endif

// Signature to set in memory to flag that a dfu reboot is requested
#define USB_BOOT_FLAG 0x55534220424f4f54 // "USB BOOT"

// Flag that bootloader is desired and reboot
void
dfu_reboot(void)
{
    if (!CONFIG_STM32_DFU_ROM_ADDRESS || !CONFIG_HAVE_BOOTLOADER_REQUEST)
        return;
    irq_disable();
    volatile uint64_t *bflag = (void*)USB_BOOT_FLAG_ADDR;
    *bflag = USB_BOOT_FLAG;
#if __CORTEX_M >= 7
    SCB_CleanDCache_by_Addr((void*)bflag, sizeof(*bflag));
#endif
    NVIC_SystemReset();
}

// Check if rebooting into system DFU Bootloader
void
dfu_reboot_check(void)
{
    if (!CONFIG_STM32_DFU_ROM_ADDRESS || !CONFIG_HAVE_BOOTLOADER_REQUEST)
        return;
    volatile uint64_t *bflag = (void*)USB_BOOT_FLAG_ADDR;
    if (*bflag != USB_BOOT_FLAG)
        return;
    *bflag = 0;
    uint32_t *sysbase = (uint32_t*)CONFIG_STM32_DFU_ROM_ADDRESS;
    asm volatile("mov sp, %0\n bx %1"
                 : : "r"(sysbase[0]), "r"(sysbase[1]));
}
