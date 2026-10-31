#include <stdint.h>
#include <stdio.h>
#include "autoconf.h"
#include "board/gpio.h"
#include "board/irq.h"
#include "command.h"
#include "sched.h"
#include "board/misc.h"
#include "runtime.h"
#include "mcu_transport_dispatch.h"
#if CONFIG_MACH_STM32
#include "stm32/phase_stepping_spi.h"
#elif CONFIG_MACH_LINUX
#include "linux/phase_stepping_spi.h"
#endif


extern void *runtime_handle;

void
command_runtime_seed_position(uint32_t *args)
{
    int32_t x_q16 = (int32_t)args[0];
    int32_t y_q16 = (int32_t)args[1];
    int32_t z_q16 = (int32_t)args[2];
    if (!runtime_handle)
        return;
    (void)runtime_seed_position(runtime_handle, x_q16, y_q16, z_q16);
}
DECL_COMMAND(command_runtime_seed_position,
    "runtime_seed_position x_q16=%i y_q16=%i z_q16=%i");

enum { TMC_SPI_MODE = 3 };

void
command_runtime_register_phase_bus(uint32_t *args)
{
#if CONFIG_MACH_STM32 || CONFIG_MACH_LINUX
    uint8_t bus_id = (uint8_t)args[0];
    uint32_t rate = args[1];
    struct spi_config cfg = spi_setup(bus_id, TMC_SPI_MODE, rate);
    phase_stepping_register_bus(bus_id, cfg);
    sendf("kalico_register_phase_bus_response result=%i", 0);
#else
    (void)args;
    sendf("kalico_register_phase_bus_response result=%i", -88);
#endif
}
DECL_COMMAND(command_runtime_register_phase_bus,
    "runtime_register_phase_bus bus_id=%c rate=%u");

// Wire param must stay cs_pin_id, not cs_pin: msgproto resolves any `*_pin`
// param against the pin enumeration, breaking the raw port*16+pin GPIO encoding.
void
command_runtime_register_phase_motor(uint32_t *args)
{
#if CONFIG_MACH_STM32 || CONFIG_MACH_LINUX
    uint8_t motor_idx = (uint8_t)args[0];
    uint8_t bus_id    = (uint8_t)args[1];
    uint8_t cs_pin_id = (uint8_t)args[2];
    uint8_t slot_idx  = (uint8_t)args[3];
    if (!runtime_handle)
        shutdown("register_phase_motor before runtime init");
    phase_stepping_register_motor(motor_idx, bus_id, cs_pin_id);
    int32_t rc = runtime_bind_phase_motor(runtime_handle,
                                                 motor_idx, slot_idx);
    if (rc != 0)
        shutdown("register_phase_motor bind rejected by runtime");
    sendf("kalico_register_phase_motor_response result=%i", 0);
#else
    (void)args;
    sendf("kalico_register_phase_motor_response result=%i", -88);
#endif
}
DECL_COMMAND(command_runtime_register_phase_motor,
    "runtime_register_phase_motor motor_idx=%c bus_id=%c cs_pin_id=%c"
    " slot_idx=%c");

#if CONFIG_MCU_SIM
void
command_runtime_sim_axis_window(uint32_t *args)
{
    uint32_t axis = args[0];
    uint64_t start = 0, end = 0;
    uint32_t occupancy = 0;
    int32_t armed = -7;
    if (runtime_handle) {
        irqstatus_t flag = irq_save();
        armed = runtime_axis_head_window(runtime_handle, axis,
                                         &start, &end, &occupancy);
        irq_restore(flag);
    }
    sendf("runtime_sim_axis_window_response armed=%i occupancy=%u"
          " start_lo=%u start_hi=%u end_lo=%u end_hi=%u",
          armed, occupancy,
          (uint32_t)start, (uint32_t)(start >> 32),
          (uint32_t)end, (uint32_t)(end >> 32));
}
DECL_COMMAND(command_runtime_sim_axis_window, "runtime_sim_axis_window axis=%u");
#endif

