# Additional linux build rules

dirs-y += src/linux src/generic

src-y += linux/main.c linux/timer.c linux/console.c linux/watchdog.c
src-y += linux/pca9685.c linux/spidev.c linux/analog.c linux/hard_pwm.c
src-$(CONFIG_MCU_SIM_TMCUART_BYPASS) += linux/sim_chip_socket.c
src-y += linux/i2c.c linux/gpio.c generic/crc16_ccitt.c generic/alloc.c
src-y += linux/sensor_ds18b20.c

ifeq ($(CONFIG_MOTION_RUNTIME),y)
src-y += linux/runtime_tick_host.c
src-y += runtime_commands.c
src-y += linux/phase_stepping_spi.c
endif

ifeq ($(CONFIG_MCU_SIM),y)
src-y += linux/sim_vtime_pacer.c
endif
src-y += linux/fault_handler_stub.c
src-y += runtime_tick.c

# Generated FFI header (cbindgen) shared with the H7 build.
CFLAGS += -Irust/c-api/include

ifeq ($(CONFIG_MOTION_RUNTIME),y)
# Link libc_api.a. The Linux build has a mutual-callback edge:
# host_tick.o calls into libkalico (runtime_tick) and libkalico
# calls back out (runtime_tick_enable / read_cyccnt). Wrap with
# --start-group/--end-group so ld iterates until both sides resolve.
# MCU_RUST_LIB is set by src/Makefile's MACH_LINUX block (its own target-linux
# dir, mirroring the per-arch STM32 paths) — included before this file.
CFLAGS_klipper.elf := -Wl,--start-group $(MCU_RUST_LIB) -Wl,--end-group $(CFLAGS_klipper.elf)
$(OUT)klipper.elf: $(MCU_RUST_LIB)
endif

CFLAGS_klipper.elf += -lutil -lrt -lpthread

flash: $(OUT)klipper.elf
	@echo "  Flashing"
	$(Q)sudo ./scripts/flash-linux.sh $(OUT)
