// LD_PRELOAD stand-in for one GPIO character device of the Klipper
// linux-process MCU, so a live test can home a stepper without hardware.
//
// /dev/gpiochip<KV2_GPIO_SIM_CHIP> exists only in memory. Its line
// KV2_GPIO_SIM_STEP counts a step on every rising edge (the stepper is
// configured with step pulses), KV2_GPIO_SIM_DIR gives the direction (high
// is positive), and input line KV2_GPIO_SIM_ENDSTOP reads 1 once the
// stepper is KV2_GPIO_SIM_TRIP_STEPS (signed) or further from where it
// started.
// Every other path and descriptor goes to libc.
#define _GNU_SOURCE
#include <dlfcn.h>
#include <fcntl.h>
#include <linux/gpio.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

enum role { NONE, CHIP, STEP, DIR, ENDSTOP, OTHER_LINE };

#define MAX_FDS 4096

static enum role roles[MAX_FDS];
static int sim_chip = -1, step_line, dir_line, endstop_line;
static long trip_steps;
static long position;
static int step_level, dir_level;

static int (*real_open)(const char *, int, ...);
static int (*real_access)(const char *, int);
static int (*real_ioctl)(int, unsigned long, ...);
static int (*real_close)(int);

static int
env_int(const char *name)
{
    const char *v = getenv(name);
    if (!v) {
        fprintf(stderr, "gpio_sim: %s is not set\n", name);
        abort();
    }
    return atoi(v);
}

__attribute__((constructor)) static void
gpio_sim_init(void)
{
    real_open = dlsym(RTLD_NEXT, "open");
    real_access = dlsym(RTLD_NEXT, "access");
    real_ioctl = dlsym(RTLD_NEXT, "ioctl");
    real_close = dlsym(RTLD_NEXT, "close");
    sim_chip = env_int("KV2_GPIO_SIM_CHIP");
    step_line = env_int("KV2_GPIO_SIM_STEP");
    dir_line = env_int("KV2_GPIO_SIM_DIR");
    endstop_line = env_int("KV2_GPIO_SIM_ENDSTOP");
    trip_steps = env_int("KV2_GPIO_SIM_TRIP_STEPS");
}

static int
is_sim_chip(const char *path)
{
    char want[64];
    snprintf(want, sizeof(want), "/dev/gpiochip%d", sim_chip);
    return strcmp(path, want) == 0;
}

static int
placeholder_fd(enum role role)
{
    int fd = real_open("/dev/null", O_RDWR | O_CLOEXEC);
    if (fd < 0 || fd >= MAX_FDS)
        abort();
    roles[fd] = role;
    return fd;
}

int
access(const char *path, int mode)
{
    if (is_sim_chip(path))
        return 0;
    return real_access(path, mode);
}

int
open(const char *path, int flags, ...)
{
    if (is_sim_chip(path))
        return placeholder_fd(CHIP);
    int mode = 0;
    if (flags & (O_CREAT | O_TMPFILE)) {
        va_list ap;
        va_start(ap, flags);
        mode = va_arg(ap, int);
        va_end(ap);
    }
    return real_open(path, flags, mode);
}

int
close(int fd)
{
    if (fd >= 0 && fd < MAX_FDS)
        roles[fd] = NONE;
    return real_close(fd);
}

static int
endstop_closed(void)
{
    return trip_steps < 0 ? position <= trip_steps : position >= trip_steps;
}

static void
set_level(enum role role, int level)
{
    if (role == STEP && level != step_level) {
        if (level)
            position += dir_level ? 1 : -1;
        step_level = level;
    } else if (role == DIR) {
        dir_level = level;
    }
}

int
ioctl(int fd, unsigned long request, ...)
{
    va_list ap;
    va_start(ap, request);
    void *arg = va_arg(ap, void *);
    va_end(ap);
    enum role role = fd >= 0 && fd < MAX_FDS ? roles[fd] : NONE;
    if (role == NONE)
        return real_ioctl(fd, request, arg);
    if (role == CHIP && request == GPIO_GET_LINEHANDLE_IOCTL) {
        struct gpiohandle_request *req = arg;
        int line = req->lineoffsets[0];
        enum role lr = line == step_line ? STEP
            : line == dir_line ? DIR
            : line == endstop_line ? ENDSTOP : OTHER_LINE;
        req->fd = placeholder_fd(lr);
        if (req->flags & GPIOHANDLE_REQUEST_OUTPUT) {
            int level = !!req->default_values[0];
            if (lr == STEP)
                step_level = level;
            set_level(lr, level);
        }
        return 0;
    }
    if (request == GPIOHANDLE_SET_LINE_VALUES_IOCTL) {
        struct gpiohandle_data *data = arg;
        set_level(role, !!data->values[0]);
        return 0;
    }
    if (request == GPIOHANDLE_GET_LINE_VALUES_IOCTL) {
        struct gpiohandle_data *data = arg;
        data->values[0] = role == ENDSTOP && endstop_closed();
        return 0;
    }
    fprintf(stderr, "gpio_sim: unexpected ioctl 0x%lx on fd %d\n", request, fd);
    abort();
}
