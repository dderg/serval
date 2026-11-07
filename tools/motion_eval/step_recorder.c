// LD_PRELOAD for the Klipper linux-process MCU: records every step it
// executes, with no hardware. Derived from test/gpio_sim.c on kv2/ethercat.
//
// /dev/gpiochip<KV2_STEP_REC_CHIP> exists only in memory. KV2_STEP_REC_LINES
// lists "step:dir" line pairs, one per stepper ("0:1,3:4"). Every rising
// edge of a step line appends {CLOCK_MONOTONIC ns, stepper, dir level} to
// the file KV2_STEP_REC_OUT. The MCU locks its memory (mlockall) under a
// small RLIMIT_MEMLOCK, so records go through a fixed buffer, written out
// when full and on SIGTERM, which then ends the process. Other lines are
// plain outputs, and inputs read 0. Every other path and descriptor goes
// to libc.
#define _GNU_SOURCE
#include <dlfcn.h>
#include <fcntl.h>
#include <linux/gpio.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <signal.h>
#include <time.h>
#include <unistd.h>

#define MAX_FDS 4096
#define MAX_STEPPERS 8
#define BUFFERED 4096

struct record {
    int64_t ns;
    int32_t stepper;
    int32_t dir;
};

enum role { NONE, CHIP, STEP, DIR, OTHER_LINE };

static enum role roles[MAX_FDS];
static int stepper_of[MAX_FDS];
static int sim_chip;
static int nsteppers;
static int step_line[MAX_STEPPERS], dir_line[MAX_STEPPERS];
static int step_level[MAX_STEPPERS], dir_level[MAX_STEPPERS];
static struct record buffered[BUFFERED];
static volatile int nbuffered;
static int out_fd = -1;

static int (*real_open)(const char *, int, ...);
static int (*real_access)(const char *, int);
static int (*real_ioctl)(int, unsigned long, ...);
static int (*real_close)(int);

static void
fail(const char *msg)
{
    fprintf(stderr, "step_recorder: %s\n", msg);
    abort();
}

static const char *
env(const char *name)
{
    const char *v = getenv(name);
    if (!v)
        fail("an environment variable KV2_STEP_REC_* is not set");
    return v;
}

static void
flush_records(void)
{
    size_t size = nbuffered * sizeof(struct record);
    const char *p = (const char *)buffered;
    while (size) {
        ssize_t n = write(out_fd, p, size);
        if (n <= 0)
            abort();
        p += n;
        size -= n;
    }
    nbuffered = 0;
}

static void
on_sigterm(int sig)
{
    flush_records();
    _exit(0);
}

__attribute__((constructor)) static void
step_recorder_init(void)
{
    real_open = dlsym(RTLD_NEXT, "open");
    real_access = dlsym(RTLD_NEXT, "access");
    real_ioctl = dlsym(RTLD_NEXT, "ioctl");
    real_close = dlsym(RTLD_NEXT, "close");
    sim_chip = atoi(env("KV2_STEP_REC_CHIP"));
    const char *p = env("KV2_STEP_REC_LINES");
    while (*p) {
        if (nsteppers == MAX_STEPPERS)
            fail("too many steppers");
        char *end;
        step_line[nsteppers] = strtol(p, &end, 10);
        if (*end != ':')
            fail("KV2_STEP_REC_LINES must be step:dir[,step:dir...]");
        dir_line[nsteppers] = strtol(end + 1, &end, 10);
        nsteppers++;
        p = *end == ',' ? end + 1 : end;
    }
    out_fd = real_open(env("KV2_STEP_REC_OUT"),
                       O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0644);
    if (out_fd < 0)
        fail("cannot create KV2_STEP_REC_OUT");
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = on_sigterm;
    if (sigaction(SIGTERM, &sa, NULL))
        fail("cannot install the SIGTERM handler");
}

static int
is_sim_chip(const char *path)
{
    char want[64];
    snprintf(want, sizeof(want), "/dev/gpiochip%d", sim_chip);
    return strcmp(path, want) == 0;
}

static int
placeholder_fd(enum role role, int stepper)
{
    int fd = real_open("/dev/null", O_RDWR | O_CLOEXEC);
    if (fd < 0 || fd >= MAX_FDS)
        fail("no descriptor for a GPIO line");
    roles[fd] = role;
    stepper_of[fd] = stepper;
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
        return placeholder_fd(CHIP, -1);
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

static void
record_step(int stepper)
{
    if (nbuffered == BUFFERED)
        flush_records();
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    struct record *r = &buffered[nbuffered];
    r->ns = (int64_t)ts.tv_sec * 1000000000 + ts.tv_nsec;
    r->stepper = stepper;
    r->dir = dir_level[stepper];
    nbuffered++;
}

static void
set_level(enum role role, int stepper, int level)
{
    if (role == STEP) {
        if (level && !step_level[stepper])
            record_step(stepper);
        step_level[stepper] = level;
    } else if (role == DIR) {
        dir_level[stepper] = level;
    }
}

static void
line_role(int line, enum role *role, int *stepper)
{
    for (int i = 0; i < nsteppers; i++) {
        if (line == step_line[i]) {
            *role = STEP;
            *stepper = i;
            return;
        }
        if (line == dir_line[i]) {
            *role = DIR;
            *stepper = i;
            return;
        }
    }
    *role = OTHER_LINE;
    *stepper = -1;
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
        enum role lr;
        int stepper;
        line_role(req->lineoffsets[0], &lr, &stepper);
        req->fd = placeholder_fd(lr, stepper);
        if (req->flags & GPIOHANDLE_REQUEST_OUTPUT) {
            int level = !!req->default_values[0];
            if (lr == STEP)
                step_level[stepper] = level;
            else if (lr == DIR)
                dir_level[stepper] = level;
        }
        return 0;
    }
    if (request == GPIOHANDLE_SET_LINE_VALUES_IOCTL) {
        struct gpiohandle_data *data = arg;
        set_level(role, stepper_of[fd], !!data->values[0]);
        return 0;
    }
    if (request == GPIOHANDLE_GET_LINE_VALUES_IOCTL) {
        struct gpiohandle_data *data = arg;
        data->values[0] = 0;
        return 0;
    }
    fprintf(stderr, "step_recorder: unexpected ioctl 0x%lx on fd %d\n",
            request, fd);
    abort();
}
