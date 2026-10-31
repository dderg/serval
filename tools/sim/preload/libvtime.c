
#define _GNU_SOURCE
#include "vtime.h"

#include <dlfcn.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/futex.h>
#include <poll.h>
#include <pthread.h>
#include <semaphore.h>
#include <signal.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/epoll.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <time.h>
#include <unistd.h>

static int vtime_debug = 0;
static double vtime_speed = 100.0;
static uint64_t vtime_real_t0_ns = 0;
#define VLOG(fmt, ...) do { \
    if (vtime_debug) fprintf(stderr, "[vtime:%d] " fmt "\n", \
                             (int)getpid(), ##__VA_ARGS__); \
} while (0)

static struct vtime_shm *vshm = NULL;

static int (*real_clock_gettime)(clockid_t, struct timespec *) = NULL;
static int (*real_clock_nanosleep)(clockid_t, int, const struct timespec *,
                                   struct timespec *) = NULL;
static int (*real_nanosleep)(const struct timespec *, struct timespec *) = NULL;
static int (*real_poll)(struct pollfd *, nfds_t, int) = NULL;
static int (*real_ppoll)(struct pollfd *, nfds_t, const struct timespec *,
                          const sigset_t *) = NULL;
static int (*real_select)(int, fd_set *, fd_set *, fd_set *,
                           struct timeval *) = NULL;
static int (*real_pselect)(int, fd_set *, fd_set *, fd_set *,
                            const struct timespec *, const sigset_t *) = NULL;
static int (*real_epoll_pwait)(int, struct epoll_event *, int, int,
                                const sigset_t *) = NULL;
static int (*real_epoll_pwait2)(int, struct epoll_event *, int,
                                 const struct timespec *,
                                 const sigset_t *) = NULL;
static long (*real_syscall)(long, ...) = NULL;
static int (*real_pthread_cond_timedwait)(pthread_cond_t *, pthread_mutex_t *,
                                          const struct timespec *) = NULL;
static int (*real_pthread_cond_clockwait)(pthread_cond_t *, pthread_mutex_t *,
                                          clockid_t,
                                          const struct timespec *) = NULL;
static int (*real_sem_clockwait)(sem_t *, clockid_t,
                                 const struct timespec *) = NULL;
static int (*real_timer_create)(clockid_t, struct sigevent *,
                                 timer_t *) = NULL;
static int (*real_timer_settime)(timer_t, int, const struct itimerspec *,
                                  struct itimerspec *) = NULL;
static int (*real_usleep)(useconds_t) = NULL;

static uint64_t
real_monotonic_ns(void)
{
    struct timespec ts;
    if (real_clock_gettime)
        real_clock_gettime(CLOCK_MONOTONIC, &ts);
    else
        clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
}

// Real time this process observed pacers holding virtual time back,
// subtracted from the speed cap: after a pacer stall the cap resumes its
// slope from the stalled point instead of fast-forwarding through the lost
// span. A catch-up chase would run virtual time faster than the configured
// speed and burn the pump's piece-lead margin.
static _Atomic uint64_t vtime_stall_offset_ns = 0;

static uint64_t
vtime_speed_cap(void)
{
    if (vtime_speed <= 0)
        return UINT64_MAX;
    uint64_t real_elapsed = real_monotonic_ns() - vtime_real_t0_ns;
    uint64_t vtime_start = 1000000000ULL;
    return vtime_start + (uint64_t)(real_elapsed * vtime_speed)
           - atomic_load_explicit(&vtime_stall_offset_ns,
                                  memory_order_acquire);
}

static struct {
    pthread_t owner;
    int armed;
    uint64_t target_ns;
    pthread_mutex_t lock;
} vtimer = { .lock = PTHREAD_MUTEX_INITIALIZER };

static uint64_t
ts_to_ns(const struct timespec *ts)
{
    return (uint64_t)ts->tv_sec * 1000000000ULL + (uint64_t)ts->tv_nsec;
}

static void
ns_to_ts(uint64_t ns, struct timespec *ts)
{
    ts->tv_sec = ns / 1000000000ULL;
    ts->tv_nsec = ns % 1000000000ULL;
}

static uint64_t
vtime_now(void)
{
    return atomic_load_explicit(&vshm->nanos, memory_order_acquire);
}

static int owned_pacer_slots[VTIME_MAX_PACERS];

// Default distance virtual time may run ahead of a pacer's floor. Zero slack
// serializes every pacer step through a real sleep quantum — with two MCU
// processes that drags virtual time to a fraction of the configured speed.
// The slack is the burst bound: a preempted tick thread replays at most
// this much virtual time at once, so position observed against the virtual
// clock is never more than slack x velocity stale (2 ms at probing speeds
// is ~10 um). Keep it far below the ~1 ms endstop poll error budget only
// if probing speeds rise well past 5 mm/s. Pacers whose consumer needs a
// tighter bound register their own via vtime_pacer_register_slack.
#define VTIME_PACER_SLACK_NS 2000000ULL

static uint64_t
pacer_floor_min(void)
{
    uint64_t floor = UINT64_MAX;
    uint32_t used = atomic_load_explicit(&vshm->pacer_slots_used,
                                         memory_order_acquire);
    if (used > VTIME_MAX_PACERS)
        used = VTIME_MAX_PACERS;
    for (uint32_t i = 0; i < used; i++) {
        if (!atomic_load_explicit(&vshm->pacers[i].active,
                                  memory_order_acquire))
            continue;
        uint64_t f = atomic_load_explicit(&vshm->pacers[i].floor_ns,
                                          memory_order_acquire)
                     + atomic_load_explicit(&vshm->pacers[i].slack_ns,
                                            memory_order_acquire);
        if (f < floor)
            floor = f;
    }
    return floor;
}

static void
vtime_raise_to(uint64_t bounded_target)
{
    uint64_t cur;
    do {
        cur = atomic_load_explicit(&vshm->nanos, memory_order_acquire);
        if (bounded_target <= cur)
            return;
    } while (!atomic_compare_exchange_weak_explicit(
        &vshm->nanos, &cur, bounded_target,
        memory_order_acq_rel, memory_order_acquire));
}

static void
vtime_advance_to(uint64_t target_ns)
{
    for (;;) {
        if (vtime_now() >= target_ns)
            return;
        uint64_t bound = vtime_speed_cap();
        uint64_t floor = pacer_floor_min();
        if (floor < bound)
            bound = floor;
        if (bound > vtime_now()) {
            uint64_t step = target_ns < bound ? target_ns : bound;
            vtime_raise_to(step);
            if (step >= target_ns)
                return;
        }
        struct timespec w = { 0, 100000 };
        real_nanosleep(&w, NULL);
    }
}

static void vtimer_check_and_fire(void);

// Non-blocking: raise virtual time toward `target` as far as the speed cap
// and every pacer floor allow.
static void
vtime_nudge(uint64_t target)
{
    uint64_t bound = vtime_speed_cap();
    uint64_t floor = pacer_floor_min();
    if (floor < bound)
        bound = floor;
    vtime_raise_to(target < bound ? target : bound);
}

#define WAIT_SLICE_MIN_NS 50000ULL
#define WAIT_SLICE_MAX_NS 20000000ULL
#define NO_DEADLINE UINT64_MAX

// A timed wait in the thread that owns the virtual timer (the MCU main
// thread sleeping in ppoll until SIGALRM) must also stop at that timer.
static uint64_t
wait_target(uint64_t deadline)
{
    pthread_mutex_lock(&vtimer.lock);
    if (vtimer.armed && pthread_equal(vtimer.owner, pthread_self())
        && vtimer.target_ns < deadline)
        deadline = vtimer.target_ns;
    pthread_mutex_unlock(&vtimer.lock);
    return deadline;
}

// One step of every timed wait on a virtual-clock deadline: advance virtual
// time toward it, fire a due virtual timer, and return how long to block in
// real time before re-checking — 0 once the deadline has passed. Virtual time
// moves at most vtime_speed x real, so blocking remaining/speed never sleeps
// past the deadline.
static uint64_t
wait_step(uint64_t deadline)
{
    uint64_t target = wait_target(deadline);
    vtime_nudge(target);
    vtimer_check_and_fire();
    uint64_t now = vtime_now();
    if (now >= deadline)
        return 0;
    double slice = target > now ? (double)(target - now) / vtime_speed : 0;
    if (slice < WAIT_SLICE_MIN_NS)
        return WAIT_SLICE_MIN_NS;
    if (slice > WAIT_SLICE_MAX_NS)
        return WAIT_SLICE_MAX_NS;
    return (uint64_t)slice;
}

static uint64_t
deadline_after(uint64_t delta_ns)
{
    uint64_t now = vtime_now();
    return delta_ns > NO_DEADLINE - now ? NO_DEADLINE : now + delta_ns;
}

static struct timespec
real_deadline_after(uint64_t slice_ns)
{
    struct timespec ts;
    ns_to_ts(real_monotonic_ns() + slice_ns, &ts);
    return ts;
}

static int
is_monotonic(clockid_t clk)
{
    return clk == CLOCK_MONOTONIC || clk == CLOCK_MONOTONIC_RAW
           || clk == CLOCK_MONOTONIC_COARSE;
}

// Steady wall driver: raises virtual time toward the speed cap every 1ms
// so vtime is a linear function of real time (speed x real) instead of
// demand-driven. Demand-driven advancement stalls while nothing sleeps
// and then leaps to the cap, which klippy's clocksync tracks as a wildly
// nonuniform MCU clock — projections made across a leap land seconds off
// (PieceStartInPast, trip-time resolution failures).
// Time the floor held back is folded into vtime_stall_offset_ns so the
// cap resumes at the configured speed instead of chasing.
static void *
vtime_driver_main(void *arg)
{
    (void)arg;
    for (;;) {
        uint64_t cap = vtime_speed_cap();
        uint64_t floor = pacer_floor_min();
        if (floor < cap && cap != UINT64_MAX)
            atomic_fetch_add_explicit(&vtime_stall_offset_ns, cap - floor,
                                      memory_order_acq_rel);
        vtime_raise_to(floor < cap ? floor : cap);
        vtimer_check_and_fire();
        struct timespec w = { 0, 1000000 };
        real_nanosleep(&w, NULL);
    }
    return NULL;
}

static void vtimer_check_and_fire(void);

__attribute__((visibility("default"))) int
vtime_pacer_register_slack(uint64_t period_ns, uint64_t slack_ns)
{
    if (!vshm || period_ns == 0)
        return -1;
    uint32_t slot = atomic_fetch_add_explicit(&vshm->pacer_slots_used, 1,
                                              memory_order_acq_rel);
    if (slot >= VTIME_MAX_PACERS)
        return -1;
    atomic_store_explicit(&vshm->pacers[slot].slack_ns, slack_ns,
                          memory_order_release);
    atomic_store_explicit(&vshm->pacers[slot].floor_ns,
                          vtime_now() + period_ns, memory_order_release);
    atomic_store_explicit(&vshm->pacers[slot].active, 1,
                          memory_order_release);
    owned_pacer_slots[slot] = 1;
    VLOG("pacer %u registered, period=%lu ns, slack=%lu ns", slot,
         (unsigned long)period_ns, (unsigned long)slack_ns);
    return (int)slot;
}

__attribute__((visibility("default"))) int
vtime_pacer_register(uint64_t period_ns)
{
    return vtime_pacer_register_slack(period_ns, VTIME_PACER_SLACK_NS);
}

__attribute__((visibility("default"))) void
vtime_pacer_advance(int slot, uint64_t target_ns, uint64_t period_ns)
{
    if (!vshm || slot < 0 || slot >= VTIME_MAX_PACERS)
        return;
    (void)period_ns;
    // Floor equals the tick target exactly: virtual time may never outrun
    // the tick the pacer is about to execute, so the engine's clock read at
    // tick N is at most one period past any piece start it adopts.
    atomic_store_explicit(&vshm->pacers[slot].floor_ns, target_ns,
                          memory_order_release);
    vtime_advance_to(target_ns);
    vtimer_check_and_fire();
    sched_yield();
}

// Non-blocking floor update for pacers that gate on scheduled work without
// consuming virtual time themselves (the MCU main thread's timer queue:
// virtual time must not run past a pending software timer, or timers
// dispatch late in virtual terms and every position sampled by one — the
// endstop polls above all — lands a velocity-scaled distance off).
__attribute__((visibility("default"))) void
vtime_pacer_set_floor(int slot, uint64_t target_ns)
{
    if (!vshm || slot < 0 || slot >= VTIME_MAX_PACERS)
        return;
    atomic_store_explicit(&vshm->pacers[slot].floor_ns, target_ns,
                          memory_order_release);
}

__attribute__((destructor))
static void
vtime_pacer_cleanup(void)
{
    if (!vshm)
        return;
    for (int i = 0; i < VTIME_MAX_PACERS; i++)
        if (owned_pacer_slots[i])
            atomic_store_explicit(&vshm->pacers[i].active, 0,
                                  memory_order_release);
}

static void
vtimer_check_and_fire(void)
{
    pthread_mutex_lock(&vtimer.lock);
    if (vtimer.armed) {
        uint64_t now = vtime_now();
        if (now >= vtimer.target_ns) {
            vtimer.armed = 0;
            pthread_t owner = vtimer.owner;
            pthread_mutex_unlock(&vtimer.lock);
            pthread_kill(owner, SIGALRM);
            return;
        }
    }
    pthread_mutex_unlock(&vtimer.lock);
}

__attribute__((constructor(102)))
static void
vtime_init(void)
{
    const char *v = getenv("VTIME_DEBUG");
    vtime_debug = (v && v[0] == '1');

    const char *speed_env = getenv("VTIME_SPEED");
    if (speed_env)
        vtime_speed = atof(speed_env);
    if (vtime_speed <= 0.0)
        vtime_speed = 100.0;

    real_clock_gettime = dlsym(RTLD_NEXT, "clock_gettime");
    real_clock_nanosleep = dlsym(RTLD_NEXT, "clock_nanosleep");
    real_nanosleep = dlsym(RTLD_NEXT, "nanosleep");
    real_poll = dlsym(RTLD_NEXT, "poll");
    real_ppoll = dlsym(RTLD_NEXT, "ppoll");
    real_select = dlsym(RTLD_NEXT, "select");
    real_pselect = dlsym(RTLD_NEXT, "pselect");
    real_epoll_pwait = dlsym(RTLD_NEXT, "epoll_pwait");
    real_epoll_pwait2 = dlsym(RTLD_NEXT, "epoll_pwait2");
    if (!real_syscall)
        real_syscall = dlsym(RTLD_NEXT, "syscall");
    real_pthread_cond_timedwait = dlsym(RTLD_NEXT, "pthread_cond_timedwait");
    real_pthread_cond_clockwait = dlsym(RTLD_NEXT, "pthread_cond_clockwait");
    real_sem_clockwait = dlsym(RTLD_NEXT, "sem_clockwait");
    real_timer_create = dlsym(RTLD_NEXT, "timer_create");
    real_timer_settime = dlsym(RTLD_NEXT, "timer_settime");
    real_usleep = dlsym(RTLD_NEXT, "usleep");

    const char *shm_name = getenv("VTIME_SHM_NAME");
    if (!shm_name || !shm_name[0])
        shm_name = VTIME_SHM_NAME;
    int fd = shm_open(shm_name, O_RDWR, 0);
    if (fd < 0) {
        fd = shm_open(shm_name, O_CREAT | O_RDWR, 0666);
        if (fd < 0) {
            fprintf(stderr, "[vtime] shm_open failed: %s\n", strerror(errno));
            return;
        }
        if (ftruncate(fd, sizeof(struct vtime_shm)) < 0) {
            fprintf(stderr, "[vtime] ftruncate failed: %s\n", strerror(errno));
            close(fd);
            return;
        }
    }

    vshm = mmap(NULL, sizeof(struct vtime_shm), PROT_READ | PROT_WRITE,
                MAP_SHARED, fd, 0);
    close(fd);
    if (vshm == MAP_FAILED) {
        fprintf(stderr, "[vtime] mmap failed: %s\n", strerror(errno));
        vshm = NULL;
        return;
    }

    vtime_real_t0_ns = real_monotonic_ns();
    atomic_fetch_add(&vshm->num_participants, 1);

    pthread_t driver;
    if (pthread_create(&driver, NULL, vtime_driver_main, NULL) == 0)
        pthread_detach(driver);
    else {
        fprintf(stderr, "[vtime] driver thread create failed\n");
        abort();
    }

    VLOG("init, participants=%u, vtime=%lu ns, speed=%.0fx",
         atomic_load(&vshm->num_participants),
         (unsigned long)vtime_now(), vtime_speed);
}

int
clock_gettime(clockid_t clk_id, struct timespec *tp)
{
    if (!vshm || !tp || !is_monotonic(clk_id))
        return real_clock_gettime(clk_id, tp);
    ns_to_ts(vtime_now(), tp);
    return 0;
}

// A signal interrupts a virtual sleep exactly as it would a real one.
static int
sleep_until(uint64_t deadline)
{
    for (;;) {
        uint64_t slice = wait_step(deadline);
        if (!slice)
            return 0;
        struct timespec ts;
        ns_to_ts(slice, &ts);
        int ret = real_clock_nanosleep(CLOCK_MONOTONIC, 0, &ts, NULL);
        if (ret)
            return ret;
    }
}

static void
store_remaining(uint64_t deadline, struct timespec *rem)
{
    if (!rem)
        return;
    uint64_t now = vtime_now();
    ns_to_ts(deadline > now ? deadline - now : 0, rem);
}

int
clock_nanosleep(clockid_t clk_id, int flags, const struct timespec *req,
                struct timespec *rem)
{
    if (!vshm || !is_monotonic(clk_id))
        return real_clock_nanosleep(clk_id, flags, req, rem);
    uint64_t deadline = flags & TIMER_ABSTIME ? ts_to_ns(req)
                                              : deadline_after(ts_to_ns(req));
    int ret = sleep_until(deadline);
    if (ret && !(flags & TIMER_ABSTIME))
        store_remaining(deadline, rem);
    return ret;
}

int
nanosleep(const struct timespec *req, struct timespec *rem)
{
    if (!vshm)
        return real_nanosleep(req, rem);
    uint64_t deadline = deadline_after(ts_to_ns(req));
    int ret = sleep_until(deadline);
    if (!ret)
        return 0;
    store_remaining(deadline, rem);
    errno = ret;
    return -1;
}

int
usleep(useconds_t usec)
{
    if (!vshm)
        return real_usleep(usec);
    int ret = sleep_until(deadline_after((uint64_t)usec * 1000ULL));
    if (!ret)
        return 0;
    errno = ret;
    return -1;
}

static uint64_t
deadline_from_ts(const struct timespec *timeout)
{
    return timeout ? deadline_after(ts_to_ns(timeout)) : NO_DEADLINE;
}

int
ppoll(struct pollfd *fds, nfds_t nfds, const struct timespec *timeout,
      const sigset_t *sigmask)
{
    if (!vshm)
        return real_ppoll(fds, nfds, timeout, sigmask);
    uint64_t deadline = deadline_from_ts(timeout);
    for (;;) {
        uint64_t slice = wait_step(deadline);
        struct timespec ts;
        ns_to_ts(slice, &ts);
        int ret = real_ppoll(fds, nfds, &ts, sigmask);
        if (ret || !slice)
            return ret;
    }
}

int
poll(struct pollfd *fds, nfds_t nfds, int timeout_ms)
{
    if (!vshm || timeout_ms == 0)
        return real_poll(fds, nfds, timeout_ms);
    struct timespec ts;
    ns_to_ts((uint64_t)timeout_ms * 1000000ULL, &ts);
    return ppoll(fds, nfds, timeout_ms > 0 ? &ts : NULL, NULL);
}

static int
pselect_until(int nfds, fd_set *readfds, fd_set *writefds, fd_set *exceptfds,
              uint64_t deadline, const sigset_t *sigmask)
{
    fd_set r0, w0, e0;
    if (readfds)
        r0 = *readfds;
    if (writefds)
        w0 = *writefds;
    if (exceptfds)
        e0 = *exceptfds;
    for (;;) {
        uint64_t slice = wait_step(deadline);
        struct timespec ts;
        ns_to_ts(slice, &ts);
        int ret = real_pselect(nfds, readfds, writefds, exceptfds, &ts,
                               sigmask);
        if (ret || !slice)
            return ret;
        if (readfds)
            *readfds = r0;
        if (writefds)
            *writefds = w0;
        if (exceptfds)
            *exceptfds = e0;
    }
}

int
select(int nfds, fd_set *readfds, fd_set *writefds, fd_set *exceptfds,
       struct timeval *timeout)
{
    if (!vshm || (timeout && !timeout->tv_sec && !timeout->tv_usec))
        return real_select(nfds, readfds, writefds, exceptfds, timeout);
    uint64_t deadline = NO_DEADLINE;
    if (timeout)
        deadline = deadline_after((uint64_t)timeout->tv_sec * 1000000000ULL
                                  + (uint64_t)timeout->tv_usec * 1000ULL);
    int ret = pselect_until(nfds, readfds, writefds, exceptfds, deadline,
                            NULL);
    if (timeout) {
        struct timespec rem;
        store_remaining(deadline, &rem);
        timeout->tv_sec = rem.tv_sec;
        timeout->tv_usec = rem.tv_nsec / 1000;
    }
    return ret;
}

int
pselect(int nfds, fd_set *readfds, fd_set *writefds, fd_set *exceptfds,
        const struct timespec *timeout, const sigset_t *sigmask)
{
    if (!vshm)
        return real_pselect(nfds, readfds, writefds, exceptfds, timeout,
                            sigmask);
    return pselect_until(nfds, readfds, writefds, exceptfds,
                         deadline_from_ts(timeout), sigmask);
}

int
epoll_pwait2(int epfd, struct epoll_event *events, int maxevents,
             const struct timespec *timeout, const sigset_t *sigmask)
{
    if (!vshm)
        return real_epoll_pwait2(epfd, events, maxevents, timeout, sigmask);
    uint64_t deadline = deadline_from_ts(timeout);
    for (;;) {
        uint64_t slice = wait_step(deadline);
        int ret;
        if (real_epoll_pwait2) {
            struct timespec ts;
            ns_to_ts(slice, &ts);
            ret = real_epoll_pwait2(epfd, events, maxevents, &ts, sigmask);
        } else {
            ret = real_epoll_pwait(epfd, events, maxevents,
                                   (int)((slice + 999999ULL) / 1000000ULL),
                                   sigmask);
        }
        if (ret || !slice)
            return ret;
    }
}

int
epoll_pwait(int epfd, struct epoll_event *events, int maxevents,
            int timeout_ms, const sigset_t *sigmask)
{
    if (!vshm || timeout_ms == 0)
        return real_epoll_pwait(epfd, events, maxevents, timeout_ms, sigmask);
    struct timespec ts;
    ns_to_ts((uint64_t)timeout_ms * 1000000ULL, &ts);
    return epoll_pwait2(epfd, events, maxevents, timeout_ms > 0 ? &ts : NULL,
                        sigmask);
}

int
epoll_wait(int epfd, struct epoll_event *events, int maxevents,
           int timeout_ms)
{
    return epoll_pwait(epfd, events, maxevents, timeout_ms, NULL);
}

// Rust std (park, Condvar, channel timeouts) and parking_lot wait through
// raw futex syscalls; the bitset form takes an absolute CLOCK_MONOTONIC
// deadline that Rust computed from the virtual clock.
static long
futex_wait_until(uint32_t *uaddr, int op, uint32_t val, uint64_t deadline,
                 uint32_t bitset)
{
    int bitset_op = (op & ~FUTEX_CMD_MASK) | FUTEX_WAIT_BITSET;
    for (;;) {
        uint64_t slice = wait_step(deadline);
        if (!slice) {
            errno = ETIMEDOUT;
            return -1;
        }
        struct timespec ts = real_deadline_after(slice);
        long ret = real_syscall(SYS_futex, uaddr, bitset_op, val, &ts, NULL,
                                bitset);
        if (ret == 0 || errno != ETIMEDOUT)
            return ret;
    }
}

long
syscall(long number, ...)
{
    va_list ap;
    va_start(ap, number);
    long a1 = va_arg(ap, long), a2 = va_arg(ap, long), a3 = va_arg(ap, long);
    long a4 = va_arg(ap, long), a5 = va_arg(ap, long), a6 = va_arg(ap, long);
    va_end(ap);
    if (!real_syscall)
        real_syscall = dlsym(RTLD_NEXT, "syscall");
    if (vshm && number == SYS_futex) {
        int op = (int)a2, cmd = op & FUTEX_CMD_MASK;
        const struct timespec *timeout = (const struct timespec *)a4;
        if (timeout && !(op & FUTEX_CLOCK_REALTIME)
            && (cmd == FUTEX_WAIT || cmd == FUTEX_WAIT_BITSET)) {
            uint64_t deadline = cmd == FUTEX_WAIT
                                    ? deadline_after(ts_to_ns(timeout))
                                    : ts_to_ns(timeout);
            uint32_t bitset = cmd == FUTEX_WAIT ? FUTEX_BITSET_MATCH_ANY
                                                : (uint32_t)a6;
            return futex_wait_until((uint32_t *)a1, op, (uint32_t)a3,
                                    deadline, bitset);
        }
    }
    return real_syscall(number, a1, a2, a3, a4, a5, a6);
}

static int
cond_wait_until(pthread_cond_t *cond, pthread_mutex_t *mutex,
                uint64_t deadline)
{
    for (;;) {
        uint64_t slice = wait_step(deadline);
        if (!slice)
            return ETIMEDOUT;
        struct timespec ts = real_deadline_after(slice);
        int ret = real_pthread_cond_clockwait(cond, mutex, CLOCK_MONOTONIC,
                                              &ts);
        if (ret != ETIMEDOUT)
            return ret;
    }
}

// CPython's GIL hand-off waits on a CLOCK_MONOTONIC condvar; glibc records
// the condvar's clock in bit 1 of __wrefs.
int
pthread_cond_timedwait(pthread_cond_t *cond, pthread_mutex_t *mutex,
                       const struct timespec *abstime)
{
    if (!vshm || !(cond->__data.__wrefs & 2))
        return real_pthread_cond_timedwait(cond, mutex, abstime);
    return cond_wait_until(cond, mutex, ts_to_ns(abstime));
}

int
pthread_cond_clockwait(pthread_cond_t *cond, pthread_mutex_t *mutex,
                       clockid_t clockid, const struct timespec *abstime)
{
    if (!vshm || !is_monotonic(clockid))
        return real_pthread_cond_clockwait(cond, mutex, clockid, abstime);
    return cond_wait_until(cond, mutex, ts_to_ns(abstime));
}

// CPython's timed lock acquire (queue.get, Event.wait, Lock.acquire).
int
sem_clockwait(sem_t *sem, clockid_t clockid, const struct timespec *abstime)
{
    if (!vshm || !is_monotonic(clockid))
        return real_sem_clockwait(sem, clockid, abstime);
    uint64_t deadline = ts_to_ns(abstime);
    for (;;) {
        uint64_t slice = wait_step(deadline);
        if (!slice) {
            if (sem_trywait(sem) == 0)
                return 0;
            errno = ETIMEDOUT;
            return -1;
        }
        struct timespec ts = real_deadline_after(slice);
        int ret = real_sem_clockwait(sem, CLOCK_MONOTONIC, &ts);
        if (ret == 0 || errno != ETIMEDOUT)
            return ret;
    }
}

int
timer_create(clockid_t clk_id, struct sigevent *sevp, timer_t *timerid)
{
    if (!vshm)
        return real_timer_create(clk_id, sevp, timerid);

    // Klipper creates a CLOCK_MONOTONIC timer that fires SIGALRM.
    // We intercept this and use virtual time instead.
    if (clk_id == CLOCK_MONOTONIC) {
        pthread_mutex_lock(&vtimer.lock);
        vtimer.armed = 0;
        vtimer.target_ns = 0;
        vtimer.owner = pthread_self();
        pthread_mutex_unlock(&vtimer.lock);
        *timerid = (timer_t)0xCAFE;
        VLOG("timer_create: virtual timer created");
        return 0;
    }
    return real_timer_create(clk_id, sevp, timerid);
}

int
timer_settime(timer_t timerid, int flags, const struct itimerspec *new_value,
              struct itimerspec *old_value)
{
    if (!vshm || timerid != (timer_t)0xCAFE)
        return real_timer_settime(timerid, flags, new_value, old_value);

    if (old_value) {
        memset(old_value, 0, sizeof(*old_value));
    }

    pthread_mutex_lock(&vtimer.lock);
    if (new_value->it_value.tv_sec == 0 && new_value->it_value.tv_nsec == 0) {
        vtimer.armed = 0;
    } else if (flags & TIMER_ABSTIME) {
        vtimer.target_ns = ts_to_ns(&new_value->it_value);
        vtimer.armed = 1;
    } else {
        vtimer.target_ns = vtime_now() + ts_to_ns(&new_value->it_value);
        vtimer.armed = 1;
    }
    pthread_mutex_unlock(&vtimer.lock);

    vtimer_check_and_fire();

    return 0;
}
