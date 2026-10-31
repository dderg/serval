
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
#include <sys/socket.h>
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
static int (*real_pthread_cond_wait)(pthread_cond_t *, pthread_mutex_t *);
static int (*real_sem_wait)(sem_t *);
static int (*real_sem_timedwait)(sem_t *, const struct timespec *);
static ssize_t (*real_read)(int, void *, size_t);
static ssize_t (*real_recv)(int, void *, size_t, int);
static ssize_t (*real_recvfrom)(int, void *, size_t, int, struct sockaddr *,
                                socklen_t *);
static ssize_t (*real_recvmsg)(int, struct msghdr *, int);
static int (*real_accept4)(int, struct sockaddr *, socklen_t *, int);
static int (*real_pthread_join)(pthread_t, void **);
static int (*real_pthread_create)(pthread_t *, const pthread_attr_t *,
                                  void *(*)(void *), void *);
static void (*real_pthread_exit)(void *);

// Resolves on first use: these can run before the constructor (another
// preloaded library's init, or a thread started before it).
#define REAL(fn) \
    (real_##fn ? real_##fn \
               : (real_##fn = (__typeof__(real_##fn))dlsym(RTLD_NEXT, #fn)))

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

#define VTIME_KICK_SIGNAL (SIGRTMIN + 7)

// The kick exists only to cut a real-time sleep short. Its EINTR goes back
// to the caller like any other: every participant already retries on EINTR
// (CPython PEP 475, Rust std, the firmware's irq_wait), and a kick can land
// in the same wait as a real signal, so swallowing it would swallow that
// signal's EINTR too.
static void
kick_handler(int sig)
{
    (void)sig;
}

static uint32_t
waiter_slots_used(void)
{
    uint32_t used = atomic_load_explicit(&vshm->waiter_slots_used,
                                         memory_order_acquire);
    return used > VTIME_MAX_WAITERS ? VTIME_MAX_WAITERS : used;
}

// Wake every waiter whose target virtual time has reached: it would
// otherwise notice only when its real-time sleep slice expires, leaving the
// clock parked on its target until then.
static void
kick_due_waiters(uint64_t now)
{
    uint32_t used = waiter_slots_used();
    for (uint32_t i = 0; i < used; i++) {
        uint64_t target = atomic_load_explicit(&vshm->waiter_target[i],
                                               memory_order_acquire);
        uint32_t idle = 0;
        if (!target || target > now
            || !atomic_compare_exchange_strong(&vshm->waiter_kicked[i],
                                               &idle, 1))
            continue;
        uint64_t who = atomic_load_explicit(&vshm->waiter_thread[i],
                                            memory_order_acquire);
        real_syscall(SYS_tgkill, (long)(who >> 32), (long)(uint32_t)who,
                     (long)VTIME_KICK_SIGNAL);
    }
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
    kick_due_waiters(bounded_target);
}

static int is_host;

// A host thread woken by a message still counts as idle until its wait
// returns; in that window virtual time can only reach the next pacer floor
// or host deadline, which bounds the error to sub-millisecond lateness.
static int
host_holds_time(void)
{
    return atomic_load_explicit(&vshm->host_present, memory_order_acquire)
           && atomic_load_explicit(&vshm->host_running,
                                   memory_order_acquire) > 0;
}

static uint64_t
host_wait_floor(void)
{
    uint64_t floor = UINT64_MAX;
    uint32_t used = waiter_slots_used();
    for (uint32_t i = 0; i < used; i++) {
        if (!atomic_load_explicit(&vshm->waiter_is_floor[i],
                                  memory_order_acquire))
            continue;
        uint64_t target = atomic_load_explicit(&vshm->waiter_target[i],
                                               memory_order_acquire);
        if (target && target < floor)
            floor = target;
    }
    return floor;
}

static int
harness_paced(void)
{
    return atomic_load_explicit(&vshm->harness_paced, memory_order_acquire);
}

// The speed cap, tightened to real time while the harness is paced.
static uint64_t
vtime_cap(void)
{
    uint64_t cap = vtime_speed_cap();
    if (!harness_paced())
        return cap;
    uint64_t paced = atomic_load_explicit(&vshm->harness_pace_vtime,
                                          memory_order_acquire)
                     + real_monotonic_ns()
                     - atomic_load_explicit(&vshm->harness_pace_real,
                                            memory_order_acquire);
    return paced < cap ? paced : cap;
}

// How far virtual time may advance right now: the cap, every pacer floor,
// the earliest host wait deadline, and nowhere at all while the host is
// busy.
static uint64_t
vtime_bound(void)
{
    uint64_t bound = vtime_cap();
    uint64_t floor = pacer_floor_min();
    if (floor < bound)
        bound = floor;
    floor = host_wait_floor();
    if (floor < bound)
        bound = floor;
    if (host_holds_time()) {
        uint64_t now = vtime_now();
        if (now < bound)
            bound = now;
    }
    return bound;
}

static __thread int waiter_slot = -1;

static int
own_waiter_slot(void)
{
    if (waiter_slot < 0) {
        uint32_t slot = atomic_fetch_add_explicit(&vshm->waiter_slots_used, 1,
                                                  memory_order_acq_rel);
        if (slot >= VTIME_MAX_WAITERS) {
            fprintf(stderr, "[vtime] more than %d threads wait on the virtual "
                            "clock\n", VTIME_MAX_WAITERS);
            abort();
        }
        atomic_store_explicit(&vshm->waiter_thread[slot],
                              (uint64_t)getpid() << 32 | (uint32_t)gettid(),
                              memory_order_release);
        atomic_store_explicit(&vshm->waiter_is_floor[slot], is_host,
                              memory_order_release);
        waiter_slot = (int)slot;
    }
    return waiter_slot;
}

static uint64_t wait_target(uint64_t deadline);

// Entering a blocking wait: publish the virtual time it waits for so
// whoever reaches it can wake this thread, and stop counting a host thread
// as running.
static void
thread_blocks(uint64_t deadline)
{
    if (!vshm)
        return;
    uint64_t target = wait_target(deadline);
    if (target != UINT64_MAX) {
        int slot = own_waiter_slot();
        atomic_store_explicit(&vshm->waiter_kicked[slot], 0,
                              memory_order_release);
        atomic_store_explicit(&vshm->waiter_target[slot], target,
                              memory_order_release);
    }
    if (is_host)
        atomic_fetch_sub_explicit(&vshm->host_running, 1,
                                  memory_order_acq_rel);
}

static void
thread_runs(void)
{
    if (!vshm)
        return;
    if (is_host)
        atomic_fetch_add_explicit(&vshm->host_running, 1,
                                  memory_order_acq_rel);
    if (waiter_slot >= 0)
        atomic_store_explicit(&vshm->waiter_target[waiter_slot], 0,
                              memory_order_release);
}

// Host thread creation and exit: a host thread counts as running from the
// moment its creator spawns it until it exits.
static void
host_threads_add(int delta)
{
    if (is_host)
        atomic_fetch_add_explicit(&vshm->host_running, delta,
                                  memory_order_acq_rel);
}

static void
vtime_advance_to(uint64_t target_ns)
{
    for (;;) {
        if (vtime_now() >= target_ns)
            return;
        uint64_t bound = vtime_bound();
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

#define WAIT_SLICE_MIN_NS 50000ULL
#define WAIT_SLICE_MAX_NS 20000000ULL
#define WAIT_SLICE_HELD_NS WAIT_SLICE_MIN_NS
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
// past the deadline; time held by the host or a pacer is re-checked soon.
static uint64_t
wait_step(uint64_t deadline)
{
    uint64_t target = wait_target(deadline);
    uint64_t cap = vtime_cap();
    uint64_t bound = vtime_bound();
    vtime_raise_to(target < bound ? target : bound);
    vtimer_check_and_fire();
    uint64_t now = vtime_now();
    if (now >= deadline)
        return 0;
    if (bound < cap)
        return WAIT_SLICE_HELD_NS;
    double rate = harness_paced() ? 1.0 : vtime_speed;
    double slice = target > now ? (double)(target - now) / rate : 0;
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

// Backstop advancer: every 1 ms of real time, raise virtual time as far as
// the cap and every floor allow, in case no waiter is awake to do it.
// Waiters normally advance the clock themselves; this only keeps it from
// stalling when all of them sleep in long real-time slices.
// Time the floor held back is folded into vtime_stall_offset_ns so the
// cap resumes at the configured speed instead of chasing.
static void *
vtime_driver_main(void *arg)
{
    (void)arg;
    for (;;) {
        uint64_t cap = vtime_cap();
        uint64_t bound = vtime_bound();
        if (bound < cap && cap != UINT64_MAX)
            atomic_fetch_add_explicit(&vtime_stall_offset_ns, cap - bound,
                                      memory_order_acq_rel);
        vtime_raise_to(bound);
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
    if (is_host)
        atomic_store_explicit(&vshm->host_present, 0, memory_order_release);
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

    struct sigaction kick = { .sa_handler = kick_handler,
                              .sa_flags = SA_RESTART };
    sigemptyset(&kick.sa_mask);
    sigaction(VTIME_KICK_SIGNAL, &kick, NULL);

    // Only this process is the host: children it spawns inherit the
    // preload but not the role.
    const char *host = getenv("VTIME_HOST");
    if (host && host[0] == '1') {
        unsetenv("VTIME_HOST");
        is_host = 1;
        // A restarted host's threads are gone: retire their slots so a
        // deadline left behind by a killed waiter cannot hold the clock.
        for (uint32_t i = 0; i < waiter_slots_used(); i++)
            if (atomic_load(&vshm->waiter_is_floor[i])) {
                atomic_store(&vshm->waiter_is_floor[i], 0);
                atomic_store(&vshm->waiter_target[i], 0);
            }
        atomic_store(&vshm->host_running, 1);
        atomic_store(&vshm->host_present, 1);
    }

    // The driver is clock infrastructure, not host work: never counted.
    pthread_t driver;
    if (REAL(pthread_create)(&driver, NULL, vtime_driver_main, NULL) == 0)
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
sleep_loop(uint64_t deadline)
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

static int
sleep_until(uint64_t deadline)
{
    thread_blocks(deadline);
    int ret = sleep_loop(deadline);
    thread_runs();
    return ret;
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

static int
ppoll_loop(struct pollfd *fds, nfds_t nfds, uint64_t deadline,
           const sigset_t *sigmask)
{
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
ppoll(struct pollfd *fds, nfds_t nfds, const struct timespec *timeout,
      const sigset_t *sigmask)
{
    if (!vshm)
        return real_ppoll(fds, nfds, timeout, sigmask);
    uint64_t deadline = deadline_from_ts(timeout);
    thread_blocks(deadline);
    int ret = ppoll_loop(fds, nfds, deadline, sigmask);
    thread_runs();
    return ret;
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
pselect_loop(int nfds, fd_set *readfds, fd_set *writefds, fd_set *exceptfds,
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

static int
pselect_until(int nfds, fd_set *readfds, fd_set *writefds, fd_set *exceptfds,
              uint64_t deadline, const sigset_t *sigmask)
{
    thread_blocks(deadline);
    int ret = pselect_loop(nfds, readfds, writefds, exceptfds, deadline,
                           sigmask);
    thread_runs();
    return ret;
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

static int
epoll_loop(int epfd, struct epoll_event *events, int maxevents,
           uint64_t deadline, const sigset_t *sigmask)
{
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
epoll_pwait2(int epfd, struct epoll_event *events, int maxevents,
             const struct timespec *timeout, const sigset_t *sigmask)
{
    if (!vshm)
        return real_epoll_pwait2(epfd, events, maxevents, timeout, sigmask);
    uint64_t deadline = deadline_from_ts(timeout);
    thread_blocks(deadline);
    int ret = epoll_loop(epfd, events, maxevents, deadline, sigmask);
    thread_runs();
    return ret;
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
futex_wait_loop(uint32_t *uaddr, int op, uint32_t val, uint64_t deadline,
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
    int op = (int)a2, cmd = op & FUTEX_CMD_MASK;
    if (!vshm || number != SYS_futex
        || (cmd != FUTEX_WAIT && cmd != FUTEX_WAIT_BITSET))
        return real_syscall(number, a1, a2, a3, a4, a5, a6);
    const struct timespec *timeout = (const struct timespec *)a4;
    long ret;
    if (timeout && !(op & FUTEX_CLOCK_REALTIME)) {
        uint64_t deadline = cmd == FUTEX_WAIT
                                ? deadline_after(ts_to_ns(timeout))
                                : ts_to_ns(timeout);
        uint32_t bitset = cmd == FUTEX_WAIT ? FUTEX_BITSET_MATCH_ANY
                                            : (uint32_t)a6;
        thread_blocks(deadline);
        ret = futex_wait_loop((uint32_t *)a1, op, (uint32_t)a3, deadline,
                              bitset);
    } else {
        thread_blocks(NO_DEADLINE);
        ret = real_syscall(number, a1, a2, a3, a4, a5, a6);
    }
    thread_runs();
    return ret;
}

static int
cond_wait_loop(pthread_cond_t *cond, pthread_mutex_t *mutex,
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

int
pthread_cond_wait(pthread_cond_t *cond, pthread_mutex_t *mutex)
{
    thread_blocks(NO_DEADLINE);
    int ret = REAL(pthread_cond_wait)(cond, mutex);
    thread_runs();
    return ret;
}

// CPython's GIL hand-off waits on a CLOCK_MONOTONIC condvar; glibc records
// the condvar's clock in bit 1 of __wrefs.
int
pthread_cond_timedwait(pthread_cond_t *cond, pthread_mutex_t *mutex,
                       const struct timespec *abstime)
{
    int monotonic = vshm && (cond->__data.__wrefs & 2);
    thread_blocks(monotonic ? ts_to_ns(abstime) : NO_DEADLINE);
    int ret = monotonic ? cond_wait_loop(cond, mutex, ts_to_ns(abstime))
                        : REAL(pthread_cond_timedwait)(cond, mutex, abstime);
    thread_runs();
    return ret;
}

int
pthread_cond_clockwait(pthread_cond_t *cond, pthread_mutex_t *mutex,
                       clockid_t clockid, const struct timespec *abstime)
{
    int monotonic = vshm && is_monotonic(clockid);
    thread_blocks(monotonic ? ts_to_ns(abstime) : NO_DEADLINE);
    int ret = monotonic ? cond_wait_loop(cond, mutex, ts_to_ns(abstime))
                        : REAL(pthread_cond_clockwait)(cond, mutex, clockid,
                                                       abstime);
    thread_runs();
    return ret;
}

static int
sem_wait_loop(sem_t *sem, uint64_t deadline)
{
    for (;;) {
        uint64_t slice = wait_step(deadline);
        if (!slice) {
            if (sem_trywait(sem) == 0)
                return 0;
            errno = ETIMEDOUT;
            return -1;
        }
        struct timespec ts = real_deadline_after(slice);
        int ret = REAL(sem_clockwait)(sem, CLOCK_MONOTONIC, &ts);
        if (ret == 0 || errno != ETIMEDOUT)
            return ret;
    }
}

// CPython's timed lock acquire (queue.get, Event.wait, Lock.acquire).
int
sem_clockwait(sem_t *sem, clockid_t clockid, const struct timespec *abstime)
{
    int monotonic = vshm && is_monotonic(clockid);
    thread_blocks(monotonic ? ts_to_ns(abstime) : NO_DEADLINE);
    int ret = monotonic ? sem_wait_loop(sem, ts_to_ns(abstime))
                        : REAL(sem_clockwait)(sem, clockid, abstime);
    thread_runs();
    return ret;
}

int
sem_timedwait(sem_t *sem, const struct timespec *abstime)
{
    thread_blocks(NO_DEADLINE);
    int ret = REAL(sem_timedwait)(sem, abstime);
    thread_runs();
    return ret;
}

int
sem_wait(sem_t *sem)
{
    thread_blocks(NO_DEADLINE);
    int ret = REAL(sem_wait)(sem);
    thread_runs();
    return ret;
}

static int
fd_blocks(int fd)
{
    int flags = fcntl(fd, F_GETFL);
    return flags >= 0 && !(flags & O_NONBLOCK);
}

ssize_t
read(int fd, void *buf, size_t count)
{
    if (!is_host || !fd_blocks(fd))
        return REAL(read)(fd, buf, count);
    thread_blocks(NO_DEADLINE);
    ssize_t ret = REAL(read)(fd, buf, count);
    thread_runs();
    return ret;
}

ssize_t
recv(int fd, void *buf, size_t len, int flags)
{
    if (!is_host || (flags & MSG_DONTWAIT) || !fd_blocks(fd))
        return REAL(recv)(fd, buf, len, flags);
    thread_blocks(NO_DEADLINE);
    ssize_t ret = REAL(recv)(fd, buf, len, flags);
    thread_runs();
    return ret;
}

ssize_t
recvfrom(int fd, void *buf, size_t len, int flags, struct sockaddr *addr,
         socklen_t *addrlen)
{
    if (!is_host || (flags & MSG_DONTWAIT) || !fd_blocks(fd))
        return REAL(recvfrom)(fd, buf, len, flags, addr, addrlen);
    thread_blocks(NO_DEADLINE);
    ssize_t ret = REAL(recvfrom)(fd, buf, len, flags, addr, addrlen);
    thread_runs();
    return ret;
}

ssize_t
recvmsg(int fd, struct msghdr *msg, int flags)
{
    if (!is_host || (flags & MSG_DONTWAIT) || !fd_blocks(fd))
        return REAL(recvmsg)(fd, msg, flags);
    thread_blocks(NO_DEADLINE);
    ssize_t ret = REAL(recvmsg)(fd, msg, flags);
    thread_runs();
    return ret;
}

int
accept4(int fd, struct sockaddr *addr, socklen_t *addrlen, int flags)
{
    if (!is_host || !fd_blocks(fd))
        return REAL(accept4)(fd, addr, addrlen, flags);
    thread_blocks(NO_DEADLINE);
    int ret = REAL(accept4)(fd, addr, addrlen, flags);
    thread_runs();
    return ret;
}

int
accept(int fd, struct sockaddr *addr, socklen_t *addrlen)
{
    return accept4(fd, addr, addrlen, 0);
}

int
pthread_join(pthread_t thread, void **retval)
{
    thread_blocks(NO_DEADLINE);
    int ret = REAL(pthread_join)(thread, retval);
    thread_runs();
    return ret;
}

struct host_thread_start {
    void *(*fn)(void *);
    void *arg;
};

static void *
host_thread_main(void *p)
{
    struct host_thread_start start = *(struct host_thread_start *)p;
    free(p);
    void *ret = start.fn(start.arg);
    host_threads_add(-1);
    return ret;
}

// Every host thread counts as running from creation until it exits; the
// creator counts it before it can start so the host never looks idle in
// between.
int
pthread_create(pthread_t *thread, const pthread_attr_t *attr,
               void *(*fn)(void *), void *arg)
{
    if (!is_host)
        return REAL(pthread_create)(thread, attr, fn, arg);
    struct host_thread_start *start = malloc(sizeof(*start));
    if (!start)
        return EAGAIN;
    start->fn = fn;
    start->arg = arg;
    host_threads_add(1);
    int ret = REAL(pthread_create)(thread, attr, host_thread_main, start);
    if (ret) {
        host_threads_add(-1);
        free(start);
    }
    return ret;
}

void
pthread_exit(void *retval)
{
    host_threads_add(-1);
    REAL(pthread_exit)(retval);
    __builtin_unreachable();
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
