
#define _GNU_SOURCE
#include "vtime.h"

#include <dlfcn.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/futex.h>
#include <math.h>
#include <poll.h>
#include <pthread.h>
#include <semaphore.h>
#include <signal.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/epoll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <sys/sysmacros.h>
#include <sys/uio.h>
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
static ssize_t (*real_write)(int, const void *, size_t);
static ssize_t (*real_writev)(int, const struct iovec *, int);
static ssize_t (*real_send)(int, const void *, size_t, int);
static ssize_t (*real_sendto)(int, const void *, size_t, int,
                              const struct sockaddr *, socklen_t);
static ssize_t (*real_sendmsg)(int, const struct msghdr *, int);
static int (*real_pthread_cond_signal)(pthread_cond_t *);
static int (*real_pthread_cond_broadcast)(pthread_cond_t *);
static int (*real_sem_post)(sem_t *);
static int (*real_openpty)(int *, int *, char *, const struct termios *,
                           const struct winsize *);

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

#define NO_DEADLINE UINT64_MAX
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

// The process's own oscillator: VTIME_CRYSTAL_PPM makes every clock this
// process reads run that many ppm fast against the shared virtual timeline,
// as a real MCU crystal does against the host's. Times the process hands
// in (deadlines, pacer floors) are on its own clock and map back.
static double crystal_scale = 1.0;

static uint64_t
local_ns(uint64_t virtual_ns)
{
    if (crystal_scale == 1.0 || virtual_ns == NO_DEADLINE)
        return virtual_ns;
    return (uint64_t)llround((double)virtual_ns * crystal_scale);
}

static uint64_t
virtual_ns(uint64_t local)
{
    if (crystal_scale == 1.0 || local == NO_DEADLINE)
        return local;
    return (uint64_t)ceil((double)local / crystal_scale);
}

static uint64_t
local_now(void)
{
    return local_ns(vtime_now());
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

static int is_host;

static int
process_gone(uint32_t pid)
{
    return kill((pid_t)pid, 0) < 0 && errno == ESRCH;
}

static void
clock_lock(void)
{
    int saved_errno = errno;
    uint32_t self = (uint32_t)getpid();
    for (unsigned spins = 1;; spins++) {
        uint32_t owner = 0;
        if (atomic_compare_exchange_weak_explicit(&vshm->clock_lock_owner,
                                                  &owner, self,
                                                  memory_order_acquire,
                                                  memory_order_relaxed))
            break;
        if (owner && spins % 1024 == 0 && process_gone(owner))
            atomic_compare_exchange_strong(&vshm->clock_lock_owner, &owner,
                                           0);
        sched_yield();
    }
    errno = saved_errno;
}

static void
clock_unlock(void)
{
    atomic_store_explicit(&vshm->clock_lock_owner, 0, memory_order_release);
}

static uint32_t stat_cache_pid;
static uint32_t stat_cache_tid[VTIME_MAX_HOST_THREADS];
static int stat_cache_fd[VTIME_MAX_HOST_THREADS];

static void
reset_stat_cache(uint32_t pid)
{
    for (int i = 0; i < VTIME_MAX_HOST_THREADS; i++) {
        if (stat_cache_tid[i] && stat_cache_fd[i] >= 0)
            close(stat_cache_fd[i]);
        stat_cache_tid[i] = 0;
    }
    stat_cache_pid = pid;
}

static int
host_thread_runnable(int slot, uint32_t pid, uint32_t tid)
{
    if (stat_cache_tid[slot] != tid) {
        if (stat_cache_tid[slot] && stat_cache_fd[slot] >= 0)
            close(stat_cache_fd[slot]);
        char path[64];
        snprintf(path, sizeof(path), "/proc/%u/task/%u/stat", pid, tid);
        stat_cache_fd[slot] = open(path, O_RDONLY | O_CLOEXEC);
        stat_cache_tid[slot] = tid;
    }
    if (stat_cache_fd[slot] < 0)
        return 0;
    char stat[512];
    ssize_t n = pread(stat_cache_fd[slot], stat, sizeof(stat) - 1, 0);
    if (n <= 0)
        return 0;
    stat[n] = '\0';
    char *comm_end = strrchr(stat, ')');
    return comm_end && comm_end[1] == ' ' && comm_end[2] == 'R';
}

static int
host_has_runnable_waiter(void)
{
    uint32_t pid = atomic_load_explicit(&vshm->host_pid, memory_order_acquire);
    if (pid != stat_cache_pid)
        reset_stat_cache(pid);
    uint32_t self = is_host ? (uint32_t)gettid() : 0;
    for (int i = 0; i < VTIME_MAX_HOST_THREADS; i++) {
        uint32_t tid = atomic_load_explicit(&vshm->host_tid[i],
                                            memory_order_acquire);
        if (tid && tid != self && host_thread_runnable(i, pid, tid))
            return 1;
    }
    return 0;
}

static int
host_idle(void)
{
    if (!atomic_load_explicit(&vshm->host_present, memory_order_acquire))
        return 1;
    if (atomic_load_explicit(&vshm->host_running, memory_order_acquire) > 0)
        return 0;
    uint64_t epoch = atomic_load_explicit(&vshm->host_wake_epoch,
                                          memory_order_acquire);
    if (epoch == atomic_load_explicit(&vshm->host_idle_epoch,
                                      memory_order_acquire))
        return 1;
    if (host_has_runnable_waiter())
        return 0;
    atomic_store_explicit(&vshm->host_idle_epoch, epoch, memory_order_release);
    return 1;
}

static void
host_may_have_woken_a_thread(void)
{
    if (is_host && vshm)
        atomic_fetch_add_explicit(&vshm->host_wake_epoch, 1,
                                  memory_order_acq_rel);
}

static uint32_t
link_slots_used(void)
{
    uint32_t used = atomic_load_explicit(&vshm->link_slots_used,
                                         memory_order_acquire);
    return used > VTIME_MAX_LINKS ? VTIME_MAX_LINKS : used;
}

static int device_link_fd = -1;
static int device_link_slot = -1;
static __thread int device_handling;
static __thread int stepping_own_timer;
static __thread int reads_device_link;

static int
device_input_pending(void)
{
    if (atomic_load_explicit(&vshm->devices_handling, memory_order_acquire))
        return 1;
    uint32_t used = link_slots_used();
    for (uint32_t i = 0; i < used; i++) {
        struct vtime_device_link *link = &vshm->links[i];
        if (stepping_own_timer && (int)i == device_link_slot)
            continue;
        if (atomic_load_explicit(&link->owner_stepping_own_timer,
                                 memory_order_acquire))
            continue;
        if (atomic_load_explicit(&link->pts_plus_one, memory_order_acquire)
            && atomic_load_explicit(&link->written, memory_order_acquire)
                   > atomic_load_explicit(&link->consumed,
                                          memory_order_acquire))
            return 1;
    }
    return 0;
}

static int
pts_of(const struct stat *st, uint32_t *pts)
{
    unsigned int major_num = major(st->st_rdev);
    if (!S_ISCHR(st->st_mode) || major_num < 136 || major_num > 143)
        return 0;
    *pts = (major_num - 136) * 256 + minor(st->st_rdev);
    return 1;
}

static void
host_wrote(int fd, uint64_t bytes)
{
    int saved_errno = errno;
    struct stat st;
    uint32_t pts;
    if (fstat(fd, &st) == 0) {
        if (pts_of(&st, &pts)) {
            uint32_t used = link_slots_used();
            for (uint32_t i = 0; i < used; i++)
                if (atomic_load_explicit(&vshm->links[i].pts_plus_one,
                                         memory_order_acquire)
                    == pts + 1) {
                    atomic_fetch_add_explicit(&vshm->links[i].written, bytes,
                                              memory_order_acq_rel);
                    VLOG("host fd %d wrote %lu bytes to device pts%u", fd,
                         (unsigned long)bytes, pts);
                }
        } else if (!S_ISREG(st.st_mode)) {
            host_may_have_woken_a_thread();
        }
    }
    errno = saved_errno;
}

static void
register_device_link(int master_fd)
{
    if (device_link_fd >= 0) {
        fprintf(stderr, "[vtime] device process opened a second pty\n");
        abort();
    }
    unsigned int pts;
    if (ioctl(master_fd, TIOCGPTN, &pts)) {
        fprintf(stderr, "[vtime] TIOCGPTN failed: %s\n", strerror(errno));
        abort();
    }
    uint32_t slot = atomic_fetch_add_explicit(&vshm->link_slots_used, 1,
                                              memory_order_acq_rel);
    if (slot >= VTIME_MAX_LINKS) {
        fprintf(stderr, "[vtime] more than %d device links\n",
                VTIME_MAX_LINKS);
        abort();
    }
    struct vtime_device_link *link = &vshm->links[slot];
    atomic_store_explicit(&link->written, 0, memory_order_release);
    atomic_store_explicit(&link->consumed, 0, memory_order_release);
    atomic_store_explicit(&link->pts_plus_one, pts + 1, memory_order_release);
    device_link_fd = master_fd;
    device_link_slot = (int)slot;
    VLOG("device link: master fd %d is pts%u, slot %u", master_fd, pts, slot);
}

static void
device_read(uint64_t bytes)
{
    reads_device_link = 1;
    if (!device_handling) {
        device_handling = 1;
        atomic_fetch_add_explicit(&vshm->devices_handling, 1,
                                  memory_order_acq_rel);
    }
    atomic_fetch_add_explicit(&vshm->links[device_link_slot].consumed, bytes,
                              memory_order_acq_rel);
    VLOG("device read %lu bytes", (unsigned long)bytes);
}

static void
finish_device_handling(void)
{
    if (!device_handling)
        return;
    device_handling = 0;
    atomic_fetch_sub_explicit(&vshm->devices_handling, 1,
                              memory_order_acq_rel);
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

static uint64_t
locked_vtime_bound(void)
{
    uint64_t bound = vtime_cap();
    uint64_t floor = pacer_floor_min();
    if (floor < bound)
        bound = floor;
    floor = host_wait_floor();
    if (floor < bound)
        bound = floor;
    uint64_t now = vtime_now();
    if (bound > now && (device_input_pending() || !host_idle()))
        return now;
    return bound;
}

static uint64_t
raise_toward(uint64_t target)
{
    clock_lock();
    uint64_t bound = locked_vtime_bound();
    uint64_t step = target < bound ? target : bound;
    int raised = step > vtime_now();
    if (raised)
        atomic_store_explicit(&vshm->nanos, step, memory_order_release);
    clock_unlock();
    if (raised)
        kick_due_waiters(step);
    return bound;
}

static void
store_pacer_floor(int slot, uint64_t floor_ns)
{
    uint64_t old = atomic_load_explicit(&vshm->pacers[slot].floor_ns,
                                        memory_order_acquire);
    if (floor_ns >= old) {
        atomic_store_explicit(&vshm->pacers[slot].floor_ns, floor_ns,
                              memory_order_release);
        return;
    }
    clock_lock();
    atomic_store_explicit(&vshm->pacers[slot].floor_ns, floor_ns,
                          memory_order_release);
    clock_unlock();
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

static __thread uint64_t blocked_until;
static uint64_t test_woken_spin_ns;

static void
spin_real(uint64_t ns)
{
    uint64_t end = real_monotonic_ns() + ns;
    while (real_monotonic_ns() < end)
        ;
}

static void
thread_blocks(uint64_t deadline)
{
    if (!vshm)
        return;
    finish_device_handling();
    blocked_until = deadline;
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
    if (is_host) {
        if (test_woken_spin_ns && vtime_now() < blocked_until)
            spin_real(test_woken_spin_ns);
        clock_lock();
        atomic_fetch_add_explicit(&vshm->host_running, 1,
                                  memory_order_acq_rel);
        clock_unlock();
    }
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
    finish_device_handling();
    struct vtime_device_link *own_link =
        device_link_slot >= 0 && reads_device_link
            ? &vshm->links[device_link_slot]
            : NULL;
    if (own_link)
        atomic_store_explicit(&own_link->owner_stepping_own_timer, 1,
                              memory_order_release);
    stepping_own_timer = 1;
    while (vtime_now() < target_ns) {
        raise_toward(target_ns);
        if (vtime_now() >= target_ns)
            break;
        struct timespec w = { 0, 100000 };
        real_nanosleep(&w, NULL);
    }
    stepping_own_timer = 0;
    if (own_link)
        atomic_store_explicit(&own_link->owner_stepping_own_timer, 0,
                              memory_order_release);
}

static void vtimer_check_and_fire(void);

#define WAIT_SLICE_MIN_NS 50000ULL
#define WAIT_SLICE_MAX_NS 20000000ULL
#define WAIT_SLICE_HELD_NS WAIT_SLICE_MIN_NS
#define HOST_WAIT_SLICE_HELD_NS 1000000ULL

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
    uint64_t bound = raise_toward(target);
    vtimer_check_and_fire();
    uint64_t now = vtime_now();
    if (now >= deadline)
        return 0;
    if (bound < cap)
        return is_host ? HOST_WAIT_SLICE_HELD_NS : WAIT_SLICE_HELD_NS;
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
    uint64_t now = local_now();
    return delta_ns > NO_DEADLINE - now ? NO_DEADLINE
                                        : virtual_ns(now + delta_ns);
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
        uint64_t bound = raise_toward(UINT64_MAX);
        if (bound < cap && cap != UINT64_MAX)
            atomic_fetch_add_explicit(&vtime_stall_offset_ns, cap - bound,
                                      memory_order_acq_rel);
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
    clock_lock();
    atomic_store_explicit(&vshm->pacers[slot].slack_ns, slack_ns,
                          memory_order_release);
    atomic_store_explicit(&vshm->pacers[slot].floor_ns,
                          deadline_after(period_ns), memory_order_release);
    atomic_store_explicit(&vshm->pacers[slot].active, 1,
                          memory_order_release);
    clock_unlock();
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
    finish_device_handling();
    store_pacer_floor(slot, virtual_ns(target_ns));
    vtime_advance_to(virtual_ns(target_ns));
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
    store_pacer_floor(slot, virtual_ns(target_ns));
}

// The shared timeline itself, for the step-tracking shim: step timestamps
// are compared across processes, so they must not carry this process's
// crystal error.
__attribute__((visibility("default"))) uint64_t
vtime_shared_now_ns(void)
{
    return vshm ? vtime_now() : 0;
}

static __thread int host_tid_slot = -1;

static void
register_host_thread(void)
{
    uint32_t tid = (uint32_t)gettid();
    for (int i = 0; i < VTIME_MAX_HOST_THREADS; i++) {
        uint32_t free_slot = 0;
        if (atomic_compare_exchange_strong(&vshm->host_tid[i], &free_slot,
                                           tid)) {
            host_tid_slot = i;
            return;
        }
    }
    fprintf(stderr, "[vtime] more than %d host threads\n",
            VTIME_MAX_HOST_THREADS);
    abort();
}

static void
unregister_host_thread(void)
{
    if (host_tid_slot < 0)
        return;
    atomic_store_explicit(&vshm->host_tid[host_tid_slot], 0,
                          memory_order_release);
    host_tid_slot = -1;
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
    finish_device_handling();
    if (device_link_slot >= 0)
        atomic_store_explicit(&vshm->links[device_link_slot].pts_plus_one, 0,
                              memory_order_release);
}

static void
forked_child(void)
{
    is_host = 0;
    waiter_slot = -1;
    host_tid_slot = -1;
    blocked_until = 0;
    device_handling = 0;
    device_link_fd = -1;
    device_link_slot = -1;
    memset(owned_pacer_slots, 0, sizeof(owned_pacer_slots));
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

    const char *crystal_ppm = getenv("VTIME_CRYSTAL_PPM");
    if (crystal_ppm)
        crystal_scale = 1.0 + atof(crystal_ppm) * 1e-6;

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
    const char *spin_env = getenv("VTIME_TEST_WOKEN_SPIN_US");
    if (spin_env)
        test_woken_spin_ns = strtoull(spin_env, NULL, 10) * 1000ULL;

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
    pthread_atfork(NULL, NULL, forked_child);

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
        for (int i = 0; i < VTIME_MAX_HOST_THREADS; i++)
            atomic_store(&vshm->host_tid[i], 0);
        atomic_store(&vshm->host_pid, (uint32_t)getpid());
        atomic_fetch_add(&vshm->host_wake_epoch, 1);
        register_host_thread();
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
    ns_to_ts(local_now(), tp);
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
    uint64_t now = local_now();
    uint64_t end = local_ns(deadline);
    ns_to_ts(end > now ? end - now : 0, rem);
}

int
clock_nanosleep(clockid_t clk_id, int flags, const struct timespec *req,
                struct timespec *rem)
{
    if (!vshm || !is_monotonic(clk_id))
        return real_clock_nanosleep(clk_id, flags, req, rem);
    uint64_t deadline = flags & TIMER_ABSTIME ? virtual_ns(ts_to_ns(req))
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

extern void __chk_fail(void) __attribute__((noreturn));

int
__ppoll_chk(struct pollfd *fds, nfds_t nfds, const struct timespec *timeout,
            const sigset_t *sigmask, size_t fdslen)
{
    if (fdslen / sizeof(*fds) < nfds)
        __chk_fail();
    return ppoll(fds, nfds, timeout, sigmask);
}

int
__poll_chk(struct pollfd *fds, nfds_t nfds, int timeout_ms, size_t fdslen)
{
    if (fdslen / sizeof(*fds) < nfds)
        __chk_fail();
    return poll(fds, nfds, timeout_ms);
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
        || (cmd != FUTEX_WAIT && cmd != FUTEX_WAIT_BITSET)) {
        long woke = real_syscall(number, a1, a2, a3, a4, a5, a6);
        if (vshm && number == SYS_futex && woke > 0
            && (cmd == FUTEX_WAKE || cmd == FUTEX_WAKE_BITSET
                || cmd == FUTEX_WAKE_OP || cmd == FUTEX_REQUEUE
                || cmd == FUTEX_CMP_REQUEUE))
            host_may_have_woken_a_thread();
        return woke;
    }
    const struct timespec *timeout = (const struct timespec *)a4;
    long ret;
    if (timeout && !(op & FUTEX_CLOCK_REALTIME)) {
        uint64_t deadline = cmd == FUTEX_WAIT
                                ? deadline_after(ts_to_ns(timeout))
                                : virtual_ns(ts_to_ns(timeout));
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

int
pthread_cond_signal(pthread_cond_t *cond)
{
    host_may_have_woken_a_thread();
    return REAL(pthread_cond_signal)(cond);
}

int
pthread_cond_broadcast(pthread_cond_t *cond)
{
    host_may_have_woken_a_thread();
    return REAL(pthread_cond_broadcast)(cond);
}

int
sem_post(sem_t *sem)
{
    host_may_have_woken_a_thread();
    return REAL(sem_post)(sem);
}

// CPython's GIL hand-off waits on a CLOCK_MONOTONIC condvar; glibc records
// the condvar's clock in bit 1 of __wrefs.
int
pthread_cond_timedwait(pthread_cond_t *cond, pthread_mutex_t *mutex,
                       const struct timespec *abstime)
{
    int monotonic = vshm && (cond->__data.__wrefs & 2);
    thread_blocks(monotonic ? virtual_ns(ts_to_ns(abstime)) : NO_DEADLINE);
    int ret = monotonic ? cond_wait_loop(cond, mutex,
                                         virtual_ns(ts_to_ns(abstime)))
                        : REAL(pthread_cond_timedwait)(cond, mutex, abstime);
    thread_runs();
    return ret;
}

int
pthread_cond_clockwait(pthread_cond_t *cond, pthread_mutex_t *mutex,
                       clockid_t clockid, const struct timespec *abstime)
{
    int monotonic = vshm && is_monotonic(clockid);
    thread_blocks(monotonic ? virtual_ns(ts_to_ns(abstime)) : NO_DEADLINE);
    int ret = monotonic ? cond_wait_loop(cond, mutex,
                                         virtual_ns(ts_to_ns(abstime)))
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
    thread_blocks(monotonic ? virtual_ns(ts_to_ns(abstime)) : NO_DEADLINE);
    int ret = monotonic ? sem_wait_loop(sem, virtual_ns(ts_to_ns(abstime)))
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
    if (!is_host) {
        ssize_t ret = REAL(read)(fd, buf, count);
        if (ret > 0 && fd == device_link_fd)
            device_read((uint64_t)ret);
        return ret;
    }
    if (!fd_blocks(fd))
        return REAL(read)(fd, buf, count);
    thread_blocks(NO_DEADLINE);
    ssize_t ret = REAL(read)(fd, buf, count);
    thread_runs();
    return ret;
}

ssize_t
__read_chk(int fd, void *buf, size_t count, size_t buflen)
{
    if (count > buflen)
        __chk_fail();
    return read(fd, buf, count);
}

ssize_t
write(int fd, const void *buf, size_t count)
{
    ssize_t ret = REAL(write)(fd, buf, count);
    if (is_host && vshm && ret > 0)
        host_wrote(fd, (uint64_t)ret);
    return ret;
}

ssize_t
writev(int fd, const struct iovec *iov, int iovcnt)
{
    ssize_t ret = REAL(writev)(fd, iov, iovcnt);
    if (is_host && vshm && ret > 0)
        host_wrote(fd, (uint64_t)ret);
    return ret;
}

ssize_t
send(int fd, const void *buf, size_t len, int flags)
{
    ssize_t ret = REAL(send)(fd, buf, len, flags);
    if (is_host && vshm && ret > 0)
        host_may_have_woken_a_thread();
    return ret;
}

ssize_t
sendto(int fd, const void *buf, size_t len, int flags,
       const struct sockaddr *addr, socklen_t addrlen)
{
    ssize_t ret = REAL(sendto)(fd, buf, len, flags, addr, addrlen);
    if (is_host && vshm && ret > 0)
        host_may_have_woken_a_thread();
    return ret;
}

ssize_t
sendmsg(int fd, const struct msghdr *msg, int flags)
{
    ssize_t ret = REAL(sendmsg)(fd, msg, flags);
    if (is_host && vshm && ret > 0)
        host_may_have_woken_a_thread();
    return ret;
}

int
openpty(int *amaster, int *aslave, char *name, const struct termios *termp,
        const struct winsize *winp)
{
    int ret = REAL(openpty)(amaster, aslave, name, termp, winp);
    if (ret == 0 && vshm && !is_host)
        register_device_link(*amaster);
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
    register_host_thread();
    void *ret = start.fn(start.arg);
    unregister_host_thread();
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
    if (is_host)
        unregister_host_thread();
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
        vtimer.target_ns = virtual_ns(ts_to_ns(&new_value->it_value));
        vtimer.armed = 1;
    } else {
        vtimer.target_ns = deadline_after(ts_to_ns(&new_value->it_value));
        vtimer.armed = 1;
    }
    pthread_mutex_unlock(&vtimer.lock);

    vtimer_check_and_fire();

    return 0;
}
