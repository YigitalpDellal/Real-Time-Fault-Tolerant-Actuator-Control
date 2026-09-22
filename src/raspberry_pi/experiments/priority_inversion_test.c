/*
 * Project:
 * Real-Time Safety-Critical Actuator Control & Fault-Injection Testbed
 *
 * Experiment:
 * Priority Inversion and Priority Inheritance
 *
 * Purpose:
 *
 * Demonstrate unbounded priority inversion using three SCHED_FIFO threads:
 *
 *   HIGH   priority 80
 *   MEDIUM priority 60
 *   LOW    priority 40
 *
 * Scenario:
 *
 * 1. LOW starts first and locks a shared mutex.
 * 2. HIGH becomes ready and attempts to lock the same mutex.
 * 3. HIGH blocks because LOW owns the mutex.
 * 4. MEDIUM becomes ready.
 *
 * Without priority inheritance:
 *
 *      MEDIUM preempts LOW.
 *      LOW cannot release the mutex.
 *      HIGH remains blocked.
 *
 * With PTHREAD_PRIO_INHERIT:
 *
 *      LOW temporarily inherits HIGH priority.
 *      MEDIUM can no longer preempt LOW.
 *      LOW finishes the critical section sooner.
 *      HIGH obtains the mutex much earlier.
 *
 * Important:
 *
 * Run this experiment pinned to ONE CPU core:
 *
 * sudo taskset -c 3 ./priority_inversion_test
 *
 * Otherwise HIGH, MEDIUM and LOW could execute simultaneously on
 * different CPU cores, destroying the intended single-processor
 * priority-inversion experiment.
 */

#define _GNU_SOURCE

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>

#include <time.h>
#include <pthread.h>
#include <sched.h>
#include <unistd.h>


/* ==========================================================================
 * Real-time priorities
 * ========================================================================== */

#define HIGH_PRIORITY       80
#define MEDIUM_PRIORITY     60
#define LOW_PRIORITY        40


/* ==========================================================================
 * Release offsets
 * ========================================================================== */

/*
 * LOW starts immediately.
 *
 * HIGH starts 10 ms later and blocks on LOW's mutex.
 *
 * MEDIUM starts 20 ms after experiment start.
 */
#define LOW_RELEASE_NS          0LL
#define HIGH_RELEASE_NS  10000000LL
#define MEDIUM_RELEASE_NS 20000000LL


/* ==========================================================================
 * CPU workloads
 * ========================================================================== */

/*
 * LOW must perform 80 ms of actual CPU work while holding the mutex.
 *
 * MEDIUM performs 150 ms of CPU work.
 *
 * CPU time is deliberately used instead of wall-clock time.
 *
 * Therefore, if LOW gets preempted by MEDIUM, LOW's workload does
 * NOT magically finish while it is not executing.
 */
#define LOW_CRITICAL_CPU_NS     80000000LL
#define MEDIUM_CPU_NS          150000000LL


/* ==========================================================================
 * Trial type
 * ========================================================================== */

typedef enum
{
    TRIAL_NO_INHERITANCE = 0,
    TRIAL_PRIORITY_INHERITANCE

} trial_type_t;


/* ==========================================================================
 * Experiment result structure
 * ========================================================================== */

typedef struct
{
    double low_mutex_hold_ms;

    double high_blocking_ms;

    double medium_wall_time_ms;

    double low_lock_time_ms;
    double high_request_time_ms;
    double high_acquire_time_ms;
    double medium_start_time_ms;
    double medium_finish_time_ms;
    double low_unlock_time_ms;

} trial_result_t;


/* ==========================================================================
 * Thread context
 * ========================================================================== */

typedef struct
{
    pthread_mutex_t *shared_mutex;

    struct timespec experiment_start;

    trial_result_t *result;

} thread_context_t;


/* ==========================================================================
 * Time helpers
 * ========================================================================== */

static int64_t TimespecToNs(
    const struct timespec *time_value
)
{
    return
        ((int64_t)time_value->tv_sec *
         1000000000LL) +
        (int64_t)time_value->tv_nsec;
}


static void AddNs(
    struct timespec *time_value,
    int64_t nanoseconds
)
{
    time_value->tv_nsec +=
        nanoseconds;


    while(time_value->tv_nsec >=
          1000000000L)
    {
        time_value->tv_nsec -=
            1000000000L;

        time_value->tv_sec++;
    }
}


static int64_t MonotonicNowNs(void)
{
    struct timespec current_time;


    clock_gettime(
        CLOCK_MONOTONIC,
        &current_time
    );


    return
        TimespecToNs(
            &current_time
        );
}


static double ElapsedMs(
    const struct timespec *origin
)
{
    return
        (
            MonotonicNowNs() -
            TimespecToNs(origin)
        ) /
        1000000.0;
}


/* ==========================================================================
 * Absolute release helper
 * ========================================================================== */

static void SleepUntilOffset(
    const struct timespec *origin,
    int64_t offset_ns
)
{
    struct timespec release_time =
        *origin;


    AddNs(
        &release_time,
        offset_ns
    );


    int result;


    do
    {
        result =
            clock_nanosleep(
                CLOCK_MONOTONIC,
                TIMER_ABSTIME,
                &release_time,
                NULL
            );
    }
    while(result == EINTR);


    if(result != 0)
    {
        fprintf(
            stderr,
            "clock_nanosleep failed: %s\n",
            strerror(result)
        );
    }
}


/* ==========================================================================
 * CPU-time workload
 * ========================================================================== */

/*
 * Consume the requested amount of CPU time on the calling thread.
 *
 * CLOCK_THREAD_CPUTIME_ID is critical here.
 *
 * If the thread is preempted, its CPU-time clock stops advancing.
 * This allows us to observe real priority inversion.
 */
static void BurnThreadCpuTime(
    int64_t required_cpu_ns
)
{
    struct timespec start_time;
    struct timespec current_time;


    volatile uint64_t activity =
        0ULL;


    clock_gettime(
        CLOCK_THREAD_CPUTIME_ID,
        &start_time
    );


    const int64_t start_ns =
        TimespecToNs(
            &start_time
        );


    while(true)
    {
        /*
         * Perform some CPU activity so this is not merely
         * a clock-reading loop.
         */
        activity =
            (activity * 1664525ULL) +
            1013904223ULL;


        clock_gettime(
            CLOCK_THREAD_CPUTIME_ID,
            &current_time
        );


        const int64_t consumed_ns =
            TimespecToNs(
                &current_time
            ) -
            start_ns;


        if(consumed_ns >=
           required_cpu_ns)
        {
            break;
        }
    }


    (void)activity;
}


/* ==========================================================================
 * LOW-priority thread
 * ========================================================================== */

static void *LowThread(
    void *argument
)
{
    thread_context_t *context =
        (thread_context_t *)argument;


    /*
     * LOW is released first.
     */
    SleepUntilOffset(
        &context->experiment_start,
        LOW_RELEASE_NS
    );


    pthread_mutex_lock(
        context->shared_mutex
    );


    context->result->
        low_lock_time_ms =
            ElapsedMs(
                &context->experiment_start
            );


    const int64_t lock_start_ns =
        MonotonicNowNs();


    /*
     * LOW performs a long critical section while holding
     * the resource required by HIGH.
     */
    BurnThreadCpuTime(
        LOW_CRITICAL_CPU_NS
    );


    context->result->
        low_unlock_time_ms =
            ElapsedMs(
                &context->experiment_start
            );


    const int64_t lock_finish_ns =
        MonotonicNowNs();


    context->result->
        low_mutex_hold_ms =
            (
                lock_finish_ns -
                lock_start_ns
            ) /
            1000000.0;


    pthread_mutex_unlock(
        context->shared_mutex
    );


    return NULL;
}


/* ==========================================================================
 * HIGH-priority thread
 * ========================================================================== */

static void *HighThread(
    void *argument
)
{
    thread_context_t *context =
        (thread_context_t *)argument;


    /*
     * HIGH arrives after LOW already owns the mutex.
     */
    SleepUntilOffset(
        &context->experiment_start,
        HIGH_RELEASE_NS
    );


    context->result->
        high_request_time_ms =
            ElapsedMs(
                &context->experiment_start
            );


    const int64_t request_ns =
        MonotonicNowNs();


    /*
     * HIGH blocks here until LOW releases the mutex.
     */
    pthread_mutex_lock(
        context->shared_mutex
    );


    const int64_t acquire_ns =
        MonotonicNowNs();


    context->result->
        high_acquire_time_ms =
            ElapsedMs(
                &context->experiment_start
            );


    context->result->
        high_blocking_ms =
            (
                acquire_ns -
                request_ns
            ) /
            1000000.0;


    /*
     * HIGH only needs the shared resource briefly.
     */
    volatile uint32_t high_activity =
        0U;


    high_activity++;


    (void)high_activity;


    pthread_mutex_unlock(
        context->shared_mutex
    );


    return NULL;
}


/* ==========================================================================
 * MEDIUM-priority thread
 * ========================================================================== */

static void *MediumThread(
    void *argument
)
{
    thread_context_t *context =
        (thread_context_t *)argument;


    /*
     * MEDIUM arrives after HIGH has already blocked on LOW.
     */
    SleepUntilOffset(
        &context->experiment_start,
        MEDIUM_RELEASE_NS
    );


    context->result->
        medium_start_time_ms =
            ElapsedMs(
                &context->experiment_start
            );


    const int64_t start_ns =
        MonotonicNowNs();


    /*
     * MEDIUM does not require the mutex.
     *
     * Without inheritance, MEDIUM preempts LOW and indirectly
     * delays HIGH.
     */
    BurnThreadCpuTime(
        MEDIUM_CPU_NS
    );


    const int64_t finish_ns =
        MonotonicNowNs();


    context->result->
        medium_finish_time_ms =
            ElapsedMs(
                &context->experiment_start
            );


    context->result->
        medium_wall_time_ms =
            (
                finish_ns -
                start_ns
            ) /
            1000000.0;


    return NULL;
}


/* ==========================================================================
 * Real-time thread creation helper
 * ========================================================================== */

static int CreateRtThread(
    pthread_t *thread,
    void *(*thread_function)(void *),
    void *argument,
    int priority
)
{
    pthread_attr_t attribute;


    struct sched_param scheduling_parameter;


    int result;


    pthread_attr_init(
        &attribute
    );


    pthread_attr_setinheritsched(
        &attribute,
        PTHREAD_EXPLICIT_SCHED
    );


    pthread_attr_setschedpolicy(
        &attribute,
        SCHED_FIFO
    );


    memset(
        &scheduling_parameter,
        0,
        sizeof(scheduling_parameter)
    );


    scheduling_parameter.
        sched_priority =
            priority;


    pthread_attr_setschedparam(
        &attribute,
        &scheduling_parameter
    );


    result =
        pthread_create(
            thread,
            &attribute,
            thread_function,
            argument
        );


    pthread_attr_destroy(
        &attribute
    );


    return result;
}


/* ==========================================================================
 * CPU-affinity check
 * ========================================================================== */

static int GetAllowedCpuCount(void)
{
    cpu_set_t cpu_set;


    CPU_ZERO(
        &cpu_set
    );


    if(sched_getaffinity(
            0,
            sizeof(cpu_set),
            &cpu_set) != 0)
    {
        return -1;
    }


    return
        CPU_COUNT(
            &cpu_set
        );
}


/* ==========================================================================
 * Execute one priority-inversion trial
 * ========================================================================== */

static int RunTrial(
    trial_type_t trial_type,
    trial_result_t *result
)
{
    pthread_mutex_t shared_mutex;


    pthread_mutexattr_t mutex_attribute;


    pthread_t low_thread;
    pthread_t medium_thread;
    pthread_t high_thread;


    thread_context_t context;


    int protocol;


    memset(
        result,
        0,
        sizeof(*result)
    );


    /* ----------------------------------------------------------------------
     * Configure mutex protocol
     * ---------------------------------------------------------------------- */

    pthread_mutexattr_init(
        &mutex_attribute
    );


    if(trial_type ==
       TRIAL_PRIORITY_INHERITANCE)
    {
        protocol =
            PTHREAD_PRIO_INHERIT;
    }
    else
    {
        protocol =
            PTHREAD_PRIO_NONE;
    }


    int result_code =
        pthread_mutexattr_setprotocol(
            &mutex_attribute,
            protocol
        );


    if(result_code != 0)
    {
        fprintf(
            stderr,
            "pthread_mutexattr_setprotocol failed: %s\n",
            strerror(result_code)
        );


        pthread_mutexattr_destroy(
            &mutex_attribute
        );


        return -1;
    }


    result_code =
        pthread_mutex_init(
            &shared_mutex,
            &mutex_attribute
        );


    pthread_mutexattr_destroy(
        &mutex_attribute
    );


    if(result_code != 0)
    {
        fprintf(
            stderr,
            "pthread_mutex_init failed: %s\n",
            strerror(result_code)
        );


        return -1;
    }


    /* ----------------------------------------------------------------------
     * Establish common release origin
     * ---------------------------------------------------------------------- */

    clock_gettime(
        CLOCK_MONOTONIC,
        &context.experiment_start
    );


    /*
     * One second gives all threads enough time to be created
     * before LOW's release.
     */
    AddNs(
        &context.experiment_start,
        1000000000LL
    );


    context.shared_mutex =
        &shared_mutex;


    context.result =
        result;


    /* ----------------------------------------------------------------------
     * Create threads
     * ---------------------------------------------------------------------- */

    result_code =
        CreateRtThread(
            &low_thread,
            LowThread,
            &context,
            LOW_PRIORITY
        );


    if(result_code != 0)
    {
        fprintf(
            stderr,
            "Failed to create LOW: %s\n",
            strerror(result_code)
        );


        pthread_mutex_destroy(
            &shared_mutex
        );


        return -1;
    }


    result_code =
        CreateRtThread(
            &medium_thread,
            MediumThread,
            &context,
            MEDIUM_PRIORITY
        );


    if(result_code != 0)
    {
        fprintf(
            stderr,
            "Failed to create MEDIUM: %s\n",
            strerror(result_code)
        );


        return -1;
    }


    result_code =
        CreateRtThread(
            &high_thread,
            HighThread,
            &context,
            HIGH_PRIORITY
        );


    if(result_code != 0)
    {
        fprintf(
            stderr,
            "Failed to create HIGH: %s\n",
            strerror(result_code)
        );


        return -1;
    }


    /* ----------------------------------------------------------------------
     * Wait for experiment completion
     * ---------------------------------------------------------------------- */

    pthread_join(
        low_thread,
        NULL
    );


    pthread_join(
        medium_thread,
        NULL
    );


    pthread_join(
        high_thread,
        NULL
    );


    pthread_mutex_destroy(
        &shared_mutex
    );


    return 0;
}


/* ==========================================================================
 * Result printer
 * ========================================================================== */

static void PrintTrialResult(
    const char *title,
    const trial_result_t *result
)
{
    printf(
        "\n=== %s ===\n",
        title
    );


    printf(
        "LOW mutex acquired       : %8.3f ms\n",
        result->low_lock_time_ms
    );


    printf(
        "HIGH requested mutex     : %8.3f ms\n",
        result->high_request_time_ms
    );


    printf(
        "MEDIUM started           : %8.3f ms\n",
        result->medium_start_time_ms
    );


    printf(
        "MEDIUM finished          : %8.3f ms\n",
        result->medium_finish_time_ms
    );


    printf(
        "LOW released mutex       : %8.3f ms\n",
        result->low_unlock_time_ms
    );


    printf(
        "HIGH acquired mutex      : %8.3f ms\n",
        result->high_acquire_time_ms
    );


    printf(
        "\nLOW mutex hold wall time : %8.3f ms\n",
        result->low_mutex_hold_ms
    );


    printf(
        "HIGH blocking time       : %8.3f ms\n",
        result->high_blocking_ms
    );


    printf(
        "MEDIUM wall time         : %8.3f ms\n",
        result->medium_wall_time_ms
    );
}


/* ==========================================================================
 * Main
 * ========================================================================== */

int main(void)
{
    trial_result_t no_inheritance;
    trial_result_t priority_inheritance;


    printf(
        "=== PRIORITY INVERSION EXPERIMENT ===\n\n"
    );


    printf(
        "Scheduler priorities:\n"
    );


    printf(
        "  HIGH   : %d\n",
        HIGH_PRIORITY
    );


    printf(
        "  MEDIUM : %d\n",
        MEDIUM_PRIORITY
    );


    printf(
        "  LOW    : %d\n\n",
        LOW_PRIORITY
    );


    printf(
        "LOW critical CPU workload : %.1f ms\n",
        LOW_CRITICAL_CPU_NS /
            1000000.0
    );


    printf(
        "MEDIUM CPU workload       : %.1f ms\n\n",
        MEDIUM_CPU_NS /
            1000000.0
    );


    const int cpu_count =
        GetAllowedCpuCount();


    printf(
        "Allowed CPU count         : %d\n",
        cpu_count
    );


    if(cpu_count != 1)
    {
        printf(
            "[WARNING] Experiment should be pinned to exactly one CPU.\n"
        );


        printf(
            "Use: sudo taskset -c 3 ./priority_inversion_test\n"
        );
    }


    /* ----------------------------------------------------------------------
     * Trial 1: standard mutex
     * ---------------------------------------------------------------------- */

    printf(
        "\nRunning trial 1: PTHREAD_PRIO_NONE...\n"
    );


    if(RunTrial(
            TRIAL_NO_INHERITANCE,
            &no_inheritance) != 0)
    {
        fprintf(
            stderr,
            "\nTrial 1 failed.\n"
            "Run the program with sudo so SCHED_FIFO threads can be created.\n"
        );


        return 1;
    }


    /* ----------------------------------------------------------------------
     * Trial 2: priority-inheritance mutex
     * ---------------------------------------------------------------------- */

    /*
     * Small separation between trials.
     */
    usleep(
        500000
    );


    printf(
        "Running trial 2: PTHREAD_PRIO_INHERIT...\n"
    );


    if(RunTrial(
            TRIAL_PRIORITY_INHERITANCE,
            &priority_inheritance) != 0)
    {
        fprintf(
            stderr,
            "\nTrial 2 failed.\n"
        );


        return 1;
    }


    /* ----------------------------------------------------------------------
     * Print measured data
     * ---------------------------------------------------------------------- */

    PrintTrialResult(
        "WITHOUT PRIORITY INHERITANCE",
        &no_inheritance
    );


    PrintTrialResult(
        "WITH PRIORITY INHERITANCE",
        &priority_inheritance
    );


    /* ----------------------------------------------------------------------
     * Comparison
     * ---------------------------------------------------------------------- */

    printf(
        "\n=== COMPARISON ===\n"
    );


    printf(
        "HIGH blocking without inheritance : %.3f ms\n",
        no_inheritance.
            high_blocking_ms
    );


    printf(
        "HIGH blocking with inheritance    : %.3f ms\n",
        priority_inheritance.
            high_blocking_ms
    );


    const double blocking_reduction_ms =
        no_inheritance.high_blocking_ms -
        priority_inheritance.high_blocking_ms;


    printf(
        "Blocking-time reduction           : %.3f ms\n",
        blocking_reduction_ms
    );


    if(no_inheritance.high_blocking_ms >
       0.0)
    {
        const double reduction_percent =
            (
                blocking_reduction_ms /
                no_inheritance.
                    high_blocking_ms
            ) *
            100.0;


        printf(
            "Blocking reduction               : %.2f %%\n",
            reduction_percent
        );
    }


    /*
     * The important experimental result:
     *
     * priority inheritance should substantially reduce HIGH's
     * mutex blocking time.
     */
    if(priority_inheritance.high_blocking_ms <
       no_inheritance.high_blocking_ms)
    {
        printf(
            "\n[PASS] Priority inheritance reduced "
            "high-priority blocking.\n"
        );
    }
    else
    {
        printf(
            "\n[CHECK] Expected blocking reduction was not observed.\n"
        );
    }


    return 0;
}
