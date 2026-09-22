/*
 * Project:
 * Real-Time Safety-Critical Actuator Control & Fault-Injection Testbed
 *
 * Experiment:
 * Rate Monotonic vs Deadline Monotonic Fixed-Priority Scheduling
 *
 * Purpose:
 * - Compare RM and DM using the exact same periodic task set.
 * - Demonstrate why relative deadlines can affect fixed priorities.
 * - Measure response time and deadline misses experimentally.
 *
 * Task set:
 *
 * Task       Execution    Period      Deadline
 * ------------------------------------------------
 * FAST_A       8 ms        40 ms        40 ms
 * URGENT_B     8 ms        50 ms        12 ms
 * SLOW_C       4 ms       100 ms       100 ms
 *
 * RM priority:
 * shorter period -> higher priority
 *
 *   FAST_A   = 80
 *   URGENT_B = 70
 *   SLOW_C   = 60
 *
 * DM priority:
 * shorter relative deadline -> higher priority
 *
 *   URGENT_B = 80
 *   FAST_A   = 70
 *   SLOW_C   = 60
 *
 * Run on ONE CPU:
 *
 * sudo taskset -c 3 ./rm_vs_dm_test
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
 * Experiment configuration
 * ========================================================================== */

#define TASK_COUNT                  3U
#define EXPERIMENT_DURATION_NS      5000000000LL


/* ==========================================================================
 * Scheduling mode
 * ========================================================================== */

typedef enum
{
    POLICY_RM = 0,
    POLICY_DM

} scheduling_mode_t;


/* ==========================================================================
 * Task model
 * ========================================================================== */

typedef enum
{
    TASK_FAST_A = 0,
    TASK_URGENT_B,
    TASK_SLOW_C

} task_id_t;


typedef struct
{
    task_id_t id;

    const char *name;

    int64_t execution_ns;
    int64_t period_ns;
    int64_t deadline_ns;

    int priority;

    uint32_t cycles;

    double average_jitter_us;
    double worst_jitter_us;

    double average_execution_us;
    double worst_execution_us;

    double average_response_us;
    double worst_response_us;

    uint32_t deadline_misses;

} periodic_task_t;


/* ==========================================================================
 * Trial context
 * ========================================================================== */

typedef struct
{
    periodic_task_t tasks[TASK_COUNT];

    struct timespec start_time;

    scheduling_mode_t mode;

} experiment_t;


/* ==========================================================================
 * Timing helpers
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
    struct timespec now;


    clock_gettime(
        CLOCK_MONOTONIC,
        &now
    );


    return
        TimespecToNs(
            &now
        );
}


/* ==========================================================================
 * CPU workload
 * ========================================================================== */

/*
 * Consume actual CPU time rather than wall-clock time.
 *
 * If the task is preempted, CLOCK_THREAD_CPUTIME_ID stops advancing.
 * Therefore measured interference comes from scheduling rather than
 * the workload silently completing while the task is not running.
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
 * Task initialization
 * ========================================================================== */

static void InitializeTaskSet(
    experiment_t *experiment,
    scheduling_mode_t mode
)
{
    memset(
        experiment,
        0,
        sizeof(*experiment)
    );


    experiment->mode =
        mode;


    /* ----------------------------------------------------------------------
     * FAST_A
     * ---------------------------------------------------------------------- */

    experiment->tasks[TASK_FAST_A].id =
        TASK_FAST_A;

    experiment->tasks[TASK_FAST_A].name =
        "FAST_A";

    experiment->tasks[TASK_FAST_A].execution_ns =
        8000000LL;

    experiment->tasks[TASK_FAST_A].period_ns =
        40000000LL;

    experiment->tasks[TASK_FAST_A].deadline_ns =
        40000000LL;


    /* ----------------------------------------------------------------------
     * URGENT_B
     *
     * Important:
     *
     * Its deadline is much shorter than its period.
     * This is what causes RM and DM to choose different priorities.
     * ---------------------------------------------------------------------- */

    experiment->tasks[TASK_URGENT_B].id =
        TASK_URGENT_B;

    experiment->tasks[TASK_URGENT_B].name =
        "URGENT_B";

    experiment->tasks[TASK_URGENT_B].execution_ns =
        8000000LL;

    experiment->tasks[TASK_URGENT_B].period_ns =
        50000000LL;

    experiment->tasks[TASK_URGENT_B].deadline_ns =
        12000000LL;


    /* ----------------------------------------------------------------------
     * SLOW_C
     * ---------------------------------------------------------------------- */

    experiment->tasks[TASK_SLOW_C].id =
        TASK_SLOW_C;

    experiment->tasks[TASK_SLOW_C].name =
        "SLOW_C";

    experiment->tasks[TASK_SLOW_C].execution_ns =
        4000000LL;

    experiment->tasks[TASK_SLOW_C].period_ns =
        100000000LL;

    experiment->tasks[TASK_SLOW_C].deadline_ns =
        100000000LL;


    /* ----------------------------------------------------------------------
     * Assign fixed priorities
     * ---------------------------------------------------------------------- */

    if(mode ==
       POLICY_RM)
    {
        /*
         * Rate Monotonic:
         *
         * 40 ms < 50 ms < 100 ms
         */
        experiment->tasks[TASK_FAST_A].priority =
            80;

        experiment->tasks[TASK_URGENT_B].priority =
            70;

        experiment->tasks[TASK_SLOW_C].priority =
            60;
    }
    else
    {
        /*
         * Deadline Monotonic:
         *
         * 12 ms < 40 ms < 100 ms
         */
        experiment->tasks[TASK_URGENT_B].priority =
            80;

        experiment->tasks[TASK_FAST_A].priority =
            70;

        experiment->tasks[TASK_SLOW_C].priority =
            60;
    }


    /*
     * Calculate number of releases in five seconds.
     */
    uint32_t i;


    for(i = 0U;
        i < TASK_COUNT;
        i++)
    {
        experiment->tasks[i].cycles =
            (uint32_t)(
                EXPERIMENT_DURATION_NS /
                experiment->tasks[i].
                    period_ns
            );
    }
}


/* ==========================================================================
 * Periodic task thread
 * ========================================================================== */

typedef struct
{
    periodic_task_t *task;

    struct timespec start_time;

} task_context_t;


static void *PeriodicTaskThread(
    void *argument
)
{
    task_context_t *context =
        (task_context_t *)argument;


    periodic_task_t *task =
        context->task;


    struct timespec next_release =
        context->start_time;


    int64_t total_jitter_ns =
        0;

    int64_t total_execution_ns =
        0;

    int64_t total_response_ns =
        0;


    int64_t worst_jitter_ns =
        0;

    int64_t worst_execution_ns =
        0;

    int64_t worst_response_ns =
        0;


    uint32_t deadline_misses =
        0U;


    uint32_t cycle;


    for(cycle = 1U;
        cycle <= task->cycles;
        cycle++)
    {
        int sleep_result;


        do
        {
            sleep_result =
                clock_nanosleep(
                    CLOCK_MONOTONIC,
                    TIMER_ABSTIME,
                    &next_release,
                    NULL
                );
        }
        while(sleep_result ==
              EINTR);


        if(sleep_result != 0)
        {
            fprintf(
                stderr,
                "%s sleep failed: %s\n",
                task->name,
                strerror(
                    sleep_result
                )
            );


            return NULL;
        }


        struct timespec actual_start;
        struct timespec finish;


        clock_gettime(
            CLOCK_MONOTONIC,
            &actual_start
        );


        const int64_t scheduled_ns =
            TimespecToNs(
                &next_release
            );


        const int64_t start_ns =
            TimespecToNs(
                &actual_start
            );


        const int64_t jitter_ns =
            start_ns -
            scheduled_ns;


        /*
         * Execute deterministic CPU workload.
         */
        BurnThreadCpuTime(
            task->execution_ns
        );


        clock_gettime(
            CLOCK_MONOTONIC,
            &finish
        );


        const int64_t finish_ns =
            TimespecToNs(
                &finish
            );


        const int64_t execution_ns =
            finish_ns -
            start_ns;


        const int64_t response_ns =
            finish_ns -
            scheduled_ns;


        if(response_ns >
           task->deadline_ns)
        {
            deadline_misses++;
        }


        total_jitter_ns +=
            jitter_ns;


        total_execution_ns +=
            execution_ns;


        total_response_ns +=
            response_ns;


        if(jitter_ns >
           worst_jitter_ns)
        {
            worst_jitter_ns =
                jitter_ns;
        }


        if(execution_ns >
           worst_execution_ns)
        {
            worst_execution_ns =
                execution_ns;
        }


        if(response_ns >
           worst_response_ns)
        {
            worst_response_ns =
                response_ns;
        }


        AddNs(
            &next_release,
            task->period_ns
        );
    }


    task->average_jitter_us =
        (
            total_jitter_ns /
            (double)task->cycles
        ) /
        1000.0;


    task->worst_jitter_us =
        worst_jitter_ns /
        1000.0;


    task->average_execution_us =
        (
            total_execution_ns /
            (double)task->cycles
        ) /
        1000.0;


    task->worst_execution_us =
        worst_execution_ns /
        1000.0;


    task->average_response_us =
        (
            total_response_ns /
            (double)task->cycles
        ) /
        1000.0;


    task->worst_response_us =
        worst_response_ns /
        1000.0;


    task->deadline_misses =
        deadline_misses;


    return NULL;
}


/* ==========================================================================
 * One scheduling trial
 * ========================================================================== */

static int RunExperiment(
    experiment_t *experiment
)
{
    pthread_t threads[TASK_COUNT];

    pthread_attr_t attributes[TASK_COUNT];

    struct sched_param scheduling_parameter;

    task_context_t contexts[TASK_COUNT];


    uint32_t i;


    /*
     * Common task-release origin.
     */
    clock_gettime(
        CLOCK_MONOTONIC,
        &experiment->start_time
    );


    /*
     * Give all threads one second for creation.
     */
    AddNs(
        &experiment->start_time,
        1000000000LL
    );


    for(i = 0U;
        i < TASK_COUNT;
        i++)
    {
        contexts[i].task =
            &experiment->tasks[i];


        contexts[i].start_time =
            experiment->start_time;


        pthread_attr_init(
            &attributes[i]
        );


        pthread_attr_setinheritsched(
            &attributes[i],
            PTHREAD_EXPLICIT_SCHED
        );


        pthread_attr_setschedpolicy(
            &attributes[i],
            SCHED_FIFO
        );


        memset(
            &scheduling_parameter,
            0,
            sizeof(scheduling_parameter)
        );


        scheduling_parameter.
            sched_priority =
                experiment->tasks[i].
                    priority;


        pthread_attr_setschedparam(
            &attributes[i],
            &scheduling_parameter
        );


        const int result =
            pthread_create(
                &threads[i],
                &attributes[i],
                PeriodicTaskThread,
                &contexts[i]
            );


        if(result != 0)
        {
            fprintf(
                stderr,
                "Failed to create %s: %s\n",
                experiment->tasks[i].name,
                strerror(result)
            );


            return -1;
        }
    }


    for(i = 0U;
        i < TASK_COUNT;
        i++)
    {
        pthread_join(
            threads[i],
            NULL
        );


        pthread_attr_destroy(
            &attributes[i]
        );
    }


    return 0;
}


/* ==========================================================================
 * Result printer
 * ========================================================================== */

static void PrintExperiment(
    const experiment_t *experiment
)
{
    const char *mode_name =
        (
            experiment->mode ==
            POLICY_RM
        ) ?
        "RATE MONOTONIC" :
        "DEADLINE MONOTONIC";


    printf(
        "\n=== %s RESULTS ===\n\n",
        mode_name
    );


    printf(
        "%-10s %-9s %-9s %-9s %-10s\n",
        "Task",
        "Period",
        "Deadline",
        "Priority",
        "Misses"
    );


    printf(
        "-----------------------------------------------------\n"
    );


    uint32_t i;


    for(i = 0U;
        i < TASK_COUNT;
        i++)
    {
        const periodic_task_t *task =
            &experiment->tasks[i];


        printf(
            "%-10s %5.1f ms %5.1f ms %-9d %u / %u\n",
            task->name,
            task->period_ns /
                1000000.0,
            task->deadline_ns /
                1000000.0,
            task->priority,
            task->deadline_misses,
            task->cycles
        );


        printf(
            "   Avg jitter    : %9.3f us\n",
            task->average_jitter_us
        );


        printf(
            "   Worst jitter  : %9.3f us\n",
            task->worst_jitter_us
        );


        printf(
            "   Avg execution : %9.3f us\n",
            task->average_execution_us
        );


        printf(
            "   Worst exec    : %9.3f us\n",
            task->worst_execution_us
        );


        printf(
            "   Avg response  : %9.3f us\n",
            task->average_response_us
        );


        printf(
            "   Worst response: %9.3f us\n\n",
            task->worst_response_us
        );
    }
}


/* ==========================================================================
 * CPU-affinity check
 * ========================================================================== */

static int GetAllowedCpuCount(void)
{
    cpu_set_t set;


    CPU_ZERO(
        &set
    );


    if(sched_getaffinity(
            0,
            sizeof(set),
            &set) != 0)
    {
        return -1;
    }


    return
        CPU_COUNT(
            &set
        );
}


/* ==========================================================================
 * Main
 * ========================================================================== */

int main(void)
{
    experiment_t rm_experiment;
    experiment_t dm_experiment;


    printf(
        "=== RATE MONOTONIC vs DEADLINE MONOTONIC ===\n\n"
    );


    printf(
        "Task set:\n"
    );


    printf(
        "FAST_A   : C=8 ms, T=40 ms,  D=40 ms\n"
    );


    printf(
        "URGENT_B : C=8 ms, T=50 ms,  D=12 ms\n"
    );


    printf(
        "SLOW_C   : C=4 ms, T=100 ms, D=100 ms\n\n"
    );


    printf(
        "Total nominal utilization:\n"
    );


    printf(
        "U = 8/40 + 8/50 + 4/100 = 0.400\n\n"
    );


    const int cpu_count =
        GetAllowedCpuCount();


    printf(
        "Allowed CPU count: %d\n",
        cpu_count
    );


    if(cpu_count != 1)
    {
        printf(
            "[WARNING] Pin this experiment to one CPU core.\n"
        );


        printf(
            "Use: sudo taskset -c 3 ./rm_vs_dm_test\n"
        );
    }


    /* ----------------------------------------------------------------------
     * RM
     * ---------------------------------------------------------------------- */

    InitializeTaskSet(
        &rm_experiment,
        POLICY_RM
    );


    printf(
        "\nRunning Rate Monotonic trial...\n"
    );


    if(RunExperiment(
            &rm_experiment) != 0)
    {
        fprintf(
            stderr,
            "RM trial failed.\n"
        );


        return 1;
    }


    /*
     * Separation between trials.
     */
    usleep(
        500000
    );


    /* ----------------------------------------------------------------------
     * DM
     * ---------------------------------------------------------------------- */

    InitializeTaskSet(
        &dm_experiment,
        POLICY_DM
    );


    printf(
        "Running Deadline Monotonic trial...\n"
    );


    if(RunExperiment(
            &dm_experiment) != 0)
    {
        fprintf(
            stderr,
            "DM trial failed.\n"
        );


        return 1;
    }


    /* ----------------------------------------------------------------------
     * Results
     * ---------------------------------------------------------------------- */

    PrintExperiment(
        &rm_experiment
    );


    PrintExperiment(
        &dm_experiment
    );


    const periodic_task_t *rm_urgent =
        &rm_experiment.tasks[
            TASK_URGENT_B
        ];


    const periodic_task_t *dm_urgent =
        &dm_experiment.tasks[
            TASK_URGENT_B
        ];


    printf(
        "\n=== URGENT_B COMPARISON ===\n"
    );


    printf(
        "Relative deadline              : 12.000 ms\n"
    );


    printf(
        "RM priority                    : %d\n",
        rm_urgent->priority
    );


    printf(
        "DM priority                    : %d\n",
        dm_urgent->priority
    );


    printf(
        "RM deadline misses             : %u / %u\n",
        rm_urgent->deadline_misses,
        rm_urgent->cycles
    );


    printf(
        "DM deadline misses             : %u / %u\n",
        dm_urgent->deadline_misses,
        dm_urgent->cycles
    );


    printf(
        "RM worst response              : %.3f ms\n",
        rm_urgent->worst_response_us /
            1000.0
    );


    printf(
        "DM worst response              : %.3f ms\n",
        dm_urgent->worst_response_us /
            1000.0
    );


    if((rm_urgent->deadline_misses > 0U) &&
       (dm_urgent->deadline_misses == 0U))
    {
        printf(
            "\n[PASS] Deadline Monotonic protected "
            "the short-deadline task.\n"
        );
    }
    else
    {
        printf(
            "\n[CHECK] Expected RM/DM deadline behavior "
            "was not clearly observed.\n"
        );
    }


    return 0;
}
