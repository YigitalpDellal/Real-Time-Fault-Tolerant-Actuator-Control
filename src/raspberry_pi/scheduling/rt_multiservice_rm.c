/*
 * Project:
 * Real-Time Safety-Critical Actuator Control & Fault-Injection Testbed
 *
 * Milestone:
 * Multi-Service POSIX Real-Time System using Rate Monotonic Scheduling
 *
 * Purpose:
 * - Create five periodic real-time services
 * - Assign fixed priorities according to Rate Monotonic scheduling
 * - Use SCHED_FIFO on Linux
 * - Measure release jitter, execution time and response time
 * - Detect deadline misses for every service
 * - Estimate observed processor utilization
 *
 * Rate Monotonic principle:
 *
 * Shorter period -> higher fixed priority
 *
 * Service     Period     Priority
 * --------------------------------
 * CONTROL       20 ms       80
 * COMM          50 ms       70
 * HEALTH       100 ms       60
 * MONITOR      200 ms       50
 * LOGGER      1000 ms       40
 *
 * IMPORTANT:
 * The process will later be pinned to one CPU core using taskset.
 * This makes the scheduling experiment equivalent to a single-processor
 * fixed-priority scheduling problem.
 */

#define _GNU_SOURCE

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <errno.h>
#include <string.h>
#include <pthread.h>
#include <sched.h>


/* --------------------------------------------------------------------------
 * Experiment configuration
 * -------------------------------------------------------------------------- */

/*
 * Total experiment duration:
 *
 * 10 seconds
 *
 * Resulting releases:
 *
 * CONTROL  -> 500
 * COMM     -> 200
 * HEALTH   -> 100
 * MONITOR  -> 50
 * LOGGER   -> 10
 */
#define EXPERIMENT_DURATION_NS  10000000000LL


/*
 * Number of real-time services.
 */
#define SERVICE_COUNT           5U


/* --------------------------------------------------------------------------
 * Service description
 * -------------------------------------------------------------------------- */

typedef struct
{
    const char *name;

    /*
     * Period and relative deadline.
     *
     * For this first RM experiment:
     * deadline = period
     */
    int64_t period_ns;
    int64_t deadline_ns;

    /*
     * Fixed SCHED_FIFO priority.
     */
    int priority;

    /*
     * Synthetic workload size.
     *
     * Different values represent different service execution demands.
     */
    uint32_t workload_iterations;

    /*
     * Number of releases during the experiment.
     */
    uint32_t cycles;

    /*
     * Runtime statistics.
     */
    double average_jitter_us;
    double worst_jitter_us;

    double average_execution_us;
    double observed_max_execution_us;

    double average_response_us;
    double worst_response_us;

    uint32_t deadline_misses;

} service_t;


/* --------------------------------------------------------------------------
 * Common experiment start time
 * -------------------------------------------------------------------------- */

/*
 * All services use the same global timing origin.
 *
 * This provides a common release timeline and avoids each thread starting
 * from an unrelated clock instant.
 */
static struct timespec g_start_time;


/* --------------------------------------------------------------------------
 * Service table
 * -------------------------------------------------------------------------- */

static service_t g_services[SERVICE_COUNT] =
{
    {
        .name = "CONTROL",
        .period_ns = 20000000LL,
        .deadline_ns = 20000000LL,
        .priority = 80,
        .workload_iterations = 10000U
    },

    {
        .name = "COMM",
        .period_ns = 50000000LL,
        .deadline_ns = 50000000LL,
        .priority = 70,
        .workload_iterations = 7000U
    },

    {
        .name = "HEALTH",
        .period_ns = 100000000LL,
        .deadline_ns = 100000000LL,
        .priority = 60,
        .workload_iterations = 5000U
    },

    {
        .name = "MONITOR",
        .period_ns = 200000000LL,
        .deadline_ns = 200000000LL,
        .priority = 50,
        .workload_iterations = 3000U
    },

    {
        .name = "LOGGER",
        .period_ns = 1000000000LL,
        .deadline_ns = 1000000000LL,
        .priority = 40,
        .workload_iterations = 1000U
    }
};


/* --------------------------------------------------------------------------
 * Timing helpers
 * -------------------------------------------------------------------------- */

/*
 * Convert timespec into nanoseconds.
 */
static int64_t timespec_to_ns(
    const struct timespec *time_value
)
{
    return
        ((int64_t)time_value->tv_sec * 1000000000LL) +
        (int64_t)time_value->tv_nsec;
}


/*
 * Add nanoseconds to an absolute timespec value.
 */
static void add_ns(
    struct timespec *time_value,
    int64_t nanoseconds
)
{
    time_value->tv_nsec += nanoseconds;

    while(time_value->tv_nsec >= 1000000000L)
    {
        time_value->tv_nsec -= 1000000000L;
        time_value->tv_sec++;
    }
}


/* --------------------------------------------------------------------------
 * Synthetic service workload
 * -------------------------------------------------------------------------- */

/*
 * Perform deterministic CPU work.
 *
 * At this milestone the workload is synthetic so that scheduling behavior
 * can be studied independently from UART and actuator I/O.
 *
 * Later:
 *
 * CONTROL -> real actuator command generation
 * COMM    -> Raspberry Pi / TM4C communication
 * HEALTH  -> communication and system health supervision
 * MONITOR -> deadline / status monitoring
 * LOGGER  -> telemetry logging
 */
static void execute_workload(
    uint32_t iterations
)
{
    volatile uint64_t result = 0ULL;
    uint32_t i;

    for(i = 0U; i < iterations; i++)
    {
        result +=
            ((uint64_t)i * 3ULL) + 1ULL;
    }

    /*
     * Prevent the compiler from completely eliminating the workload.
     */
    (void)result;
}


/* --------------------------------------------------------------------------
 * Periodic service thread
 * -------------------------------------------------------------------------- */

static void *service_thread(void *argument)
{
    service_t *service =
        (service_t *)argument;

    struct timespec next_release;
    struct timespec actual_start;
    struct timespec finish;

    int64_t scheduled_ns;
    int64_t start_ns;
    int64_t finish_ns;

    int64_t jitter_ns;
    int64_t execution_ns;
    int64_t response_ns;

    int64_t total_jitter_ns = 0;
    int64_t total_execution_ns = 0;
    int64_t total_response_ns = 0;

    int64_t worst_jitter_ns = 0;
    int64_t max_execution_ns = 0;
    int64_t worst_response_ns = 0;

    uint32_t deadline_misses = 0U;
    uint32_t cycle;


    /*
     * Every service begins from the same global start time.
     */
    next_release = g_start_time;


    for(cycle = 1U;
        cycle <= service->cycles;
        cycle++)
    {
        int sleep_result;


        /* ------------------------------------------------------------------
         * Wait until absolute release instant
         * ------------------------------------------------------------------ */

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
        while(sleep_result == EINTR);


        if(sleep_result != 0)
        {
            fprintf(
                stderr,
                "%s clock_nanosleep failed: %s\n",
                service->name,
                strerror(sleep_result)
            );

            return NULL;
        }


        /* ------------------------------------------------------------------
         * Measure actual task start
         * ------------------------------------------------------------------ */

        clock_gettime(
            CLOCK_MONOTONIC,
            &actual_start
        );


        scheduled_ns =
            timespec_to_ns(&next_release);

        start_ns =
            timespec_to_ns(&actual_start);


        /*
         * Release jitter:
         *
         * actual start - intended release
         */
        jitter_ns =
            start_ns - scheduled_ns;


        /* ------------------------------------------------------------------
         * Execute service workload
         * ------------------------------------------------------------------ */

        execute_workload(
            service->workload_iterations
        );


        /* ------------------------------------------------------------------
         * Measure completion
         * ------------------------------------------------------------------ */

        clock_gettime(
            CLOCK_MONOTONIC,
            &finish
        );


        finish_ns =
            timespec_to_ns(&finish);


        /*
         * Execution time:
         *
         * completion - actual start
         */
        execution_ns =
            finish_ns - start_ns;


        /*
         * Response time:
         *
         * completion - intended release
         */
        response_ns =
            finish_ns - scheduled_ns;


        /* ------------------------------------------------------------------
         * Deadline supervision
         * ------------------------------------------------------------------ */

        if(response_ns >
           service->deadline_ns)
        {
            deadline_misses++;
        }


        /* ------------------------------------------------------------------
         * Accumulate statistics
         * ------------------------------------------------------------------ */

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
           max_execution_ns)
        {
            max_execution_ns =
                execution_ns;
        }


        if(response_ns >
           worst_response_ns)
        {
            worst_response_ns =
                response_ns;
        }


        /* ------------------------------------------------------------------
         * Schedule next release
         * ------------------------------------------------------------------ */

        add_ns(
            &next_release,
            service->period_ns
        );
    }


    /* ----------------------------------------------------------------------
     * Store final service statistics
     * ---------------------------------------------------------------------- */

    service->average_jitter_us =
        (total_jitter_ns /
         (double)service->cycles) /
        1000.0;


    service->worst_jitter_us =
        worst_jitter_ns /
        1000.0;


    service->average_execution_us =
        (total_execution_ns /
         (double)service->cycles) /
        1000.0;


    service->observed_max_execution_us =
        max_execution_ns /
        1000.0;


    service->average_response_us =
        (total_response_ns /
         (double)service->cycles) /
        1000.0;


    service->worst_response_us =
        worst_response_ns /
        1000.0;


    service->deadline_misses =
        deadline_misses;


    return NULL;
}


/* --------------------------------------------------------------------------
 * Main
 * -------------------------------------------------------------------------- */

int main(void)
{
    pthread_t threads[SERVICE_COUNT];

    pthread_attr_t attributes[SERVICE_COUNT];

    struct sched_param scheduling_parameters;

    uint32_t i;

    double total_observed_utilization = 0.0;


    /* ----------------------------------------------------------------------
     * Calculate number of service releases
     * ---------------------------------------------------------------------- */

    for(i = 0U;
        i < SERVICE_COUNT;
        i++)
    {
        g_services[i].cycles =
            (uint32_t)(
                EXPERIMENT_DURATION_NS /
                g_services[i].period_ns
            );
    }


    /* ----------------------------------------------------------------------
     * Establish common start time
     * ---------------------------------------------------------------------- */

    if(clock_gettime(
            CLOCK_MONOTONIC,
            &g_start_time) != 0)
    {
        perror("clock_gettime");

        return 1;
    }


    /*
     * Give the program one second to create all threads before
     * periodic releases begin.
     */
    add_ns(
        &g_start_time,
        1000000000LL
    );


    /* ----------------------------------------------------------------------
     * Display task model
     * ---------------------------------------------------------------------- */

    printf(
        "=== RATE MONOTONIC MULTI-SERVICE TEST ===\n\n"
    );

    printf(
        "%-10s %-10s %-10s %-10s\n",
        "Service",
        "Period",
        "Priority",
        "Cycles"
    );

    printf(
        "------------------------------------------\n"
    );


    for(i = 0U;
        i < SERVICE_COUNT;
        i++)
    {
        printf(
            "%-10s %-7.0f ms %-10d %-10u\n",
            g_services[i].name,
            g_services[i].period_ns /
                1000000.0,
            g_services[i].priority,
            g_services[i].cycles
        );
    }


    printf(
        "\nScheduler: SCHED_FIFO / Rate Monotonic\n"
    );

    printf(
        "Experiment duration: 10 seconds\n\n"
    );


    /* ----------------------------------------------------------------------
     * Create real-time service threads
     * ---------------------------------------------------------------------- */

    for(i = 0U;
        i < SERVICE_COUNT;
        i++)
    {
        int result;


        pthread_attr_init(
            &attributes[i]
        );


        /*
         * Use explicit scheduling attributes instead of inheriting
         * the parent thread's scheduler.
         */
        pthread_attr_setinheritsched(
            &attributes[i],
            PTHREAD_EXPLICIT_SCHED
        );


        /*
         * All services use SCHED_FIFO.
         */
        pthread_attr_setschedpolicy(
            &attributes[i],
            SCHED_FIFO
        );


        memset(
            &scheduling_parameters,
            0,
            sizeof(scheduling_parameters)
        );


        scheduling_parameters.sched_priority =
            g_services[i].priority;


        /*
         * Assign Rate Monotonic fixed priority.
         */
        pthread_attr_setschedparam(
            &attributes[i],
            &scheduling_parameters
        );


        /*
         * Create service thread.
         */
        result =
            pthread_create(
                &threads[i],
                &attributes[i],
                service_thread,
                &g_services[i]
            );


        if(result != 0)
        {
            fprintf(
                stderr,
                "Failed to create %s thread: %s\n",
                g_services[i].name,
                strerror(result)
            );

            fprintf(
                stderr,
                "Run with sudo because "
                "SCHED_FIFO requires real-time privileges.\n"
            );

            return 1;
        }
    }


    /* ----------------------------------------------------------------------
     * Wait for all services to finish
     * ---------------------------------------------------------------------- */

    for(i = 0U;
        i < SERVICE_COUNT;
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


    /* ----------------------------------------------------------------------
     * Print measured results
     * ---------------------------------------------------------------------- */

    printf(
        "\n=== SERVICE TIMING RESULTS ===\n\n"
    );


    for(i = 0U;
        i < SERVICE_COUNT;
        i++)
    {
        double observed_utilization;


        /*
         * Observed utilization estimate:
         *
         * maximum measured execution time / period
         *
         * This is experimental rather than a formally proven WCET.
         */
        observed_utilization =
            (
                g_services[i].
                    observed_max_execution_us *
                1000.0
            ) /
            g_services[i].period_ns;


        total_observed_utilization +=
            observed_utilization;


        printf(
            "[%s]\n",
            g_services[i].name
        );

        printf(
            "  Period              : %.3f ms\n",
            g_services[i].period_ns /
                1000000.0
        );

        printf(
            "  Priority            : %d\n",
            g_services[i].priority
        );

        printf(
            "  Average jitter      : %.3f us\n",
            g_services[i].
                average_jitter_us
        );

        printf(
            "  Worst jitter        : %.3f us\n",
            g_services[i].
                worst_jitter_us
        );

        printf(
            "  Average execution   : %.3f us\n",
            g_services[i].
                average_execution_us
        );

        printf(
            "  Observed max exec   : %.3f us\n",
            g_services[i].
                observed_max_execution_us
        );

        printf(
            "  Average response    : %.3f us\n",
            g_services[i].
                average_response_us
        );

        printf(
            "  Worst response      : %.3f us\n",
            g_services[i].
                worst_response_us
        );

        printf(
            "  Deadline misses     : %u / %u\n",
            g_services[i].
                deadline_misses,
            g_services[i].
                cycles
        );

        printf(
            "  Observed utilization: %.6f\n\n",
            observed_utilization
        );
    }


    /* ----------------------------------------------------------------------
     * Overall utilization
     * ---------------------------------------------------------------------- */

    printf(
        "=== OBSERVED PROCESSOR UTILIZATION ===\n"
    );

    printf(
        "Sum(C_i / T_i) = %.6f\n",
        total_observed_utilization
    );


    /*
     * Liu & Layland sufficient RM utilization bound:
     *
     * U(n) = n * (2^(1/n) - 1)
     *
     * For n = 5:
     *
     * U(5) approximately 0.7435
     *
     * We use the numeric value here so no math library is required.
     */
    printf(
        "RM sufficient bound for 5 tasks ~= 0.7435\n"
    );


    if(total_observed_utilization <= 0.7435)
    {
        printf(
            "Observed utilization is below "
            "the classical RM sufficient bound.\n"
        );
    }
    else
    {
        printf(
            "Observed utilization exceeds "
            "the classical RM sufficient bound.\n"
        );
    }


    return 0;
}
