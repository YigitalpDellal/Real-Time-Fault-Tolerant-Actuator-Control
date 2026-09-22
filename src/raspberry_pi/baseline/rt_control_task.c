/*
 * Project:
 * Real-Time Safety-Critical Actuator Control & Fault-Injection Testbed
 *
 * Milestone:
 * POSIX Real-Time CONTROL Task using SCHED_FIFO
 *
 * Purpose:
 * - Run the CONTROL task periodically every 20 ms
 * - Execute the task under the POSIX SCHED_FIFO scheduling policy
 * - Measure release jitter
 * - Measure execution time
 * - Measure response time
 * - Detect deadline misses
 * - Calculate average and worst-case timing values
 * - Log every task instance to CSV
 *
 * Comparison:
 *
 * Previous experiment:
 *     Linux default scheduler -> SCHED_OTHER
 *
 * Current experiment:
 *     POSIX real-time scheduler -> SCHED_FIFO
 *
 * The task workload, period, deadline and number of cycles are kept
 * identical so the scheduling policies can be compared fairly.
 */

#define _GNU_SOURCE

#include <stdio.h>
#include <stdint.h>
#include <time.h>
#include <errno.h>
#include <string.h>
#include <pthread.h>
#include <sched.h>


/* --------------------------------------------------------------------------
 * CONTROL task configuration
 * -------------------------------------------------------------------------- */

/*
 * CONTROL period:
 *
 * 20 ms = 20,000,000 ns
 */
#define CONTROL_PERIOD_NS       20000000LL

/*
 * Relative deadline equals the period.
 */
#define CONTROL_DEADLINE_NS     CONTROL_PERIOD_NS

/*
 * 1000 cycles x 20 ms = approximately 20 seconds.
 */
#define TEST_CYCLES             1000U

/*
 * SCHED_FIFO priority.
 *
 * Linux normally provides SCHED_FIFO priorities from 1 to 99.
 * A relatively high priority is used for the CONTROL task.
 */
#define CONTROL_PRIORITY        80

/*
 * Dedicated CSV file for this experiment.
 *
 * This prevents the previous SCHED_OTHER measurements
 * from being overwritten.
 */
#define CSV_FILE_NAME           "control_timing_sched_fifo.csv"


/* --------------------------------------------------------------------------
 * Timing helper functions
 * -------------------------------------------------------------------------- */

/*
 * Convert a POSIX timespec structure to nanoseconds.
 */
static int64_t timespec_to_ns(const struct timespec *time_value)
{
    return ((int64_t)time_value->tv_sec * 1000000000LL) +
           (int64_t)time_value->tv_nsec;
}


/*
 * Add nanoseconds to an absolute timespec value.
 *
 * Any overflow from tv_nsec is moved into tv_sec.
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
 * CONTROL workload
 * -------------------------------------------------------------------------- */

/*
 * Temporary deterministic CONTROL workload.
 *
 * The same workload used in the SCHED_OTHER baseline is preserved
 * so the scheduling comparison remains meaningful.
 *
 * A later milestone will replace this synthetic computation with
 * real Raspberry Pi -> TM4C123 actuator communication.
 */
static void control_workload(void)
{
    volatile uint32_t result = 0U;
    uint32_t i;

    for(i = 0U; i < 10000U; i++)
    {
        result += i;
    }

    /*
     * Prevent the compiler from eliminating the workload.
     */
    (void)result;
}


/* --------------------------------------------------------------------------
 * Real-time scheduling configuration
 * -------------------------------------------------------------------------- */

/*
 * Configure the current POSIX thread to use SCHED_FIFO.
 *
 * SCHED_FIFO is a fixed-priority real-time scheduling policy.
 *
 * A runnable SCHED_FIFO thread can preempt normal SCHED_OTHER tasks
 * with lower scheduling priority.
 *
 * This operation normally requires root privileges or CAP_SYS_NICE.
 */
static int configure_realtime_scheduling(void)
{
    struct sched_param parameters;

    int minimum_priority;
    int maximum_priority;
    int result;

    /*
     * Read the priority range supported by the operating system.
     */
    minimum_priority =
        sched_get_priority_min(SCHED_FIFO);

    maximum_priority =
        sched_get_priority_max(SCHED_FIFO);

    if((minimum_priority == -1) ||
       (maximum_priority == -1))
    {
        perror("sched_get_priority");

        return -1;
    }

    /*
     * Verify that the selected CONTROL priority is valid.
     */
    if((CONTROL_PRIORITY < minimum_priority) ||
       (CONTROL_PRIORITY > maximum_priority))
    {
        fprintf(
            stderr,
            "Invalid SCHED_FIFO priority %d "
            "(valid range: %d-%d)\n",
            CONTROL_PRIORITY,
            minimum_priority,
            maximum_priority
        );

        return -1;
    }

    memset(
        &parameters,
        0,
        sizeof(parameters)
    );

    parameters.sched_priority =
        CONTROL_PRIORITY;

    /*
     * Apply SCHED_FIFO to the current POSIX thread.
     */
    result =
        pthread_setschedparam(
            pthread_self(),
            SCHED_FIFO,
            &parameters
        );

    if(result != 0)
    {
        fprintf(
            stderr,
            "pthread_setschedparam failed: %s\n",
            strerror(result)
        );

        fprintf(
            stderr,
            "Run the program with sudo because "
            "SCHED_FIFO requires real-time scheduling privileges.\n"
        );

        return -1;
    }

    return 0;
}


/* --------------------------------------------------------------------------
 * Main timing experiment
 * -------------------------------------------------------------------------- */

int main(void)
{
    struct timespec next_release;
    struct timespec actual_start;
    struct timespec finish;

    int64_t scheduled_ns;
    int64_t start_ns;
    int64_t finish_ns;

    int64_t jitter_ns;
    int64_t execution_ns;
    int64_t response_ns;

    /*
     * Aggregate values used to calculate averages.
     */
    int64_t total_jitter_ns = 0;
    int64_t total_execution_ns = 0;
    int64_t total_response_ns = 0;

    /*
     * Worst observed timing values.
     */
    int64_t worst_jitter_ns = 0;
    int64_t wcet_ns = 0;
    int64_t worst_response_ns = 0;

    uint32_t deadline_misses = 0U;
    uint32_t cycle;

    FILE *csv_file;


    /* ----------------------------------------------------------------------
     * Enable POSIX real-time scheduling
     * ---------------------------------------------------------------------- */

    if(configure_realtime_scheduling() != 0)
    {
        return 1;
    }


    /* ----------------------------------------------------------------------
     * Create CSV output file
     * ---------------------------------------------------------------------- */

    csv_file = fopen(
        CSV_FILE_NAME,
        "w"
    );

    if(csv_file == NULL)
    {
        perror("fopen");

        return 1;
    }

    fprintf(
        csv_file,
        "cycle,scheduled_ns,start_ns,finish_ns,"
        "jitter_us,execution_us,response_us,deadline_miss\n"
    );


    /* ----------------------------------------------------------------------
     * Establish the periodic release timeline
     * ---------------------------------------------------------------------- */

    /*
     * CLOCK_MONOTONIC is independent of normal wall-clock adjustments
     * and is therefore appropriate for periodic timing measurements.
     */
    if(clock_gettime(
            CLOCK_MONOTONIC,
            &next_release) != 0)
    {
        perror("clock_gettime");

        fclose(csv_file);

        return 1;
    }

    /*
     * Schedule the first release one full period in the future.
     */
    add_ns(
        &next_release,
        CONTROL_PERIOD_NS
    );


    /* ----------------------------------------------------------------------
     * Experiment information
     * ---------------------------------------------------------------------- */

    printf("=== CONTROL TASK REAL-TIME TIMING TEST ===\n");
    printf("Scheduler : SCHED_FIFO\n");
    printf("Priority  : %d\n", CONTROL_PRIORITY);
    printf("Period    : 20.000 ms\n");
    printf("Deadline  : 20.000 ms\n");
    printf("Cycles    : %u\n", TEST_CYCLES);
    printf("CSV       : %s\n\n", CSV_FILE_NAME);


    /* ----------------------------------------------------------------------
     * Periodic CONTROL loop
     * ---------------------------------------------------------------------- */

    for(cycle = 1U; cycle <= TEST_CYCLES; cycle++)
    {
        int sleep_result;
        int deadline_miss;


        /*
         * Wait until the absolute release instant.
         *
         * TIMER_ABSTIME prevents accumulated drift because every task
         * release remains tied to the original periodic timeline.
         */
        do
        {
            sleep_result = clock_nanosleep(
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
                "clock_nanosleep failed: %s\n",
                strerror(sleep_result)
            );

            fclose(csv_file);

            return 1;
        }


        /*
         * Capture the real task start instant.
         */
        if(clock_gettime(
                CLOCK_MONOTONIC,
                &actual_start) != 0)
        {
            perror("clock_gettime");

            fclose(csv_file);

            return 1;
        }


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
         * Execute CONTROL workload
         * ------------------------------------------------------------------ */

        control_workload();


        /*
         * Record task completion time.
         */
        if(clock_gettime(
                CLOCK_MONOTONIC,
                &finish) != 0)
        {
            perror("clock_gettime");

            fclose(csv_file);

            return 1;
        }


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

        deadline_miss =
            (response_ns > CONTROL_DEADLINE_NS);

        if(deadline_miss)
        {
            deadline_misses++;
        }


        /* ------------------------------------------------------------------
         * Update timing statistics
         * ------------------------------------------------------------------ */

        total_jitter_ns += jitter_ns;

        total_execution_ns +=
            execution_ns;

        total_response_ns +=
            response_ns;


        if(jitter_ns > worst_jitter_ns)
        {
            worst_jitter_ns =
                jitter_ns;
        }


        /*
         * Maximum observed execution time.
         *
         * This is an experimentally observed WCET, not a formal
         * mathematical upper bound.
         */
        if(execution_ns > wcet_ns)
        {
            wcet_ns =
                execution_ns;
        }


        if(response_ns > worst_response_ns)
        {
            worst_response_ns =
                response_ns;
        }


        /* ------------------------------------------------------------------
         * Store raw timing sample
         * ------------------------------------------------------------------ */

        fprintf(
            csv_file,
            "%u,%lld,%lld,%lld,%.3f,%.3f,%.3f,%d\n",
            cycle,
            (long long)scheduled_ns,
            (long long)start_ns,
            (long long)finish_ns,
            jitter_ns / 1000.0,
            execution_ns / 1000.0,
            response_ns / 1000.0,
            deadline_miss
        );


        /*
         * Limit terminal output because console I/O itself can disturb
         * timing behavior.
         */
        if((cycle == 1U) ||
           ((cycle % 100U) == 0U))
        {
            printf(
                "Cycle %4u | "
                "Jitter: %8.3f us | "
                "Exec: %8.3f us | "
                "Response: %8.3f us\n",
                cycle,
                jitter_ns / 1000.0,
                execution_ns / 1000.0,
                response_ns / 1000.0
            );
        }


        /*
         * Move to the next absolute 20 ms release instant.
         */
        add_ns(
            &next_release,
            CONTROL_PERIOD_NS
        );
    }


    /* ----------------------------------------------------------------------
     * Finish logging
     * ---------------------------------------------------------------------- */

    fclose(csv_file);


    /* ----------------------------------------------------------------------
     * Final experiment summary
     * ---------------------------------------------------------------------- */

    printf("\n=== TIMING SUMMARY ===\n");

    printf(
        "Average jitter        : %.3f us\n",
        (total_jitter_ns / (double)TEST_CYCLES) /
        1000.0
    );

    printf(
        "Worst observed jitter : %.3f us\n",
        worst_jitter_ns / 1000.0
    );

    printf(
        "Average execution     : %.3f us\n",
        (total_execution_ns / (double)TEST_CYCLES) /
        1000.0
    );

    printf(
        "Observed WCET         : %.3f us\n",
        wcet_ns / 1000.0
    );

    printf(
        "Average response      : %.3f us\n",
        (total_response_ns / (double)TEST_CYCLES) /
        1000.0
    );

    printf(
        "Worst response        : %.3f us\n",
        worst_response_ns / 1000.0
    );

    printf(
        "Deadline misses       : %u / %u\n",
        deadline_misses,
        TEST_CYCLES
    );

    printf(
        "CSV saved             : %s\n",
        CSV_FILE_NAME
    );


    return 0;
}
