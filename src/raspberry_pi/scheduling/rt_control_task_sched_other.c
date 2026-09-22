/*
 * Project:
 * Real-Time Safety-Critical Actuator Control & Fault-Injection Testbed
 *
 * Milestone:
 * Periodic CONTROL Task Timing Analysis with CSV Logging
 *
 * Purpose:
 * - Run a periodic CONTROL task every 20 ms
 * - Measure release jitter
 * - Measure execution time
 * - Measure response time
 * - Detect deadline misses
 * - Calculate average and worst-case timing values
 * - Save every cycle to a CSV file for later analysis
 *
 * Current scheduler:
 * Normal Linux scheduling (SCHED_OTHER)
 *
 * This measurement will be used as the baseline before enabling
 * POSIX real-time scheduling with SCHED_FIFO.
 */

#define _GNU_SOURCE

#include <stdio.h>
#include <stdint.h>
#include <time.h>
#include <errno.h>
#include <string.h>


/* --------------------------------------------------------------------------
 * CONTROL task configuration
 * -------------------------------------------------------------------------- */

/*
 * CONTROL task period:
 *
 * 20 ms = 20,000,000 ns
 */
#define CONTROL_PERIOD_NS       20000000LL

/*
 * For the baseline experiment, relative deadline equals period.
 */
#define CONTROL_DEADLINE_NS     CONTROL_PERIOD_NS

/*
 * Number of task releases in one experiment.
 *
 * 1000 cycles x 20 ms = approximately 20 seconds.
 */
#define TEST_CYCLES             1000U

/*
 * CSV output file.
 *
 * The file will be created in the directory where the program is run.
 */
#define CSV_FILE_NAME           "control_timing.csv"


/* --------------------------------------------------------------------------
 * Timing helper functions
 * -------------------------------------------------------------------------- */

/*
 * Convert a POSIX timespec structure to nanoseconds.
 *
 * Using one common unit simplifies calculations for:
 * - jitter
 * - execution time
 * - response time
 */
static int64_t timespec_to_ns(const struct timespec *time_value)
{
    return ((int64_t)time_value->tv_sec * 1000000000LL) +
           (int64_t)time_value->tv_nsec;
}


/*
 * Add nanoseconds to an absolute timespec value.
 *
 * tv_nsec must always remain below 1,000,000,000.
 * Any overflow is transferred into tv_sec.
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
 * Temporary deterministic workload.
 *
 * At this stage, the objective is to characterize the timing behavior
 * of a periodic task before integrating physical actuator communication.
 *
 * Later milestones will replace this synthetic workload with real
 * Raspberry Pi -> TM4C123 control transactions.
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
     * Prevent the compiler from completely removing the workload
     * during optimization.
     */
    (void)result;
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
     * Aggregate timing values used to calculate averages.
     */
    int64_t total_jitter_ns = 0;
    int64_t total_execution_ns = 0;
    int64_t total_response_ns = 0;

    /*
     * Worst-case timing values observed during the entire experiment.
     */
    int64_t worst_jitter_ns = 0;
    int64_t wcet_ns = 0;
    int64_t worst_response_ns = 0;

    uint32_t deadline_misses = 0U;
    uint32_t cycle;

    FILE *csv_file;


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

    /*
     * CSV header.
     *
     * Individual timing samples are stored in microseconds to make
     * the resulting file easier to inspect and plot.
     */
    fprintf(
        csv_file,
        "cycle,scheduled_ns,start_ns,finish_ns,"
        "jitter_us,execution_us,response_us,deadline_miss\n"
    );


    /* ----------------------------------------------------------------------
     * Establish first absolute release time
     * ---------------------------------------------------------------------- */

    /*
     * CLOCK_MONOTONIC is used because it moves forward continuously
     * and is not affected by normal wall-clock corrections.
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
     * First CONTROL release occurs one full period from now.
     */
    add_ns(
        &next_release,
        CONTROL_PERIOD_NS
    );


    /* ----------------------------------------------------------------------
     * Experiment information
     * ---------------------------------------------------------------------- */

    printf("=== CONTROL TASK BASELINE TIMING TEST ===\n");
    printf("Scheduler : SCHED_OTHER\n");
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
         * Sleep until the exact absolute release time.
         *
         * TIMER_ABSTIME is important because a relative sleep would allow
         * execution delays to accumulate as drift over many task periods.
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
         * Capture actual task start time immediately after wake-up.
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
         * actual start - scheduled release
         *
         * This measures how late the task began relative to its
         * intended periodic release.
         */
        jitter_ns =
            start_ns - scheduled_ns;


        /* ------------------------------------------------------------------
         * Execute CONTROL workload
         * ------------------------------------------------------------------ */

        control_workload();


        /*
         * Capture completion time.
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
         * task finish - actual task start
         */
        execution_ns =
            finish_ns - start_ns;


        /*
         * Response time:
         *
         * task finish - scheduled release
         *
         * This includes both release jitter and execution time.
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
         * Update statistics
         * ------------------------------------------------------------------ */

        total_jitter_ns += jitter_ns;
        total_execution_ns += execution_ns;
        total_response_ns += response_ns;


        if(jitter_ns > worst_jitter_ns)
        {
            worst_jitter_ns = jitter_ns;
        }


        /*
         * WCET:
         *
         * Maximum measured execution time during the experiment.
         *
         * Strictly speaking, this is an observed WCET rather than a
         * mathematically guaranteed upper bound.
         */
        if(execution_ns > wcet_ns)
        {
            wcet_ns = execution_ns;
        }


        if(response_ns > worst_response_ns)
        {
            worst_response_ns = response_ns;
        }


        /* ------------------------------------------------------------------
         * Store raw measurement in CSV
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
         * Terminal printing is deliberately limited.
         *
         * Printing every cycle would add significant terminal I/O
         * interference to the timing experiment.
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
         * Advance the absolute release timeline by exactly one period.
         */
        add_ns(
            &next_release,
            CONTROL_PERIOD_NS
        );
    }


    /* ----------------------------------------------------------------------
     * Finish CSV logging
     * ---------------------------------------------------------------------- */

    fclose(csv_file);


    /* ----------------------------------------------------------------------
     * Final timing statistics
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
