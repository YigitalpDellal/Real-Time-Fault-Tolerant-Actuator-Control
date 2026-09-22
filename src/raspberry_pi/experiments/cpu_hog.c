/*
 * Project:
 * Real-Time Safety-Critical Actuator Control & Fault-Injection Testbed
 *
 * Utility:
 * Controlled CPU Load Generator
 *
 * Purpose:
 * - Generate continuous CPU load during scheduling experiments
 * - Create processor contention for the CONTROL task
 * - Allow SCHED_OTHER and SCHED_FIFO behavior to be compared
 *   under the same CPU-load condition
 *
 * Important:
 * This program does not represent a real application task.
 * It is an experimental workload generator used only during
 * timing and scheduling tests.
 */

#include <stdint.h>
#include <stdio.h>
#include <signal.h>
#include <stdbool.h>


/*
 * Controls the main loop.
 *
 * 'sig_atomic_t' is used because this variable is modified
 * from a signal handler.
 */
static volatile sig_atomic_t g_running = 1;


/*
 * Handle Ctrl+C / SIGINT so the load generator can terminate cleanly.
 */
static void handle_sigint(int signal_number)
{
    (void)signal_number;

    g_running = 0;
}


int main(void)
{
    /*
     * Volatile prevents the compiler from removing the calculation
     * as an unused optimization.
     */
    volatile uint64_t workload = 0ULL;

    /*
     * Install a clean Ctrl+C handler.
     */
    signal(SIGINT, handle_sigint);

    printf("=== CPU LOAD GENERATOR ===\n");
    printf("Generating continuous processor load.\n");
    printf("Press Ctrl+C to stop.\n\n");

    /*
     * Continuous arithmetic keeps one CPU core busy when this process
     * is pinned to a specific core with taskset.
     */
    while(g_running)
    {
        workload++;

        /*
         * Periodically modify the value to avoid trivial overflow behavior
         * during long experiments.
         */
        if(workload == UINT64_MAX)
        {
            workload = 0ULL;
        }
    }

    printf("\nCPU load generator stopped cleanly.\n");

    return 0;
}
