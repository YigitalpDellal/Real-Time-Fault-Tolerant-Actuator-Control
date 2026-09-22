/*
 * Project:
 * Real-Time Safety-Critical Actuator Control & Fault-Injection Testbed
 *
 * Milestone:
 * Multi-Service Rate Monotonic System with Real TM4C123 UART Control
 *
 * Architecture:
 *
 * CONTROL  - 20 ms
 *     Generates safe PAN/TILT setpoints.
 *
 * COMM     - 50 ms
 *     Sends changed setpoints to the TM4C123 over UART.
 *
 * HEALTH   - 100 ms
 *     Supervises communication acknowledgements.
 *
 * MONITOR  - 200 ms
 *     Represents periodic system monitoring activity.
 *
 * LOGGER   - 1000 ms
 *     Represents low-priority telemetry/logging activity.
 *
 * Scheduling:
 * SCHED_FIFO + Rate Monotonic fixed priorities
 *
 *     CONTROL  -> priority 80
 *     COMM     -> priority 70
 *     HEALTH   -> priority 60
 *     MONITOR  -> priority 50
 *     LOGGER   -> priority 40
 *
 * Hardware path:
 *
 * Raspberry Pi CONTROL task
 *          |
 *          v
 * Shared PAN/TILT setpoint
 *          |
 *          v
 *       COMM task
 *          |
 *          v
 *    /dev/serial0
 *          |
 *          v
 *       TM4C123
 *          |
 *          v
 * Hardware PWM on PB6 / PB7
 *
 * UART:
 * 115200 baud, 8N1
 */

#define _GNU_SOURCE

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include <time.h>
#include <errno.h>

#include <pthread.h>
#include <sched.h>

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <poll.h>


/* --------------------------------------------------------------------------
 * Experiment configuration
 * -------------------------------------------------------------------------- */

/*
 * Total timed experiment duration:
 *
 * 10 seconds
 */
#define EXPERIMENT_DURATION_NS  60000000000LL

#define SERVICE_COUNT           5U


/* --------------------------------------------------------------------------
 * UART configuration
 * -------------------------------------------------------------------------- */

#define UART_DEVICE             "/dev/serial0"

/*
 * Maximum time allowed for one TM4C acknowledgement.
 */
#define UART_RESPONSE_TIMEOUT_MS    10


/* --------------------------------------------------------------------------
 * Safe actuator positions
 * -------------------------------------------------------------------------- */

/*
 * These commands stay well inside the mechanical limits already enforced
 * by the TM4C firmware.
 *
 * PAN:
 * 45 - 135 degrees allowed
 *
 * TILT:
 * 55 - 125 degrees allowed
 */
typedef struct
{
    uint32_t pan;
    uint32_t tilt;

} actuator_position_t;


/*
 * CONTROL task cycles through this deterministic motion pattern.
 *
 * Each position is maintained for approximately one second.
 */
static const actuator_position_t g_motion_pattern[] =
{
    {90U,  90U},
    {70U,  90U},
    {110U, 90U},
    {90U,  70U},
    {90U,  110U},
    {90U,  90U}
};

#define MOTION_PATTERN_COUNT \
    (sizeof(g_motion_pattern) / sizeof(g_motion_pattern[0]))


/* --------------------------------------------------------------------------
 * Shared CONTROL -> COMM setpoint
 * -------------------------------------------------------------------------- */

typedef struct
{
    uint32_t pan;
    uint32_t tilt;

    /*
     * Generation is incremented whenever CONTROL publishes
     * a new actuator target.
     */
    uint64_t generation;

} shared_setpoint_t;


static shared_setpoint_t g_setpoint =
{
    .pan = 90U,
    .tilt = 90U,
    .generation = 0U
};


/*
 * CONTROL and COMM share the setpoint.
 *
 * The lock is intentionally held only for a very short copy/update.
 * UART I/O is never performed while this mutex is locked.
 */
static pthread_mutex_t g_setpoint_mutex =
    PTHREAD_MUTEX_INITIALIZER;


/* --------------------------------------------------------------------------
 * Communication health information
 * -------------------------------------------------------------------------- */

typedef struct
{
    uint64_t successful_transactions;
    uint64_t failed_transactions;
    uint32_t consecutive_failures;

    int64_t last_ack_ns;

    bool healthy;

} communication_health_t;


static communication_health_t g_comm_health =
{
    .successful_transactions = 0U,
    .failed_transactions = 0U,
    .consecutive_failures = 0U,
    .last_ack_ns = 0,
    .healthy = false
};


static pthread_mutex_t g_health_mutex =
    PTHREAD_MUTEX_INITIALIZER;


/* --------------------------------------------------------------------------
 * UART file descriptor
 * -------------------------------------------------------------------------- */

static int g_uart_fd = -1;


/* --------------------------------------------------------------------------
 * Common timing origin
 * -------------------------------------------------------------------------- */

static struct timespec g_start_time;


/* --------------------------------------------------------------------------
 * Service model
 * -------------------------------------------------------------------------- */

typedef enum
{
    SERVICE_CONTROL = 0,
    SERVICE_COMM,
    SERVICE_HEALTH,
    SERVICE_MONITOR,
    SERVICE_LOGGER

} service_id_t;


typedef struct
{
    service_id_t id;

    const char *name;

    int64_t period_ns;
    int64_t deadline_ns;

    int priority;

    uint32_t cycles;

    /*
     * Timing statistics.
     */
    double average_jitter_us;
    double worst_jitter_us;

    double average_execution_us;
    double observed_max_execution_us;

    double average_response_us;
    double worst_response_us;

    uint32_t deadline_misses;

} service_t;


/*
 * Rate Monotonic service table.
 *
 * Shorter period -> higher fixed priority.
 */
static service_t g_services[SERVICE_COUNT] =
{
    {
        .id = SERVICE_CONTROL,
        .name = "CONTROL",
        .period_ns = 20000000LL,
        .deadline_ns = 20000000LL,
        .priority = 80
    },

    {
        .id = SERVICE_COMM,
        .name = "COMM",
        .period_ns = 50000000LL,
        .deadline_ns = 50000000LL,
        .priority = 70
    },

    {
        .id = SERVICE_HEALTH,
        .name = "HEALTH",
        .period_ns = 100000000LL,
        .deadline_ns = 100000000LL,
        .priority = 60
    },

    {
        .id = SERVICE_MONITOR,
        .name = "MONITOR",
        .period_ns = 200000000LL,
        .deadline_ns = 200000000LL,
        .priority = 50
    },

    {
        .id = SERVICE_LOGGER,
        .name = "LOGGER",
        .period_ns = 1000000000LL,
        .deadline_ns = 1000000000LL,
        .priority = 40
    }
};


/* --------------------------------------------------------------------------
 * Timing helpers
 * -------------------------------------------------------------------------- */

static int64_t timespec_to_ns(
    const struct timespec *time_value
)
{
    return
        ((int64_t)time_value->tv_sec * 1000000000LL) +
        (int64_t)time_value->tv_nsec;
}


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


/*
 * Obtain current CLOCK_MONOTONIC time directly in nanoseconds.
 */
static int64_t monotonic_now_ns(void)
{
    struct timespec current_time;

    clock_gettime(
        CLOCK_MONOTONIC,
        &current_time
    );

    return timespec_to_ns(
        &current_time
    );
}


/* --------------------------------------------------------------------------
 * UART initialization
 * -------------------------------------------------------------------------- */

static int UART_Open(void)
{
    struct termios options;


    /*
     * Open Raspberry Pi UART.
     */
    g_uart_fd = open(
        UART_DEVICE,
        O_RDWR |
        O_NOCTTY |
        O_SYNC
    );


    if(g_uart_fd < 0)
    {
        perror("open UART");

        return -1;
    }


    /*
     * Read current serial-port configuration.
     */
    if(tcgetattr(
            g_uart_fd,
            &options) != 0)
    {
        perror("tcgetattr");

        close(g_uart_fd);

        g_uart_fd = -1;

        return -1;
    }


    /*
     * Raw serial mode:
     *
     * no terminal interpretation,
     * no echo,
     * no software line processing.
     */
    cfmakeraw(
        &options
    );


    /*
     * Baud rate:
     * 115200 bits/s
     */
    cfsetispeed(
        &options,
        B115200
    );

    cfsetospeed(
        &options,
        B115200
    );


    /*
     * UART format:
     *
     * 8 data bits
     * no parity
     * 1 stop bit
     * no hardware flow control
     */
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;

    options.c_cflag |=
        CLOCAL |
        CREAD;

    options.c_cflag &=
        ~(PARENB |
          PARODD |
          CSTOPB |
          CRTSCTS);


    /*
     * Reads are controlled using poll(), so termios-level
     * timeout behavior is disabled.
     */
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 0;


    if(tcsetattr(
            g_uart_fd,
            TCSANOW,
            &options) != 0)
    {
        perror("tcsetattr");

        close(g_uart_fd);

        g_uart_fd = -1;

        return -1;
    }


    /*
     * Remove any startup text that may already be waiting,
     * such as "TM4C READY".
     */
    tcflush(
        g_uart_fd,
        TCIOFLUSH
    );


    return 0;
}


/* --------------------------------------------------------------------------
 * UART write helper
 * -------------------------------------------------------------------------- */

static bool UART_WriteCommand(
    const char *command
)
{
    char transmit_buffer[64];

    size_t total_length;
    size_t total_written = 0U;


    int length = snprintf(
        transmit_buffer,
        sizeof(transmit_buffer),
        "%s\r\n",
        command
    );


    if((length <= 0) ||
       ((size_t)length >= sizeof(transmit_buffer)))
    {
        return false;
    }


    total_length =
        (size_t)length;


    /*
     * Handle possible partial writes.
     */
    while(total_written < total_length)
    {
        ssize_t written =
            write(
                g_uart_fd,
                transmit_buffer +
                    total_written,
                total_length -
                    total_written
            );


        if(written < 0)
        {
            if(errno == EINTR)
            {
                continue;
            }

            return false;
        }


        total_written +=
            (size_t)written;
    }


    return true;
}


/* --------------------------------------------------------------------------
 * UART response reader
 * -------------------------------------------------------------------------- */

static bool UART_ReadLine(
    char *buffer,
    size_t buffer_size,
    int timeout_ms
)
{
    struct pollfd poll_descriptor;

    int64_t start_ns;

    size_t index = 0U;


    if(buffer_size == 0U)
    {
        return false;
    }


    poll_descriptor.fd =
        g_uart_fd;

    poll_descriptor.events =
        POLLIN;


    start_ns =
        monotonic_now_ns();


    while(true)
    {
        int64_t elapsed_ns;

        int remaining_ms;

        int poll_result;


        elapsed_ns =
            monotonic_now_ns() -
            start_ns;


        remaining_ms =
            timeout_ms -
            (int)(elapsed_ns /
                  1000000LL);


        if(remaining_ms <= 0)
        {
            return false;
        }


        poll_result =
            poll(
                &poll_descriptor,
                1,
                remaining_ms
            );


        if(poll_result < 0)
        {
            if(errno == EINTR)
            {
                continue;
            }

            return false;
        }


        if(poll_result == 0)
        {
            return false;
        }


        if(poll_descriptor.revents &
           POLLIN)
        {
            char received_character;

            ssize_t received =
                read(
                    g_uart_fd,
                    &received_character,
                    1
                );


            if(received != 1)
            {
                continue;
            }


            /*
             * Ignore carriage return.
             */
            if(received_character == '\r')
            {
                continue;
            }


            /*
             * Line feed terminates TM4C response.
             */
            if(received_character == '\n')
            {
                if(index == 0U)
                {
                    continue;
                }


                buffer[index] = '\0';

                return true;
            }


            if(index <
               (buffer_size - 1U))
            {
                buffer[index++] =
                    received_character;
            }
            else
            {
                return false;
            }
        }
    }
}


/* --------------------------------------------------------------------------
 * Communication-health update
 * -------------------------------------------------------------------------- */

static void RecordCommunicationResult(
    bool success
)
{
    pthread_mutex_lock(
        &g_health_mutex
    );


    if(success)
    {
        g_comm_health.
            successful_transactions++;

        g_comm_health.
            consecutive_failures = 0U;

        g_comm_health.
            last_ack_ns =
                monotonic_now_ns();
    }
    else
    {
        g_comm_health.
            failed_transactions++;

        g_comm_health.
            consecutive_failures++;
    }


    pthread_mutex_unlock(
        &g_health_mutex
    );
}


/* --------------------------------------------------------------------------
 * UART request/response transaction
 * -------------------------------------------------------------------------- */

static bool UART_Transaction(
    const char *command,
    const char *expected_response
)
{
    char response[64];

    bool success = false;


    /*
     * Discard stale bytes before beginning a new request.
     */
    tcflush(
        g_uart_fd,
        TCIFLUSH
    );


    if(UART_WriteCommand(command))
    {
        if(UART_ReadLine(
                response,
                sizeof(response),
                UART_RESPONSE_TIMEOUT_MS))
        {
            if(strcmp(
                    response,
                    expected_response) == 0)
            {
                success = true;
            }
        }
    }


    RecordCommunicationResult(
        success
    );


    return success;
}


/* --------------------------------------------------------------------------
 * CONTROL service
 * -------------------------------------------------------------------------- */

static void CONTROL_Job(
    uint32_t cycle
)
{
    /*
     * 20 ms CONTROL period.
     *
     * 50 cycles = approximately one second.
     */
    uint32_t pattern_index =
        ((cycle - 1U) / 50U) %
        MOTION_PATTERN_COUNT;

    static uint32_t previous_pattern_index =
        UINT32_MAX;


    /*
     * Publish only when the requested target changes.
     */
    if(pattern_index !=
       previous_pattern_index)
    {
        pthread_mutex_lock(
            &g_setpoint_mutex
        );


        g_setpoint.pan =
            g_motion_pattern[
                pattern_index
            ].pan;


        g_setpoint.tilt =
            g_motion_pattern[
                pattern_index
            ].tilt;


        g_setpoint.generation++;


        pthread_mutex_unlock(
            &g_setpoint_mutex
        );


        previous_pattern_index =
            pattern_index;
    }
}


/* --------------------------------------------------------------------------
 * COMM service
 * -------------------------------------------------------------------------- */

static void COMM_Job(void)
{
    shared_setpoint_t local_setpoint;

    static uint64_t last_successful_generation =
        UINT64_MAX;


    /*
     * Copy the CONTROL target quickly.
     *
     * UART I/O happens after releasing this mutex.
     */
    pthread_mutex_lock(
        &g_setpoint_mutex
    );


    local_setpoint =
        g_setpoint;


    pthread_mutex_unlock(
        &g_setpoint_mutex
    );


    /*
     * No new CONTROL command.
     */
    if(local_setpoint.generation ==
   last_successful_generation)
   {
    /*
     * No new actuator target is waiting.
     *
     * Send a lightweight heartbeat so the HEALTH service can
     * distinguish an idle control system from a lost UART link.
     */
    UART_Transaction(
        "PING",
        "ACK"
    );

    return;
   }


    char command[32];

    bool pan_ok;
    bool tilt_ok;


    /* ----------------------------------------------------------------------
     * Send PAN command
     * ---------------------------------------------------------------------- */

    snprintf(
        command,
        sizeof(command),
        "PAN %u",
        local_setpoint.pan
    );


    pan_ok =
        UART_Transaction(
            command,
            "PAN_OK"
        );


    /* ----------------------------------------------------------------------
     * Send TILT command
     * ---------------------------------------------------------------------- */

    snprintf(
        command,
        sizeof(command),
        "TILT %u",
        local_setpoint.tilt
    );


    tilt_ok =
        UART_Transaction(
            command,
            "TILT_OK"
        );


    /*
     * Mark this setpoint as completed only after both axis
     * transactions have succeeded.
     *
     * A failed command will automatically be retried during
     * the next COMM release.
     */
    if(pan_ok &&
       tilt_ok)
    {
        last_successful_generation =
            local_setpoint.generation;
    }
}


/* --------------------------------------------------------------------------
 * HEALTH service
 * -------------------------------------------------------------------------- */

static void HEALTH_Job(void)
{
    int64_t last_ack_ns;

    uint32_t consecutive_failures;


    pthread_mutex_lock(
        &g_health_mutex
    );


    last_ack_ns =
        g_comm_health.last_ack_ns;


    consecutive_failures =
        g_comm_health.
            consecutive_failures;


    pthread_mutex_unlock(
        &g_health_mutex
    );


    /*
     * Current preliminary health rule:
     *
     * - fewer than 3 consecutive communication failures
     * - most recent ACK not older than 300 ms
     *
     * A later milestone will convert this into the explicit
     * NORMAL / DEGRADED / SAFE state machine.
     */
    bool healthy =
        (consecutive_failures < 3U) &&
        (last_ack_ns != 0) &&
        ((monotonic_now_ns() -
          last_ack_ns) <
         300000000LL);


    pthread_mutex_lock(
        &g_health_mutex
    );


    g_comm_health.healthy =
        healthy;


    pthread_mutex_unlock(
        &g_health_mutex
    );
}


/* --------------------------------------------------------------------------
 * MONITOR workload
 * -------------------------------------------------------------------------- */

static void MONITOR_Job(void)
{
    /*
     * Placeholder for system-level monitoring.
     *
     * Keep a small deterministic workload so the service
     * remains visible to the scheduler.
     */
    volatile uint32_t result = 0U;

    uint32_t i;


    for(i = 0U;
        i < 3000U;
        i++)
    {
        result += i;
    }


    (void)result;
}


/* --------------------------------------------------------------------------
 * LOGGER workload
 * -------------------------------------------------------------------------- */

static void LOGGER_Job(void)
{
    /*
     * No terminal output is performed here because printf()
     * during the timed experiment would contaminate timing results.
     */
    volatile uint32_t result = 0U;

    uint32_t i;


    for(i = 0U;
        i < 1000U;
        i++)
    {
        result += i;
    }


    (void)result;
}


/* --------------------------------------------------------------------------
 * Service dispatcher
 * -------------------------------------------------------------------------- */

static void ExecuteServiceJob(
    service_t *service,
    uint32_t cycle
)
{
    switch(service->id)
    {
        case SERVICE_CONTROL:

            CONTROL_Job(
                cycle
            );

            break;


        case SERVICE_COMM:

            COMM_Job();

            break;


        case SERVICE_HEALTH:

            HEALTH_Job();

            break;


        case SERVICE_MONITOR:

            MONITOR_Job();

            break;


        case SERVICE_LOGGER:

            LOGGER_Job();

            break;


        default:

            break;
    }
}


/* --------------------------------------------------------------------------
 * Common periodic service thread
 * -------------------------------------------------------------------------- */

static void *ServiceThread(
    void *argument
)
{
    service_t *service =
        (service_t *)argument;


    struct timespec next_release;

    struct timespec actual_start;

    struct timespec finish;


    int64_t total_jitter_ns = 0;
    int64_t total_execution_ns = 0;
    int64_t total_response_ns = 0;

    int64_t worst_jitter_ns = 0;
    int64_t max_execution_ns = 0;
    int64_t worst_response_ns = 0;

    uint32_t deadline_misses = 0U;

    uint32_t cycle;


    next_release =
        g_start_time;


    for(cycle = 1U;
        cycle <= service->cycles;
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


        clock_gettime(
            CLOCK_MONOTONIC,
            &actual_start
        );


        int64_t scheduled_ns =
            timespec_to_ns(
                &next_release
            );


        int64_t start_ns =
            timespec_to_ns(
                &actual_start
            );


        int64_t jitter_ns =
            start_ns -
            scheduled_ns;


        /*
         * Execute the actual service.
         */
        ExecuteServiceJob(
            service,
            cycle
        );


        clock_gettime(
            CLOCK_MONOTONIC,
            &finish
        );


        int64_t finish_ns =
            timespec_to_ns(
                &finish
            );


        int64_t execution_ns =
            finish_ns -
            start_ns;


        int64_t response_ns =
            finish_ns -
            scheduled_ns;


        if(response_ns >
           service->deadline_ns)
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


        /*
         * Absolute-time periodic release.
         */
        add_ns(
            &next_release,
            service->period_ns
        );
    }


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

    double total_observed_utilization =
        0.0;


    /* ----------------------------------------------------------------------
     * Open Raspberry Pi UART
     * ---------------------------------------------------------------------- */

    if(UART_Open() != 0)
    {
        return 1;
    }


    /*
     * Give the serial interface a short settling period.
     */
    usleep(
        100000
    );


    /*
     * Clear possible TM4C startup text.
     */
    tcflush(
        g_uart_fd,
        TCIFLUSH
    );


    /* ----------------------------------------------------------------------
     * Verify TM4C communication before starting RT services
     * ---------------------------------------------------------------------- */

    printf(
        "Verifying TM4C123 communication...\n"
    );


    if(!UART_Transaction(
            "PING",
            "ACK"))
    {
        fprintf(
            stderr,
            "[FAIL] TM4C123 did not respond to PING.\n"
        );

        close(
            g_uart_fd
        );

        return 1;
    }


    printf(
        "[PASS] TM4C123 UART verified.\n\n"
    );


    /* ----------------------------------------------------------------------
     * Calculate task releases
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
     * Common release origin
     * ---------------------------------------------------------------------- */

    clock_gettime(
        CLOCK_MONOTONIC,
        &g_start_time
    );


    /*
     * Give all threads one second to be created before release zero.
     */
    add_ns(
        &g_start_time,
        1000000000LL
    );


    /* ----------------------------------------------------------------------
     * Print architecture
     * ---------------------------------------------------------------------- */

    printf(
        "=== REAL-TIME UART MULTI-SERVICE TEST ===\n\n"
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
        "\nCONTROL -> shared setpoint -> COMM -> UART -> TM4C123\n"
    );

    printf(
        "Scheduler: SCHED_FIFO / Rate Monotonic\n"
    );

    printf(
        "Duration : 60 seconds\n\n"
    );


    /* ----------------------------------------------------------------------
     * Create SCHED_FIFO service threads
     * ---------------------------------------------------------------------- */

    for(i = 0U;
        i < SERVICE_COUNT;
        i++)
    {
        int result;


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
            &scheduling_parameters,
            0,
            sizeof(scheduling_parameters)
        );


        scheduling_parameters.
            sched_priority =
                g_services[i].
                    priority;


        pthread_attr_setschedparam(
            &attributes[i],
            &scheduling_parameters
        );


        result =
            pthread_create(
                &threads[i],
                &attributes[i],
                ServiceThread,
                &g_services[i]
            );


        if(result != 0)
        {
            fprintf(
                stderr,
                "Failed to create %s: %s\n",
                g_services[i].name,
                strerror(result)
            );


            close(
                g_uart_fd
            );


            return 1;
        }
    }


    /* ----------------------------------------------------------------------
     * Wait for all RT services
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
     * Timing results
     * ---------------------------------------------------------------------- */

    printf(
        "\n=== SERVICE TIMING RESULTS ===\n\n"
    );


    for(i = 0U;
        i < SERVICE_COUNT;
        i++)
    {
        double utilization;


        utilization =
            (
                g_services[i].
                    observed_max_execution_us *
                1000.0
            ) /
            g_services[i].
                period_ns;


        total_observed_utilization +=
            utilization;


        printf(
            "[%s]\n",
            g_services[i].name
        );


        printf(
            "  Period            : %.3f ms\n",
            g_services[i].
                period_ns /
                1000000.0
        );


        printf(
            "  Priority          : %d\n",
            g_services[i].
                priority
        );


        printf(
            "  Average jitter    : %.3f us\n",
            g_services[i].
                average_jitter_us
        );


        printf(
            "  Worst jitter      : %.3f us\n",
            g_services[i].
                worst_jitter_us
        );


        printf(
            "  Average execution : %.3f us\n",
            g_services[i].
                average_execution_us
        );


        printf(
            "  Observed max exec : %.3f us\n",
            g_services[i].
                observed_max_execution_us
        );


        printf(
            "  Average response  : %.3f us\n",
            g_services[i].
                average_response_us
        );


        printf(
            "  Worst response    : %.3f us\n",
            g_services[i].
                worst_response_us
        );


        printf(
            "  Deadline misses   : %u / %u\n",
            g_services[i].
                deadline_misses,
            g_services[i].
                cycles
        );


        printf(
            "  Observed util.    : %.6f\n\n",
            utilization
        );
    }


    /* ----------------------------------------------------------------------
     * Communication health summary
     * ---------------------------------------------------------------------- */

    pthread_mutex_lock(
        &g_health_mutex
    );


    communication_health_t final_health =
        g_comm_health;


    pthread_mutex_unlock(
        &g_health_mutex
    );


    printf(
        "=== COMMUNICATION HEALTH ===\n"
    );


    printf(
        "Successful UART transactions : %llu\n",
        (unsigned long long)
            final_health.
                successful_transactions
    );


    printf(
        "Failed UART transactions     : %llu\n",
        (unsigned long long)
            final_health.
                failed_transactions
    );


    printf(
        "Consecutive failures         : %u\n",
        final_health.
            consecutive_failures
    );


    printf(
        "HEALTH state                 : %s\n",
        final_health.healthy ?
            "HEALTHY" :
            "UNHEALTHY"
    );


    /* ----------------------------------------------------------------------
     * Final commanded actuator position
     * ---------------------------------------------------------------------- */

    pthread_mutex_lock(
        &g_setpoint_mutex
    );


    shared_setpoint_t final_setpoint =
        g_setpoint;


    pthread_mutex_unlock(
        &g_setpoint_mutex
    );


    printf(
        "\n=== FINAL SETPOINT ===\n"
    );


    printf(
        "PAN  : %u degrees\n",
        final_setpoint.pan
    );


    printf(
        "TILT : %u degrees\n",
        final_setpoint.tilt
    );


    /* ----------------------------------------------------------------------
     * RM utilization summary
     * ---------------------------------------------------------------------- */

    printf(
        "\n=== OBSERVED PROCESSOR UTILIZATION ===\n"
    );


    printf(
        "Sum(C_i / T_i) = %.6f\n",
        total_observed_utilization
    );


    printf(
        "RM sufficient bound for 5 tasks ~= 0.7435\n"
    );


    if(total_observed_utilization <=
       0.7435)
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


    /*
     * Return TM4C to center before leaving the experiment.
     */
    UART_Transaction(
        "CENTER",
        "CENTER_OK"
    );


    close(
        g_uart_fd
    );


    return 0;
}
