/*
 * Project:
 * Real-Time Safety-Critical Actuator Control & Fault-Injection Testbed
 *
 * Milestone:
 * Fault-Tolerant Multi-Service Real-Time Control System
 *
 * Purpose:
 * - Run five periodic services using SCHED_FIFO / Rate Monotonic scheduling
 * - Control a TM4C123-based dual-axis actuator through UART
 * - Supervise communication health
 * - Inject a controlled communication-loss fault
 * - Transition through NORMAL, DEGRADED and SAFE states
 * - Recover automatically after communication becomes stable again
 *
 * System architecture:
 *
 * CONTROL (20 ms)
 *      |
 *      v
 * shared actuator setpoint
 *      |
 *      v
 * COMM (50 ms)
 *      |
 *      v
 * UART -> TM4C123 -> hardware PWM -> PAN/TILT
 *
 * HEALTH (100 ms)
 *      |
 *      +--> NORMAL
 *      +--> DEGRADED
 *      +--> SAFE
 *
 * TM4C123 additionally contains an independent local communication
 * watchdog. If Raspberry Pi traffic disappears for 300 ms, the MCU
 * centers both actuators without requiring a CENTER command.
 *
 * Fault-injection interval:
 *
 * 0 - 4 s  : normal communication
 * 4 - 6 s  : Raspberry Pi deliberately suppresses UART traffic
 * 6 - 10 s : communication restored
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


/* ==========================================================================
 * Experiment configuration
 * ========================================================================== */

#define EXPERIMENT_DURATION_NS       10000000000LL

#define SERVICE_COUNT                5U

#define FAULT_START_NS               4000000000LL
#define FAULT_END_NS                 6000000000LL


/* ==========================================================================
 * UART configuration
 * ========================================================================== */

#define UART_DEVICE                  "/dev/serial0"
#define UART_RESPONSE_TIMEOUT_MS     10


/* ==========================================================================
 * System-state thresholds
 * ========================================================================== */

/*
 * One or two consecutive failures:
 * NORMAL -> DEGRADED
 */
#define DEGRADED_FAILURE_THRESHOLD   1U

/*
 * Three consecutive failures:
 * DEGRADED -> SAFE
 */
#define SAFE_FAILURE_THRESHOLD       3U

/*
 * SAFE mode requires several consecutive successful communication
 * transactions before returning to NORMAL.
 */
#define RECOVERY_SUCCESS_THRESHOLD   5U


/* ==========================================================================
 * System state
 * ========================================================================== */

typedef enum
{
    SYSTEM_NORMAL = 0,
    SYSTEM_DEGRADED,
    SYSTEM_SAFE

} system_state_t;


typedef enum
{
    FAULT_NONE = 0,
    FAULT_COMMUNICATION_LOSS

} fault_reason_t;


static system_state_t g_system_state =
    SYSTEM_NORMAL;


static fault_reason_t g_fault_reason =
    FAULT_NONE;


static pthread_mutex_t g_state_mutex =
    PTHREAD_MUTEX_INITIALIZER;


/* ==========================================================================
 * State-transition log
 * ========================================================================== */

#define MAX_STATE_EVENTS 16U


typedef struct
{
    double time_seconds;

    system_state_t previous_state;
    system_state_t new_state;

    fault_reason_t reason;

} state_event_t;


static state_event_t g_state_events[
    MAX_STATE_EVENTS
];


static uint32_t g_state_event_count =
    0U;


/* ==========================================================================
 * Actuator setpoint
 * ========================================================================== */

typedef struct
{
    uint32_t pan;
    uint32_t tilt;

} actuator_position_t;


/*
 * Motion sequence used while the system is in NORMAL state.
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


typedef struct
{
    uint32_t pan;
    uint32_t tilt;

    uint64_t generation;

} shared_setpoint_t;


static shared_setpoint_t g_setpoint =
{
    .pan = 90U,
    .tilt = 90U,
    .generation = 0U
};


static pthread_mutex_t g_setpoint_mutex =
    PTHREAD_MUTEX_INITIALIZER;


/* ==========================================================================
 * Communication health
 * ========================================================================== */

typedef struct
{
    uint64_t successful_transactions;
    uint64_t failed_transactions;

    uint32_t consecutive_failures;
    uint32_t consecutive_successes;

    int64_t last_ack_ns;

} communication_health_t;


static communication_health_t g_comm_health =
{
    .successful_transactions = 0U,
    .failed_transactions = 0U,

    .consecutive_failures = 0U,
    .consecutive_successes = 0U,

    .last_ack_ns = 0
};


static pthread_mutex_t g_health_mutex =
    PTHREAD_MUTEX_INITIALIZER;


/* ==========================================================================
 * UART file descriptor
 * ========================================================================== */

static int g_uart_fd =
    -1;


/* ==========================================================================
 * Timing origin
 * ========================================================================== */

static struct timespec g_start_time;


/* ==========================================================================
 * Service definitions
 * ========================================================================== */

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
     * Timing results.
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
 * Rate Monotonic ordering:
 *
 * shorter period -> higher fixed priority
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


/* ==========================================================================
 * Timing helpers
 * ========================================================================== */

static int64_t timespec_to_ns(
    const struct timespec *time_value
)
{
    return
        ((int64_t)time_value->tv_sec *
         1000000000LL) +
        (int64_t)time_value->tv_nsec;
}


static void add_ns(
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


/*
 * Return elapsed experiment time.
 */
static int64_t experiment_elapsed_ns(void)
{
    return
        monotonic_now_ns() -
        timespec_to_ns(
            &g_start_time
        );
}


/* ==========================================================================
 * Text helpers
 * ========================================================================== */

static const char *SystemStateToString(
    system_state_t state
)
{
    switch(state)
    {
        case SYSTEM_NORMAL:
            return "NORMAL";

        case SYSTEM_DEGRADED:
            return "DEGRADED";

        case SYSTEM_SAFE:
            return "SAFE";

        default:
            return "UNKNOWN";
    }
}


static const char *FaultReasonToString(
    fault_reason_t reason
)
{
    switch(reason)
    {
        case FAULT_NONE:
            return "NONE";

        case FAULT_COMMUNICATION_LOSS:
            return "COMMUNICATION_LOSS";

        default:
            return "UNKNOWN";
    }
}


/* ==========================================================================
 * State handling
 * ========================================================================== */

static system_state_t GetSystemState(void)
{
    system_state_t state;


    pthread_mutex_lock(
        &g_state_mutex
    );


    state =
        g_system_state;


    pthread_mutex_unlock(
        &g_state_mutex
    );


    return state;
}


/*
 * Store state transitions in memory instead of printing from the
 * real-time thread. Console output during the timing experiment
 * would distort timing measurements.
 */
static void SetSystemState(
    system_state_t new_state,
    fault_reason_t reason
)
{
    pthread_mutex_lock(
        &g_state_mutex
    );


    if(new_state !=
       g_system_state)
    {
        system_state_t previous_state =
            g_system_state;


        if(g_state_event_count <
           MAX_STATE_EVENTS)
        {
            state_event_t *event =
                &g_state_events[
                    g_state_event_count
                ];


            event->time_seconds =
                experiment_elapsed_ns() /
                1000000000.0;


            event->previous_state =
                previous_state;


            event->new_state =
                new_state;


            event->reason =
                reason;


            g_state_event_count++;
        }


        g_system_state =
            new_state;


        g_fault_reason =
            reason;
    }


    pthread_mutex_unlock(
        &g_state_mutex
    );
}


/* ==========================================================================
 * UART initialization
 * ========================================================================== */

static int UART_Open(void)
{
    struct termios options;


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


    if(tcgetattr(
            g_uart_fd,
            &options) != 0)
    {
        perror("tcgetattr");

        close(
            g_uart_fd
        );


        g_uart_fd =
            -1;


        return -1;
    }


    /*
     * Disable terminal-style character processing.
     */
    cfmakeraw(
        &options
    );


    cfsetispeed(
        &options,
        B115200
    );


    cfsetospeed(
        &options,
        B115200
    );


    /*
     * 8N1, no hardware flow control.
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


    options.c_cc[VMIN] =
        0;

    options.c_cc[VTIME] =
        0;


    if(tcsetattr(
            g_uart_fd,
            TCSANOW,
            &options) != 0)
    {
        perror("tcsetattr");

        close(
            g_uart_fd
        );


        g_uart_fd =
            -1;


        return -1;
    }


    tcflush(
        g_uart_fd,
        TCIOFLUSH
    );


    return 0;
}


/* ==========================================================================
 * UART helpers
 * ========================================================================== */

static bool UART_WriteCommand(
    const char *command
)
{
    char transmit_buffer[64];

    size_t total_length;
    size_t total_written = 0U;


    int length =
        snprintf(
            transmit_buffer,
            sizeof(transmit_buffer),
            "%s\r\n",
            command
        );


    if((length <= 0) ||
       ((size_t)length >=
        sizeof(transmit_buffer)))
    {
        return false;
    }


    total_length =
        (size_t)length;


    while(total_written <
          total_length)
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


static bool UART_ReadLine(
    char *buffer,
    size_t buffer_size,
    int timeout_ms
)
{
    struct pollfd poll_descriptor;

    int64_t start_ns =
        monotonic_now_ns();

    size_t index =
        0U;


    poll_descriptor.fd =
        g_uart_fd;

    poll_descriptor.events =
        POLLIN;


    while(true)
    {
        int64_t elapsed_ns =
            monotonic_now_ns() -
            start_ns;


        int remaining_ms =
            timeout_ms -
            (int)(
                elapsed_ns /
                1000000LL
            );


        if(remaining_ms <= 0)
        {
            return false;
        }


        int poll_result =
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


            if(received_character ==
               '\r')
            {
                continue;
            }


            if(received_character ==
               '\n')
            {
                if(index == 0U)
                {
                    continue;
                }


                buffer[index] =
                    '\0';


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


/* ==========================================================================
 * Communication health
 * ========================================================================== */

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
            consecutive_successes++;


        g_comm_health.
            consecutive_failures =
                0U;


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


        g_comm_health.
            consecutive_successes =
                0U;
    }


    pthread_mutex_unlock(
        &g_health_mutex
    );
}


/* ==========================================================================
 * UART transaction
 * ========================================================================== */

static bool UART_Transaction(
    const char *command,
    const char *expected_response
)
{
    char response[64];

    bool success =
        false;


    /*
     * Remove stale receive data.
     */
    tcflush(
        g_uart_fd,
        TCIFLUSH
    );


    if(UART_WriteCommand(
            command))
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
                success =
                    true;
            }
        }
    }


    RecordCommunicationResult(
        success
    );


    return success;
}


/* ==========================================================================
 * Fault injection
 * ========================================================================== */

static bool CommunicationFaultActive(void)
{
    int64_t elapsed_ns =
        experiment_elapsed_ns();


    return
        (elapsed_ns >=
         FAULT_START_NS) &&
        (elapsed_ns <
         FAULT_END_NS);
}


/* ==========================================================================
 * CONTROL service
 * ========================================================================== */

static void CONTROL_Job(
    uint32_t cycle
)
{
    /*
     * Motion generation is inhibited unless the system is NORMAL.
     */
    if(GetSystemState() !=
       SYSTEM_NORMAL)
    {
        return;
    }


    /*
     * CONTROL period is 20 ms.
     *
     * 50 cycles correspond to approximately one second.
     */
    uint32_t pattern_index =
        ((cycle - 1U) / 50U) %
        MOTION_PATTERN_COUNT;


    static uint32_t previous_pattern_index =
        UINT32_MAX;


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


/* ==========================================================================
 * COMM service
 * ========================================================================== */

static void COMM_Job(void)
{
    shared_setpoint_t local_setpoint;


    static uint64_t last_successful_generation =
        UINT64_MAX;


    /*
     * Fault injection:
     *
     * Deliberately suppress ALL UART traffic.
     *
     * No heartbeat.
     * No actuator command.
     *
     * This simultaneously:
     *
     * 1. Creates communication failures on Raspberry Pi.
     * 2. Causes the independent TM4C local watchdog to expire.
     */
    if(CommunicationFaultActive())
    {
        RecordCommunicationResult(
            false
        );


        return;
    }


    pthread_mutex_lock(
        &g_setpoint_mutex
    );


    local_setpoint =
        g_setpoint;


    pthread_mutex_unlock(
        &g_setpoint_mutex
    );


    /*
     * No new setpoint:
     *
     * send lightweight heartbeat.
     */
    if(local_setpoint.generation ==
       last_successful_generation)
    {
        UART_Transaction(
            "PING",
            "ACK"
        );


        return;
    }


    char command[32];


    bool pan_ok;
    bool tilt_ok;


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


    if(pan_ok &&
       tilt_ok)
    {
        last_successful_generation =
            local_setpoint.generation;
    }
}


/* ==========================================================================
 * HEALTH service
 * ========================================================================== */

static void HEALTH_Job(void)
{
    uint32_t consecutive_failures;
    uint32_t consecutive_successes;


    pthread_mutex_lock(
        &g_health_mutex
    );


    consecutive_failures =
        g_comm_health.
            consecutive_failures;


    consecutive_successes =
        g_comm_health.
            consecutive_successes;


    pthread_mutex_unlock(
        &g_health_mutex
    );


    system_state_t current_state =
        GetSystemState();


    /*
     * Critical communication loss.
     */
    if(consecutive_failures >=
       SAFE_FAILURE_THRESHOLD)
    {
        SetSystemState(
            SYSTEM_SAFE,
            FAULT_COMMUNICATION_LOSS
        );


        return;
    }


    /*
     * Initial communication degradation.
     */
    if(consecutive_failures >=
       DEGRADED_FAILURE_THRESHOLD)
    {
        SetSystemState(
            SYSTEM_DEGRADED,
            FAULT_COMMUNICATION_LOSS
        );


        return;
    }


    /*
     * Recovery from SAFE requires several consecutive
     * successful communication transactions.
     */
    if(current_state ==
       SYSTEM_SAFE)
    {
        if(consecutive_successes >=
           RECOVERY_SUCCESS_THRESHOLD)
        {
            SetSystemState(
                SYSTEM_NORMAL,
                FAULT_NONE
            );
        }


        return;
    }


    /*
     * DEGRADED recovery also requires stable communication.
     */
    if(current_state ==
       SYSTEM_DEGRADED)
    {
        if(consecutive_successes >=
           RECOVERY_SUCCESS_THRESHOLD)
        {
            SetSystemState(
                SYSTEM_NORMAL,
                FAULT_NONE
            );
        }


        return;
    }


    /*
     * Healthy steady state.
     */
    SetSystemState(
        SYSTEM_NORMAL,
        FAULT_NONE
    );
}


/* ==========================================================================
 * MONITOR and LOGGER
 * ========================================================================== */

static void MONITOR_Job(void)
{
    volatile uint32_t result =
        0U;


    uint32_t i;


    for(i = 0U;
        i < 3000U;
        i++)
    {
        result +=
            i;
    }


    (void)result;
}


static void LOGGER_Job(void)
{
    volatile uint32_t result =
        0U;


    uint32_t i;


    for(i = 0U;
        i < 1000U;
        i++)
    {
        result +=
            i;
    }


    (void)result;
}


/* ==========================================================================
 * Service dispatcher
 * ========================================================================== */

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


/* ==========================================================================
 * Periodic service thread
 * ========================================================================== */

static void *ServiceThread(
    void *argument
)
{
    service_t *service =
        (service_t *)argument;


    struct timespec next_release =
        g_start_time;


    int64_t total_jitter_ns =
        0;

    int64_t total_execution_ns =
        0;

    int64_t total_response_ns =
        0;


    int64_t worst_jitter_ns =
        0;

    int64_t max_execution_ns =
        0;

    int64_t worst_response_ns =
        0;


    uint32_t deadline_misses =
        0U;


    uint32_t cycle;


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
                "%s sleep failed: %s\n",
                service->name,
                strerror(sleep_result)
            );


            return NULL;
        }


        struct timespec actual_start;
        struct timespec finish;


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


/* ==========================================================================
 * Main
 * ========================================================================== */

int main(void)
{
    pthread_t threads[
        SERVICE_COUNT
    ];


    pthread_attr_t attributes[
        SERVICE_COUNT
    ];


    struct sched_param scheduling_parameters;


    uint32_t i;


    double total_observed_utilization =
        0.0;


    /* ----------------------------------------------------------------------
     * Open UART
     * ---------------------------------------------------------------------- */

    if(UART_Open() != 0)
    {
        return 1;
    }


    usleep(
        100000
    );


    tcflush(
        g_uart_fd,
        TCIFLUSH
    );


    /* ----------------------------------------------------------------------
     * Pre-flight communication test
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
            "[FAIL] TM4C123 communication unavailable.\n"
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
     * Calculate number of releases
     * ---------------------------------------------------------------------- */

    for(i = 0U;
        i < SERVICE_COUNT;
        i++)
    {
        g_services[i].cycles =
            (uint32_t)(
                EXPERIMENT_DURATION_NS /
                g_services[i].
                    period_ns
            );
    }


    /* ----------------------------------------------------------------------
     * Common timing origin
     * ---------------------------------------------------------------------- */

    clock_gettime(
        CLOCK_MONOTONIC,
        &g_start_time
    );


    /*
     * Give threads one second for creation before
     * periodic execution begins.
     */
    add_ns(
        &g_start_time,
        1000000000LL
    );


    /* ----------------------------------------------------------------------
     * Display experiment configuration
     * ---------------------------------------------------------------------- */

    printf(
        "=== FAULT-TOLERANT REAL-TIME CONTROL TEST ===\n\n"
    );


    printf(
        "Fault injection : 4.0 s -> 6.0 s\n"
    );


    printf(
        "Injected fault  : COMPLETE UART TRAFFIC LOSS\n"
    );


    printf(
        "Pi states       : NORMAL -> DEGRADED -> SAFE -> NORMAL\n"
    );


    printf(
        "TM4C watchdog   : 300 ms local fail-safe\n\n"
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
            g_services[i].
                period_ns /
                1000000.0,
            g_services[i].
                priority,
            g_services[i].
                cycles
        );
    }


    printf("\n");


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
     * Wait for services
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
            "  Worst response    : %.3f us\n",
            g_services[i].
                worst_response_us
        );


        printf(
            "  Deadline misses   : %u / %u\n\n",
            g_services[i].
                deadline_misses,
            g_services[i].
                cycles
        );
    }


    /* ----------------------------------------------------------------------
     * Communication statistics
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
        "=== COMMUNICATION SUMMARY ===\n"
    );


    printf(
        "Successful transactions : %llu\n",
        (unsigned long long)
            final_health.
                successful_transactions
    );


    printf(
        "Failed transactions     : %llu\n",
        (unsigned long long)
            final_health.
                failed_transactions
    );


    printf(
        "Consecutive failures    : %u\n",
        final_health.
            consecutive_failures
    );


    printf(
        "Consecutive successes   : %u\n",
        final_health.
            consecutive_successes
    );


    /* ----------------------------------------------------------------------
     * State-transition history
     * ---------------------------------------------------------------------- */

    printf(
        "\n=== STATE TRANSITION LOG ===\n"
    );


    pthread_mutex_lock(
        &g_state_mutex
    );


    for(i = 0U;
        i < g_state_event_count;
        i++)
    {
        state_event_t *event =
            &g_state_events[i];


        printf(
            "t=%6.3f s | %-8s -> %-8s | %s\n",
            event->time_seconds,
            SystemStateToString(
                event->previous_state
            ),
            SystemStateToString(
                event->new_state
            ),
            FaultReasonToString(
                event->reason
            )
        );
    }


    system_state_t final_state =
        g_system_state;


    fault_reason_t final_fault =
        g_fault_reason;


    pthread_mutex_unlock(
        &g_state_mutex
    );


    printf(
        "\nFinal system state : %s\n",
        SystemStateToString(
            final_state
        )
    );


    printf(
        "Final fault reason : %s\n",
        FaultReasonToString(
            final_fault
        )
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


    /*
     * Return physical mechanism to center before exit.
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
