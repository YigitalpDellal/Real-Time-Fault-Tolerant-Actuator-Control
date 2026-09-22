/*
 * Project:
 * Real-Time Safety-Critical Actuator Control & Fault-Injection Testbed
 *
 * Milestone:
 * Supervised Timing-Fault Recovery
 *
 * Purpose:
 * - Run five periodic services with SCHED_FIFO / Rate Monotonic priorities
 * - Control the TM4C123 dual-axis actuator over UART
 * - Inject a CONTROL-task CPU overload between 4 and 6 seconds
 * - Detect CONTROL deadline overruns
 * - Transition NORMAL -> DEGRADED -> SAFE
 * - Shed the faulty workload after entering SAFE
 * - Command the physical actuator to CENTER
 * - Recover automatically after stable timing returns
 *
 * CONTROL:
 *   Period   = 20 ms
 *   Deadline = 20 ms
 *
 * Injected fault:
 *   25 ms busy workload
 *
 * Since 25 ms > 20 ms, CONTROL intentionally violates its deadline.
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

#define OVERLOAD_START_NS            4000000000LL
#define OVERLOAD_END_NS              6000000000LL

#define CONTROL_OVERLOAD_NS            25000000LL


/* ==========================================================================
 * Timing-fault supervision configuration
 * ========================================================================== */

/*
 * First consecutive CONTROL deadline miss:
 *
 * NORMAL -> DEGRADED
 */
#define DEGRADED_MISS_THRESHOLD          1U


/*
 * Three consecutive CONTROL deadline misses:
 *
 * DEGRADED -> SAFE
 */
#define SAFE_MISS_THRESHOLD              3U


/*
 * 25 healthy CONTROL periods:
 *
 * 25 * 20 ms = 500 ms
 *
 * required before recovery.
 */
#define RECOVERY_GOOD_CYCLES_THRESHOLD  25U


/* ==========================================================================
 * UART configuration
 * ========================================================================== */

#define UART_DEVICE                  "/dev/serial0"

#define UART_RESPONSE_TIMEOUT_MS     10


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
    FAULT_CONTROL_DEADLINE_OVERRUN

} fault_reason_t;


static system_state_t g_system_state =
    SYSTEM_NORMAL;


static fault_reason_t g_fault_reason =
    FAULT_NONE;


static pthread_mutex_t g_state_mutex =
    PTHREAD_MUTEX_INITIALIZER;


/*
 * When SAFE is entered, COMM must command
 * the actuator to its safe center position.
 */
static bool g_safe_center_pending =
    false;


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
 * CONTROL timing-health counters
 * ========================================================================== */

static uint32_t g_control_consecutive_misses =
    0U;


static uint32_t g_control_recovery_cycles =
    0U;


/* ==========================================================================
 * Actuator setpoints
 * ========================================================================== */

typedef struct
{
    uint32_t pan;
    uint32_t tilt;

} actuator_position_t;


/*
 * Normal deterministic motion pattern.
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

    int64_t last_ack_ns;

} communication_health_t;


static communication_health_t g_comm_health =
{
    .successful_transactions = 0U,
    .failed_transactions = 0U,

    .consecutive_failures = 0U,

    .last_ack_ns = 0
};


static pthread_mutex_t g_health_mutex =
    PTHREAD_MUTEX_INITIALIZER;


/* ==========================================================================
 * UART descriptor
 * ========================================================================== */

static int g_uart_fd =
    -1;


/* ==========================================================================
 * Common RT timing origin
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

    double average_jitter_us;
    double worst_jitter_us;

    double average_execution_us;
    double observed_max_execution_us;

    double average_response_us;
    double worst_response_us;

    uint32_t deadline_misses;

} service_t;


/*
 * Rate Monotonic priority ordering:
 *
 * shorter period -> higher priority
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


    return
        timespec_to_ns(
            &current_time
        );
}


static int64_t experiment_elapsed_ns(void)
{
    return
        monotonic_now_ns() -
        timespec_to_ns(
            &g_start_time
        );
}


/* ==========================================================================
 * Controlled CPU-overload generator
 * ========================================================================== */

/*
 * Intentionally consumes CPU for the requested interval.
 *
 * This is different from sleep():
 *
 * sleep() releases the CPU.
 * Busy waiting keeps CONTROL runnable.
 */
static void BusyWaitNs(
    int64_t duration_ns
)
{
    const int64_t finish_ns =
        monotonic_now_ns() +
        duration_ns;


    volatile uint64_t activity =
        0ULL;


    while(monotonic_now_ns() <
          finish_ns)
    {
        activity++;
    }


    (void)activity;
}


/* ==========================================================================
 * State text helpers
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

        case FAULT_CONTROL_DEADLINE_OVERRUN:
            return "CONTROL_DEADLINE_OVERRUN";

        default:
            return "UNKNOWN";
    }
}


/* ==========================================================================
 * System-state handling
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
 * State changes are stored in memory instead of printed immediately.
 *
 * This prevents terminal output from contaminating the RT measurement.
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


        /*
         * Request physical safe-center action once.
         */
        if(new_state ==
           SYSTEM_SAFE)
        {
            g_safe_center_pending =
                true;
        }
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


    g_uart_fd =
        open(
            UART_DEVICE,
            O_RDWR |
            O_NOCTTY |
            O_SYNC
        );


    if(g_uart_fd < 0)
    {
        perror(
            "open UART"
        );


        return -1;
    }


    if(tcgetattr(
            g_uart_fd,
            &options) != 0)
    {
        perror(
            "tcgetattr"
        );


        close(
            g_uart_fd
        );


        g_uart_fd =
            -1;


        return -1;
    }


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
     * 8 data bits, no parity, one stop bit.
     */
    options.c_cflag &= ~CSIZE;

    options.c_cflag |=
        CS8;


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
        perror(
            "tcsetattr"
        );


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
 * UART transmission
 * ========================================================================== */

static bool UART_WriteCommand(
    const char *command
)
{
    char transmit_buffer[64];


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


    size_t total_length =
        (size_t)length;


    size_t total_written =
        0U;


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
            if(errno ==
               EINTR)
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


/* ==========================================================================
 * UART receive
 * ========================================================================== */

static bool UART_ReadLine(
    char *buffer,
    size_t buffer_size,
    int timeout_ms
)
{
    struct pollfd poll_descriptor;


    poll_descriptor.fd =
        g_uart_fd;


    poll_descriptor.events =
        POLLIN;


    int64_t start_ns =
        monotonic_now_ns();


    size_t index =
        0U;


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
            if(errno ==
               EINTR)
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
 * Communication-health tracking
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
    }


    pthread_mutex_unlock(
        &g_health_mutex
    );
}


/* ==========================================================================
 * Complete UART request/response transaction
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
     * Remove stale input before starting the request.
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
 * CONTROL service
 * ========================================================================== */

static void CONTROL_Job(
    uint32_t cycle
)
{
    const int64_t elapsed_ns =
        experiment_elapsed_ns();


    /*
     * Deliberate deadline fault.
     *
     * The overload remains active in NORMAL and DEGRADED.
     *
     * Once SAFE is reached, the workload is shed.
     */
    if((GetSystemState() !=
        SYSTEM_SAFE) &&
       (elapsed_ns >=
        OVERLOAD_START_NS) &&
       (elapsed_ns <
        OVERLOAD_END_NS))
    {
        BusyWaitNs(
            CONTROL_OVERLOAD_NS
        );
    }


    /*
     * Actuator-motion generation is permitted only
     * while the system is fully NORMAL.
     */
    if(GetSystemState() !=
       SYSTEM_NORMAL)
    {
        return;
    }


    /*
     * CONTROL period = 20 ms.
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
 * CONTROL timing supervision
 * ========================================================================== */

static void UpdateControlTimingHealth(
    bool deadline_missed
)
{
    if(deadline_missed)
    {
        /*
         * Consecutive deadline failures are the timing-fault signal.
         */
        g_control_consecutive_misses++;


        g_control_recovery_cycles =
            0U;


        if(g_control_consecutive_misses >=
           SAFE_MISS_THRESHOLD)
        {
            SetSystemState(
                SYSTEM_SAFE,
                FAULT_CONTROL_DEADLINE_OVERRUN
            );
        }
        else if(g_control_consecutive_misses >=
                DEGRADED_MISS_THRESHOLD)
        {
            SetSystemState(
                SYSTEM_DEGRADED,
                FAULT_CONTROL_DEADLINE_OVERRUN
            );
        }


        return;
    }


    /*
     * Current CONTROL job met its deadline.
     */
    g_control_consecutive_misses =
        0U;


    system_state_t current_state =
        GetSystemState();


    if(current_state ==
       SYSTEM_NORMAL)
    {
        return;
    }


    /*
     * Do not begin timing recovery while the deliberate
     * overload window is still active.
     *
     * SAFE has shed the workload, but the external fault
     * condition conceptually remains active until 6 s.
     */
    if(experiment_elapsed_ns() <
       OVERLOAD_END_NS)
    {
        g_control_recovery_cycles =
            0U;


        return;
    }


    /*
     * Count consecutive healthy CONTROL cycles only
     * after the injected fault interval has ended.
     */
    g_control_recovery_cycles++;


    if(g_control_recovery_cycles >=
       RECOVERY_GOOD_CYCLES_THRESHOLD)
    {
        SetSystemState(
            SYSTEM_NORMAL,
            FAULT_NONE
        );
    }
}


/* ==========================================================================
 * COMM service
 * ========================================================================== */

static void COMM_Job(void)
{
    /*
     * Check whether entering SAFE requested a physical
     * actuator-center command.
     */
    pthread_mutex_lock(
        &g_state_mutex
    );


    bool center_pending =
        g_safe_center_pending;


    pthread_mutex_unlock(
        &g_state_mutex
    );


    /*
     * First SAFE action:
     *
     * command PAN/TILT to the safe center position.
     */
    if(center_pending)
    {
        if(UART_Transaction(
                "CENTER",
                "CENTER_OK"))
        {
            pthread_mutex_lock(
                &g_state_mutex
            );


            g_safe_center_pending =
                false;


            pthread_mutex_unlock(
                &g_state_mutex
            );
        }


        return;
    }


    /*
     * While SAFE, maintain communication but inhibit motion.
     */
    if(GetSystemState() ==
       SYSTEM_SAFE)
    {
        UART_Transaction(
            "PING",
            "ACK"
        );


        return;
    }


    shared_setpoint_t local_setpoint;


    static uint64_t last_successful_generation =
        UINT64_MAX;


    pthread_mutex_lock(
        &g_setpoint_mutex
    );


    local_setpoint =
        g_setpoint;


    pthread_mutex_unlock(
        &g_setpoint_mutex
    );


    /*
     * No new actuator target:
     *
     * send a heartbeat.
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
    /*
     * In this experiment, timing-fault state transitions are driven
     * directly by CONTROL deadline supervision.
     *
     * HEALTH still exists as an independent periodic service and
     * communication-health infrastructure remains active.
     */
    volatile uint32_t health_snapshot =
        0U;


    pthread_mutex_lock(
        &g_health_mutex
    );


    health_snapshot =
        g_comm_health.
            consecutive_failures;


    pthread_mutex_unlock(
        &g_health_mutex
    );


    (void)health_snapshot;
}


/* ==========================================================================
 * MONITOR service
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


/* ==========================================================================
 * LOGGER service
 * ========================================================================== */

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
 * Common periodic service thread
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
        while(sleep_result ==
              EINTR);


        if(sleep_result != 0)
        {
            fprintf(
                stderr,
                "%s sleep failed: %s\n",
                service->name,
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
         * Run the actual service.
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


        bool deadline_missed =
            response_ns >
            service->deadline_ns;


        if(deadline_missed)
        {
            deadline_misses++;
        }


        /*
         * CONTROL timing directly feeds the fault supervisor.
         */
        if(service->id ==
           SERVICE_CONTROL)
        {
            UpdateControlTimingHealth(
                deadline_missed
            );
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
         * Absolute periodic scheduling.
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
     * UART initialization
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
     * Pre-flight TM4C verification
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
     * Calculate periodic releases
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
     * Establish common scheduling origin
     * ---------------------------------------------------------------------- */

    clock_gettime(
        CLOCK_MONOTONIC,
        &g_start_time
    );


    /*
     * One-second setup interval before periodic execution begins.
     */
    add_ns(
        &g_start_time,
        1000000000LL
    );


    /* ----------------------------------------------------------------------
     * Experiment description
     * ---------------------------------------------------------------------- */

    printf(
        "=== SUPERVISED TIMING-FAULT TEST ===\n\n"
    );


    printf(
        "CONTROL period/deadline : 20 ms\n"
    );


    printf(
        "Injected workload       : 25 ms\n"
    );


    printf(
        "Fault interval          : 4.0 s -> 6.0 s\n"
    );


    printf(
        "SAFE threshold          : 3 consecutive CONTROL misses\n"
    );


    printf(
        "Recovery requirement    : 25 healthy CONTROL cycles\n\n"
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
            g_services[i].
                name,
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
     * Create SCHED_FIFO threads
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
                g_services[i].
                    name,
                strerror(
                    result
                )
            );


            close(
                g_uart_fd
            );


            return 1;
        }
    }


    /* ----------------------------------------------------------------------
     * Wait for all services
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
     * Service timing results
     * ---------------------------------------------------------------------- */

    printf(
        "\n=== SERVICE TIMING RESULTS ===\n\n"
    );


    for(i = 0U;
        i < SERVICE_COUNT;
        i++)
    {
        double utilization =
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
            g_services[i].
                name
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
     * Communication summary
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


    /* ----------------------------------------------------------------------
     * Timing-fault state history
     * ---------------------------------------------------------------------- */

    printf(
        "\n=== TIMING FAULT STATE LOG ===\n"
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
        "\nFinal system state        : %s\n",
        SystemStateToString(
            final_state
        )
    );


    printf(
        "Final fault reason        : %s\n",
        FaultReasonToString(
            final_fault
        )
    );


    printf(
        "CONTROL consecutive misses: %u\n",
        g_control_consecutive_misses
    );


    printf(
        "CONTROL recovery cycles   : %u\n",
        g_control_recovery_cycles
    );


    /* ----------------------------------------------------------------------
     * Final actuator target
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
     * Utilization summary
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
            "Observed maximum execution-time sum exceeds "
            "the classical RM sufficient bound.\n"
        );
    }


    /*
     * Always leave the physical mechanism in a known safe position.
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
