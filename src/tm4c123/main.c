#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "inc/hw_memmap.h"

#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
#include "driverlib/pwm.h"
#include "driverlib/systick.h"
#include "driverlib/interrupt.h"


/* ==========================================================================
 * Project:
 * Real-Time Safety-Critical Actuator Control & Fault-Injection Testbed
 *
 * Firmware milestone:
 * Dual-Axis Actuator Controller with Local Communication Watchdog
 *
 * Architecture:
 *
 * Raspberry Pi
 *      |
 *      | UART commands / heartbeat
 *      v
 * TM4C123
 *      |
 *      +--> PAN hardware PWM
 *      |
 *      +--> TILT hardware PWM
 *      |
 *      +--> 300 ms local communication watchdog
 *
 * Fail-safe behavior:
 *
 * If no VALID Raspberry Pi command is received for 300 ms:
 *
 *      PAN  -> 90 degrees
 *      TILT -> 90 degrees
 *
 * This behavior is implemented locally on the TM4C123.
 *
 * Therefore the safe position does NOT depend on Raspberry Pi
 * communication remaining available.
 *
 * UART:
 * PB0 -> UART1 RX
 * PB1 -> UART1 TX
 *
 * PWM:
 * PB6 -> M0PWM0 -> PAN
 * PB7 -> M0PWM1 -> TILT
 * ========================================================================== */


/* --------------------------------------------------------------------------
 * UART configuration
 * -------------------------------------------------------------------------- */

#define UART_BAUD_RATE              115200U


/* --------------------------------------------------------------------------
 * PWM / servo configuration
 * -------------------------------------------------------------------------- */

#define PWM_FREQUENCY_HZ            50U
#define PWM_DIVIDER                 64U

#define SERVO_MIN_PULSE_US          500U
#define SERVO_MAX_PULSE_US          2500U


/* --------------------------------------------------------------------------
 * Mechanical safety limits
 * -------------------------------------------------------------------------- */

#define PAN_MIN_ANGLE               45U
#define PAN_CENTER_ANGLE            90U
#define PAN_MAX_ANGLE               135U

#define TILT_MIN_ANGLE              55U
#define TILT_CENTER_ANGLE           90U
#define TILT_MAX_ANGLE              125U


/* --------------------------------------------------------------------------
 * Communication watchdog
 * -------------------------------------------------------------------------- */

/*
 * Communication timeout:
 *
 * Raspberry Pi normally sends communication activity every 50 ms.
 *
 * 300 ms therefore allows several missed communication periods
 * before the local fail-safe is activated.
 */
#define COMM_WATCHDOG_TIMEOUT_MS    300U


/* --------------------------------------------------------------------------
 * PWM runtime values
 * -------------------------------------------------------------------------- */

static uint32_t g_pwmClock;
static uint32_t g_pwmPeriod;


/* --------------------------------------------------------------------------
 * Watchdog state
 * -------------------------------------------------------------------------- */

/*
 * Incremented once every millisecond by SysTick.
 *
 * Reset whenever a valid protocol command is received.
 */
static volatile uint32_t g_msSinceValidMessage = 0U;


/*
 * Set by the SysTick interrupt when communication has been absent
 * for at least COMM_WATCHDOG_TIMEOUT_MS.
 */
static volatile bool g_watchdogExpired = false;


/*
 * Tracks whether the actuators are currently in the local fail-safe state.
 */
static volatile bool g_failsafeActive = false;


/* --------------------------------------------------------------------------
 * UART initialization
 * -------------------------------------------------------------------------- */

static void UART1_Init(void)
{
    SysCtlPeripheralEnable(
        SYSCTL_PERIPH_GPIOB
    );

    SysCtlPeripheralEnable(
        SYSCTL_PERIPH_UART1
    );


    while(!SysCtlPeripheralReady(
              SYSCTL_PERIPH_GPIOB))
    {
    }


    while(!SysCtlPeripheralReady(
              SYSCTL_PERIPH_UART1))
    {
    }


    /*
     * PB0 -> U1RX
     * PB1 -> U1TX
     */
    GPIOPinConfigure(
        GPIO_PB0_U1RX
    );

    GPIOPinConfigure(
        GPIO_PB1_U1TX
    );


    GPIOPinTypeUART(
        GPIO_PORTB_BASE,
        GPIO_PIN_0 |
        GPIO_PIN_1
    );


    /*
     * UART1:
     * 115200 baud
     * 8 data bits
     * no parity
     * 1 stop bit
     */
    UARTConfigSetExpClk(
        UART1_BASE,
        SysCtlClockGet(),
        UART_BAUD_RATE,
        UART_CONFIG_WLEN_8 |
        UART_CONFIG_STOP_ONE |
        UART_CONFIG_PAR_NONE
    );
}


/* --------------------------------------------------------------------------
 * UART transmit helper
 * -------------------------------------------------------------------------- */

static void UART1_SendString(
    const char *text
)
{
    while(*text)
    {
        UARTCharPut(
            UART1_BASE,
            *text++
        );
    }
}


/* --------------------------------------------------------------------------
 * Servo conversion
 * -------------------------------------------------------------------------- */

static uint32_t ServoAngleToPulseUs(
    uint32_t angle
)
{
    uint32_t pulseRange =
        SERVO_MAX_PULSE_US -
        SERVO_MIN_PULSE_US;


    return
        SERVO_MIN_PULSE_US +
        ((angle * pulseRange) /
         180U);
}


static uint32_t ServoAngleToPulseTicks(
    uint32_t angle
)
{
    uint32_t pulseUs =
        ServoAngleToPulseUs(
            angle
        );


    /*
     * 64-bit intermediate arithmetic prevents precision loss.
     */
    return (uint32_t)(
        ((uint64_t)g_pwmClock *
         pulseUs) /
        1000000ULL
    );
}


/* --------------------------------------------------------------------------
 * PAN actuator
 * -------------------------------------------------------------------------- */

static bool PAN_SetAngle(
    uint32_t angle
)
{
    if((angle < PAN_MIN_ANGLE) ||
       (angle > PAN_MAX_ANGLE))
    {
        return false;
    }


    PWMPulseWidthSet(
        PWM0_BASE,
        PWM_OUT_0,
        ServoAngleToPulseTicks(
            angle
        )
    );


    return true;
}


/* --------------------------------------------------------------------------
 * TILT actuator
 * -------------------------------------------------------------------------- */

static bool TILT_SetAngle(
    uint32_t angle
)
{
    if((angle < TILT_MIN_ANGLE) ||
       (angle > TILT_MAX_ANGLE))
    {
        return false;
    }


    PWMPulseWidthSet(
        PWM0_BASE,
        PWM_OUT_1,
        ServoAngleToPulseTicks(
            angle
        )
    );


    return true;
}


/* --------------------------------------------------------------------------
 * Safe center position
 * -------------------------------------------------------------------------- */

static void Actuator_Center(void)
{
    PAN_SetAngle(
        PAN_CENTER_ANGLE
    );


    TILT_SetAngle(
        TILT_CENTER_ANGLE
    );
}


/* --------------------------------------------------------------------------
 * PWM initialization
 * -------------------------------------------------------------------------- */

static void ActuatorPWM_Init(void)
{
    SysCtlPeripheralEnable(
        SYSCTL_PERIPH_PWM0
    );

    SysCtlPeripheralEnable(
        SYSCTL_PERIPH_GPIOB
    );


    while(!SysCtlPeripheralReady(
              SYSCTL_PERIPH_PWM0))
    {
    }


    while(!SysCtlPeripheralReady(
              SYSCTL_PERIPH_GPIOB))
    {
    }


    /*
     * 80 MHz / 64 = 1.25 MHz PWM clock.
     */
    SysCtlPWMClockSet(
        SYSCTL_PWMDIV_64
    );


    g_pwmClock =
        SysCtlClockGet() /
        PWM_DIVIDER;


    g_pwmPeriod =
        g_pwmClock /
        PWM_FREQUENCY_HZ;


    /*
     * PB6 -> PAN
     * PB7 -> TILT
     */
    GPIOPinConfigure(
        GPIO_PB6_M0PWM0
    );

    GPIOPinConfigure(
        GPIO_PB7_M0PWM1
    );


    GPIOPinTypePWM(
        GPIO_PORTB_BASE,
        GPIO_PIN_6 |
        GPIO_PIN_7
    );


    PWMGenConfigure(
        PWM0_BASE,
        PWM_GEN_0,
        PWM_GEN_MODE_DOWN |
        PWM_GEN_MODE_NO_SYNC
    );


    PWMGenPeriodSet(
        PWM0_BASE,
        PWM_GEN_0,
        g_pwmPeriod
    );


    /*
     * Start from a known safe position.
     */
    Actuator_Center();


    PWMOutputState(
        PWM0_BASE,
        PWM_OUT_0_BIT |
        PWM_OUT_1_BIT,
        true
    );


    PWMGenEnable(
        PWM0_BASE,
        PWM_GEN_0
    );
}


/* --------------------------------------------------------------------------
 * SysTick interrupt
 * -------------------------------------------------------------------------- */

/*
 * Called every 1 ms.
 *
 * This timing mechanism runs independently from UART parsing.
 */
static void SysTickHandler(void)
{
    /*
     * Saturating increment avoids integer wraparound
     * during very long operation.
     */
    if(g_msSinceValidMessage <
       UINT32_MAX)
    {
        g_msSinceValidMessage++;
    }


    /*
     * Do NOT manipulate the servos directly inside the ISR.
     *
     * Instead, set a flag and let the main loop perform
     * the fail-safe transition.
     */
    if(g_msSinceValidMessage >=
       COMM_WATCHDOG_TIMEOUT_MS)
    {
        g_watchdogExpired =
            true;
    }
}


/* --------------------------------------------------------------------------
 * Watchdog initialization
 * -------------------------------------------------------------------------- */

static void CommunicationWatchdog_Init(void)
{
    /*
     * 1 ms SysTick period:
     *
     * SysTick frequency =
     * system clock / 1000
     */
    SysTickPeriodSet(
        SysCtlClockGet() /
        1000U
    );


    /*
     * Dynamically register the SysTick interrupt handler.
     */
    SysTickIntRegister(
        SysTickHandler
    );


    SysTickIntEnable();
    SysTickEnable();


    /*
     * Enable processor interrupts globally.
     */
    IntMasterEnable();
}


/* --------------------------------------------------------------------------
 * Communication watchdog reset
 * -------------------------------------------------------------------------- */

/*
 * Called only after a valid protocol command has been recognized.
 */
static void MarkCommunicationAlive(void)
{
    g_msSinceValidMessage =
        0U;


    g_watchdogExpired =
        false;


    /*
     * A valid command means communication has recovered.
     */
    g_failsafeActive =
        false;
}


/* --------------------------------------------------------------------------
 * Local fail-safe supervision
 * -------------------------------------------------------------------------- */

static void CheckCommunicationFailsafe(void)
{
    /*
     * Enter fail-safe only once per communication-loss event.
     */
    if(g_watchdogExpired &&
       !g_failsafeActive)
    {
        /*
         * Critical behavior:
         *
         * This does NOT require Raspberry Pi communication.
         */
        Actuator_Center();


        g_failsafeActive =
            true;
    }
}


/* --------------------------------------------------------------------------
 * Numeric command parser
 * -------------------------------------------------------------------------- */

static bool ParseUnsignedInteger(
    const char *text,
    uint32_t *value
)
{
    uint32_t result = 0U;


    if(*text == '\0')
    {
        return false;
    }


    while(*text != '\0')
    {
        if((*text < '0') ||
           (*text > '9'))
        {
            return false;
        }


        result =
            (result * 10U) +
            (uint32_t)(
                *text - '0'
            );


        text++;
    }


    *value =
        result;


    return true;
}


/* --------------------------------------------------------------------------
 * Command processor
 * -------------------------------------------------------------------------- */

/*
 * Supported protocol:
 *
 * PING
 *      -> ACK
 *
 * PAN <angle>
 *      -> PAN_OK
 *
 * TILT <angle>
 *      -> TILT_OK
 *
 * CENTER
 *      -> CENTER_OK
 *
 * RANGE
 *      Requested angle outside safe mechanical range.
 *
 * ERROR
 *      Invalid numeric parameter.
 *
 * UNKNOWN
 *      Unsupported command.
 *
 * Watchdog behavior:
 *
 * A successfully recognized and accepted protocol command resets
 * the local communication watchdog.
 */
static void ProcessCommand(
    const char *command
)
{
    uint32_t angle;


    /* ----------------------------------------------------------------------
     * Heartbeat
     * ---------------------------------------------------------------------- */

    if(strcmp(
            command,
            "PING") == 0)
    {
        MarkCommunicationAlive();


        UART1_SendString(
            "ACK\r\n"
        );


        return;
    }


    /* ----------------------------------------------------------------------
     * Safe center command
     * ---------------------------------------------------------------------- */

    if(strcmp(
            command,
            "CENTER") == 0)
    {
        MarkCommunicationAlive();


        Actuator_Center();


        UART1_SendString(
            "CENTER_OK\r\n"
        );


        return;
    }


    /* ----------------------------------------------------------------------
     * PAN command
     * ---------------------------------------------------------------------- */

    if(strncmp(
            command,
            "PAN ",
            4U) == 0)
    {
        if(!ParseUnsignedInteger(
                command + 4,
                &angle))
        {
            UART1_SendString(
                "ERROR\r\n"
            );


            return;
        }


        if(!PAN_SetAngle(
                angle))
        {
            UART1_SendString(
                "RANGE\r\n"
            );


            return;
        }


        MarkCommunicationAlive();


        UART1_SendString(
            "PAN_OK\r\n"
        );


        return;
    }


    /* ----------------------------------------------------------------------
     * TILT command
     * ---------------------------------------------------------------------- */

    if(strncmp(
            command,
            "TILT ",
            5U) == 0)
    {
        if(!ParseUnsignedInteger(
                command + 5,
                &angle))
        {
            UART1_SendString(
                "ERROR\r\n"
            );


            return;
        }


        if(!TILT_SetAngle(
                angle))
        {
            UART1_SendString(
                "RANGE\r\n"
            );


            return;
        }


        MarkCommunicationAlive();


        UART1_SendString(
            "TILT_OK\r\n"
        );


        return;
    }


    /*
     * Unknown messages do NOT reset the watchdog.
     */
    UART1_SendString(
        "UNKNOWN\r\n"
    );
}


/* --------------------------------------------------------------------------
 * Main
 * -------------------------------------------------------------------------- */

int main(void)
{
    char commandBuffer[32];

    uint32_t commandIndex =
        0U;


    /* ----------------------------------------------------------------------
     * System clock
     * ---------------------------------------------------------------------- */

    /*
     * External crystal = 16 MHz
     * PLL              = enabled
     * CPU clock        = 80 MHz
     */
    SysCtlClockSet(
        SYSCTL_SYSDIV_2_5 |
        SYSCTL_USE_PLL |
        SYSCTL_OSC_MAIN |
        SYSCTL_XTAL_16MHZ
    );


    /* ----------------------------------------------------------------------
     * Hardware initialization
     * ---------------------------------------------------------------------- */

    UART1_Init();

    ActuatorPWM_Init();

    CommunicationWatchdog_Init();


    UART1_SendString(
        "TM4C READY\r\n"
    );


    /* ----------------------------------------------------------------------
     * Main supervision loop
     * ---------------------------------------------------------------------- */

    while(1)
    {
        /*
         * Continuously evaluate the local watchdog.
         *
         * Because UART reception below is non-blocking, this code
         * remains responsive even when Raspberry Pi sends nothing.
         */
        CheckCommunicationFailsafe();


        /*
         * Process UART only when a character is available.
         *
         * This replaces the previous blocking UARTCharGet()
         * implementation.
         */
        if(UARTCharsAvail(
                UART1_BASE))
        {
            char receivedChar =
                (char)UARTCharGetNonBlocking(
                    UART1_BASE
                );


            /*
             * CR or LF terminates one command.
             */
            if((receivedChar == '\r') ||
               (receivedChar == '\n'))
            {
                if(commandIndex > 0U)
                {
                    commandBuffer[
                        commandIndex
                    ] = '\0';


                    ProcessCommand(
                        commandBuffer
                    );


                    commandIndex =
                        0U;
                }
            }
            else
            {
                /*
                 * Protect the command buffer from overflow.
                 */
                if(commandIndex <
                   (sizeof(commandBuffer) -
                    1U))
                {
                    commandBuffer[
                        commandIndex++
                    ] =
                        receivedChar;
                }
                else
                {
                    /*
                     * Oversized malformed command.
                     *
                     * Reset the parser but do not reset
                     * the communication watchdog.
                     */
                    commandIndex =
                        0U;


                    UART1_SendString(
                        "ERROR\r\n"
                    );
                }
            }
        }
    }
}