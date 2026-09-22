# Problems Encountered and Solutions

The project was built iteratively on real hardware. This page records the failures that materially changed the design.

## Servo instability under shared power

**Problem:** servo motion was unstable and inconsistent.

**Cause:** servo current transients disturbed the supply.

**Resolution:** the servos were moved to a separate regulated 5 V supply, the grounds were tied together, and supply smoothing was added.

## Mechanical noise near the tilt limit

**Problem:** the tilt servo produced mechanical noise near the edge of travel.

**Cause:** the requested position approached the mechanical limit of the bracket.

**Resolution:** the usable tilt range was reduced to 55°–125°. PAN was constrained to 45°–135°.

## Uncontrolled actuator motion during bring-up

**Problem:** one development run produced unintended excessive servo movement.

**Resolution:** the mechanical horn mounting was corrected and explicit software angle limits were retained in the TM4C123 firmware.

## Raspberry Pi / UART bring-up

**Problem:** early tests did not always expose the expected serial device or communication response immediately.

**Resolution:** serial configuration was verified independently and a small `uart_ping_test.py` program was used before running the full scheduler.

## Debugging the complete actuator protocol was too broad

**Problem:** testing the full real-time system before proving individual commands made failures difficult to localize.

**Resolution:** `actuator_command_test.py` was added to verify `PING`, `PAN`, `TILT`, and `CENTER` independently.

## Communication loss could leave the last actuator command active

**Problem:** if the high-level controller disappeared, the actuator could otherwise remain at its last requested setpoint.

**Resolution:** a 300 ms communication watchdog was implemented locally on the TM4C123. A timeout commands both axes to 90° without requiring Linux.

## CPU overload caused cascading deadline misses

**Problem:** a deliberate 25 ms CONTROL workload was inserted into a 20 ms period/deadline task. The raw run produced misses across all lower-priority services.

**Raw result:**

```text
CONTROL : 100 / 500
COMM    : 41 / 200
HEALTH  : 20 / 100
MONITOR : 10 / 50
LOGGER  : 2 / 10
```

**Resolution:** deadline-based supervision, `DEGRADED`/`SAFE` states, motion inhibition, and workload shedding were added.

**After supervision:**

```text
CONTROL : 3 / 500
COMM    : 1 / 200
HEALTH  : 0 / 100
MONITOR : 0 / 50
LOGGER  : 0 / 10
```

## Communication health looked healthy during scheduler starvation

**Problem:** the original health logic mainly counted failed UART transactions. If COMM was starved by a higher-priority CPU overload, it could not run often enough to attempt transactions, so the communication counters alone did not describe the real failure.

**Resolution:** CONTROL deadline health and scheduler timing were included in supervision instead of treating UART transaction failures as the only health signal.

## Priority inversion

**Problem:** LOW owned a mutex, HIGH blocked on that mutex, and MEDIUM preempted LOW even though HIGH indirectly depended on LOW.

**Measured HIGH blocking:** 220.222 ms.

**Resolution:** the mutex protocol was configured with `PTHREAD_PRIO_INHERIT`.

**Measured HIGH blocking after the change:** 70.119 ms, a 68.16% reduction.

## Rate Monotonic did not protect a constrained deadline

**Problem:** `URGENT_B` had a 50 ms period but a 12 ms relative deadline. RM assigned it lower priority than the 40 ms-period task.

**Measured RM result:** 25 / 100 deadline misses, 16.083 ms worst response.

**Resolution:** Deadline Monotonic priority assignment was evaluated.

**Measured DM result:** 0 / 100 misses, 8.112 ms worst response.

## Long-run validation label mismatch

**Problem:** the experiment duration was increased from 10 seconds to 60 seconds while one old terminal status string still displayed `10 seconds`.

**Resolution:** the display string was corrected, the program was rebuilt, and the final 60-second validation was rerun. The retained final logs show the corrected duration.
