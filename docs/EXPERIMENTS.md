# Experiment Notes and Evidence

This document collects the detailed test evidence behind the summary in the main README.

## 1. UART and actuator bring-up

The communication path was verified before enabling the complete real-time service set.

![Serial port verification](media/02-uart/00-serial-port-verification.png)

![PING/ACK verification](media/02-uart/02-ping-ack-terminal.png)

![Actuator protocol verification](media/03-actuator/03-actuator-protocol-verification.png)

![Dual-axis protocol verification](media/03-actuator/05-dual-axis-protocol-verification.png)

Video evidence:

- [Center command test](media/03-actuator/02-center-test.mp4)
- [UART PAN control](media/03-actuator/04-uart-pan-control-demo.mp4)
- [Dual-axis motion](media/03-actuator/06-dual-axis-motion-demo.mp4)

## 2. CONTROL scheduling baseline

The baseline CONTROL task used a 20 ms period and 20 ms deadline.

![CONTROL baseline](media/04-scheduling/01-control-task-baseline.png)

![SCHED_OTHER summary](media/04-scheduling/02-sched-other-baseline-summary.png)

![SCHED_FIFO summary](media/04-scheduling/03-sched-fifo-timing-summary.png)

Measured CONTROL values from the retained CSV data:

| Condition | Average jitter | Worst jitter | Average response | Worst response | Deadline misses |
|---|---:|---:|---:|---:|---:|
| SCHED_OTHER / no load | 82.074 µs | 212.422 µs | 164.325 µs | 296.953 µs | 0 / 1000 |
| SCHED_FIFO / no load | 21.618 µs | 77.969 µs | 101.880 µs | 196.288 µs | 0 / 1000 |
| SCHED_OTHER / CPU load | 120.019 µs | 8203.196 µs | 156.095 µs | 8239.341 µs | 0 / 1000 |
| SCHED_FIFO / CPU load | 10.052 µs | 60.820 µs | 46.066 µs | 97.278 µs | 0 / 1000 |

![Average jitter comparison](../results/figures/average_jitter_comparison.png)

![Jitter under CPU load](../results/figures/jitter_under_cpu_load.png)

![Response time under CPU load](../results/figures/response_time_under_cpu_load.png)

## 3. Rate Monotonic multi-service baseline

The periodic service set is:

| Service | Period | Priority |
|---|---:|---:|
| CONTROL | 20 ms | 80 |
| COMM | 50 ms | 70 |
| HEALTH | 100 ms | 60 |
| MONITOR | 200 ms | 50 |
| LOGGER | 1000 ms | 40 |

![RM service results](media/05-rate-monotonic/01-rm-service-results-top.png)

![RM utilization summary](media/05-rate-monotonic/02-rm-utilization-summary.png)

## 4. Physical UART integration

The multi-service scheduler was connected to the TM4C123 actuator node over UART.

![RT UART health test](media/06-integration/01-rt-uart-health-test.png)

![Physical service results](media/06-integration/03-physical-rt-service-results.png)

![Physical health summary](media/06-integration/04-physical-rt-health-summary.png)

[Real-time pan-tilt video](media/06-integration/02-realtime-pan-tilt-demo.mp4)

## 5. Communication-loss fault injection

Complete UART traffic loss was injected between 4.0 s and 6.0 s.

Observed communication summary:

```text
Successful transactions : 170
Failed transactions     : 40
```

The 40 failed transactions correspond to the 2-second fault interval at a 50 ms COMM period.

Observed state sequence:

```text
NORMAL -> DEGRADED -> SAFE -> NORMAL
```

![Watchdog recovery terminal](media/07-fault-injection/02-watchdog-recovery-terminal.png)

![State-machine transition log](media/07-fault-injection/03-state-machine-transition-log.png)

![Fault timing results](media/07-fault-injection/04-fault-test-timing-results.png)

![Physical safe-state log](media/07-fault-injection/06-physical-safe-state-log.png)

Videos:

- [TM4C123 local watchdog fail-safe](media/07-fault-injection/01-local-watchdog-failsafe-demo.mp4)
- [Physical SAFE-state demonstration](media/07-fault-injection/05-safe-state-physical-demo.mp4)

## 6. Raw deadline fault

A 25 ms CPU workload was deliberately injected into CONTROL, whose period/deadline is 20 ms.

Raw overload results:

| Service | Deadline misses |
|---|---:|
| CONTROL | 100 / 500 |
| COMM | 41 / 200 |
| HEALTH | 20 / 100 |
| MONITOR | 10 / 50 |
| LOGGER | 2 / 10 |

![Raw CONTROL overload](media/08-deadline-fault/01-overload-deadline-misses.png)

![Cascading deadline misses](media/08-deadline-fault/02-overload-cascade-results.png)

## 7. Supervised timing-fault recovery

The supervisor was extended to use CONTROL deadline health directly. Three consecutive CONTROL misses cause `SAFE`. The faulty workload is then shed, actuator motion is inhibited, and recovery requires 25 healthy CONTROL cycles after the injected fault interval.

Measured state transitions:

```text
t=4.025 s | NORMAL   -> DEGRADED | CONTROL_DEADLINE_OVERRUN
t=4.075 s | DEGRADED -> SAFE     | CONTROL_DEADLINE_OVERRUN
t=6.480 s | SAFE     -> NORMAL   | NONE
```

After supervision:

| Service | Deadline misses |
|---|---:|
| CONTROL | 3 / 500 |
| COMM | 1 / 200 |
| HEALTH | 0 / 100 |
| MONITOR | 0 / 50 |
| LOGGER | 0 / 10 |

![Supervised timing results](media/08-deadline-fault/03-supervised-timing-results.png)

![Recovery state log](media/08-deadline-fault/04-supervised-state-recovery.png)

[Physical supervised timing-fault demo](media/08-deadline-fault/05-supervised-timing-safe-demo.mp4)

## 8. Priority inversion and priority inheritance

Single-core `SCHED_FIFO` priorities:

```text
HIGH   = 80
MEDIUM = 60
LOW    = 40
```

Without priority inheritance:

```text
LOW mutex hold wall time : 230.172 ms
HIGH blocking time       : 220.222 ms
MEDIUM wall time         : 150.003 ms
```

With `PTHREAD_PRIO_INHERIT`:

```text
LOW mutex hold wall time : 80.064 ms
HIGH blocking time       : 70.119 ms
MEDIUM wall time         : 150.011 ms
```

Measured HIGH blocking reduction:

```text
150.103 ms
68.16%
```

![Priority inversion comparison](media/09-priority-inversion/01-priority-inheritance-comparison.png)

## 9. Rate Monotonic vs Deadline Monotonic

Task set:

| Task | Execution | Period | Deadline |
|---|---:|---:|---:|
| FAST_A | 8 ms | 40 ms | 40 ms |
| URGENT_B | 8 ms | 50 ms | 12 ms |
| SLOW_C | 4 ms | 100 ms | 100 ms |

For `URGENT_B`:

| Policy | Priority | Misses | Worst response |
|---|---:|---:|---:|
| Rate Monotonic | 70 | 25 / 100 | 16.083 ms |
| Deadline Monotonic | 80 | 0 / 100 | 8.112 ms |

![Full RM/DM results](media/10-rm-vs-dm/01-rm-dm-full-results.png)

![URGENT_B comparison](media/10-rm-vs-dm/02-urgent-task-comparison.png)

## 10. 60-second final validation

A 60-second run was recorded both as a normal long-run validation and as a powered physical validation.

Final powered result:

```text
CONTROL : 0 / 3000 deadline misses
COMM    : 0 / 1200
HEALTH  : 0 / 600
MONITOR : 0 / 300
LOGGER  : 0 / 60

Successful UART transactions : 1261
Failed UART transactions     : 0
Consecutive failures         : 0
HEALTH state                 : HEALTHY

PAN  : 90 degrees
TILT : 90 degrees

Sum(C_i / T_i) = 0.069792
```

![60-second timing results](media/11-validation/03-60s-physical-validation-timing.png)

![60-second summary](media/11-validation/04-60s-physical-validation-summary.png)

Raw logs:

- [`final_60s_validation.txt`](../results/logs/final_60s_validation.txt)
- [`final_60s_physical_validation.txt`](../results/logs/final_60s_physical_validation.txt)

A repository-sized video preview is available here:

- [60-second physical validation preview](media/11-validation/05-60s-physical-validation-preview.mp4)
