# Real-Time Fault-Tolerant Dual-Axis Actuator Control

A hardware-in-the-loop real-time control testbed built with a **Raspberry Pi 3 Model B+** and a **TM4C123GXL LaunchPad**.

The Raspberry Pi runs fixed-priority Linux real-time tasks for control, communication, supervision, logging, timing analysis and fault injection. The TM4C123 performs deterministic low-level PWM control for the PAN/TILT mechanism and implements an independent local communication watchdog.

> **Final powered validation:** 5,160 periodic jobs, 1,261 successful UART transactions, **0 deadline misses** and **0 UART failures** during a 60-second run.

## What this project demonstrates

- Linux `SCHED_FIFO` fixed-priority scheduling
- Rate Monotonic periodic service organization
- Release jitter, execution-time and response-time measurement
- Raspberry Pi ↔ TM4C123 UART communication
- Dual-axis PAN/TILT hardware PWM control
- Independent 300 ms MCU communication watchdog
- `NORMAL → DEGRADED → SAFE → NORMAL` fault handling
- CPU-overload fault injection and recovery
- Priority inversion and `PTHREAD_PRIO_INHERIT`
- Rate Monotonic vs Deadline Monotonic scheduling
- 60-second powered physical validation

## Key measured results

| Experiment | Baseline / fault case | Improved / comparison case | Result |
|---|---:|---:|---|
| Average CONTROL jitter under CPU load | `SCHED_OTHER`: 120.019 µs | `SCHED_FIFO`: 10.052 µs | 91.6% lower |
| Worst CONTROL jitter under CPU load | 8203.196 µs | 60.820 µs | major worst-case reduction |
| CONTROL overload | 100 / 500 misses | 3 / 500 after supervision | fault contained |
| COMM during CONTROL overload | 41 / 200 misses | 1 / 200 after supervision | cascading misses reduced |
| Priority inversion | HIGH blocked 220.222 ms | 70.119 ms with inheritance | 68.16% reduction |
| Constrained-deadline task | RM: 25 / 100 misses | DM: 0 / 100 misses | deadline protected |
| 60 s physical validation | 5,160 jobs | 0 deadline misses | stable |
| UART in physical validation | 1,261 successful | 0 failed | stable |

## System architecture

```text
Raspberry Pi 3 Model B+
┌───────────────────────────────────────┐
│ CONTROL   20 ms   priority 80         │
│ COMM      50 ms   priority 70         │
│ HEALTH   100 ms   priority 60         │
│ MONITOR  200 ms   priority 50         │
│ LOGGER  1000 ms   priority 40         │
│                                       │
│ SCHED_FIFO / timing measurement       │
│ fault supervision / fault injection   │
└───────────────────┬───────────────────┘
                    │ UART
                    ▼
TM4C123GXL
┌───────────────────────────────────────┐
│ UART parser                           │
│ PAN/TILT hardware PWM                 │
│ angle limits                          │
│ 300 ms communication watchdog         │
│ independent CENTER fail-safe          │
└──────────────┬──────────────┬─────────┘
               │              │
               ▼              ▼
           PAN servo      TILT servo
```

## Hardware

- Raspberry Pi 3 Model B+
- TM4C123GXL LaunchPad
- 2 × MG90S micro servo
- Dual-axis pan-tilt bracket
- External regulated 5 V servo supply
- Breadboard and jumper wiring
- Supply smoothing capacitor

| Axis | TM4C123 output | Minimum | Center | Maximum |
|---|---|---:|---:|---:|
| PAN | PB6 / M0PWM0 | 45° | 90° | 135° |
| TILT | PB7 / M0PWM1 | 55° | 90° | 125° |

The servos are powered from the external 5 V supply; controller grounds are shared.

## UART protocol

```text
PING
PAN 70
PAN 110
TILT 70
TILT 110
CENTER
```

Expected acknowledgements include:

```text
ACK
PAN_OK
TILT_OK
CENTER_OK
```

## Independent fail-safe

The TM4C123 implements a **300 ms local communication watchdog**. If valid Raspberry Pi commands stop arriving, the MCU returns both axes to the safe center:

```text
PAN  -> 90°
TILT -> 90°
```

The Raspberry Pi supervisor separately manages:

```text
NORMAL -> DEGRADED -> SAFE -> NORMAL
```

## Scheduler comparison

Measured average CONTROL release jitter under CPU load:

- `SCHED_OTHER`: **120.019 µs**
- `SCHED_FIFO`: **10.052 µs**

![Average jitter comparison](results/figures/average_jitter_comparison.png)

![Jitter under CPU load](results/figures/jitter_under_cpu_load.png)

![Response time under CPU load](results/figures/response_time_under_cpu_load.png)

Raw CSV data:

- [SCHED_OTHER baseline](results/csv/control_timing_sched_other_baseline.csv)
- [SCHED_FIFO baseline](results/csv/control_timing_sched_fifo_baseline.csv)
- [SCHED_OTHER under CPU load](results/csv/control_timing_sched_other_load.csv)
- [SCHED_FIFO under CPU load](results/csv/control_timing_sched_fifo_load.csv)

## CPU-overload fault containment

A deliberate 25 ms CONTROL workload was inserted into a task with a 20 ms period/deadline.

| Service | Raw overload | With supervision |
|---|---:|---:|
| CONTROL | 100 / 500 | 3 / 500 |
| COMM | 41 / 200 | 1 / 200 |
| HEALTH | 20 / 100 | 0 / 100 |
| MONITOR | 10 / 50 | 0 / 50 |
| LOGGER | 2 / 10 | 0 / 10 |

Measured supervised transitions:

```text
t=4.025 s | NORMAL   -> DEGRADED | CONTROL_DEADLINE_OVERRUN
t=4.075 s | DEGRADED -> SAFE     | CONTROL_DEADLINE_OVERRUN
t=6.480 s | SAFE     -> NORMAL   | NONE
```

![Supervised timing results](docs/media/08-deadline-fault/03-supervised-timing-results.png)

![Supervised state recovery](docs/media/08-deadline-fault/04-supervised-state-recovery.png)

## Priority inversion experiment

Without priority inheritance, the HIGH-priority thread was blocked for **220.222 ms**. With `PTHREAD_PRIO_INHERIT`, measured blocking fell to **70.119 ms**.

![Priority inheritance comparison](docs/media/09-priority-inversion/01-priority-inheritance-comparison.png)

## Rate Monotonic vs Deadline Monotonic

For the constrained-deadline task `URGENT_B`:

```text
Execution = 8 ms
Period    = 50 ms
Deadline  = 12 ms
```

- Rate Monotonic: **25 / 100** deadline misses, worst response **16.083 ms**
- Deadline Monotonic: **0 / 100** deadline misses, worst response **8.112 ms**

![RM vs DM comparison](docs/media/10-rm-vs-dm/02-urgent-task-comparison.png)

## 60-second powered validation

| Service | Jobs | Deadline misses |
|---|---:|---:|
| CONTROL | 3000 | 0 |
| COMM | 1200 | 0 |
| HEALTH | 600 | 0 |
| MONITOR | 300 | 0 |
| LOGGER | 60 | 0 |
| **Total** | **5160** | **0** |

```text
Successful UART transactions : 1261
Failed UART transactions     : 0
Consecutive failures         : 0
HEALTH state                 : HEALTHY

PAN  : 90 degrees
TILT : 90 degrees

Sum(C_i / T_i) = 0.069792
```

![Physical validation timing](docs/media/11-validation/03-60s-physical-validation-timing.png)

![Physical validation summary](docs/media/11-validation/04-60s-physical-validation-summary.png)

Raw logs:

- [60-second validation](results/logs/final_60s_validation.txt)
- [60-second powered physical validation](results/logs/final_60s_physical_validation.txt)

## Build on Raspberry Pi

```bash
sudo apt update
sudo apt install build-essential python3-pip
python3 -m pip install -r requirements.txt
make
```

The real-time examples require permission to create `SCHED_FIFO` threads. Example:

```bash
sudo taskset -c 3 ./build/rt_multiservice_uart
sudo taskset -c 3 ./build/rt_fault_tolerant
sudo taskset -c 3 ./build/rt_timing_fault_supervised
sudo taskset -c 3 ./build/priority_inversion_test
sudo taskset -c 3 ./build/rm_vs_dm_test
```

UART configuration:

```text
/dev/serial0
115200 baud
8-N-1
```

## Repository structure

```text
.
├── README.md
├── Makefile
├── requirements.txt
├── src/
│   ├── raspberry_pi/
│   │   ├── baseline/
│   │   ├── scheduling/
│   │   ├── fault_tolerance/
│   │   └── experiments/
│   └── tm4c123/
├── tests/
├── results/
│   ├── csv/
│   ├── figures/
│   └── logs/
└── docs/
    ├── EXPERIMENTS.md
    ├── TROUBLESHOOTING.md
    └── media/
```

## Documentation

- [Detailed experiment notes](docs/EXPERIMENTS.md)
- [Problems encountered and solutions](docs/TROUBLESHOOTING.md)

## Measurement note

The reported timing values are measurements from this Raspberry Pi 3 Model B+ test setup, not universal guarantees for Linux or other hardware.

This repository is an experimental real-time embedded-systems testbed, not a certified safety-critical product.
