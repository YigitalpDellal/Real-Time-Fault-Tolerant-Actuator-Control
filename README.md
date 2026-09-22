# Real-Time Fault-Tolerant Dual-Axis Actuator Control

A hardware-in-the-loop real-time control testbed built with a **Raspberry Pi 3 Model B+** and a **TM4C123GXL LaunchPad**. The Raspberry Pi runs fixed-priority Linux real-time tasks for control, communication, supervision, logging, and fault injection; the TM4C123 performs deterministic PWM actuator control and provides an independent local fail-safe.

> **Final powered validation:** 5,160 periodic jobs, 1,261 successful UART transactions, **0 deadline misses**, and **0 UART failures** during a 60-second run with the pan-tilt mechanism powered.

![System overview](docs/media/01-hardware/system-overview.jpg)

## Why this project exists

The physical mechanism is intentionally simple. The engineering focus is the behavior around it: scheduling latency, deadline failures, communication faults, resource contention, safe-state transitions, and recovery.

The project demonstrates:

- Linux `SCHED_FIFO` fixed-priority scheduling on Raspberry Pi
- Rate Monotonic multi-service scheduling
- Release jitter, execution-time, and response-time measurement
- Raspberry Pi ↔ TM4C123 UART command/response communication
- Dual-axis hardware PWM actuator control
- Independent 300 ms MCU communication watchdog
- `NORMAL → DEGRADED → SAFE → NORMAL` fault handling
- CPU-overload fault injection and workload shedding
- Priority inversion and `PTHREAD_PRIO_INHERIT`
- Rate Monotonic vs Deadline Monotonic scheduling
- Long-duration powered validation on the physical platform

## Key measured results

| Experiment | Before / baseline | After / comparison | Result |
|---|---:|---:|---|
| Average release jitter under CPU load | `SCHED_OTHER`: 120.019 µs | `SCHED_FIFO`: 10.052 µs | 91.6% lower average jitter |
| Worst release jitter under CPU load | 8203.196 µs | 60.820 µs | large reduction in worst observed jitter |
| CONTROL overload | 100 / 500 deadline misses | 3 / 500 after supervision | overload contained |
| COMM during CONTROL overload | 41 / 200 misses | 1 / 200 after supervision | cascading failure largely removed |
| Priority inversion | HIGH blocked 220.222 ms | 70.119 ms with priority inheritance | 68.16% reduction |
| Constrained-deadline task | RM: 25 / 100 misses | DM: 0 / 100 misses | deadline protected |
| 60 s powered validation | 5,160 jobs | 0 deadline misses | stable |
| UART during powered validation | 1,261 successful | 0 failed | stable |

The scheduler CSV files and final validation logs are retained in [`results/`](results/).

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
│ software angle limits                 │
│ 300 ms local communication watchdog   │
│ independent CENTER fail-safe          │
└──────────────┬──────────────┬─────────┘
               │              │
               ▼              ▼
           PAN servo      TILT servo
```

The split is deliberate: Linux handles higher-level scheduling and supervision, while the MCU owns the actuator-level behavior and can force a local safe position without relying on Linux.

## Hardware

- Raspberry Pi 3 Model B+
- TM4C123GXL LaunchPad
- 2 × MG90S micro servo
- Dual-axis pan-tilt bracket
- External regulated 5 V servo supply
- Breadboard and jumper wiring
- Supply smoothing capacitor

### Actuator configuration

| Axis | TM4C123 output | Minimum | Center | Maximum |
|---|---|---:|---:|---:|
| PAN | PB6 / M0PWM0 | 45° | 90° | 135° |
| TILT | PB7 / M0PWM1 | 55° | 90° | 125° |

**Power note:** the servos are powered from the external 5 V supply rather than from the Raspberry Pi power rail. The controller grounds are shared.

<p>
  <img src="docs/media/01-hardware/wiring-and-power-overview.jpg" width="48%" alt="Wiring and power overview">
  <img src="docs/media/01-hardware/full-assembly.jpg" width="48%" alt="Full hardware assembly">
</p>

## UART protocol

The Raspberry Pi sends simple request/response commands:

```text
PING
PAN 70
PAN 110
TILT 70
TILT 110
CENTER
```

Expected responses include:

```text
ACK
PAN_OK
TILT_OK
CENTER_OK
```

Before the real-time multi-service program starts, the TM4C123 link is explicitly verified.

## Safety behavior

The TM4C123 implements a **300 ms local communication watchdog**. If valid Raspberry Pi commands stop arriving, the MCU centers both actuators:

```text
PAN  -> 90°
TILT -> 90°
```

The Raspberry Pi supervisor independently manages system state:

```text
NORMAL
   │ fault detected
   ▼
DEGRADED
   │ persistent fault
   ▼
SAFE
   │ stable recovery
   ▼
NORMAL
```

In the timing-fault experiment, entering `SAFE` also sheds the deliberately faulty CONTROL workload and inhibits new motion commands.

## Demo videos

Click a thumbnail to open the corresponding MP4.

### Real-time pan-tilt operation

[![Real-time pan-tilt demo](docs/media/06-integration/02-realtime-pan-tilt-demo-thumb.jpg)](docs/media/06-integration/02-realtime-pan-tilt-demo.mp4)

### Independent local watchdog fail-safe

[![Local watchdog demo](docs/media/07-fault-injection/01-local-watchdog-failsafe-demo-thumb.jpg)](docs/media/07-fault-injection/01-local-watchdog-failsafe-demo.mp4)

### Supervised timing fault and safe-state behavior

[![Timing fault demo](docs/media/08-deadline-fault/05-supervised-timing-safe-demo-thumb.jpg)](docs/media/08-deadline-fault/05-supervised-timing-safe-demo.mp4)

### 60-second powered validation preview

The repository includes a short preview of the 60-second physical run. The complete timing evidence is preserved in the terminal screenshots and raw log.

[![60-second validation preview](docs/media/11-validation/05-60s-physical-validation-preview-thumb.jpg)](docs/media/11-validation/05-60s-physical-validation-preview.mp4)

## Experiment highlights

### `SCHED_OTHER` vs `SCHED_FIFO`

Under the same generated CPU load, measured average CONTROL release jitter changed from **120.019 µs** with `SCHED_OTHER` to **10.052 µs** with `SCHED_FIFO`.

![Jitter comparison](results/figures/average_jitter_comparison.png)

![Jitter under CPU load](results/figures/jitter_under_cpu_load.png)

Raw data:

- [`control_timing_sched_other_baseline.csv`](results/csv/control_timing_sched_other_baseline.csv)
- [`control_timing_sched_fifo_baseline.csv`](results/csv/control_timing_sched_fifo_baseline.csv)
- [`control_timing_sched_other_load.csv`](results/csv/control_timing_sched_other_load.csv)
- [`control_timing_sched_fifo_load.csv`](results/csv/control_timing_sched_fifo_load.csv)

### CPU-overload fault containment

A 25 ms CONTROL workload was injected into a task with a 20 ms period/deadline.

| Service | Raw overload | With supervision |
|---|---:|---:|
| CONTROL | 100 / 500 | 3 / 500 |
| COMM | 41 / 200 | 1 / 200 |
| HEALTH | 20 / 100 | 0 / 100 |
| MONITOR | 10 / 50 | 0 / 50 |
| LOGGER | 2 / 10 | 0 / 10 |

Measured supervised state transitions:

```text
t=4.025 s | NORMAL   -> DEGRADED | CONTROL_DEADLINE_OVERRUN
t=4.075 s | DEGRADED -> SAFE     | CONTROL_DEADLINE_OVERRUN
t=6.480 s | SAFE     -> NORMAL   | NONE
```

![Supervised timing results](docs/media/08-deadline-fault/03-supervised-timing-results.png)

![Supervised state recovery](docs/media/08-deadline-fault/04-supervised-state-recovery.png)

### Priority inversion

The experiment uses three `SCHED_FIFO` threads on one CPU:

```text
HIGH   priority 80
MEDIUM priority 60
LOW    priority 40
```

Without priority inheritance, HIGH was blocked for **220.222 ms**. With `PTHREAD_PRIO_INHERIT`, blocking fell to **70.119 ms**, a measured reduction of **68.16%**.

![Priority inheritance comparison](docs/media/09-priority-inversion/01-priority-inheritance-comparison.png)

### Rate Monotonic vs Deadline Monotonic

`URGENT_B` was configured with:

```text
C = 8 ms
T = 50 ms
D = 12 ms
```

Under Rate Monotonic scheduling it missed **25 / 100** deadlines with a worst response of **16.083 ms**. Under Deadline Monotonic its priority increased, deadline misses fell to **0 / 100**, and worst response fell to **8.112 ms**.

![RM vs DM comparison](docs/media/10-rm-vs-dm/02-urgent-task-comparison.png)

## 60-second powered validation

The complete Raspberry Pi + TM4C123 + externally powered pan-tilt system was operated for 60 seconds.

| Service | Jobs | Deadline misses |
|---|---:|---:|
| CONTROL | 3000 | 0 |
| COMM | 1200 | 0 |
| HEALTH | 600 | 0 |
| MONITOR | 300 | 0 |
| LOGGER | 60 | 0 |
| **Total** | **5160** | **0** |

UART summary:

```text
Successful UART transactions : 1261
Failed UART transactions     : 0
Consecutive failures         : 0
HEALTH state                 : HEALTHY

Final setpoint:
PAN  : 90 degrees
TILT : 90 degrees

Sum(C_i / T_i) = 0.069792
```

![Physical validation timing](docs/media/11-validation/03-60s-physical-validation-timing.png)

![Physical validation summary](docs/media/11-validation/04-60s-physical-validation-summary.png)

Raw log: [`results/logs/final_60s_physical_validation.txt`](results/logs/final_60s_physical_validation.txt)

## Build on Raspberry Pi

Prerequisites:

```bash
sudo apt update
sudo apt install build-essential python3-pip
python3 -m pip install -r requirements.txt
```

Build the Raspberry Pi C programs:

```bash
make
```

Binaries are written to `build/`.

The real-time experiments require permission to create `SCHED_FIFO` threads. The examples were run pinned to CPU 3:

```bash
sudo taskset -c 3 ./build/rt_multiservice_uart
sudo taskset -c 3 ./build/rt_fault_tolerant
sudo taskset -c 3 ./build/rt_timing_fault_supervised
sudo taskset -c 3 ./build/priority_inversion_test
sudo taskset -c 3 ./build/rm_vs_dm_test
```

The Raspberry Pi UART program expects the TM4C123 link at:

```text
/dev/serial0
115200 baud
8-N-1
```

The TM4C123 firmware source used for the final hardware test is in [`src/tm4c123/main.c`](src/tm4c123/main.c).

## Repository layout

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

- [Detailed experiment notes and evidence](docs/EXPERIMENTS.md)
- [Problems encountered and solutions](docs/TROUBLESHOOTING.md)

## Notes on the measurements

The reported timing values are **measurements from this Raspberry Pi 3 Model B+ test setup**, not universal guarantees for Linux or for other hardware. The goal is to make scheduling and fault behavior observable and reproducible on the project platform.

This repository is an experimental real-time embedded-systems testbed, not a certified safety-critical product.
