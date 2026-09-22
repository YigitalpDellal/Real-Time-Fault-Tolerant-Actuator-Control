"""
Project:
Real-Time Safety-Critical Actuator Control & Fault-Injection Testbed

Analysis:
SCHED_OTHER vs SCHED_FIFO Timing Comparison

Purpose:
- Read raw timing measurements from four experiments
- Compare normal Linux scheduling with POSIX SCHED_FIFO
- Compare unloaded and CPU-loaded conditions
- Generate figures for engineering analysis and GitHub documentation

Experiments:
1. SCHED_OTHER - baseline
2. SCHED_FIFO  - baseline
3. SCHED_OTHER - CPU load
4. SCHED_FIFO  - CPU load
"""

import csv
import statistics
from pathlib import Path

import matplotlib.pyplot as plt


# ---------------------------------------------------------------------------
# Input files
# ---------------------------------------------------------------------------

FILES = {
    "SCHED_OTHER / No Load":
        "control_timing_sched_other_baseline.csv",

    "SCHED_FIFO / No Load":
        "control_timing_sched_fifo_baseline.csv",

    "SCHED_OTHER / CPU Load":
        "control_timing_sched_other_load.csv",

    "SCHED_FIFO / CPU Load":
        "control_timing_sched_fifo_load.csv",
}


# ---------------------------------------------------------------------------
# CSV loader
# ---------------------------------------------------------------------------

def load_timing_data(filename):
    """
    Load one timing experiment from CSV.

    Returns a dictionary containing:
    - cycle number
    - jitter
    - execution time
    - response time
    - deadline-miss information
    """

    data = {
        "cycle": [],
        "jitter_us": [],
        "execution_us": [],
        "response_us": [],
        "deadline_miss": [],
    }

    with open(filename, "r", newline="") as csv_file:
        reader = csv.DictReader(csv_file)

        for row in reader:
            data["cycle"].append(
                int(row["cycle"])
            )

            data["jitter_us"].append(
                float(row["jitter_us"])
            )

            data["execution_us"].append(
                float(row["execution_us"])
            )

            data["response_us"].append(
                float(row["response_us"])
            )

            data["deadline_miss"].append(
                int(row["deadline_miss"])
            )

    return data


# ---------------------------------------------------------------------------
# Statistical summary
# ---------------------------------------------------------------------------

def print_summary(name, data):
    """
    Print key timing statistics for one experiment.
    """

    print(f"\n{name}")
    print("-" * len(name))

    print(
        f"Average jitter   : "
        f"{statistics.mean(data['jitter_us']):.3f} us"
    )

    print(
        f"Worst jitter     : "
        f"{max(data['jitter_us']):.3f} us"
    )

    print(
        f"Average execution: "
        f"{statistics.mean(data['execution_us']):.3f} us"
    )

    print(
        f"Max execution    : "
        f"{max(data['execution_us']):.3f} us"
    )

    print(
        f"Average response : "
        f"{statistics.mean(data['response_us']):.3f} us"
    )

    print(
        f"Worst response   : "
        f"{max(data['response_us']):.3f} us"
    )

    print(
        f"Deadline misses  : "
        f"{sum(data['deadline_miss'])}"
    )


# ---------------------------------------------------------------------------
# Plot: jitter under CPU load
# ---------------------------------------------------------------------------

def plot_loaded_jitter(other_data, fifo_data):
    """
    Compare release jitter under CPU contention.

    A logarithmic vertical scale is used because SCHED_OTHER may contain
    millisecond-scale outliers while SCHED_FIFO remains in the
    microsecond range.
    """

    plt.figure(figsize=(11, 6))

    plt.plot(
        other_data["cycle"],
        other_data["jitter_us"],
        label="SCHED_OTHER"
    )

    plt.plot(
        fifo_data["cycle"],
        fifo_data["jitter_us"],
        label="SCHED_FIFO"
    )

    plt.yscale("log")

    plt.xlabel("Task Cycle")
    plt.ylabel("Release Jitter (us)")
    plt.title(
        "CONTROL Task Release Jitter Under CPU Load"
    )

    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()

    plt.savefig(
        "jitter_under_cpu_load.png",
        dpi=200
    )

    plt.close()


# ---------------------------------------------------------------------------
# Plot: response time under CPU load
# ---------------------------------------------------------------------------

def plot_loaded_response(other_data, fifo_data):
    """
    Compare CONTROL-task response time while both schedulers compete
    with the same CPU load.
    """

    plt.figure(figsize=(11, 6))

    plt.plot(
        other_data["cycle"],
        other_data["response_us"],
        label="SCHED_OTHER"
    )

    plt.plot(
        fifo_data["cycle"],
        fifo_data["response_us"],
        label="SCHED_FIFO"
    )

    plt.yscale("log")

    plt.xlabel("Task Cycle")
    plt.ylabel("Response Time (us)")
    plt.title(
        "CONTROL Task Response Time Under CPU Load"
    )

    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()

    plt.savefig(
        "response_time_under_cpu_load.png",
        dpi=200
    )

    plt.close()


# ---------------------------------------------------------------------------
# Plot: four-experiment average jitter comparison
# ---------------------------------------------------------------------------

def plot_average_jitter(all_results):
    """
    Compare average release jitter across all four experiments.
    """

    names = list(all_results.keys())

    averages = [
        statistics.mean(
            all_results[name]["jitter_us"]
        )
        for name in names
    ]

    plt.figure(figsize=(11, 6))

    plt.bar(
        names,
        averages
    )

    plt.ylabel("Average Release Jitter (us)")
    plt.title(
        "Average CONTROL Task Jitter by Scheduling Condition"
    )

    plt.xticks(
        rotation=20,
        ha="right"
    )

    plt.grid(
        axis="y",
        alpha=0.3
    )

    plt.tight_layout()

    plt.savefig(
        "average_jitter_comparison.png",
        dpi=200
    )

    plt.close()


# ---------------------------------------------------------------------------
# Main analysis
# ---------------------------------------------------------------------------

def main():
    """
    Load all experiments, verify the input files, print timing summaries,
    and generate comparison figures.
    """

    results = {}

    print(
        "=== REAL-TIME SCHEDULER ANALYSIS ==="
    )

    for experiment_name, filename in FILES.items():

        if not Path(filename).exists():
            print(
                f"[ERROR] Missing input file: {filename}"
            )
            return

        results[experiment_name] = (
            load_timing_data(filename)
        )

        print_summary(
            experiment_name,
            results[experiment_name]
        )


    plot_loaded_jitter(
        results["SCHED_OTHER / CPU Load"],
        results["SCHED_FIFO / CPU Load"]
    )

    plot_loaded_response(
        results["SCHED_OTHER / CPU Load"],
        results["SCHED_FIFO / CPU Load"]
    )

    plot_average_jitter(
        results
    )


    print(
        "\nGenerated figures:"
    )

    print(
        " - jitter_under_cpu_load.png"
    )

    print(
        " - response_time_under_cpu_load.png"
    )

    print(
        " - average_jitter_comparison.png"
    )


if __name__ == "__main__":
    main()
