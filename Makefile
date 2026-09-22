CC := gcc
CFLAGS := -O2 -Wall -Wextra -pthread
BUILD := build

PROGRAMS := \
	rt_control_task \
	rt_control_task_sched_other \
	rt_control_task_sched_fifo \
	rt_multiservice_rm \
	rt_multiservice_rm_synthetic \
	rt_multiservice_uart \
	rt_fault_tolerant \
	rt_timing_fault_supervised \
	cpu_hog \
	priority_inversion_test \
	rm_vs_dm_test \
	rt_deadline_fault_raw \
	rt_long_run_validation

SOURCES_rt_control_task := src/raspberry_pi/baseline/rt_control_task.c
SOURCES_rt_control_task_sched_other := src/raspberry_pi/scheduling/rt_control_task_sched_other.c
SOURCES_rt_control_task_sched_fifo := src/raspberry_pi/scheduling/rt_control_task_sched_fifo.c
SOURCES_rt_multiservice_rm := src/raspberry_pi/scheduling/rt_multiservice_rm.c
SOURCES_rt_multiservice_rm_synthetic := src/raspberry_pi/scheduling/rt_multiservice_rm_synthetic.c
SOURCES_rt_multiservice_uart := src/raspberry_pi/fault_tolerance/rt_multiservice_uart.c
SOURCES_rt_fault_tolerant := src/raspberry_pi/fault_tolerance/rt_fault_tolerant.c
SOURCES_rt_timing_fault_supervised := src/raspberry_pi/fault_tolerance/rt_timing_fault_supervised.c
SOURCES_cpu_hog := src/raspberry_pi/experiments/cpu_hog.c
SOURCES_priority_inversion_test := src/raspberry_pi/experiments/priority_inversion_test.c
SOURCES_rm_vs_dm_test := src/raspberry_pi/experiments/rm_vs_dm_test.c
SOURCES_rt_deadline_fault_raw := src/raspberry_pi/experiments/rt_deadline_fault_raw.c
SOURCES_rt_long_run_validation := src/raspberry_pi/experiments/rt_long_run_validation.c

.PHONY: all clean

all: $(PROGRAMS:%=$(BUILD)/%)

$(BUILD):
	mkdir -p $(BUILD)

define BUILD_template
$(BUILD)/$(1): $$(SOURCES_$(1)) | $(BUILD)
	$$(CC) $$(CFLAGS) -o $$@ $$<
endef

$(foreach p,$(PROGRAMS),$(eval $(call BUILD_template,$(p))))

clean:
	rm -rf $(BUILD)
