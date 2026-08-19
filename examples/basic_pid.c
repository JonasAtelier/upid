/*
 * Portable upid example: no platform headers, runs on a host or a board.
 *
 * On real hardware, replace read_measurement() and apply_output() with your
 * sensor and actuator, and take dt_s from your own clock - micros(),
 * esp_timer_get_time(), HAL_GetTick() - converted to seconds.
 */
#include "upid.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static const float setpoint = 50.0f;
static const float dt_s = 0.01f;

/* Stand-in plant: output heats a value that also leaks away. */
static float plant = 20.0f;

static float read_measurement(void)
{
	return plant;
}

static void apply_output(float output)
{
	plant += (output - (plant - 20.0f) * 0.1f) * dt_s;
}

int main(void)
{
	struct upid_cfg cfg;
	struct upid pid;
	int step;

	upid_cfg_init(&cfg);
	cfg.kp = 2.0f;
	cfg.ki = 0.5f;
	cfg.kd = 0.1f;
	cfg.o_min = 0.0f;
	cfg.o_max = 100.0f;
	cfg.d_filter_tau = 0.02f;
	cfg.dir = UPID_DIRECT;

	if (upid_create(&pid, &cfg) != UPID_OK) {
		printf("bad cfg\n");
		return 1;
	}

	for (step = 0; step < 2000; step++) {
		upid_sta status = upid_spin(&pid, setpoint,
					      read_measurement(), dt_s);
		float output;

		if (status != UPID_OK) {
			printf("update failed: %d\n", (int)status);
			return 1;
		}

		output = upid_get_output(&pid);
		assert(output >= cfg.o_min && output <= cfg.o_max);

		apply_output(output);

		if (step % 200 == 0)
			printf("t=%5.2fs  measurement=%7.3f  output=%7.3f\n",
			       step * dt_s, (double)read_measurement(),
			       (double)output);
	}

	/* This example is the repo's only runnable check: run it after edits. */
	assert(fabsf(read_measurement() - setpoint) < 0.5f);

	printf("settled at %.3f (setpoint %.1f)\n",
	       (double)read_measurement(), (double)setpoint);

	return 0;
}
