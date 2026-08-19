/*
 * upid - portable PID controller.
 *
 * No platform headers, no hardware access, no allocation, no clock of its
 * own. You supply the elapsed time and call upid_spin() once per control
 * sample.
 *
 * C99, libm only.
 */
#ifndef UPID_UPID_H
#define UPID_UPID_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Result of a controller operation. */
typedef enum {
	UPID_OK = 0,
	UPID_EINVAL_TSTAMP,	/* dt was zero or negative */
	UPID_EINVAL_INPUT,	/* a value was not finite, or overflowed */
	UPID_EINVAL_CFG,	/* the controller holds no valid tuning */
	UPID_EINVAL_MODE,	/* wrong mode for the call */
} upid_sta;

/* Relationship between controller output and process measurement. */
typedef enum {
	UPID_DIRECT = 0,	/* more output raises the measurement */
	UPID_REVERSE,		/* more output lowers it */
} upid_dir;

/* Selects whether the caller or the PID calculation owns the output. */
typedef enum {
	UPID_MANUAL = 0,
	UPID_AUTO,
} upid_mode;

/*
 * PID gains, limits, and derivative filtering.
 *
 * Gains use the parallel form: P + integral(I) - derivative(measurement).
 */
struct upid_cfg {
	float kp;
	float ki;
	float kd;
	float o_min;
	float o_max;
	float d_filter_tau;	/* derivative filter tau; 0 disables filtering */
	upid_dir dir;
};

/*
 * Controller state. Declare one wherever you like - on the stack, in a
 * static, inside your own struct - and hand its address to every call.
 * Nothing here is allocated.
 *
 * The fields are yours to read and nothing else; use the functions to
 * change them. Until upid_create() has run the contents mean nothing, and
 * every call refuses to touch it.
 */
struct upid {
	uint32_t marker;	/* set by upid_create; proves it ran */
	struct upid_cfg cfg;
	upid_sta sta;
	float prev_o;
	float integral;
	upid_mode mode;
	bool is_auto_pend;
	bool is_mea_init;
	float prev_mea;
	float filter_mea_rate;
	float alpha_dt;		/* dt the cached filter coefficient belongs to */
	float alpha;		/* cached first-order filter coefficient */
};

/*
 * Load default tuning into cfg: gains zero, output range 0 to 1, no
 * derivative filter, direct acting.
 *
 * Start here rather than from a zeroed struct - o_min < o_max is required,
 * so all-zero is not a valid cfg:
 *
 *     struct upid_cfg cfg;
 *     upid_cfg_init(&cfg);
 *     cfg.kp = 2.0f;
 *     cfg.o_max = 255.0f;
 */
upid_sta upid_cfg_init(struct upid_cfg *cfg);

/*
 * Initialise, or reinitialise, a controller: clears the running state,
 * validates the tuning, installs it, and starts in automatic mode. This is
 * the only call needed to get from a declared struct to a running one.
 *
 * Returns UPID_EINVAL_CFG for tuning that cannot be honoured, in which case
 * the controller stays unrunnable rather than running on bad numbers. The
 * tuning is copied in, so cfg does not have to outlive the call, and
 * passing &pid->cfg is safe.
 */
upid_sta upid_create(struct upid *pid, const struct upid_cfg *cfg);

/* Whether the controller currently holds a valid cfg. */
upid_sta upid_cfg_status(const struct upid *pid);

/* The active cfg. NULL if pid is NULL. */
const struct upid_cfg *upid_get_cfg(const struct upid *pid);

/* Validate and apply new tuning, preserving controller state. */
upid_sta upid_set_cfg(struct upid *pid, const struct upid_cfg *cfg);

/* Process one sample. dt_s is the elapsed time in seconds. */
upid_sta upid_spin(struct upid *pid, float target, float mea, float dt_s);

/* Hand the output back to the PID, with a bumpless transition. */
upid_sta upid_set_auto(struct upid *pid);

/* Take the output over, clamped to the configured limits. */
upid_sta upid_set_manual(struct upid *pid, float output);

/* Reset history around known process and output values. */
upid_sta upid_reset(struct upid *pid, float mea, float out);

/* Who owns the output right now. UPID_MANUAL if pid is NULL. */
upid_mode upid_get_mode(const struct upid *pid);

/* The most recent valid, clamped output. 0 if pid is NULL. */
float upid_get_output(const struct upid *pid);

#ifdef __cplusplus
}
#endif

#endif /* UPID_UPID_H */
