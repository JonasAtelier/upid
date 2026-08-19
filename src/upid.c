#include "upid.h"

#include <float.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

/* Exponent field of an IEEE-754 single. All ones means infinity or NaN. */
#define FLOAT_EXP_MASK		0x7F800000u

/* Output ceiling a default cfg starts with, so that the required
 * o_min < o_max already holds before the caller touches anything. */
#define DEFAULT_O_MAX		1.0f

/* No filter coefficient cached yet. Safe as a sentinel because a cached
 * entry always belongs to a dt above zero. */
#define ALPHA_UNCACHED		0.0f

/* Neutral coefficient: an unfiltered derivative passes straight through. */
#define ALPHA_UNFILTERED	1.0f

/*
 * Written by upid_create, checked by everything else. A controller that was
 * never created - a struct on the stack, or a static one sitting in zeroed
 * BSS - will not carry this pattern, so it is reported as holding no
 * valid cfg instead of running on whatever the memory happened to hold.
 */
#define UPID_MARKER		0x55504944u	/* "UPID" */

/* Error sign for each control direction. */
#define SIGN_DIRECT		1.0f
#define SIGN_REVERSE		(-1.0f)

static bool is_finite(float val)
{
	uint32_t bits;

	memcpy(&bits, &val, sizeof bits);

	return ((bits & FLOAT_EXP_MASK) != FLOAT_EXP_MASK);
}

/*
 * Usable means: a real pointer, created, and holding tuning that passed
 * validation. Everything that reads controller state goes through here.
 */
static upid_sta is_ready(const struct upid *pid)
{
	if (!pid)
		return UPID_EINVAL_INPUT;

	if (pid->marker != UPID_MARKER || pid->sta != UPID_OK)
		return UPID_EINVAL_CFG;

	return UPID_OK;
}

static bool is_cfg_finite(const struct upid_cfg *cfg)
{
	return (is_finite(cfg->kp) && is_finite(cfg->ki) &&
		is_finite(cfg->kd) && is_finite(cfg->o_min) &&
		is_finite(cfg->o_max) && is_finite(cfg->d_filter_tau));
}

static upid_sta is_cfg_valid(const struct upid_cfg *cfg)
{
	if (!is_cfg_finite(cfg))
		return UPID_EINVAL_INPUT;

	if (cfg->kp < 0.0f || cfg->ki < 0.0f || cfg->kd < 0.0f ||
	    cfg->d_filter_tau < 0.0f ||
	    (cfg->dir != UPID_DIRECT && cfg->dir != UPID_REVERSE) ||
	    !(cfg->o_min < cfg->o_max))
		return UPID_EINVAL_CFG;

	return UPID_OK;
}

static float clamp_output(float val, const struct upid_cfg *cfg)
{
	/*
	 * Preserve the configured sign of a zero-valued boundary for IEEE-754
	 * callers that distinguish +0.0 from -0.0.
	 */
	if (val < cfg->o_min ||
	    (val == 0.0f && cfg->o_min == 0.0f && !signbit(cfg->o_min)))
		return cfg->o_min;

	if (val > cfg->o_max ||
	    (val == 0.0f && cfg->o_max == 0.0f && signbit(cfg->o_max)))
		return cfg->o_max;

	return val;
}

upid_sta upid_cfg_status(const struct upid *pid)
{
	if (!pid)
		return UPID_EINVAL_INPUT;

	if (pid->marker != UPID_MARKER)
		return UPID_EINVAL_CFG;

	return pid->sta;
}

const struct upid_cfg *upid_get_cfg(const struct upid *pid)
{
	if (!pid || pid->marker != UPID_MARKER)
		return NULL;

	return &pid->cfg;
}

upid_sta upid_set_cfg(struct upid *pid, const struct upid_cfg *cfg)
{
	upid_sta res;
	float output_candidate;
	double adjusted_integral;

	if (!pid || !cfg)
		return UPID_EINVAL_INPUT;

	/* Not is_ready: this call is what makes a controller valid. */
	if (pid->marker != UPID_MARKER)
		return UPID_EINVAL_CFG;

	res = is_cfg_valid(cfg);

	if (res != UPID_OK)
		return res;

	output_candidate = clamp_output(pid->prev_o, cfg);
	adjusted_integral = (double)pid->integral +
			    (double)output_candidate -
			    (double)pid->prev_o;

	if (adjusted_integral > FLT_MAX || adjusted_integral < -FLT_MAX)
		return UPID_EINVAL_INPUT;

	pid->cfg = *cfg;
	pid->alpha_dt = ALPHA_UNCACHED;	/* tau may have moved; recompute it */
	pid->prev_o = output_candidate;
	pid->integral = (float)adjusted_integral;
	pid->sta = UPID_OK;

	return UPID_OK;
}

/*
 * One control cycle: validate the sample, work out P, I and D, clamp the
 * result and keep the history the next cycle needs.
 */
upid_sta upid_spin(struct upid *pid, float target, float mea, float dt_s)
{
	float sign, err, p_out, i_out, d_out, output;
	float mea_rate, filter_mea_rate, alpha, base;
	upid_sta res;

	res = is_ready(pid);

	if (res != UPID_OK)
		return res;

	if (pid->mode == UPID_MANUAL && !pid->is_auto_pend)
		return UPID_EINVAL_MODE;

	if (!is_finite(dt_s))
		return UPID_EINVAL_INPUT;

	if (dt_s <= 0.0f)
		return UPID_EINVAL_TSTAMP;

	if (!is_finite(target) || !is_finite(mea))
		return UPID_EINVAL_INPUT;

	d_out = 0.0f;
	filter_mea_rate = 0.0f;

	sign = (pid->cfg.dir == UPID_DIRECT) ? SIGN_DIRECT : SIGN_REVERSE;
	err = sign * (target - mea);

	/* proportional term */
	p_out = pid->cfg.kp * err;

	if (!is_finite(err) || !is_finite(p_out))
		return UPID_EINVAL_INPUT;

	if (pid->is_auto_pend) {
		/*
		 * Align the integral term with the manual output so automatic
		 * control can take ownership without causing an output jump.
		 */
		i_out = pid->prev_o - p_out;

		if (!is_finite(i_out))
			return UPID_EINVAL_INPUT;

		pid->prev_mea = mea;
		pid->filter_mea_rate = 0.0f;
		pid->is_mea_init = true;
		pid->integral = i_out;
		pid->mode = UPID_AUTO;
		pid->is_auto_pend = false;

		return UPID_OK;
	}

	if (pid->is_mea_init) {
		/*
		 * Differentiate measurement, not error, to avoid derivative
		 * kick when the setpoint changes abruptly.
		 */
		base = ALPHA_UNFILTERED;
		alpha = ALPHA_UNFILTERED;

		mea_rate = (mea - pid->prev_mea) / dt_s;

		if (!is_finite(mea_rate))
			return UPID_EINVAL_INPUT;

		if (pid->cfg.d_filter_tau != 0.0f) {
			/*
			 * First-order low-pass filter coefficient; tau == 0
			 * deliberately selects an unfiltered derivative.
			 *
			 * It depends only on dt and tau, so a fixed-rate loop
			 * would divide to reach the same number every cycle.
			 * Cache it against the dt that produced it; anything
			 * that can change tau clears alpha_dt, and dt_s is
			 * always above zero, so 0 is a safe "nothing cached".
			 */
			if (dt_s == pid->alpha_dt) {
				alpha = pid->alpha;
			} else {
				base = pid->cfg.d_filter_tau + dt_s;

				if (!is_finite(base))
					return UPID_EINVAL_INPUT;

				alpha = dt_s / base;

				if (!is_finite(alpha))
					return UPID_EINVAL_INPUT;

				pid->alpha_dt = dt_s;
				pid->alpha = alpha;
			}
		}

		/* filtered = prev + alpha * (current - prev) */
		filter_mea_rate = pid->filter_mea_rate +
				  alpha * (mea_rate - pid->filter_mea_rate);

		if (!is_finite(filter_mea_rate))
			return UPID_EINVAL_INPUT;

		/* derivative term */
		d_out = sign * pid->cfg.kd * filter_mea_rate;

		if (!is_finite(d_out))
			return UPID_EINVAL_INPUT;
	}

	/* integral term */
	i_out = pid->integral + pid->cfg.ki * err * dt_s;

	if (!is_finite(i_out))
		return UPID_EINVAL_INPUT;

	output = p_out + i_out - d_out;

	if (!is_finite(output))
		return UPID_EINVAL_INPUT;

	/*
	 * Conditional integration blocks windup but permits movement back
	 * from saturation toward the configured output range.
	 */
	if ((output >= pid->cfg.o_min && output <= pid->cfg.o_max) ||
	    (output > pid->cfg.o_max && err < 0.0f) ||
	    (output < pid->cfg.o_min && err > 0.0f))
		pid->integral = i_out;

	pid->prev_o = clamp_output(output, &pid->cfg);
	pid->is_mea_init = true;
	pid->prev_mea = mea;
	pid->filter_mea_rate = filter_mea_rate;

	return UPID_OK;
}

upid_sta upid_set_auto(struct upid *pid)
{
	upid_sta res = is_ready(pid);

	if (res != UPID_OK)
		return res;

	if (pid->mode == UPID_MANUAL)
		pid->is_auto_pend = true;

	return UPID_OK;
}

upid_sta upid_set_manual(struct upid *pid, float output)
{
	upid_sta res = is_ready(pid);

	if (res != UPID_OK)
		return res;

	if (!is_finite(output))
		return UPID_EINVAL_INPUT;

	pid->mode = UPID_MANUAL;
	pid->is_auto_pend = false;
	pid->prev_o = clamp_output(output, &pid->cfg);

	return UPID_OK;
}

upid_sta upid_reset(struct upid *pid, float mea, float out)
{
	float output_candidate;
	upid_sta res = is_ready(pid);

	if (res != UPID_OK)
		return res;

	if (!is_finite(mea) || !is_finite(out))
		return UPID_EINVAL_INPUT;

	output_candidate = clamp_output(out, &pid->cfg);

	pid->prev_mea = mea;
	pid->filter_mea_rate = 0.0f;
	pid->is_mea_init = true;
	pid->integral = output_candidate;
	pid->prev_o = output_candidate;

	return UPID_OK;
}

upid_mode upid_get_mode(const struct upid *pid)
{
	if (!pid || pid->marker != UPID_MARKER)
		return UPID_MANUAL;

	return pid->mode;
}

float upid_get_output(const struct upid *pid)
{
	if (!pid || pid->marker != UPID_MARKER)
		return 0.0f;

	return pid->prev_o;
}

/* ---- setting a controller up. Kept last: everything above is the
 * running path, and this is what you call once before entering it. ----
 */

upid_sta upid_cfg_init(struct upid_cfg *cfg)
{
	if (!cfg)
		return UPID_EINVAL_INPUT;

	cfg->kp = 0.0f;
	cfg->ki = 0.0f;
	cfg->kd = 0.0f;
	cfg->o_min = 0.0f;
	cfg->o_max = DEFAULT_O_MAX;
	cfg->d_filter_tau = 0.0f;
	cfg->dir = UPID_DIRECT;

	return UPID_OK;
}

/* Everything except the tuning, which upid_set_cfg() installs. */
static void state_reset(struct upid *pid)
{
	pid->marker = UPID_MARKER;
	pid->sta = UPID_EINVAL_CFG;
	pid->prev_o = 0.0f;
	pid->integral = 0.0f;
	pid->mode = UPID_AUTO;
	pid->is_auto_pend = false;
	pid->is_mea_init = false;
	pid->prev_mea = 0.0f;
	pid->filter_mea_rate = 0.0f;
	pid->alpha_dt = ALPHA_UNCACHED;
	pid->alpha = 0.0f;
}

upid_sta upid_create(struct upid *pid, const struct upid_cfg *cfg)
{
	struct upid_cfg wanted;

	if (!pid || !cfg)
		return UPID_EINVAL_INPUT;

	/*
	 * Copy first: cfg is allowed to be &pid->cfg, which the reset below
	 * overwrites with the defaults.
	 */
	wanted = *cfg;

	upid_cfg_init(&pid->cfg);
	state_reset(pid);

	return upid_set_cfg(pid, &wanted);
}
