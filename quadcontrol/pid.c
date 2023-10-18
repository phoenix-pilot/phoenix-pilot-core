/*
 * Phoenix-RTOS
 *
 * Two stage R->PID controller
 * Rate -> Proportional - Integral - Derivative controller
 *
 * Copyright 2022-2023 Phoenix Systems
 * Author: Hubert Buczynski, Mateusz Niewiadomski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include "pid.h"
#include "control.h"
#include "log.h"

#include <errno.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <vec.h>

/* Stores new coefficient value in `c`. Adjusts for final min/max values and IIR filtering */
static void pid_store(pid_coef_t *c, float newVal)
{
	float val;

	/* IIR filtering */
	val = (c->f == 0.0) ? newVal : c->val.scl * c->f + (1.0 - c->f) * newVal;

	if (val > c->max) {
		val = c->max;
	}
	if (val < -c->max) {
		val = -c->max;
	}

	c->val.scl = val;
}


static void pid_store3d(pid_coef_t *c, vec_t *newVal)
{
	vec_t val = *newVal;
	float len;

	/* IIR filtering */
	if (c->f != 0.0) {
		vec_times(&val, 1.0 - c->f);
		vec_times(&c->val.vec, c->f);
		vec_add(&c->val.vec, &val);
	}
	else {
		c->val.vec = val;
	}

	len = vec_len(&c->val.vec);

	if (len > c->max) {
		vec_times(&c->val.vec, c->max / len);
	}
}


float pid_calc(pid_ctx_t *pid, float targetPos, float currPos, float currRate, time_t dt)
{
	float err, out = 0;
	float timeStep = (float)dt / 1000;

	/* Position error calculation with boundary values check */
	err = targetPos - currPos;
	if (pid->errBound != NO_BOUNDVAL) {
		if (err > pid->errBound) {
			err -= 2 * pid->errBound;
		}
		if (err < -pid->errBound) {
			err += 2 * pid->errBound;
		}
	}

	/* Target rate calculation */
	pid_store(&pid->r, err * pid->r.k);

	/* P gain */
	err = pid->r.val.scl - currRate;
	pid_store(&pid->p, err * pid->p.k);
	if ((pid->flags & PID_IGNORE_P) == 0) {
		out += pid->p.val.scl;
	}

	/* I gain */
	pid_store(&pid->i, pid->i.val.scl + err * timeStep * pid->i.k);
	if ((pid->flags & PID_RESET_I) != 0) {
		pid->i.val.scl = 0;
	}
	if ((pid->flags & PID_IGNORE_I) == 0) {
		out += pid->i.val.scl;
	}

	/* D gain */
	pid_store(&pid->d, (err - pid->prevErr.scl) * pid->d.k / timeStep);
	if ((pid->flags & PID_IGNORE_D) == 0) {
		out += pid->d.val.scl;
	}
	pid->prevErr.scl = err;

	return out;
}


void pid_calc3d(pid_ctx_t *pid, const vec_t *targetPos, const vec_t *currPos, const vec_t *currRate, time_t dt, vec_t *out)
{
	float timeStep = (float)dt / 1000;
	vec_t posErr, rateErr, tmp;
	vec_t res = { 0 };

	/* Position error calculation with boundary values check */
	vec_dif(targetPos, currPos, &posErr);

	/* Target rate calculation */

	vec_times(&posErr, pid->r.k);
	pid_store3d(&pid->r, &posErr);
	vec_dif(&pid->r.val.vec, currRate, &rateErr);

	/* P gain */
	tmp = rateErr;
	vec_times(&tmp, pid->p.k);
	pid_store3d(&pid->p, &tmp);
	if ((pid->flags & PID_IGNORE_P) == 0) {
		vec_add(&res, &pid->p.val.vec);
	}

	/* I gain */
	tmp = rateErr;
	vec_times(&tmp, timeStep * pid->i.k);
	vec_add(&tmp, &pid->i.val.vec);
	pid_store3d(&pid->i, &tmp);
	if ((pid->flags & PID_RESET_I) != 0) {
		pid->i.val.vec.x = 0;
		pid->i.val.vec.y = 0;
		pid->i.val.vec.z = 0;
	}
	if ((pid->flags & PID_IGNORE_I) == 0) {
		vec_add(&res, &pid->i.val.vec);
	}

	/* D gain */
	tmp = rateErr;
	vec_sub(&tmp, &pid->prevErr.vec);
	vec_times(&tmp, pid->d.k / timeStep);
	if ((pid->flags & PID_IGNORE_D) == 0) {
		vec_add(&res, &pid->d.val.vec);
	}
	pid->prevErr.vec = rateErr;

	*out = res;
}


/* TODO: tune PID coefficients */
int pid_tune(pid_ctx_t *pid)
{
	if (pid == NULL) {
		return -EINVAL;
	}

	return 0;
}


int pid_init(pid_ctx_t *pid)
{
	if (pid == NULL) {
		return -EINVAL;
	}

	memset(&pid->r.val, 0, sizeof(pid->r.val));
	memset(&pid->p.val, 0, sizeof(pid->r.val));
	memset(&pid->i.val, 0, sizeof(pid->r.val));
	memset(&pid->d.val, 0, sizeof(pid->r.val));
	memset(&pid->prevErr, 0, sizeof(pid->prevErr));

	pid->errBound = NO_BOUNDVAL;
	pid->flags = PID_FULL;

	return 0;
}
