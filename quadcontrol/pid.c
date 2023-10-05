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


/* Stores new coefficient value in `c`. Adjusts for final min/max values and IIR filtering */
static void pid_store(pid_coef_t *c, float newVal)
{
	float val;

	/* IIR filtering */
	val = (c->f == 0.0) ? newVal : c->val * c->f + (1.0 - c->f) * newVal;

	if (val > c->max) {
		val = c->max;
	}
	if (val < -c->max) {
		val = -c->max;
	}

	c->val = val;
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
	err = pid->r.val - currRate;
	pid_store(&pid->p, err * pid->p.k);
	if ((pid->flags & PID_IGNORE_P) == 0) {
		out += pid->p.val;
	}

	/* I gain */
	pid_store(&pid->i, pid->i.val + err * timeStep * pid->i.k);
	if ((pid->flags & PID_RESET_I) != 0) {
		pid->i.val = 0;
	}
	if ((pid->flags & PID_IGNORE_I) == 0) {
		out += pid->i.val;
	}

	/* D gain */
	pid_store(&pid->d, (err - pid->prevErr) * pid->d.k / timeStep);
	if ((pid->flags & PID_IGNORE_D) == 0) {
		out += pid->d.val;
	}
	pid->prevErr = err;

	return out;
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

	pid->prevErr = 0;
	pid->errBound = NO_BOUNDVAL;
	pid->flags = PID_FULL;

	return 0;
}
