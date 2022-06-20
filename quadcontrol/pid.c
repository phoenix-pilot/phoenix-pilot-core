/*
 * Phoenix-RTOS
 *
 * PID (Proportional – Integral – Derivative Controller)
 *
 * Copyright 2022 Phoenix Systems
 * Author: Hubert Buczynski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include "pid.h"

#include <errno.h>
#include <stdlib.h>


float pid_calc(pid_ctx_t *pid, float setVal, float currVal, float dt)
{
	float err, out;
	float p, i, d; /* Results for proportional, integral and derivative parts of PID */

	if (pid == NULL) {
		return -EINVAL;
	}

	if (__builtin_isfinite(setVal) == 0 || __builtin_isfinite(currVal) == 0 || __builtin_isfinite(dt) == 0) {
		return pid->lastPid;
	}

	err = setVal - currVal;

	/* Derivative */
	d =(pid->kd * (err - pid->prevErr)) / dt;

	/* Proportional */
	p = pid->kp * err;

	/* Integral */
	i = pid->integral + pid->ki * (err * dt);
	if (i > pid->maxInteg) {
		i = pid->maxInteg;
	}

	/* PID */
	out = p + i + d;
	if (out > pid->max) {
		out = pid->max;
	}

	if (out < pid->min) {
		out = pid->min;
	}

	pid->integral = i;
	pid->prevErr = err;
	pid->lastPid = out;

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

	pid->integral = 0;
	pid->lastPid = 0;
	pid->prevErr = 0;

	return 0;
}
