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

#ifndef _PID_H_
#define _PID_H_

#include <time.h>

#define NO_BOUNDVAL 0.0

typedef struct {
	/* Coefficients */
	float kp; /* proportional gain */
	float ki; /* integral gain */
	float kd; /* derivative gain */

	/* PID limits */
	float max;      /* Maximum allowed PID value */
	float min;      /* Minimum allowed PID value */
	float maxInteg; /* Maximum integral value */
	float minInteg; /* Minimum integral value */

	float integral; /* Accumulate for integral term */
	float prevErr;  /* Previous error */
	float lastPid;  /* Last calculated PID value */

	float errBound; /* positive boundary value for process variable (symmetric boundary value assumed) */
} pid_ctx_t;


/* Initialize pid fields. The coefficients and max/min PID values should be set by the user */
extern int pid_init(pid_ctx_t *pid);


/* Calculate the PID value based on gain values and precalculated change rate */
extern float pid_calc(pid_ctx_t *pid, float setVal, float currVal, float currValDot, time_t dt);


/* TODO: Tuning gain coefficients */
extern int pid_tune(pid_ctx_t *pid);


#endif
