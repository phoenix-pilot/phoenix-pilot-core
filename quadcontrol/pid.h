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

#ifndef _PID_H_
#define _PID_H_

#include <time.h>

#include <vec.h>

#define NO_BOUNDVAL 0.0

#define PID_FULL     0
#define PID_IGNORE_P (1 << 0)
#define PID_IGNORE_I (1 << 1)
#define PID_IGNORE_D (1 << 2)
#define PID_RESET_I  (1 << 3)

typedef struct {
	float k;   /* coefficient value */
	float max; /* maximum impact of this coefficient */
	float f;   /* coefficient IIR parameter (0 disables filtering) */

	union {
		float scl;
		vec_t vec;
	} val;
} pid_coef_t;


typedef struct {
	pid_coef_t r; /* rate error (R) coefficient */
	pid_coef_t p; /* rate error P coefficient */
	pid_coef_t i; /* rate error I coefficient */
	pid_coef_t d; /* rate error D coefficient */

	/* previous error for D controller */
	union {
		float scl;
		vec_t vec;
	} prevErr;
	float errBound; /* positive boundary value for process variable (symmetric boundary value assumed) */

	uint32_t flags; /* flags controlling pid controller behaviour */

} pid_ctx_t;


/* Initialize pid fields. The coefficients and max/min PID values should be set by the user */
extern int pid_init(pid_ctx_t *pid);


/*
 * PID controller calculation process:
 * 1) calculate position error from targetPos and currPos
 * 2) translate position error into target rate
 * 3) use target rate and currRate as base variable for standard PID controller
 * Performs cyclic boundary check on position error, and max/min checks on R,P,I and D controllers.
 */
extern float pid_calc(pid_ctx_t *pid, float targetPos, float currPos, float currRate, time_t dt);


/*
 * 3d vectorial PID controller calculation process:
 * 1) calculate position error from targetPos and currPos
 * 2) translate position error into target rate
 * 3) use target rate and currRate as base variable for standard PID controller
 * Performs cyclic boundary check on position error, and max/min checks on R,P,I and D controllers.
 */
extern void pid_calc3d(pid_ctx_t *pid, const vec_t *targetPos, const vec_t *currPos, const vec_t *currRate, time_t dt, vec_t *out);


/* TODO: Tuning gain coefficients */
extern int pid_tune(pid_ctx_t *pid);


#endif
