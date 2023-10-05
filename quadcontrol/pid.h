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

	float val; /* current value of calculated coefficient impact */
} pid_coef_t;

typedef struct {
	pid_coef_t r; /* rate error (R) coefficient */
	pid_coef_t p; /* rate error P coefficient */
	pid_coef_t i; /* rate error I coefficient */
	pid_coef_t d; /* rate error D coefficient */

	float prevErr;  /* previous error for D controller */
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
float pid_calc(pid_ctx_t *pid, float targetPos, float currPos, float currRate, time_t dt);


/* TODO: Tuning gain coefficients */
extern int pid_tune(pid_ctx_t *pid);


#endif
