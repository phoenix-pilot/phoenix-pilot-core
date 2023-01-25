/*
 * Phoenix-RTOS
 *
 * MMA (Motor Mixing Algorithm)
 *
 * Copyright 2022 Phoenix Systems
 * Author: Hubert Buczynski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _MMA_H_
#define _MMA_H_

#include "control.h"

/*
Pid input attenuation factor is relative to throttle.
Factor curve has three points (throttle, factor): [ (0, startVal), (midArg, midVal), (1, enfVal) ]
Middle point of attenuation must be within (0.1, 0.9) throttle range.
Attenuation factor values must be within (0, 2) range
*/
typedef struct {
	float startVal; /* attenuation curve value at throttle = 0 */
	float midArg;	/* attenuation curve middle point */
	float midVal;	/* attenuation curve value at throttle = midArg */
	float endVal;	/* attenuation curve value at throttle = 1 */

	float slope[2]; /* Slopes of attenuation curve. [0] for start-mid, [1] for mid-end curve */
} mma_atten_t;


/* Based on PID values, the PWM are set to the each motor */
extern int mma_control(float palt, float proll, float ppitch, float pyaw);


/* Set motors in idle state and arm motors */
extern void mma_start(void);


/* Set motors in idle state and disarm motors */
extern void mma_stop(void);


/* Disarmed motors and disable module */
extern void mma_done(void);


/* MMA module initialization */
extern int mma_init(const quad_coeffs_t *coeffs, const mma_atten_t *atten);


#endif
