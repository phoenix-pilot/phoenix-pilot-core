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


/* Based on PID values, the PWM are set to the each motor */
extern void mma_control(float palt, float proll, float ppitch, float pyaw);


/* Reduce motors velocity to minium */
extern void mma_stop(void);


/* Disable module */
extern void mma_done(void);


/* MMA module initialization */
extern int mma_init(const quad_coeffs_t *coeffs);

#endif
