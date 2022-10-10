/*
 * Phoenix-RTOS
 *
 * MMA (Motor Mixing Algorithm) for Plane
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


/* Based on PID values, the PWM are set to the each motor */
extern int mma_control(float throttle, float proll, float ppitch, float pyaw);


/* Set motors in idle state and arm motors */
extern void mma_start(void);


/* Set motors in idle state and disarm motors */
extern void mma_stop(void);


/* Disarmed motors and disable module */
extern void mma_done(void);


/* MMA module initialization */
extern int mma_init(void);


#endif
