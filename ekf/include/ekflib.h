/*
 * Phoenix-Pilot
 *
 * extended kalman filter library header file
 *
 * Copyright 2022 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#ifndef EKFLIB_H
#define EKFLIB_H

typedef struct {
	/* position in ENU frame in meters */
	float enuX;
	float enuY;
	float enuZ;

	/* vehicle attitude, ranges according to Tait–Bryan convention */
	float pitch; /* (-PI/2, PI/2) */
	float yaw;   /* (-PI, PI) */
	float roll;  /* (-PI, PI) */
} ekf_state_t;

extern int ekf_init(void);

extern int ekf_run(void);

extern void ekf_done(void);

extern void ekf_stateGet(ekf_state_t *ekf_state);

#endif
