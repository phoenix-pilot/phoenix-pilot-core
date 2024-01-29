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

/* Ekf init flags */
#define EKF_INIT_LOG_SRC (1 << 0) /* Sets logs as input data for EKF */

/* Ekf status flags */
#define EKF_RUNNING  (1 << 0) /* EKF is working */
#define EKF_ERROR    (1 << 1) /* General purpose error flag */
#define EKF_MEAS_EOF (1 << 2) /* Measurements module encountered end-of-file */


typedef struct {
	int status;

	/* position in ENU frame in meters */
	float enuX;
	float enuY;
	float enuZ;

	/* velocity in ENU frame in meters per second */
	float veloX;
	float veloY;
	float veloZ;

	/* vehicle attitude, ranges according to Tait–Bryan convention */
	float pitch; /* (-PI/2, PI/2) */
	float yaw;   /* (-PI, PI) */
	float roll;  /* (-PI, PI) */

	/* vehicle attitude as quaternion */
	float q0;
	float q1;
	float q2;
	float q3;

	/* angular rates in uav frame of reference */
	float pitchDot;
	float yawDot;
	float rollDot;

	/* accelerations in earth frame of reference */
	float accelX;
	float accelY;
	float accelZ;

	float accelBiasZ;

	/* benchmarking */
	unsigned long long int stateTime;
	unsigned long long int imuTime;
} ekf_state_t;


extern int ekf_init(int initFlags);


extern int ekf_run(void);


extern int ekf_stop(void);


extern void ekf_done(void);


extern void ekf_stateGet(ekf_state_t *ekf_state);


extern void ekf_boundsGet(float *bYaw, float *bRoll, float *bPitch);


/* calculates local ENU `east` and `north` coordinates of lat/lon geodesic coordinates (assumes height = MSL) */
extern int ekf_latlon2en(double lat, double lon, float *east, float *north);


#endif
