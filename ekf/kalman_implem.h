/*
 * Phoenix-Pilot
 *
 * extended kalman filter
 * 
 * EKF implementation specific code header file. Declares prediction and update engines routines/functions
 *
 * Copyright 2022 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#ifndef PHKALMAN_H
#define PHKALMAN_H

#include <sys/time.h>
#include <stdint.h>

#include <matrix.h>
#include <vec.h>
#include <quat.h>

#include "kalman_core.h"
#include "meas.h"

#define EARTH_G       9.80665F   /* m/s^2 */
#define UNI_GAS_CONST 8.3144598F /* J/(mol * K) */
#define AIR_MOL_MASS  0.0289644F /* kg/mol */

#define DEG2RAD 0.0174532925

/* */
/* Kalman filter index defines */
/* */

#define STATE_LENGTH     13
#define CTRL_LENGTH      6
#define MEAS_IMU_LENGTH  9
#define MEAS_BARO_LENGTH 1

/* STATE VECTOR */
/* Attitude quaternion rotates vectors from body frame of reference to inertial frame of reference */
#define QA  0 /* attitude quaternion real part */
#define QB  1 /* attitude quaternion i part */
#define QC  2 /* attitude quaternion j part */
#define QD  3 /* attitude quaternion k part */
/* Gyroscope body frame biases describes offsets of gyro measurements when stationary */
#define BWX 4 /* gyroscope x axis bias */
#define BWY 5 /* gyroscope y axis bias */
#define BWZ 6 /* gyroscope z axis bias */
/* Velocity in inertial frame of reference */
#define VX 7 /* velocity x component */
#define VY 8 /* velocity y component */
#define VZ 9 /* velocity z component */
/* Accelerometer body frame biases describes acceleration offsets in flight */
#define BAX 10 /* accelerometer x axis bias */
#define BAY 11 /* accelerometer y axis bias */
#define BAZ 12 /* accelerometer z axis bias */

/* control vector u */
#define UWX 0
#define UWY 1
#define UWZ 2
#define UAX 3
#define UAY 4
#define UAZ 5

/* IMU measurement vector indexes */
#define MGX 0
#define MGY 1
#define MGZ 2
#define MEX 3
#define MEY 4
#define MEZ 5
#define MBWX 6 /* Measurement of gyroscope x axis bias (body frame of reference) */
#define MBWY 7 /* Measurement of gyroscope y axis bias (body frame of reference) */
#define MBWZ 8 /* Measurement of gyroscope z axis bias (body frame of reference) */

/* Baro measurement vector indexes */
#define MDZ 0 /* Measurement of change in height (NED z component change) */


/* IMPORTANT: must be kept in order with 'char * configNames' in 'kalman.inits.c' */
typedef struct {
	int verbose;
	int log;

	/* State covariance error initialization values */
	float P_qerr;
	float P_verr;
	float P_baerr;
	float P_bwerr;

	/* IMU measurement model standard deviations */
	float R_astdev;
	float R_mstdev;
	float R_bwstdev;

	/* Baro measurement model standard deviations */
	float R_dzstdev;

	/* State process noise values */
	float Q_astdev;
	float Q_wstdev;
	float Q_baDotstdev;
	float Q_bwDotstdev;
} kalman_init_t;


int verbose;


extern void kmn_configRead(kalman_init_t *initVals);

/* Reads from a matrix like from a untransposed column vector directly. Invalid read returns 0.f */
static inline float kmn_vecAt(const matrix_t *M, unsigned int i)
{
	return (i < M->rows && M->cols == 1) ? M->data[i] : 0.f;
}


/* PHMATRIX MATRICES INITIALIZATIONS */

/* initializes matices related to state prediction step of kalman filter */
extern void kmn_predInit(state_engine_t *engine, const meas_calib_t *calib, const kalman_init_t *inits);

/* imu update engine composer */
extern void kmn_imuEngInit(update_engine_t *engine, const kalman_init_t *inits);

/* barometer update engine composer */
extern void kmn_baroEngInit(update_engine_t *engine, const kalman_init_t *inits);

/* GPS update engine composer */
extern void kmn_gpsEngInit(update_engine_t *engine, const kalman_init_t *inits);


#endif
