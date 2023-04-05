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

#define BARO_UPDATE_TIMEOUT 40000
#define GPS_UPDATE_TIMEOUT  200000

/* If the difference between EARTH_G and acceleration length is beyond ACC_SIGMA_THRESHOLD the accelSigma is multiplied by ACC_SIGMA_STEP_FACTOR */
#define ACC_SIGMA_STEP_THRESHOLD 3.f
#define ACC_SIGMA_STEP_FACTOR    50

#define ACC_DAMP_THRESHOLD 0.5 /* accelerometer damping threshold length (in m/s^2) */

/* */
/* Kalman filter index defines */
/* */

#define STATE_LENGTH     16
#define CTRL_LENGTH      6
#define MEAS_IMU_LENGTH  6
#define MEAS_BARO_LENGTH 2
#define MEAS_GPS_LENGTH  4

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
/* Position in inertial frame of reference (NED) */
#define RX 13 /* position x component */
#define RY 14 /* position y component */
#define RZ 15 /* position z component */


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

/* Baro measurement vector indexes */
#define MDZ 0 /* Measurement of change in height (NED z component change) */
#define MRZ 1 /* Measurement of vertical position (NED height) */

/* GPS measurement */
#define MGPSRX 0 /* NED x position */
#define MGPSRY 1 /* NED y position */
#define MGPSVX 2 /* NED x speed */
#define MGPSVY 3 /* NED y speed */

typedef struct {
	int verbose;
	int log;

	/* State covariance error initialization values */
	float P_qerr;
	float P_verr;
	float P_baerr;
	float P_bwerr;
	float P_rerr;

	/* IMU measurement model standard deviations */
	float R_astdev;
	float R_mstdev;
	float R_bwstdev;

	/* Baro measurement model standard deviations */
	float R_hstdev;

	float R_gpsxstdev;
	float R_gpsvstdev;

	/* State process noise values */
	float Q_astdev;
	float Q_wstdev;
	float Q_baDotstdev;
	float Q_bwDotstdev;
} kalman_init_t;


int verbose;


/* Function reads ekf configuration file under `path` and fills structure pointed by `initVals`  */
extern int kmn_configRead(const char *configFile, kalman_init_t *initVals);

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
