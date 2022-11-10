/*
 * Phoenix-Pilot
 *
 * extended kalman filter
 * 
 * data acquisition logic/calibration
 *
 * Copyright 2022 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */


#include <stdio.h>

#include <vec.h>
#include <quat.h>

#ifndef _EKF_MEAS_T_
#define _EKF_MEAS_T_


typedef struct {
	float lat;
	float lon;
	float h;
} meas_geodetic_t;


/* initial values calculated during calibration */
typedef struct {
	struct {
		quat_t init_q;   /* starting position quaternion */
		vec_t init_m;    /* starting magnetic field */
		vec_t gyr_nivel; /* starting gyro bias */
	} imu;

	struct {
		float base_pressure; /* starting atmospheric pressure */
		float base_temp;     /* starting temperature */
	} baro;

	struct {
		vec_t refEcef;               /* reference point ECEF coordinates */
		meas_geodetic_t refGeodetic; /* reference point geodetic coordinates */
	} gps;
} meas_calib_t;


/* CALIBRATION INITIALIZERS */

/* obtain current IMU calibration parameters */
void meas_imuCalib(void);

/* obtain current barometer calibration parameters */
void meas_baroCalib(void);

/* obtain current gps calibration parameters */
void meas_gpsCalib(void);


/* CALIBRATION GETTERS */

/* Return pointer to full calibration data */
const meas_calib_t *meas_calibGet(void);

/* returns the calibration pressure in Pascals */
float meas_calibPressGet(void);


/* MEASUREMENT ACQUISITION */

/* Returns prepared IMU data in SI units */
int meas_imuGet(vec_t *accels, vec_t *gyros, vec_t *mags, uint64_t *timestamp);

/* Returns prepared barometer data in SI units */
int meas_baroGet(float *pressure, float *temperature, uint64_t *dtBaroUs);

/* Returns prepared GPS data in SI units */
int meas_gpsGet(vec_t *enu, vec_t *enu_speed, float *hdop);

#endif
