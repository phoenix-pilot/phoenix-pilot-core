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
#include <sys/types.h>

#include <vec.h>
#include <quat.h>

#ifndef _EKF_MEAS_T_
#define _EKF_MEAS_T_


/* clang-format off */
typedef enum { srcSens = 0, srcLog } meas_sourceType_t;
/* clang-format on */


typedef struct {
	double lat;
	double lon;
	float h;

	double sinLat;
	double sinLon;

	double cosLat;
	double cosLon;
} meas_geodetic_t;


/* initial values calculated during calibration */
typedef struct {
	struct {
		quat_t initQuat; /* starting position quaternion */
		vec_t initMag;   /* starting magnetic field */
		vec_t gyroBias;  /* starting gyro bias */
	} imu;

	struct {
		float basePress; /* starting atmospheric pressure */
		float baseTemp;  /* starting temperature */
	} baro;

	struct {
		double refEcef[3];           /* reference point ECEF coordinates */
		meas_geodetic_t refGeodetic; /* reference point geodetic coordinates */
	} gps;
} meas_calib_t;

/* General structure type to return gps measurements of GPS in SI units */
typedef struct {
	/* Geodetic coordinates */
	double lat;
	double lon;

	/* Local NED coordinates */
	vec_t pos;
	vec_t vel;

	float eph;
	float epv;

	uint8_t satsNb;
	uint8_t fix;
} meas_gps_t;


extern int meas_init(meas_sourceType_t sourceType, const char *path, int senscInitFlags);


extern int meas_done(void);


/* CALIBRATION INITIALIZERS */

/* obtain current IMU calibration parameters */
extern int meas_imuCalib(void);

/* obtain current barometer calibration parameters */
extern int meas_baroCalib(void);

/* obtain current gps calibration parameters */
extern int meas_gpsCalib(void);


/* CALIBRATION GETTERS */

/* Return pointer to full calibration data */
extern const meas_calib_t *meas_calibGet(void);

/* returns the calibration pressure in Pascals */
extern float meas_calibPressGet(void);


/* MEASUREMENT ACQUISITION */

/*
 * In case of success poll functions returns 0;
 * If end-of-file is encountered returns EOF.
 * In case of an error returns EOF and sets appropriate errno value.
 */

extern int meas_imuPoll(time_t *timestamp);

extern int meas_baroPoll(void);

extern int meas_gpsPoll(void);

/* Returns prepared IMU data in SI units */
extern int meas_accelGet(vec_t *accels, vec_t *accelsRaw);

extern int meas_gyroGet(vec_t *gyro, vec_t *gyroRaw);

extern int meas_magGet(vec_t *mag);

/* Returns prepared barometer data in SI units */
extern int meas_baroGet(float *pressure, float *temperature, uint64_t *dtBaroUs);

/* Returns prepared GPS data in SI units */
extern int meas_gpsGet(meas_gps_t *gpsData, time_t *timestamp);

extern int meas_timeGet(time_t *useconds);

/* Returns (east, north) local frame coords of lat/lon  geodesic point */
extern int meas_latlon2en(double lat, double lon, float *east, float *north);

#endif
