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

#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>

#include <sys/msg.h>

#include "kalman_implem.h"

#include <libsensors.h>
#include <sensc.h>
#include <vec.h>
#include <quat.h>
#include <matrix.h>

#define EARTH_SEMI_MAJOR           6378137.0F
#define EARTH_SEMI_MINOR           6356752.3F
#define EARTH_ECCENTRICITY_SQUARED 0.006694384F

#define IMU_CALIB_AVG  1000
#define BARO_CALIB_AVG 100

static struct {
	kalman_calib_t calib;
} meas_common;

/* accelerometer calibration data */
float acc_calib1[12] = {
	/* sphere offset in m/s^2 */
	0.237000, 0.120545, 0.022396,
	/* sphere deformation */
	1.00246374e+00,  1.82159288e-04,  8.18160423e-04,
	1.82159288e-04,  9.95663211e-01, -1.46614579e-04,
	8.18160423e-04, -1.46614579e-04,  1.00188801e+00
};


vec_t geo2ecef(float lat, float lon, float h)
{
	float sinLat, sinLon, cosLat, cosLon, N;

	/* point coordinates trigonometric values */
	sinLat = sin(lat * DEG2RAD);
	sinLon = sin(lon * DEG2RAD);
	cosLat = cos(lat * DEG2RAD);
	cosLon = cos(lon * DEG2RAD);
	N = EARTH_SEMI_MAJOR / sqrt(1 - EARTH_ECCENTRICITY_SQUARED * sinLat);

	return (vec_t) {
		.x = (N + h) * cosLat * cosLon,
		.y = (N + h) * cosLat * sinLon,
		.z = ((1 - EARTH_ECCENTRICITY_SQUARED) * N + h) * sinLat
	};
}

/* convert gps geodetic coordinates into neu (north/east/up) vector with reference point */
vec_t geo2enu(float lat, float lon, float h, float latRef, float lonRef, vec_t *refEcef)
{
	float sinLatRef, sinLonRef, cosLatRef, cosLonRef;
	float rot_data[9], dif_data[3], enu_data[3];
	matrix_t rot = { .rows = 3, .cols = 3, .transposed = 0, .data = rot_data };
	matrix_t dif = { .rows = 3, .cols = 1, .transposed = 0, .data = dif_data };
	matrix_t enu = { .rows = 3, .cols = 1, .transposed = 0, .data = enu_data };
	vec_t pointEcef;

	/* reference point coordinates trigonometric values */
	sinLatRef = sin(latRef * DEG2RAD);
	sinLonRef = sin(lonRef * DEG2RAD);
	cosLatRef = cos(latRef * DEG2RAD);
	cosLonRef = cos(lonRef * DEG2RAD);

	/* rot matrix fill */
	rot.data[0] = -sinLonRef;
	rot.data[1] = cosLonRef;
	rot.data[2] = 0;
	rot.data[3] = -sinLatRef * cosLonRef;
	rot.data[4] = -sinLatRef * sinLonRef;
	rot.data[5] = cosLatRef;
	rot.data[6] = cosLatRef * cosLonRef;
	rot.data[7] = cosLatRef * sinLonRef;
	rot.data[8] = sinLatRef;

	/* diff matrix fill */
	pointEcef = geo2ecef(lat, lon, h);
	dif.data[0] = pointEcef.x - refEcef->x;
	dif.data[1] = pointEcef.y - refEcef->y;
	dif.data[2] = pointEcef.z - refEcef->z;

	/* perform ECEF to ENU by calculating matrix product (rot * dif) */
	matrix_prod(&rot, &dif, &enu);

	return (vec_t) { .x = enu.data[0], .y = enu.data[1], .z = enu.data[2] };
}

void meas_gpsCalib(void)
{
	int i, avg = 10;
	sensor_event_t gpsEvt;
	float refLat, refLon, refHeight;

	/* Assuring gps fix */
	while (1) {
		sensc_gpsGet(&gpsEvt);
		if (gpsEvt.gps.lat != 0 && gpsEvt.gps.lon != 0 && gpsEvt.gps.groundSpeed > 0) {
			break;
		}
		printf("Awaiting GPS fix...\n");
		sleep(4);
	}

	/* Assuring gps fix */
	while (1) {
		sensc_gpsGet(&gpsEvt);
		if (gpsEvt.gps.hdop < 20) {
			break;
		}
		printf("Awaiting good quality GPS (current hdop = %d)\n", gpsEvt.gps.hdop);
		sleep(4);
	}

	refLat = refLon = refHeight = 0;
	i = 0;
	while (i < avg) {
		if (sensc_gpsGet(&gpsEvt) < 0){
			sleep(1);
			continue;
		}
		printf("Sampling gps position: sample %d/%d\n", i+1, avg);
		refLat += (float)gpsEvt.gps.lat / 1e7;
		refLon += (float)gpsEvt.gps.lon / 1e7;
		refHeight += gpsEvt.gps.alt / 1e3;
		i++;
	}
	refLat /= avg;
	refLon /= avg;

	/* Calculating geodetic reference point */
	meas_common.calib.gpsRefGeodetic.lat = refLat;
	meas_common.calib.gpsRefGeodetic.lon = refLon;
	meas_common.calib.gpsRefGeodetic.h = refHeight;

	meas_common.calib.gpsRefEcef = geo2ecef(refLat, refLon, refHeight);

	printf("Acquired GPS position of (lat/lon/h): %f/%f/%f\n", meas_common.calib.gpsRefEcef.x, meas_common.calib.gpsRefEcef.y, meas_common.calib.gpsRefEcef.z);

}

static void meas_ellipCompensate(vec_t *v, float *calib)
{
	float tx, ty, tz;

	tx = v->x - calib[0];
	ty = v->y - calib[1];
	tz = v->z - calib[2];
	v->x = tx * calib[3] + ty * calib[4] + tz * calib[5];
	v->y = tx * calib[6] + ty * calib[7] + tz * calib[8];
	v->z = tx * calib[9] + ty * calib[10] + tz * calib[11];
}


static void meas_acc2si(sensor_event_t *evt, vec_t *vec)
{
	vec->x = evt->accels.accelX / 1000.F;
	vec->y = evt->accels.accelY / 1000.F;
	vec->z = evt->accels.accelZ / 1000.F;
}


static void meas_gyr2si(sensor_event_t *evt, vec_t *vec)
{
	vec->x = evt->gyro.gyroX / 1000.F;
	vec->y = evt->gyro.gyroY / 1000.F;
	vec->z = evt->gyro.gyroZ / 1000.F;
}


static void meas_mag2si(sensor_event_t *evt, vec_t *vec)
{
	vec->x = evt->mag.magX;
	vec->y = evt->mag.magY;
	vec->z = evt->mag.magZ;
}


void meas_imuCalib(void)
{
	static const vec_t nedG = { .x = 0, .y = 0, .z = -1 }; /* earth acceleration versor in NED frame of reference */
	static const vec_t nedY = { .x = 0, .y = 1, .z = 0 };  /* earth y versor (east) in NED frame of reference */

	int i, avg = IMU_CALIB_AVG;
	vec_t acc, gyr, mag, accAvg, gyrAvg, magAvg, bodyY;
	quat_t idenQuat;
	sensor_event_t accEvt, gyrEvt, magEvt;

	quat_idenWrite(&idenQuat);
	accAvg = gyrAvg = magAvg = (vec_t) { .x = 0, .y = 0, .z = 0 };

	printf("IMU calibration...\n");

	i = 0;
	while (i < avg) {
		if (sensc_imuGet(&accEvt, &gyrEvt, &magEvt) >= 0) {
			meas_acc2si(&accEvt, &acc);
			meas_gyr2si(&gyrEvt, &gyr);
			meas_mag2si(&magEvt, &mag);

			meas_ellipCompensate(&acc, acc_calib1);

			vec_add(&accAvg, &acc);
			vec_add(&gyrAvg, &gyr);
			vec_add(&magAvg, &mag);
			usleep(1000 * 5);
			i++;
		}
		else {
			usleep(1000 * 1);
		}
	}
	vec_times(&accAvg, 1. / (float)avg);
	vec_times(&gyrAvg, 1. / (float)avg);
	vec_times(&magAvg, 1. / (float)avg);

	meas_common.calib.gyr_nivel = gyrAvg; /* save gyro drift parameters */
	meas_common.calib.init_m = magAvg;    /* save initial magnetometer reading */

	/* calculate initial rotation */
	vec_normalize(&accAvg);
	vec_normalize(&magAvg);
	vec_cross(&magAvg, &accAvg, &bodyY);
	quat_frameRot(&accAvg, &bodyY, &nedG, &nedY, &meas_common.calib.init_q, &idenQuat);
}

void meas_baroCalib(void)
{
	int i, avg = BARO_CALIB_AVG;
	uint64_t press = 0, temp = 0;
	sensor_event_t baroEvt;

	printf("Barometer calibration...\n");

	i = 0;
	while (i < avg) {
		if (sensc_baroGet(&baroEvt) >= 0) {
			press += baroEvt.baro.pressure;
			temp += baroEvt.baro.temp;
			i++;
			usleep(1000 * 20);
		}
		else {
			usleep(1000 * 10);
		}
	}

	meas_common.calib.base_pressure = (float)press / avg;
	meas_common.calib.base_temp = (float)temp / avg;
}

int meas_imuGet(vec_t *accels, vec_t *gyros, vec_t *mags, uint64_t *timestamp)
{
	sensor_event_t accEvt, gyrEvt, magEvt;

	if (sensc_imuGet(&accEvt, &gyrEvt, &magEvt) < 0) {
		return -1;
	}

	/* these timestamps do not need to be very accurate */
	*timestamp = (accEvt.timestamp + gyrEvt.timestamp + magEvt.timestamp) / 3;

	meas_acc2si(&accEvt, accels); /* accelerations from mm/s^2 -> m/s^2 */
	meas_gyr2si(&gyrEvt, gyros);  /* angulars from mrad/s -> rad/s */
	meas_mag2si(&magEvt, mags);   /* only magnitude matters from geomagnetism */

	meas_ellipCompensate(accels, acc_calib1);

	/* gyro niveling */
	vec_sub(gyros, &meas_common.calib.gyr_nivel);

	return 0;
}


int meas_baroGet(float *pressure, float *temperature, uint64_t *timestamp)
{
	sensor_event_t baro_evt;

	if (sensc_baroGet(&baro_evt) < 0) {
		return -1;
	}

	*timestamp = baro_evt.timestamp;
	*temperature = baro_evt.baro.temp;
	*pressure = baro_evt.baro.pressure;

	return 0;
}


int meas_gpsGet(vec_t *enu, vec_t *enu_speed, float *hdop)
{
	sensor_event_t gps_evt;

	if (sensc_gpsGet(&gps_evt) < 0) {
		return -1;
	}

	*enu = geo2enu(
		(float)gps_evt.gps.lat / 1e7,
		(float)gps_evt.gps.lon / 1e7,
		0,
		meas_common.calib.gpsRefGeodetic.lat,
		meas_common.calib.gpsRefGeodetic.lon,
		&meas_common.calib.gpsRefEcef);

	enu_speed->x = (float)gps_evt.gps.velEast / 1e3;
	enu_speed->y = (float)gps_evt.gps.velNorth / 1e3;
	enu_speed->z = 0;

	*hdop = (float)(gps_evt.gps.hdop) / 100;

	return 0;
}


const kalman_calib_t *meas_calibGet(void)
{
	return &meas_common.calib;
}


float meas_calibPressGet(void)
{
	return meas_common.calib.base_pressure;
}
