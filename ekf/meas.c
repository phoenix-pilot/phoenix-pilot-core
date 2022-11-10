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
#include "meas.h"

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

/* different accelerometer biases sizes */
#define ACC_BIAS_SIZE    3
#define ACC_NONORTHO_SIZE 9

static struct {
	meas_calib_t calib;
} meas_common;

/* accelerometer calibration data */
static const float accCalib[ACC_BIAS_SIZE + ACC_NONORTHO_SIZE] = {
	/* sphere offset in m/s^2 */
	0.237000, 0.120545, 0.022396,
	/* sphere deformation */
	1.00246374e+00,  1.82159288e-04,  8.18160423e-04,
	1.82159288e-04,  9.95663211e-01, -1.46614579e-04,
	8.18160423e-04, -1.46614579e-04,  1.00188801e+00
};


static void meas_gps2geo(const sensor_event_t *gpsEvt, meas_geodetic_t *geo)
{
	geo->lat = (float)gpsEvt->gps.lat / 1e7;
	geo->lon = (float)gpsEvt->gps.lon / 1e7;
	geo->h = (float)gpsEvt->gps.alt / 1e3;

	geo->sinLat = sin(geo->lat * DEG2RAD);
	geo->cosLat = cos(geo->lat * DEG2RAD);

	geo->sinLon = sin(geo->lon * DEG2RAD);
	geo->cosLon = cos(geo->lon * DEG2RAD);
}


static void meas_geo2ecef(const meas_geodetic_t *geo, vec_t *ecef)
{
	float N = EARTH_SEMI_MAJOR / sqrt(1 - EARTH_ECCENTRICITY_SQUARED * geo->sinLat);

	ecef->x = (N + geo->h) * geo->cosLat * geo->cosLon;
	ecef->y = (N + geo->h) * geo->cosLat * geo->sinLon;
	ecef->z = ((1 - EARTH_ECCENTRICITY_SQUARED) * N + geo->h) * geo->sinLat;
}


/* convert gps geodetic coordinates `geo` into `enu` (east/north/up) vector with help of `refGeo` and `refEcef` coordinates */
static void meas_geo2enu(const meas_geodetic_t *geo, const meas_geodetic_t *refGeo, const vec_t *refEcef, vec_t *enu)
{
	float rot_data[9], dif_data[3], enu_data[3];
	matrix_t rot = { .rows = 3, .cols = 3, .transposed = 0, .data = rot_data };
	matrix_t dif = { .rows = 3, .cols = 1, .transposed = 0, .data = dif_data };
	matrix_t enuMatrix = { .rows = 3, .cols = 1, .transposed = 0, .data = enu_data };
	vec_t pointEcef;

	/* rot matrix fill */
	rot.data[0] = -refGeo->sinLon;
	rot.data[1] = refGeo->cosLon;
	rot.data[2] = 0;
	rot.data[3] = -refGeo->sinLat * refGeo->cosLon;
	rot.data[4] = -refGeo->sinLat * refGeo->sinLon;
	rot.data[5] = refGeo->cosLat;
	rot.data[6] = refGeo->cosLat * refGeo->cosLon;
	rot.data[7] = refGeo->cosLat * refGeo->sinLon;
	rot.data[8] = refGeo->sinLat;

	/* diff matrix fill */
	meas_geo2ecef(geo, &pointEcef);
	dif.data[0] = pointEcef.x - refEcef->x;
	dif.data[1] = pointEcef.y - refEcef->y;
	dif.data[2] = pointEcef.z - refEcef->z;

	/* perform ECEF to ENU by calculating matrix product (rot * dif) */
	matrix_prod(&rot, &dif, &enuMatrix);

	enu->x = enuMatrix.data[0];
	enu->y = enuMatrix.data[1];
	enu->z = enuMatrix.data[2];
}


void meas_gpsCalib(void)
{
	int i, avg = 10;
	sensor_event_t gpsEvt;
	meas_geodetic_t refPos = { 0 };

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

	i = 0;
	while (i < avg) {
		if (sensc_gpsGet(&gpsEvt) < 0){
			sleep(1);
			continue;
		}
		printf("Sampling gps position: sample %d/%d\n", i+1, avg);
		refPos.lat += (float)gpsEvt.gps.lat / 1e7;
		refPos.lon += (float)gpsEvt.gps.lon / 1e7;
		refPos.h += gpsEvt.gps.alt / 1e3;
		i++;
	}
	refPos.lat /= avg;
	refPos.lon /= avg;
	refPos.h /= avg;

	/* Calculating geodetic reference point */
	meas_common.calib.gps.refGeodetic = refPos;

	meas_common.calib.gps.refGeodetic.sinLat = sin(meas_common.calib.gps.refGeodetic.lat * DEG2RAD);
	meas_common.calib.gps.refGeodetic.cosLat = cos(meas_common.calib.gps.refGeodetic.lat * DEG2RAD);
	meas_common.calib.gps.refGeodetic.sinLon = sin(meas_common.calib.gps.refGeodetic.lon * DEG2RAD);
	meas_common.calib.gps.refGeodetic.cosLon = cos(meas_common.calib.gps.refGeodetic.lon * DEG2RAD);

	meas_geo2ecef(&meas_common.calib.gps.refGeodetic, &meas_common.calib.gps.refEcef);

	printf("Acquired GPS position of (lat/lon/h): %f/%f/%f\n", meas_common.calib.gps.refEcef.x, meas_common.calib.gps.refEcef.y, meas_common.calib.gps.refEcef.z);

}

static void meas_ellipCompensate(vec_t *v, const float *calib)
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

			meas_ellipCompensate(&acc, accCalib);

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

	meas_common.calib.imu.gyroBias = gyrAvg; /* save gyro drift parameters */
	meas_common.calib.imu.initMag = magAvg;  /* save initial magnetometer reading */

	/* calculate initial rotation */
	vec_normalize(&accAvg);
	vec_normalize(&magAvg);
	vec_cross(&magAvg, &accAvg, &bodyY);
	quat_frameRot(&accAvg, &bodyY, &nedG, &nedY, &meas_common.calib.imu.initQuat, &idenQuat);
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

	meas_common.calib.baro.basePress = (float)press / avg;
	meas_common.calib.baro.baseTemp = (float)temp / avg;
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

	meas_ellipCompensate(accels, accCalib);

	/* gyro niveling */
	vec_sub(gyros, &meas_common.calib.imu.gyroBias);

	return 0;
}


int meas_baroGet(float *pressure, float *temperature, uint64_t *timestamp)
{
	sensor_event_t baroEvt;

	if (sensc_baroGet(&baroEvt) < 0) {
		return -1;
	}

	*timestamp = baroEvt.timestamp;
	*temperature = baroEvt.baro.temp;
	*pressure = baroEvt.baro.pressure;

	return 0;
}


int meas_gpsGet(vec_t *enu, vec_t *enu_speed, float *hdop)
{
	sensor_event_t gpsEvt;
	meas_geodetic_t geo;

	if (sensc_gpsGet(&gpsEvt) < 0) {
		return -1;
	}

	meas_gps2geo(&gpsEvt, &geo);

	meas_geo2enu(&geo, &meas_common.calib.gps.refGeodetic, &meas_common.calib.gps.refEcef, enu);

	/* speed conversion to m/s */
	enu_speed->x = (float)gpsEvt.gps.velEast / 1e3;
	enu_speed->y = (float)gpsEvt.gps.velNorth / 1e3;
	enu_speed->z = -(float)gpsEvt.gps.velDown / 1e3;

	*hdop = (float)(gpsEvt.gps.hdop) / 100;

	return 0;
}


const meas_calib_t *meas_calibGet(void)
{
	return &meas_common.calib;
}


float meas_calibPressGet(void)
{
	return meas_common.calib.baro.basePress;
}
