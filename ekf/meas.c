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
#include "filters.h"
#include "log.h"

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

#define MAX_U32_DELTAANGLE 0x7fffffff /* Half of the u32 buffer span is max delta angle expected in one step (roughly 2147 radians) */
#define GYRO_MAX_SENSIBLE_READ  157   /* 50 pi radians per second is the largest absolute value of angular speed deemed possible */

static struct {
	meas_calib_t calib;
} meas_common;


static void meas_gps2geo(const sensor_event_t *gpsEvt, meas_geodetic_t *geo)
{
	geo->lat = (double)gpsEvt->gps.lat / 1e9;
	geo->lon = (double)gpsEvt->gps.lon / 1e9;
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


/* convert gps geodetic coordinates `geo` into `ned` (north/east/down) vector with help of `refGeo` and `refEcef` coordinates */
static void meas_geo2ned(const meas_geodetic_t *geo, const meas_geodetic_t *refGeo, const vec_t *refEcef, vec_t *ned)
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

	/* convert ENU -> NED coordinates by switching the elements */
	ned->x = enuMatrix.data[1];
	ned->y = enuMatrix.data[0];
	ned->z = -enuMatrix.data[2];
}


void meas_gpsCalib(void)
{
	int i, avg = 10;
	sensor_event_t gpsEvt;
	meas_geodetic_t refPos = { 0 };

	/* Assuring gps fix */
	while (1) {
		sensc_gpsGet(&gpsEvt);
		if (gpsEvt.gps.fix > 0) {
			break;
		}
		printf("Awaiting GPS fix...\n");
		sleep(4);
	}

	/* Assuring gps fix */
	while (1) {
		sensc_gpsGet(&gpsEvt);
		if (gpsEvt.gps.hdop < 500) {
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
		refPos.lat += (double)gpsEvt.gps.lat / 1e9;
		refPos.lon += (double)gpsEvt.gps.lon / 1e9;
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


static int meas_dAngle2si(sensor_event_t *evtNew, sensor_event_t *evtOld, vec_t *vec)
{
	uint32_t dif;
	time_t delta;
	vec_t tmp;

	delta = evtNew->timestamp - evtOld->timestamp;

	if (evtNew->timestamp == 0 || evtOld->timestamp == 0 || delta == 0) {
		return -1;
	}

	/* delta angle overflow/underflow calculations */
	dif = evtNew->gyro.dAngleX - evtOld->gyro.dAngleX;
	tmp.x = (dif < MAX_U32_DELTAANGLE) ? dif : -(float)(uint32_t)(-dif);

	dif = evtNew->gyro.dAngleY - evtOld->gyro.dAngleY;
	tmp.y = (dif < MAX_U32_DELTAANGLE) ? dif : -(float)(uint32_t)(-dif);

	dif = evtNew->gyro.dAngleZ - evtOld->gyro.dAngleZ;
	tmp.z = (dif < MAX_U32_DELTAANGLE) ? dif : -(float)(uint32_t)(-dif);

	/* values initially in [urad]. Divided by time in [us] gives output in [rad/s] */
	tmp.x /= (float)delta;
	tmp.y /= (float)delta;
	tmp.z /= (float)delta;

	/* Abort calculation of angular speed if outcome is above maximum value */
	if (fabs(tmp.x) > GYRO_MAX_SENSIBLE_READ || fabs(tmp.y) > GYRO_MAX_SENSIBLE_READ || fabs(tmp.z) > GYRO_MAX_SENSIBLE_READ) {
		return -1;
	}

	*vec = tmp;

	return 0;
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


int meas_imuGet(vec_t *accels, vec_t *accelsRaw, vec_t *gyros, vec_t *mags, uint64_t *timestamp)
{
	static sensor_event_t gyrEvtOld = { 0 };

	sensor_event_t accEvt, gyrEvt, magEvt;

	if (sensc_imuGet(&accEvt, &gyrEvt, &magEvt) < 0) {
		return -1;
	}

	/* these timestamps do not need to be very accurate */
	*timestamp = gyrEvt.timestamp;

	ekflog_write(EKFLOG_SENSC, "SI %lld %d %d %d %d %d %d\n", *timestamp, accEvt.accels.accelX, accEvt.accels.accelY, accEvt.accels.accelZ, gyrEvt.gyro.gyroX,  gyrEvt.gyro.gyroY,  gyrEvt.gyro.gyroZ);

	meas_acc2si(&accEvt, accelsRaw); /* accelerations from mm/s^2 -> m/s^2 */
	meas_mag2si(&magEvt, mags);   /* only magnitude matters from geomagnetism */

	/* If sensorhub integral values produce wrongful data (too long/short timestep) use direct gyro output */
	if (meas_dAngle2si(&gyrEvt, &gyrEvtOld, gyros) != 0) {
		meas_gyr2si(&gyrEvt, gyros);
	}

	/* gyro niveling */
	vec_sub(gyros, &meas_common.calib.imu.gyroBias);

	*accels = *accelsRaw;
	fltr_accLpf(accels);
	fltr_gyroLpf(gyros);

	ekflog_write(EKFLOG_MEAS, "MI %lld %.3f %.3f %.3f %.3f %.3f %.3f\n", *timestamp, accels->x, accels->y, accels->z, gyros->x, gyros->y, gyros->z);

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


int meas_gpsGet(meas_gps_t *gpsData, time_t *timestamp)
{
	sensor_event_t gpsEvt;
	meas_geodetic_t geo;

	if (sensc_gpsGet(&gpsEvt) < 0) {
		return -1;
	}

	ekflog_write(
		EKFLOG_GPS_MEAS,
		"MG %lld %lld %lld\n", gpsEvt.timestamp, gpsEvt.gps.lat, gpsEvt.gps.lon);

	/* save timestamp */
	*timestamp = gpsEvt.timestamp;

	/* Transformation from sensor data -> geodetic -> ned data */
	meas_gps2geo(&gpsEvt, &geo);
	meas_geo2ned(&geo, &meas_common.calib.gps.refGeodetic, &meas_common.calib.gps.refEcef, &gpsData->pos);

	gpsData->lat = geo.lat;
	gpsData->lon = geo.lon;
	gpsData->eph = (float)(gpsEvt.gps.eph) / 1000.f;
	gpsData->epv = (float)(gpsEvt.gps.evel) / 1000.f;
	gpsData->fix = gpsEvt.gps.fix;
	gpsData->satsNb = gpsEvt.gps.satsNb;
	gpsData->vel.x = (float)gpsEvt.gps.velNorth / 1e3;
	gpsData->vel.y = (float)gpsEvt.gps.velEast / 1e3;
	gpsData->vel.z = -(float)gpsEvt.gps.velDown / 1e3;

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
