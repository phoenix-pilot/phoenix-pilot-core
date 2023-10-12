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
#include "logs/writer.h"
#include "logs/reader.h"

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
#define GPS_CALIB_AVG  10

#define MAX_CONSECUTIVE_FAILS 10 /* Max amount of fails during sensor data acquisition before returning an error */

#define MAX_U32_DELTAANGLE     0x7fffffff /* Half of the u32 buffer span is max delta angle expected in one step (roughly 2147 radians) */
#define GYRO_MAX_SENSIBLE_READ 157        /* 50 pi radians per second is the largest absolute value of angular speed deemed possible */

static struct {
	meas_sourceType_t sourceType;

	int (*baroAcq)(sensor_event_t *);
	int (*gpsAcq)(sensor_event_t *);
	int (*imuAcq)(sensor_event_t *, sensor_event_t *, sensor_event_t *);
	int (*timeAcq)(time_t *);

	meas_calib_t calib;

	struct {
		/* IMU related */
		vec_t accelRaw;
		vec_t accelFltr;
		vec_t gyroRaw;
		vec_t gyroFltr;
		time_t timeImu;

		/* Magnetometer */
		vec_t mag;
		time_t timeMag;

		/* Baro related */
		float pressure;
		float temp;
		time_t timeBaro;

		/* Gps related */
		meas_gps_t gps;
		time_t timeGps;
	} data;
} meas_common;


int meas_init(meas_sourceType_t sourceType, const char *path, int senscInitFlags)
{
	meas_common.sourceType = sourceType;

	if (access(path, R_OK) != 0) {
		fprintf(stderr, "meas: program have no read access to file %s\n", path);
		return -1;
	}

	switch (sourceType) {
		case srcSens:
			meas_common.imuAcq = sensc_imuGet;
			meas_common.gpsAcq = sensc_gpsGet;
			meas_common.timeAcq = sensc_timeGet;
			meas_common.baroAcq = sensc_baroGet;

			return sensc_init(path, CORR_ENBL_ALL, senscInitFlags);

		case srcLog:
			meas_common.imuAcq = ekflog_imuRead;
			meas_common.gpsAcq = ekflog_gpsRead;
			meas_common.timeAcq = ekflog_timeRead;
			meas_common.baroAcq = ekflog_baroRead;

			return ekflog_readerInit(path);

		default:
			fprintf(stderr, "%s: unknown source type\n", __FUNCTION__);

			return -1;
	}
}


int meas_done(void)
{
	switch (meas_common.sourceType) {
		case srcSens:
			sensc_deinit();
			return 0;

		case srcLog:
			return ekflog_readerDone();

		default:
			fprintf(stderr, "%s: unknown source type\n", __FUNCTION__);
			return -1;
	}
}


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


static void meas_geo2ecef(const meas_geodetic_t *geo, double ecef[3])
{
	double N = EARTH_SEMI_MAJOR / sqrt(1 - EARTH_ECCENTRICITY_SQUARED * geo->sinLat);

	ecef[0] = (N + geo->h) * geo->cosLat * geo->cosLon;
	ecef[1] = (N + geo->h) * geo->cosLat * geo->sinLon;
	ecef[2] = ((1 - EARTH_ECCENTRICITY_SQUARED) * N + geo->h) * geo->sinLat;
}


/* convert gps geodetic coordinates `geo` into `ned` (north/east/down) vector with help of `refGeo` and `refEcef` coordinates */
static void meas_geo2ned(const meas_geodetic_t *geo, const meas_geodetic_t *refGeo, const double refEcef[3], vec_t *ned)
{
	double rot[9], dif[3], enu[3], pointEcef[3];
	int i;

	/* rot matrix fill */
	rot[0] = -refGeo->sinLon;
	rot[1] = refGeo->cosLon;
	rot[2] = 0;
	rot[3] = -refGeo->sinLat * refGeo->cosLon;
	rot[4] = -refGeo->sinLat * refGeo->sinLon;
	rot[5] = refGeo->cosLat;
	rot[6] = refGeo->cosLat * refGeo->cosLon;
	rot[7] = refGeo->cosLat * refGeo->sinLon;
	rot[8] = refGeo->sinLat;

	/* diff matrix fill */
	meas_geo2ecef(geo, pointEcef);
	dif[0] = pointEcef[0] - refEcef[0];
	dif[1] = pointEcef[1] - refEcef[1];
	dif[2] = pointEcef[2] - refEcef[2];

	/* multiplying rot[3x3] * dif [3x1] into enu[3x1] */
	for (i = 0; i < 3; i++) {
		enu[i] = rot[3 * i + 0] * dif[0] + rot[3 * i + 1] * dif[1] + rot[3 * i + 2] * dif[2];
	}

	/* convert ENU -> NED coordinates by switching the elements */
	ned->x = enu[1];
	ned->y = enu[0];
	ned->z = -enu[2];
}


int meas_gpsCalib(void)
{
	int i, avg = GPS_CALIB_AVG, fails = 0;
	sensor_event_t gpsEvt;
	meas_geodetic_t refPos = { 0 };

	/* Assuring gps fix */
	while (1) {
		if (meas_common.gpsAcq(&gpsEvt) != 0) {
			return -1;
		}
		ekflog_gpsWrite(&gpsEvt);
		if (gpsEvt.gps.fix > 0) {
			break;
		}
		printf("Awaiting GPS fix...\n");
		sleep(4);
	}

	/* Assuring gps fix */
	while (1) {
		if (meas_common.gpsAcq(&gpsEvt) != 0) {
			return -1;
		}
		ekflog_gpsWrite(&gpsEvt);
		if (gpsEvt.gps.hdop < 500) {
			break;
		}
		printf("Awaiting good quality GPS (current hdop = %d)\n", gpsEvt.gps.hdop);
		sleep(4);
	}

	i = 0;
	while (i < avg) {
		if (meas_common.gpsAcq(&gpsEvt) < 0) {
			if (++fails > MAX_CONSECUTIVE_FAILS) {
				return -1;
			}

			sleep(1);
			continue;
		}
		ekflog_gpsWrite(&gpsEvt);
		printf("Sampling gps position: sample %d/%d\n", i + 1, avg);
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

	meas_geo2ecef(&meas_common.calib.gps.refGeodetic, meas_common.calib.gps.refEcef);

	printf("Acquired GPS position of (lat/lon/h): %f/%f/%f\n", meas_common.calib.gps.refEcef[0], meas_common.calib.gps.refEcef[1], meas_common.calib.gps.refEcef[2]);

	return 0;
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
	vec->x = evt->mag.magX; /* 10^-7 T is 1 milligauss, leaving as is */
	vec->y = evt->mag.magY; /* 10^-7 T is 1 milligauss, leaving as is */
	vec->z = evt->mag.magZ; /* 10^-7 T is 1 milligauss, leaving as is */
}


int meas_imuCalib(void)
{
	static const vec_t nedG = { .x = 0, .y = 0, .z = -1 }; /* earth acceleration versor in NED frame of reference */
	static const vec_t nedY = { .x = 0, .y = 1, .z = 0 };  /* earth y versor (east) in NED frame of reference */

	int i, avg = IMU_CALIB_AVG, fails = 0;
	vec_t acc, gyr, mag, accAvg, gyrAvg, magAvg, bodyY;
	quat_t idenQuat;
	sensor_event_t accEvt, gyrEvt, magEvt;

	quat_idenWrite(&idenQuat);
	accAvg = gyrAvg = magAvg = (vec_t) { .x = 0, .y = 0, .z = 0 };

	printf("IMU calibration...\n");

	i = 0;
	while (i < avg) {
		if (meas_common.imuAcq(&accEvt, &gyrEvt, &magEvt) >= 0) {
			ekflog_imuWrite(&accEvt, &gyrEvt, &magEvt);
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
			if (++fails > MAX_CONSECUTIVE_FAILS) {
				return -1;
			}

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

	return 0;
}

int meas_baroCalib(void)
{
	int i, avg = BARO_CALIB_AVG, fails = 0;
	uint64_t press = 0, temp = 0;
	sensor_event_t baroEvt;

	printf("Barometer calibration...\n");

	i = 0;
	while (i < avg) {
		if (meas_common.baroAcq(&baroEvt) >= 0) {
			ekflog_baroWrite(&baroEvt);
			press += baroEvt.baro.pressure;
			temp += baroEvt.baro.temp;
			i++;
			usleep(1000 * 20);
		}
		else {
			if (++fails > MAX_CONSECUTIVE_FAILS) {
				return -1;
			}

			usleep(1000 * 10);
		}
	}

	meas_common.calib.baro.basePress = (float)press / avg;
	meas_common.calib.baro.baseTemp = (float)temp / avg;

	return 0;
}


int meas_imuPoll(time_t *timestamp)
{
	static sensor_event_t gyrEvtOld = { 0 };

	sensor_event_t accEvt, gyrEvt, magEvt;

	if (meas_common.imuAcq(&accEvt, &gyrEvt, &magEvt) < 0) {
		return EOF;
	}

	if (timestamp != NULL) {
		*timestamp = gyrEvt.timestamp;
	}

	/* these timestamps do not need to be very accurate */
	meas_common.data.timeImu = gyrEvt.timestamp;

	ekflog_imuWrite(&accEvt, &gyrEvt, &magEvt);

	meas_acc2si(&accEvt, &meas_common.data.accelRaw); /* accelerations from mm/s^2 -> m/s^2 */
	meas_mag2si(&magEvt, &meas_common.data.mag);      /* only magnitude matters from geomagnetism */

	/* If sensorhub integral values produce wrongful data (too long/short timestep) use direct gyro output */
	if (meas_dAngle2si(&gyrEvt, &gyrEvtOld, &meas_common.data.gyroRaw) != 0) {
		meas_gyr2si(&gyrEvt, &meas_common.data.gyroRaw);
	}

	/* gyro niveling */
	vec_sub(&meas_common.data.gyroRaw, &meas_common.calib.imu.gyroBias);

	meas_common.data.accelFltr = meas_common.data.accelRaw;
	meas_common.data.gyroFltr = meas_common.data.gyroRaw;
	fltr_accLpf(&meas_common.data.accelFltr);
	fltr_gyroLpf(&meas_common.data.gyroFltr);

	return 0;
}


int meas_baroPoll(void)
{
	sensor_event_t baroEvt;

	if (meas_common.baroAcq(&baroEvt) < 0) {
		return EOF;
	}

	ekflog_baroWrite(&baroEvt);

	meas_common.data.timeBaro = baroEvt.timestamp;
	meas_common.data.temp = baroEvt.baro.temp;
	meas_common.data.pressure = baroEvt.baro.pressure;

	return 0;
}


int meas_gpsPoll(void)
{
	sensor_event_t gpsEvt;
	meas_geodetic_t geo;

	if (meas_common.gpsAcq(&gpsEvt) < 0) {
		return EOF;
	}

	ekflog_gpsWrite(&gpsEvt);

	/* save timestamp */
	meas_common.data.timeGps = gpsEvt.timestamp;

	/* Transformation from sensor data -> geodetic -> ned data */
	meas_gps2geo(&gpsEvt, &geo);
	meas_geo2ned(&geo, &meas_common.calib.gps.refGeodetic, meas_common.calib.gps.refEcef, &meas_common.data.gps.pos);

	meas_common.data.gps.lat = geo.lat;
	meas_common.data.gps.lon = geo.lon;
	meas_common.data.gps.eph = (float)(gpsEvt.gps.eph) / 1000.f;
	meas_common.data.gps.epv = (float)(gpsEvt.gps.evel) / 1000.f;
	meas_common.data.gps.fix = gpsEvt.gps.fix;
	meas_common.data.gps.satsNb = gpsEvt.gps.satsNb;
	meas_common.data.gps.vel.x = (float)gpsEvt.gps.velNorth / 1e3;
	meas_common.data.gps.vel.y = (float)gpsEvt.gps.velEast / 1e3;
	meas_common.data.gps.vel.z = -(float)gpsEvt.gps.velDown / 1e3;

	return 0;
}


int meas_accelGet(vec_t *accels, vec_t *accelsRaw)
{
	*accels = meas_common.data.accelFltr;
	*accelsRaw = meas_common.data.accelRaw;

	return 0;
}


int meas_gyroGet(vec_t *gyro, vec_t *gyroRaw)
{
	*gyro = meas_common.data.gyroFltr;
	*gyroRaw = meas_common.data.gyroRaw;

	return 0;
}


int meas_magGet(vec_t *mag)
{
	*mag = meas_common.data.mag;

	return 0;
}


int meas_baroGet(float *pressure, float *temperature, uint64_t *timestamp)
{
	*pressure = meas_common.data.pressure;
	*temperature = meas_common.data.temp;
	*timestamp = meas_common.data.timeBaro;

	return 0;
}


int meas_timeGet(time_t *useconds)
{
	if (meas_common.timeAcq(useconds) != 0) {
		return -1;
	}

	ekflog_timeWrite(*useconds);

	return 0;
}


int meas_gpsGet(meas_gps_t *gpsData, time_t *timestamp)
{
	*gpsData = meas_common.data.gps;
	*timestamp = meas_common.data.timeGps;

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
