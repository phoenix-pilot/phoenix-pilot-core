/*
 * Phoenix-Pilot
 *
 * Mock sensorhub client functions
 *
 * Copyright 2023 Phoenix Systems
 * Author: Piotr Nieciecki
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#include "sensc.h"

#include <gettime.h>


int sensc_init(const char *path, bool corrEnable)
{
	return 0;
}


void sensc_deinit(void)
{
	return;
}


static time_t sensc_getTimestamp(void)
{
	time_t res;
	gettime(&res, NULL);

	return res;
}


int sensc_imuGet(sensor_event_t *accelEvt, sensor_event_t *gyroEvt, sensor_event_t *magEvt)
{
	accelEvt->timestamp = sensc_getTimestamp();
	accelEvt->accels.accelX = 0;
	accelEvt->accels.accelY = 0;
	accelEvt->accels.accelZ = -9806;
	accelEvt->type = SENSOR_TYPE_ACCEL;

	gyroEvt->timestamp = sensc_getTimestamp();
	gyroEvt->gyro.gyroX = 0;
	gyroEvt->gyro.gyroY = 0;
	gyroEvt->gyro.gyroZ = 0;
	gyroEvt->gyro.dAngleX = 0;
	gyroEvt->gyro.dAngleY = 0;
	gyroEvt->gyro.dAngleZ = 0;
	gyroEvt->type = SENSOR_TYPE_GYRO;

	magEvt->timestamp = sensc_getTimestamp();
	magEvt->mag.magX = 1;
	magEvt->mag.magY = 0;
	magEvt->mag.magZ = 0;
	magEvt->type = SENSOR_TYPE_MAG;

	return 0;
}


int sensc_baroGet(sensor_event_t *baroEvt)
{
	baroEvt->timestamp = sensc_getTimestamp();
	baroEvt->baro.pressure = 101325;
	baroEvt->baro.temp = 273 + 25;
	baroEvt->type = SENSOR_TYPE_BARO;

	return 0;
}


int sensc_gpsGet(sensor_event_t *gpsEvt)
{
	gpsEvt->timestamp = sensc_getTimestamp();
	gpsEvt->gps.alt = 0;
	gpsEvt->gps.lon = 0;
	gpsEvt->gps.lat = 0;
	gpsEvt->gps.hdop = 1;
	gpsEvt->gps.vdop = 1;
	gpsEvt->gps.altEllipsoid = 0;
	gpsEvt->gps.groundSpeed = 0;
	gpsEvt->gps.velNorth = 0;
	gpsEvt->gps.velEast = 0;
	gpsEvt->gps.velDown = 0;
	gpsEvt->gps.eph = 5;
	gpsEvt->gps.epv = 5;
	gpsEvt->gps.evel = 5;
	gpsEvt->gps.heading = 0;
	gpsEvt->gps.headingOffs = 0;
	gpsEvt->gps.headingAccur = 5;
	gpsEvt->gps.satsNb = 10;
	gpsEvt->gps.fix = 5;
	gpsEvt->type = SENSOR_TYPE_GPS;

	return 0;
}
