/*
 * Phoenix-Pilot
 *
 * Tools for unit tests of ekf logging module
 *
 * Copyright 2023 Phoenix Systems
 * Author: Piotr Nieciecki
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */


#ifndef _EKF_LOG_TESTS_TOOLS_
#define _EKF_LOG_TESTS_TOOLS_


#include <libsensors.h>

#include <stdio.h>
#include <stdbool.h>
#include <string.h>


bool ekflogTests_accEvtsEqual(const sensor_event_t *expected, const sensor_event_t *actual)
{
	return (expected->type == actual->type &&
		expected->timestamp == actual->timestamp &&
		expected->accels.accelX == actual->accels.accelX &&
		expected->accels.accelY == actual->accels.accelY &&
		expected->accels.accelZ == actual->accels.accelZ);
}


bool ekflogTest_gyrEvtsEqual(const sensor_event_t *expected, const sensor_event_t *actual)
{
	return (expected->type == actual->type &&
		expected->timestamp == actual->timestamp &&
		expected->gyro.gyroX == actual->gyro.gyroX &&
		expected->gyro.gyroY == actual->gyro.gyroY &&
		expected->gyro.gyroZ == actual->gyro.gyroZ &&
		expected->gyro.dAngleX == actual->gyro.dAngleX &&
		expected->gyro.dAngleY == actual->gyro.dAngleY &&
		expected->gyro.dAngleZ == actual->gyro.dAngleZ);
}


bool ekflogTest_magEvtsEqual(const sensor_event_t *expected, const sensor_event_t *actual)
{
	return (expected->type == actual->type &&
		expected->timestamp == actual->timestamp &&
		expected->mag.magX == actual->mag.magX &&
		expected->mag.magY == actual->mag.magY &&
		expected->mag.magZ == actual->mag.magZ);
}


bool ekflogTest_gpsEvtsEqual(const sensor_event_t *expected, const sensor_event_t *actual)
{
	return (expected->type == actual->type &&
		expected->timestamp == actual->timestamp &&
		expected->gps.lat == actual->gps.lat &&
		expected->gps.lon == actual->gps.lon &&
		expected->gps.alt == actual->gps.alt &&
		expected->gps.utc == actual->gps.utc &&
		expected->gps.hdop == actual->gps.hdop &&
		expected->gps.vdop == actual->gps.vdop &&
		expected->gps.altEllipsoid == actual->gps.altEllipsoid &&
		expected->gps.groundSpeed == actual->gps.groundSpeed &&
		expected->gps.velNorth == actual->gps.velNorth &&
		expected->gps.velEast == actual->gps.velEast &&
		expected->gps.velDown == actual->gps.velDown &&
		expected->gps.eph == actual->gps.eph &&
		expected->gps.epv == actual->gps.epv &&
		expected->gps.evel == actual->gps.evel &&
		expected->gps.heading == actual->gps.heading &&
		expected->gps.headingOffs == actual->gps.headingOffs &&
		expected->gps.headingAccur == actual->gps.headingAccur &&
		expected->gps.satsNb == actual->gps.satsNb &&
		expected->gps.fix == actual->gps.fix);
}


bool ekflogTest_baroEvtsEqual(const sensor_event_t *expected, const sensor_event_t *actual)
{
	return (expected->type == actual->type &&
		expected->timestamp == actual->timestamp &&
		expected->baro.pressure == actual->baro.pressure &&
		expected->baro.temp == actual->baro.temp);
}


bool ekflogsTests_sensorEvtEqual(const sensor_event_t *expected, const sensor_event_t *actual)
{
	switch (expected->type) {
		case SENSOR_TYPE_ACCEL:
			return ekflogTest_magEvtsEqual(expected, actual);

		case SENSOR_TYPE_GYRO:
			return ekflogTest_gyrEvtsEqual(expected, actual);

		case SENSOR_TYPE_MAG:
			return ekflogTest_magEvtsEqual(expected, actual);

		case SENSOR_TYPE_GPS:
			return ekflogTest_gpsEvtsEqual(expected, actual);

		case SENSOR_TYPE_BARO:
			return ekflogTest_baroEvtsEqual(expected, actual);

		default:
			fprintf(stderr, "Ekf logs tests: unknown sensor type\n");
			return false;
	}
}


void ekflogTests_sensorEvtClear(sensor_event_t *evt)
{
	memset(evt, 0, sizeof(sensor_event_t));
}


#endif
