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


static bool ekflogTests_prefixEqual(const sensor_event_t *expected, const sensor_event_t *actual)
{
	return (expected->type == actual->type && expected->timestamp == actual->timestamp);
}


bool ekflogTests_accEvtsEqual(const sensor_event_t *expected, const sensor_event_t *actual)
{
	if (ekflogTests_prefixEqual(expected, actual) != true) {
		return false;
	}

	return (memcmp(&expected->accels, &actual->accels, sizeof(expected->accels)) == 0) ? true : false;
}


bool ekflogTest_gyrEvtsEqual(const sensor_event_t *expected, const sensor_event_t *actual)
{
	if (ekflogTests_prefixEqual(expected, actual) != true) {
		return false;
	}

	return (memcmp(&expected->gyro, &actual->gyro, sizeof(expected->gyro)) == 0) ? true : false;
}


bool ekflogTest_magEvtsEqual(const sensor_event_t *expected, const sensor_event_t *actual)
{
	if (ekflogTests_prefixEqual(expected, actual) != true) {
		return false;
	}

	return (memcmp(&expected->mag, &actual->mag, sizeof(expected->mag)) == 0) ? true : false;
}


bool ekflogTest_gpsEvtsEqual(const sensor_event_t *expected, const sensor_event_t *actual)
{
	if (ekflogTests_prefixEqual(expected, actual) != true) {
		return false;
	}

	return (memcmp(&expected->gps, &actual->gps, sizeof(expected->gps)) == 0) ? true : false;
}


bool ekflogTest_baroEvtsEqual(const sensor_event_t *expected, const sensor_event_t *actual)
{
	if (ekflogTests_prefixEqual(expected, actual) != true) {
		return false;
	}

	return (memcmp(&expected->mag, &actual->mag, sizeof(expected->mag)) == 0) ? true : false;
}


bool ekflogTests_sensorEvtEqual(const sensor_event_t *expected, const sensor_event_t *actual)
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
