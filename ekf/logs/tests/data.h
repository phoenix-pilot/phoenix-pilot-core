/*
 * Phoenix-Pilot
 *
 * Data for unit tests of ekf logging module
 *
 * Copyright 2023 Phoenix Systems
 * Author: Piotr Nieciecki
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#ifndef _EKF_LOG_TESTS_DATA_
#define _EKF_LOG_TESTS_DATA_


#include <libsensors.h>


static const time_t testTimestamp1 = 123456789;
static const time_t testTimestamp2 = 987654321;


/*
 * Because of the logs format and method of test checking timestamp in
 * accelerometer, magnetometer nad magnetometer events must be equal.
 */

static const sensor_event_t testAccEvt1 = {
	.type = SENSOR_TYPE_ACCEL,
	.timestamp = 1,

	.accels.devId = 1,

	.accels.accelX = 2,
	.accels.accelY = 3,
	.accels.accelZ = 4,

	.accels.temp = 5,
};

static const sensor_event_t testGyrEvt1 = {
	.type = SENSOR_TYPE_GYRO,
	.timestamp = 1,

	.gyro.devId = 4,

	.gyro.gyroX = 5,
	.gyro.gyroY = 6,
	.gyro.gyroZ = 7,

	.gyro.dAngleX = 8,
	.gyro.dAngleY = 9,
	.gyro.dAngleZ = 10,

	.gyro.temp = 11,
};

static const sensor_event_t testMagEvt1 = {
	.type = SENSOR_TYPE_MAG,
	.timestamp = 1,

	.mag.devId = 10,

	.mag.magX = 11,
	.mag.magY = 12,
	.mag.magZ = 13,
};


static const sensor_event_t testAccEvt2 = {
	.type = SENSOR_TYPE_ACCEL,
	.timestamp = 14,

	.accels.devId = 16,

	.accels.accelX = 15,
	.accels.accelY = 16,
	.accels.accelZ = 17,

	.accels.temp = 18,
};

static const sensor_event_t testGyrEvt2 = {
	.type = SENSOR_TYPE_GYRO,
	.timestamp = 14,

	.gyro.devId = 17,

	.gyro.gyroX = 18,
	.gyro.gyroY = 19,
	.gyro.gyroZ = 20,

	.gyro.dAngleX = 21,
	.gyro.dAngleY = 22,
	.gyro.dAngleZ = 23,

	.gyro.temp = 24,
};

static const sensor_event_t testMagEvt2 = {
	.type = SENSOR_TYPE_MAG,
	.timestamp = 14,

	.mag.devId = 23,

	.mag.magX = 24,
	.mag.magY = 25,
	.mag.magZ = 26,
};


static const sensor_event_t testGpsEvt1 = {
	.type = SENSOR_TYPE_GPS,
	.timestamp = 26,

	.gps.devId = 27,

	.gps.lat = 28,
	.gps.lon = 29,
	.gps.alt = 30,

	.gps.utc = 31,

	.gps.hdop = 32,
	.gps.vdop = 33,

	.gps.altEllipsoid = 34,

	.gps.groundSpeed = 35,

	.gps.velNorth = 36,
	.gps.velEast = 37,
	.gps.velDown = 38,

	.gps.eph = 39,
	.gps.epv = 40,
	.gps.evel = 41,

	.gps.heading = 42,
	.gps.headingOffs = 43,
	.gps.headingAccur = 44,

	.gps.satsNb = 45,
	.gps.fix = 46,
};


static const sensor_event_t testGpsEvt2 = {
	.type = SENSOR_TYPE_GPS,
	.timestamp = 46,

	.gps.devId = 47,

	.gps.alt = 28,
	.gps.lat = 29,
	.gps.lon = 30,

	.gps.utc = 51,

	.gps.hdop = 52,
	.gps.vdop = 53,

	.gps.altEllipsoid = 54,

	.gps.groundSpeed = 55,

	.gps.velNorth = 56,
	.gps.velEast = 57,
	.gps.velDown = 58,

	.gps.eph = 59,
	.gps.epv = 60,
	.gps.evel = 61,

	.gps.heading = 62,
	.gps.headingOffs = 63,
	.gps.headingAccur = 64,

	.gps.satsNb = 65,
	.gps.fix = 66,
};


static const sensor_event_t testBaroEvt = {
	.type = SENSOR_TYPE_BARO,
	.timestamp = 66,

	.baro.devId = 67,

	.baro.pressure = 68,
	.baro.temp = 69,
};


#endif
