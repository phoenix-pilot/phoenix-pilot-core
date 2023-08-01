/*
 * Phoenix-Pilot
 *
 * Ekf sensor log reader
 *
 * Copyright 2023 Phoenix Systems
 * Author: Piotr Nieciecki
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#include "log_reader.h"

#include <stdio.h>
#include <stdbool.h>

/*
 * Module is splitting log from IMU into three separate sensor events:
 * - accelerometer,
 * - gyroscope,
 * - magnetometer.
 *
 * These events are published during consecutive calls to logReader_nextEntry() function.
 */


/* clang-format off */
typedef enum { notParsing = 0, gyroToPublish, magToPublish } imuParsingState_t;
/* clang-format on */


struct {
	FILE *file;

	bool error;

	imuParsingState_t imuParsingState;
	sensor_event_t accelEvt;
	sensor_event_t gyroEvt;
	sensor_event_t magEvt;
} logReader_common;


static int logReader_readField(void *buff, size_t buffLen)
{
	if (fread(buff, buffLen, 1, logReader_common.file) != 1) {
		return -1;
	}

	return 0;
}


static int logReader_readTimestamp(time_t *result)
{
	uint64_t tmp;

	if (logReader_readField(tmp, sizeof(tmp)) != 0) {
		return -1;
	}

	*result = (time_t)tmp;

	return 0;
}


static int logReader_parseTimestamp(logReader_data_t *result)
{
	result->type = timestampLog;

	return logReader_readTimestamp(&result->data.timestamp);
}


static int logReader_parseAccel(logReader_data_t *result, time_t timestamp)
{
	int err = 0;

	result->type = senscLog;
	result->data.sensc.type = SENSOR_TYPE_ACCEL;
	result->data.sensc.timestamp = timestamp;

	err |= logReader_readField(result->data.sensc.accels.accelX, sizeof(result->data.sensc.accels.accelX));
	err |= logReader_readField(result->data.sensc.accels.accelY, sizeof(result->data.sensc.accels.accelY));
	err |= logReader_readField(result->data.sensc.accels.accelZ, sizeof(result->data.sensc.accels.accelZ));

	if (err != 0) {
		fprintf(stderr, "logReader: Error while parsing acceleration data\n");
		logReader_common.error = true;
	}

	return err;
}


static int logReader_parseGyro(logReader_data_t *result, time_t timestamp)
{
	int err = 0;

	result->type = senscLog;
	result->data.sensc.type = SENSOR_TYPE_GYRO;
	result->data.sensc.timestamp = timestamp;

	err |= logReader_readField(result->data.sensc.gyro.gyroX, sizeof(result->data.sensc.gyro.gyroX));
	err |= logReader_readField(result->data.sensc.gyro.gyroY, sizeof(result->data.sensc.gyro.gyroY));
	err |= logReader_readField(result->data.sensc.gyro.gyroZ, sizeof(result->data.sensc.gyro.gyroZ));

	err |= logReader_readField(result->data.sensc.gyro.dAngleX, sizeof(result->data.sensc.gyro.dAngleX));
	err |= logReader_readField(result->data.sensc.gyro.dAngleY, sizeof(result->data.sensc.gyro.dAngleY));
	err |= logReader_readField(result->data.sensc.gyro.dAngleZ, sizeof(result->data.sensc.gyro.dAngleZ));

	if (err != 0) {
		fprintf(stderr, "logReader: Error while parsing gyroscope data\n");
		logReader_common.error = true;
	}

	return err;
}


static int logReader_parseMag(logReader_data_t *result, time_t timestamp)
{
	int err = 0;

	result->type = senscLog;
	result->data.sensc.type = SENSOR_TYPE_MAG;
	result->data.sensc.timestamp = timestamp;

	err |= logReader_readField(result->data.sensc.mag.magX, sizeof(result->data.sensc.mag.magX));
	err |= logReader_readField(result->data.sensc.mag.magY, sizeof(result->data.sensc.mag.magY));
	err |= logReader_readField(result->data.sensc.mag.magZ, sizeof(result->data.sensc.mag.magZ));

	if (err != 0) {
		fprintf(stderr, "logReader: Error while parsing magnetometer data\n");
		logReader_common.error = true;
	}

	return err;
}


static int logReader_parseImu(void)
{
	time_t timestamp;

	if (logReader_readTimestamp(&timestamp) != 0) {
		return -1;
	}

	if (logReader_parseAccel(&logReader_common.accelEvt, timestamp) != 0) {
		return -1;
	}

	if (logReader_parseGyro(&logReader_common.gyroEvt, timestamp) != 0) {
		return -1;
	}

	if (logReader_parseMag(&logReader_common.magEvt, timestamp) != 0) {
		return -1;
	}

	return 0;
}


static int logReader_parseBaro(logReader_data_t *result)
{
	int err = 0;

	result->type = senscLog;
	result->data.sensc.type = SENSOR_TYPE_BARO;

	err |= logReader_readTimestamp(result->data.timestamp);

	err |= logReader_readField(result->data.sensc.baro.pressure, sizeof(result->data.sensc.baro.pressure));
	err |= logReader_readField(result->data.sensc.baro.temp, sizeof(result->data.sensc.baro.temp));

	if (err != 0) {
		fprintf(stderr, "logReader: Error while parsing barometer data\n");
		logReader_common.error = true;
	}

	return err;
}


static int logReader_parseGPS(logReader_data_t *result)
{
	int err = 0;

	result->type = senscLog;
	result->data.sensc.type = SENSOR_TYPE_GPS;

	err |= logReader_readTimestamp(result->data.timestamp);

	err |= logReader_readField(result->data.sensc.gps.lat, sizeof(result->data.sensc.gps.lat));
	err |= logReader_readField(result->data.sensc.gps.lon, sizeof(result->data.sensc.gps.lon));
	err |= logReader_readField(result->data.sensc.gps.alt, sizeof(result->data.sensc.gps.alt));

	err |= logReader_readField(result->data.sensc.gps.utc, sizeof(result->data.sensc.gps.utc));
	err |= logReader_readField(result->data.sensc.gps.hdop, sizeof(result->data.sensc.gps.hdop));

	err |= logReader_readField(result->data.sensc.gps.altEllipsoid, sizeof(result->data.sensc.gps.altEllipsoid));
	err |= logReader_readField(result->data.sensc.gps.groundSpeed, sizeof(result->data.sensc.gps.groundSpeed));

	err |= logReader_readField(result->data.sensc.gps.velNorth, sizeof(result->data.sensc.gps.velNorth));
	err |= logReader_readField(result->data.sensc.gps.velEast, sizeof(result->data.sensc.gps.velEast));
	err |= logReader_readField(result->data.sensc.gps.velDown, sizeof(result->data.sensc.gps.velDown));

	err |= logReader_readField(result->data.sensc.gps.eph, sizeof(result->data.sensc.gps.eph));
	err |= logReader_readField(result->data.sensc.gps.epv, sizeof(result->data.sensc.gps.epv));
	err |= logReader_readField(result->data.sensc.gps.evel, sizeof(result->data.sensc.gps.evel));

	err |= logReader_readField(result->data.sensc.gps.heading, sizeof(result->data.sensc.gps.heading));
	err |= logReader_readField(result->data.sensc.gps.headingOffs, sizeof(result->data.sensc.gps.headingOffs));
	err |= logReader_readField(result->data.sensc.gps.headingAccur, sizeof(result->data.sensc.gps.headingAccur));

	err |= logReader_readField(result->data.sensc.gps.satsNb, sizeof(result->data.sensc.gps.satsNb));
	err |= logReader_readField(result->data.sensc.gps.fix, sizeof(result->data.sensc.gps.fix));

	if (err != 0) {
		fprintf(stderr, "logReader: Error while parsing GPS data\n");
		logReader_common.error = true;
	}

	return err;
}


int logReader_nextEntry(logReader_data_t *result)
{
	uint32_t logID;
	char logSpecifier;
	int err = 0;

	if (logReader_common.error == true) {
		return -1;
	}

	if (logReader_common.imuParsingState != notParsing) {
		result->type = senscLog;

		switch (logReader_common.imuParsingState) {
			case gyroToPublish:
				result->data.sensc = logReader_common.gyroEvt;
				logReader_common.imuParsingState = magToPublish;

				return 0;
			case magToPublish:
				result->data.sensc = logReader_common.magEvt;
				logReader_common.imuParsingState = notParsing;

				return 0;

			default:
				printf(stderr, "LogReader: unknown IMU parsing state\n");
				return -1;
		}
	}

	logReader_readField(logID, sizeof(logID));
	logReader_readField(logSpecifier, sizeof(logSpecifier));

	switch (logSpecifier) {
		case 'T':
			err = logReader_parseTimestamp(result);
			break;
		case 'P':
			err = logReader_parseGPS(result);
			break;
		case 'B':
			err = logReader_parseBaro(result);
			break;

		case 'I':
			if (logReader_parseImu() != 0) {
				return -1;
			}

			result->type = senscLog;
			result->data.sensc = logReader_common.accelEvt;

			logReader_common.imuParsingState = gyroToPublish;
			err = 0;
			break;

		default:
			fprintf(stderr, "logReader: invalid input file\n");
			return -1;
	}

	if (err != 0) {
		fprintf(stderr, "Error while paring an entry\n");
		logReader_common.error = true;
	}

	return err;
}


int logReader_done()
{
	return fclose(logReader_common.file);
}


int logReader_init(const char *path)
{
	logReader_common.file = fopen(path, "rb");

	logReader_common.error = false;
	logReader_common.imuParsingState = notParsing;

	return logReader_common.file != NULL ? 0 : -1;
}
