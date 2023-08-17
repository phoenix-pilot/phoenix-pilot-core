/*
 * Phoenix-Pilot
 *
 * Ekf-specific log reader module
 *
 * Copyright 2023 Phoenix Systems
 * Authors: Piotr Nieciecki
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#include "reader.h"

#include "common.h"

#include <stdio.h>


#define ekflog_fieldRead(buff) \
	ekflog_fieldReadRaw(&(buff), sizeof(buff))


/* clang-format off */
typedef enum { timeLog = 0, imuLog, gpsLog, baroLog } logType_t;
/* clang-format on */


static struct {
	FILE *file;

	long int fileOffsets[LOG_TYPES_CNT];
} ekflog_common;


static int ekflog_fieldReadRaw(void *buff, size_t buffLen)
{
	if (fread(buff, buffLen, 1, ekflog_common.file) != 1) {
		return -1;
	}

	return 0;
}


static int ekflog_timestampRead(time_t *timestamp)
{
	uint64_t tmp;

	if (ekflog_fieldRead(tmp) != 0) {
		return -1;
	}

	*timestamp = (time_t)tmp;

	return 0;
}


static int ekflog_logOmit(char logIndicator)
{
	const size_t prefixLen = sizeof(uint32_t) + sizeof(char); /* Without the timestamp */

	switch (logIndicator) {
		case TIME_LOG_INDICATOR:
			return fseek(ekflog_common.file, TIME_LOG_LEN - prefixLen, SEEK_CUR);
			;

		case IMU_LOG_INDICATOR:
			return fseek(ekflog_common.file, IMU_LOG_LEN - prefixLen, SEEK_CUR);

		case GPS_LOG_INDICATOR:
			return fseek(ekflog_common.file, GPS_LOG_LEN - prefixLen, SEEK_CUR);

		case BARO_LOG_INDICATOR:
			return fseek(ekflog_common.file, BARO_LOG_LEN - prefixLen, SEEK_CUR);

		default:
			fprintf(stderr, "ekflog reader: Invalid log indicator in file: %c\n", logIndicator);
			return -1;
	}
}


static int ekflog_nextLogSeek(char logIndicator)
{
	char actIndicator;

	do {
		/* Omitting log ID */
		if (fseek(ekflog_common.file, sizeof(uint32_t), SEEK_CUR) != 0) {
			return -1;
		}

		if (ekflog_fieldRead(actIndicator) != 0) {
			return -1;
		}

		if (actIndicator != logIndicator) {
			if (ekflog_logOmit(actIndicator) != 0) {
				return -1;
			}
		}
	} while (actIndicator != logIndicator);

	return 0;
}


int ekflog_timeRead(time_t *timestamp)
{
	if (fseek(ekflog_common.file, ekflog_common.fileOffsets[timeLog], SEEK_SET) != 0) {
		return -1;
	}

	if (ekflog_nextLogSeek(TIME_LOG_INDICATOR) != 0) {
		return -1;
	}

	if (ekflog_timestampRead(timestamp) != 0) {
		return -1;
	}

	ekflog_common.fileOffsets[timeLog] = ftell(ekflog_common.file);
	if (ekflog_common.fileOffsets[timeLog] < 0) {
		return -1;
	}

	return 0;
}


int ekflog_senscImuRead(sensor_event_t *accEvt, sensor_event_t *gyrEvt, sensor_event_t *magEvt)
{
	int err = 0;
	time_t timestamp;

	if (fseek(ekflog_common.file, ekflog_common.fileOffsets[imuLog], SEEK_SET) != 0) {
		return -1;
	}

	if (ekflog_nextLogSeek(IMU_LOG_INDICATOR) != 0) {
		return -1;
	}

	err |= ekflog_timestampRead(&timestamp);

	accEvt->type = SENSOR_TYPE_ACCEL;
	accEvt->timestamp = timestamp;
	err |= ekflog_fieldRead(accEvt->accels.accelX);
	err |= ekflog_fieldRead(accEvt->accels.accelY);
	err |= ekflog_fieldRead(accEvt->accels.accelZ);

	gyrEvt->type = SENSOR_TYPE_GYRO;
	gyrEvt->timestamp = timestamp;
	err |= ekflog_fieldRead(gyrEvt->gyro.gyroX);
	err |= ekflog_fieldRead(gyrEvt->gyro.gyroY);
	err |= ekflog_fieldRead(gyrEvt->gyro.gyroZ);
	err |= ekflog_fieldRead(gyrEvt->gyro.dAngleX);
	err |= ekflog_fieldRead(gyrEvt->gyro.dAngleY);
	err |= ekflog_fieldRead(gyrEvt->gyro.dAngleZ);

	magEvt->type = SENSOR_TYPE_MAG;
	magEvt->timestamp = timestamp;
	err |= ekflog_fieldRead(magEvt->mag.magX);
	err |= ekflog_fieldRead(magEvt->mag.magY);
	err |= ekflog_fieldRead(magEvt->mag.magZ);

	if (err != 0) {
		fprintf(stderr, "ekflog reader: error while parsing IMU log\n");
	}

	ekflog_common.fileOffsets[imuLog] = ftell(ekflog_common.file);
	if (ekflog_common.fileOffsets[timeLog] < 0) {
		return -1;
	}

	return err;
}


int ekflog_senscGpsRead(sensor_event_t *gpsEvt)
{
	int err = 0;

	if (fseek(ekflog_common.file, ekflog_common.fileOffsets[gpsLog], SEEK_SET) != 0) {
		return -1;
	}

	if (ekflog_nextLogSeek(GPS_LOG_INDICATOR) != 0) {
		return -1;
	}

	gpsEvt->type = SENSOR_TYPE_GPS;

	err |= ekflog_timestampRead(&gpsEvt->timestamp);

	err |= ekflog_fieldRead(gpsEvt->gps.lat);
	err |= ekflog_fieldRead(gpsEvt->gps.lon);
	err |= ekflog_fieldRead(gpsEvt->gps.alt);

	err |= ekflog_fieldRead(gpsEvt->gps.utc);

	err |= ekflog_fieldRead(gpsEvt->gps.hdop);
	err |= ekflog_fieldRead(gpsEvt->gps.vdop);

	err |= ekflog_fieldRead(gpsEvt->gps.altEllipsoid);
	err |= ekflog_fieldRead(gpsEvt->gps.groundSpeed);

	err |= ekflog_fieldRead(gpsEvt->gps.velNorth);
	err |= ekflog_fieldRead(gpsEvt->gps.velEast);
	err |= ekflog_fieldRead(gpsEvt->gps.velDown);

	err |= ekflog_fieldRead(gpsEvt->gps.eph);
	err |= ekflog_fieldRead(gpsEvt->gps.epv);
	err |= ekflog_fieldRead(gpsEvt->gps.evel);

	err |= ekflog_fieldRead(gpsEvt->gps.heading);
	err |= ekflog_fieldRead(gpsEvt->gps.headingOffs);
	err |= ekflog_fieldRead(gpsEvt->gps.headingAccur);

	err |= ekflog_fieldRead(gpsEvt->gps.satsNb);
	err |= ekflog_fieldRead(gpsEvt->gps.fix);

	if (err != 0) {
		fprintf(stderr, "ekflog reader: error while parsing GPS log\n");
	}

	ekflog_common.fileOffsets[gpsLog] = ftell(ekflog_common.file);
	if (ekflog_common.fileOffsets[timeLog] < 0) {
		return -1;
	}

	return err;
}


int ekflog_senscBaroRead(sensor_event_t *baroEvt)
{
	int err = 0;

	if (fseek(ekflog_common.file, ekflog_common.fileOffsets[baroLog], SEEK_SET) != 0) {
		return -1;
	}

	if (ekflog_nextLogSeek(BARO_LOG_INDICATOR) != 0) {
		return -1;
	}

	baroEvt->type = SENSOR_TYPE_BARO;

	err |= ekflog_timestampRead(&baroEvt->timestamp);

	err |= ekflog_fieldRead(baroEvt->baro.pressure);
	err |= ekflog_fieldRead(baroEvt->baro.temp);

	if (err != 0) {
		fprintf(stderr, "ekflog reader: error while parsing GPS log\n");
	}

	ekflog_common.fileOffsets[baroLog] = ftell(ekflog_common.file);
	if (ekflog_common.fileOffsets[timeLog] < 0) {
		return -1;
	}

	return err;
}


int ekflog_readerInit(char *path)
{
	int i;

	ekflog_common.file = fopen(path, "rb");
	if (ekflog_common.file == NULL) {
		return -1;
	}

	for (i = 0; i < LOG_TYPES_CNT; i++) {
		ekflog_common.fileOffsets[i] = 0;
	}

	return 0;
}


int ekflog_readerDone(void)
{
	if (ekflog_common.file != NULL) {
		return fclose(ekflog_common.file);
	}

	return 0;
}
