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
#include <string.h>


#define FIELD_GET(des, src) \
	ekflog_logFieldGet((des), (src), sizeof(*(des)))


/* clang-format off */
typedef enum { timeLog = 0, imuLog, gpsLog, baroLog } logType_t;
/* clang-format on */


static struct {
	FILE *file;

	long int fileOffsets[LOG_TYPES_CNT];
} ekflog_common;


static int ekflog_logOmit(char logIndicator)
{
	const size_t prefixLen = LOG_ID_SIZE + LOG_IDENTIFIER_SIZE; /* Without the timestamp */

	switch (logIndicator) {
		case TIME_LOG_INDICATOR:
			return fseek(ekflog_common.file, TIME_LOG_LEN - prefixLen, SEEK_CUR);

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
	int actIndicator;

	do {
		/* Omitting log ID */
		if (fseek(ekflog_common.file, sizeof(uint32_t), SEEK_CUR) != 0) {
			return -1;
		}

		actIndicator = fgetc(ekflog_common.file);
		if (actIndicator == EOF) {
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


static inline size_t ekflog_logFieldGet(void *des, const void *src, size_t size)
{
	memcpy(des, src, size);
	return size;
}


static size_t ekflog_timestampRead(time_t *timestamp, uint8_t *buff)
{
	uint64_t tmp;

	memcpy(&tmp, buff, sizeof(tmp));
	*timestamp = (time_t)tmp;

	return sizeof(time_t);
}


int ekflog_timeRead(time_t *timestamp)
{
	uint8_t buff[TIME_LOG_LEN - LOG_ID_SIZE - LOG_IDENTIFIER_SIZE];

	if (fseek(ekflog_common.file, ekflog_common.fileOffsets[timeLog], SEEK_SET) != 0) {
		return -1;
	}

	if (ekflog_nextLogSeek(TIME_LOG_INDICATOR) != 0) {
		return -1;
	}

	if (fread(buff, sizeof(buff), 1, ekflog_common.file) != 1) {
		return -1;
	}

	ekflog_timestampRead(timestamp, buff);

	ekflog_common.fileOffsets[timeLog] = ftell(ekflog_common.file);
	if (ekflog_common.fileOffsets[timeLog] < 0) {
		return -1;
	}

	return 0;
}


int ekflog_senscImuRead(sensor_event_t *accEvt, sensor_event_t *gyrEvt, sensor_event_t *magEvt)
{
	time_t timestamp;
	uint8_t buff[IMU_LOG_LEN - LOG_ID_SIZE - LOG_IDENTIFIER_SIZE];
	uint8_t *buffHead = buff;

	if (fseek(ekflog_common.file, ekflog_common.fileOffsets[imuLog], SEEK_SET) != 0) {
		return -1;
	}

	if (ekflog_nextLogSeek(IMU_LOG_INDICATOR) != 0) {
		return -1;
	}

	if (fread(buff, sizeof(buff), 1, ekflog_common.file) != 1) {
		return -1;
	}

	buffHead += ekflog_timestampRead(&timestamp, buffHead);

	accEvt->type = SENSOR_TYPE_ACCEL;
	accEvt->timestamp = timestamp;
	buffHead += FIELD_GET(&accEvt->accels.accelX, buffHead);
	buffHead += FIELD_GET(&accEvt->accels.accelY, buffHead);
	buffHead += FIELD_GET(&accEvt->accels.accelZ, buffHead);

	gyrEvt->type = SENSOR_TYPE_GYRO;
	gyrEvt->timestamp = timestamp;
	buffHead += FIELD_GET(&gyrEvt->gyro.gyroX, buffHead);
	buffHead += FIELD_GET(&gyrEvt->gyro.gyroY, buffHead);
	buffHead += FIELD_GET(&gyrEvt->gyro.gyroZ, buffHead);
	buffHead += FIELD_GET(&gyrEvt->gyro.dAngleX, buffHead);
	buffHead += FIELD_GET(&gyrEvt->gyro.dAngleY, buffHead);
	buffHead += FIELD_GET(&gyrEvt->gyro.dAngleZ, buffHead);

	magEvt->type = SENSOR_TYPE_MAG;
	magEvt->timestamp = timestamp;
	buffHead += FIELD_GET(&magEvt->mag.magX, buffHead);
	buffHead += FIELD_GET(&magEvt->mag.magY, buffHead);
	buffHead += FIELD_GET(&magEvt->mag.magZ, buffHead);

	ekflog_common.fileOffsets[imuLog] = ftell(ekflog_common.file);
	if (ekflog_common.fileOffsets[timeLog] < 0) {
		return -1;
	}

	return 0;
}


int ekflog_senscGpsRead(sensor_event_t *gpsEvt)
{
	uint8_t buff[GPS_LOG_LEN - LOG_ID_SIZE - LOG_IDENTIFIER_SIZE];
	uint8_t *buffHead = buff;

	if (fseek(ekflog_common.file, ekflog_common.fileOffsets[gpsLog], SEEK_SET) != 0) {
		return -1;
	}

	if (ekflog_nextLogSeek(GPS_LOG_INDICATOR) != 0) {
		return -1;
	}

	if (fread(buff, sizeof(buff), 1, ekflog_common.file) != 1) {
		return -1;
	}

	gpsEvt->type = SENSOR_TYPE_GPS;

	buffHead += ekflog_timestampRead(&gpsEvt->timestamp, buffHead);

	buffHead += FIELD_GET(&gpsEvt->gps.lat, buffHead);
	buffHead += FIELD_GET(&gpsEvt->gps.lon, buffHead);
	buffHead += FIELD_GET(&gpsEvt->gps.alt, buffHead);

	buffHead += FIELD_GET(&gpsEvt->gps.utc, buffHead);

	buffHead += FIELD_GET(&gpsEvt->gps.hdop, buffHead);
	buffHead += FIELD_GET(&gpsEvt->gps.vdop, buffHead);

	buffHead += FIELD_GET(&gpsEvt->gps.altEllipsoid, buffHead);
	buffHead += FIELD_GET(&gpsEvt->gps.groundSpeed, buffHead);

	buffHead += FIELD_GET(&gpsEvt->gps.velNorth, buffHead);
	buffHead += FIELD_GET(&gpsEvt->gps.velEast, buffHead);
	buffHead += FIELD_GET(&gpsEvt->gps.velDown, buffHead);

	buffHead += FIELD_GET(&gpsEvt->gps.eph, buffHead);
	buffHead += FIELD_GET(&gpsEvt->gps.epv, buffHead);
	buffHead += FIELD_GET(&gpsEvt->gps.evel, buffHead);

	buffHead += FIELD_GET(&gpsEvt->gps.heading, buffHead);
	buffHead += FIELD_GET(&gpsEvt->gps.headingOffs, buffHead);
	buffHead += FIELD_GET(&gpsEvt->gps.headingAccur, buffHead);

	buffHead += FIELD_GET(&gpsEvt->gps.satsNb, buffHead);
	buffHead += FIELD_GET(&gpsEvt->gps.fix, buffHead);

	ekflog_common.fileOffsets[gpsLog] = ftell(ekflog_common.file);
	if (ekflog_common.fileOffsets[timeLog] < 0) {
		return -1;
	}

	return 0;
}


int ekflog_senscBaroRead(sensor_event_t *baroEvt)
{
	uint8_t buff[BARO_LOG_LEN - LOG_ID_SIZE - LOG_IDENTIFIER_SIZE];
	uint8_t *buffHead = buff;

	if (fseek(ekflog_common.file, ekflog_common.fileOffsets[baroLog], SEEK_SET) != 0) {
		return -1;
	}

	if (ekflog_nextLogSeek(BARO_LOG_INDICATOR) != 0) {
		return -1;
	}

	if (fread(buff, sizeof(buff), 1, ekflog_common.file) != 1) {
		return -1;
	}

	baroEvt->type = SENSOR_TYPE_BARO;

	buffHead += ekflog_timestampRead(&baroEvt->timestamp, buffHead);

	buffHead += FIELD_GET(&baroEvt->baro.pressure, buffHead);
	buffHead += FIELD_GET(&baroEvt->baro.temp, buffHead);

	ekflog_common.fileOffsets[baroLog] = ftell(ekflog_common.file);
	if (ekflog_common.fileOffsets[timeLog] < 0) {
		return -1;
	}

	return 0;
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
