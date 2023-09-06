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
#include <errno.h>


/* clang-format off */
typedef enum { timeLog = 0, imuLog, gpsLog, baroLog, stateLog } logType_t;
/* clang-format on */


static struct {
	FILE *file;

	long int fileOffsets[LOG_TYPES_CNT];
} ekflog_common;


static void ekflog_ebadfMsg(void)
{
	fprintf(stderr, "Log reader: Invalid log file\n");
	errno = EBADF;
}


static int ekflog_logSizeGet(char logIndicator)
{
	switch (logIndicator) {
		case TIME_LOG_INDICATOR:
			return TIME_LOG_SIZE;

		case IMU_LOG_INDICATOR:
			return IMU_LOG_SIZE;

		case GPS_LOG_INDICATOR:
			return GPS_LOG_SIZE;

		case BARO_LOG_INDICATOR:
			return BARO_LOG_SIZE;

		case STATE_LOG_INDICATOR:
			return STATE_LOG_SIZE;

		default:
			fprintf(stderr, "Log reader: Invalid log indicator in file: %c\n", logIndicator);
			return -1;
	}
}


static int ekflog_logOmit(char logIndicator)
{
	const size_t prefixLen = LOG_ID_SIZE + LOG_IDENTIFIER_SIZE; /* Without the timestamp */
	size_t logSize = ekflog_logSizeGet(logIndicator);
	if (logSize < 0) {
		return -1;
	}

	return fseek(ekflog_common.file, logSize - prefixLen, SEEK_CUR);
}


static int ekflog_nextLogSeek(char logIndicator)
{
	int actIndicator;

	do {
		/* Omitting log ID */
		if (fseek(ekflog_common.file, LOG_ID_SIZE, SEEK_CUR) != 0) {
			return -1;
		}

		actIndicator = fgetc(ekflog_common.file);
		if (actIndicator == EOF) {
			return -1;
		}

		if (actIndicator != logIndicator) {
			if (ekflog_logOmit(actIndicator) != 0) {
				ekflog_ebadfMsg();
				return -1;
			}
		}
	} while (actIndicator != logIndicator);

	return 0;
}


static int ekflog_nextFind(logType_t logType, char logIndicator)
{
	errno = 0;

	if (fseek(ekflog_common.file, ekflog_common.fileOffsets[logType], SEEK_SET) != 0) {
		return -1;
	}

	if (ekflog_nextLogSeek(logIndicator) != 0) {
		return -1;
	}

	return 0;
}


static int ekflog_postStore(logType_t logType)
{
	ekflog_common.fileOffsets[logType] = ftell(ekflog_common.file);
	if (ekflog_common.fileOffsets[logType] < 0) {
		return EOF;
	}

	return 0;
}


int ekflog_timeRead(time_t *timestamp)
{
	if (ekflog_nextFind(timeLog, TIME_LOG_INDICATOR) != 0) {
		return EOF;
	}

	if (fread(timestamp, LOG_TIMESTAMP_SIZE, 1, ekflog_common.file) != 1) {
		if (errno == 0) {
			ekflog_ebadfMsg();
		}
		return EOF;
	}

	return ekflog_postStore(timeLog);
}


int ekflog_imuRead(sensor_event_t *accEvt, sensor_event_t *gyrEvt, sensor_event_t *magEvt)
{
	time_t timestamp;

	if (ekflog_nextFind(imuLog, IMU_LOG_INDICATOR) != 0) {
		return EOF;
	}

	if (fread(&timestamp, sizeof(time_t), 1, ekflog_common.file) != 1) {
		if (errno == 0) {
			ekflog_ebadfMsg();
		}
		return EOF;
	}

	if (fread(&accEvt->accels, sizeof(accEvt->accels), 1, ekflog_common.file) != 1) {
		if (errno == 0) {
			ekflog_ebadfMsg();
		}
		return EOF;
	}

	if (fread(&gyrEvt->gyro, sizeof(gyrEvt->gyro), 1, ekflog_common.file) != 1) {
		if (errno == 0) {
			ekflog_ebadfMsg();
		}
		return EOF;
	}

	if (fread(&magEvt->mag, sizeof(magEvt->mag), 1, ekflog_common.file) != 1) {
		if (errno == 0) {
			ekflog_ebadfMsg();
		}
		return EOF;
	}

	accEvt->type = SENSOR_TYPE_ACCEL;
	accEvt->timestamp = timestamp;

	gyrEvt->type = SENSOR_TYPE_GYRO;
	gyrEvt->timestamp = timestamp;

	magEvt->type = SENSOR_TYPE_MAG;
	magEvt->timestamp = timestamp;

	return ekflog_postStore(imuLog);
}


int ekflog_gpsRead(sensor_event_t *gpsEvt)
{
	if (ekflog_nextFind(gpsLog, GPS_LOG_INDICATOR) != 0) {
		return EOF;
	}

	if (fread(&gpsEvt->timestamp, LOG_TIMESTAMP_SIZE, 1, ekflog_common.file) != 1) {
		if (errno == 0) {
			ekflog_ebadfMsg();
		}
		return EOF;
	}

	if (fread(&gpsEvt->gps, sizeof(gpsEvt->gps), 1, ekflog_common.file) != 1) {
		if (errno == 0) {
			ekflog_ebadfMsg();
		}
		return EOF;
	}

	gpsEvt->type = SENSOR_TYPE_GPS;

	return ekflog_postStore(gpsLog);
}


int ekflog_baroRead(sensor_event_t *baroEvt)
{
	if (ekflog_nextFind(baroLog, BARO_LOG_INDICATOR)) {
		return EOF;
	}

	if (fread(&baroEvt->timestamp, LOG_TIMESTAMP_SIZE, 1, ekflog_common.file) != 1) {
		if (errno == 0) {
			ekflog_ebadfMsg();
		}
		return EOF;
	}

	if (fread(&baroEvt->baro, sizeof(baroEvt->baro), 1, ekflog_common.file) != 1) {
		if (errno == 0) {
			ekflog_ebadfMsg();
		}
		return EOF;
	}

	baroEvt->type = SENSOR_TYPE_BARO;

	return ekflog_postStore(baroLog);
}


int ekflog_stateRead(matrix_t *state, time_t *timestamp)
{
	if (ekflog_nextFind(stateLog, STATE_LOG_INDICATOR) != 0) {
		return EOF;
	}

	if (fread(timestamp, LOG_TIMESTAMP_SIZE, 1, ekflog_common.file) != 1) {
		if (errno == 0) {
			ekflog_ebadfMsg();
		}
		return EOF;
	}

	if (fread(state->data, STATE_LOG_SIZE - LOG_PREFIX_SIZE, 1, ekflog_common.file) != 1) {
		if (errno == 0) {
			ekflog_ebadfMsg();
		}
		return EOF;
	}

	return ekflog_postStore(stateLog);
}


int ekflog_readerInit(const char *path)
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
