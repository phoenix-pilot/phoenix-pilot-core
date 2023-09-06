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


static int ekflog_logOmit(char logIndicator)
{
	const size_t prefixLen = LOG_ID_SIZE + LOG_IDENTIFIER_SIZE; /* Without the timestamp */

	switch (logIndicator) {
		case TIME_LOG_INDICATOR:
			return fseek(ekflog_common.file, TIME_LOG_SIZE - prefixLen, SEEK_CUR);

		case IMU_LOG_INDICATOR:
			return fseek(ekflog_common.file, IMU_LOG_SIZE - prefixLen, SEEK_CUR);

		case GPS_LOG_INDICATOR:
			return fseek(ekflog_common.file, GPS_LOG_SIZE - prefixLen, SEEK_CUR);

		case BARO_LOG_INDICATOR:
			return fseek(ekflog_common.file, BARO_LOG_SIZE - prefixLen, SEEK_CUR);

		case STATE_LOG_INDICATOR:
			return fseek(ekflog_common.file, STATE_LOG_SIZE - prefixLen, SEEK_CUR);

		default:
			fprintf(stderr, "ekflog reader: Invalid log indicator in file: %c\n", logIndicator);
			errno = EBADF;
			return -1;
	}
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
				return -1;
			}
		}
	} while (actIndicator != logIndicator);

	return 0;
}


static void ekflog_ebadfMsg(void)
{
	fprintf(stderr, "Log reader: Invalid log file\n");
	errno = EBADF;
}


int ekflog_timeRead(time_t *timestamp)
{
	errno = 0;

	if (fseek(ekflog_common.file, ekflog_common.fileOffsets[timeLog], SEEK_SET) != 0) {
		return EOF;
	}

	if (ekflog_nextLogSeek(TIME_LOG_INDICATOR) != 0) {
		return EOF;
	}

	if (fread(timestamp, LOG_TIMESTAMP_SIZE, 1, ekflog_common.file) != 1) {
		if (errno == 0) {
			ekflog_ebadfMsg();
		}
		return EOF;
	}

	ekflog_common.fileOffsets[timeLog] = ftell(ekflog_common.file);
	if (ekflog_common.fileOffsets[timeLog] < 0) {
		return EOF;
	}

	return 0;
}


int ekflog_imuRead(sensor_event_t *accEvt, sensor_event_t *gyrEvt, sensor_event_t *magEvt)
{
	time_t timestamp;

	errno = 0;

	if (fseek(ekflog_common.file, ekflog_common.fileOffsets[imuLog], SEEK_SET) != 0) {
		return EOF;
	}

	if (ekflog_nextLogSeek(IMU_LOG_INDICATOR) != 0) {
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

	ekflog_common.fileOffsets[imuLog] = ftell(ekflog_common.file);
	if (ekflog_common.fileOffsets[timeLog] < 0) {
		return EOF;
	}

	return 0;
}


int ekflog_gpsRead(sensor_event_t *gpsEvt)
{
	errno = 0;

	if (fseek(ekflog_common.file, ekflog_common.fileOffsets[gpsLog], SEEK_SET) != 0) {
		return EOF;
	}

	if (ekflog_nextLogSeek(GPS_LOG_INDICATOR) != 0) {
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

	ekflog_common.fileOffsets[gpsLog] = ftell(ekflog_common.file);
	if (ekflog_common.fileOffsets[timeLog] < 0) {
		return EOF;
	}

	return 0;
}


int ekflog_baroRead(sensor_event_t *baroEvt)
{
	errno = 0;

	if (fseek(ekflog_common.file, ekflog_common.fileOffsets[baroLog], SEEK_SET) != 0) {
		return EOF;
	}

	if (ekflog_nextLogSeek(BARO_LOG_INDICATOR) != 0) {
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

	ekflog_common.fileOffsets[baroLog] = ftell(ekflog_common.file);
	if (ekflog_common.fileOffsets[baroLog] < 0) {
		return EOF;
	}

	return 0;
}


int ekflog_stateRead(matrix_t *state, time_t *timestamp)
{
	errno = 0;

	if (fseek(ekflog_common.file, ekflog_common.fileOffsets[stateLog], SEEK_SET) != 0) {
		return EOF;
	}

	if (ekflog_nextLogSeek(STATE_LOG_INDICATOR) != 0) {
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

	ekflog_common.fileOffsets[stateLog] = ftell(ekflog_common.file);
	if (ekflog_common.fileOffsets[stateLog] < 0) {
		return EOF;
	}

	return 0;
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
