/*
 * Phoenix-Pilot
 *
 * Ekf-specific log module
 *
 * Copyright 2022, 2023 Phoenix Systems
 * Authors: Mateusz Niewiadomski, Piotr Nieciecki
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#include "writer.h"

#include "common.h"

#include <stdio.h>
#include <pthread.h>
#include <stdbool.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>

#include <plog.h>

#ifdef LOG_VOL_CHECK
#include "max_logs.h"
#endif

#define BUFFS_CAPACITY 1024 * 8

#define PHOENIX_THREAD_PRIO 4


typedef struct {
	uint8_t buff[BUFFS_CAPACITY];
	bool dirty;
	int size; /* Used capacity */
} ekflog_buff_t;


static struct {
	uint32_t logFlags;
	bool logsEnabled;

	plog_t *ctx;
} ekflog_common;


int ekflog_timeWrite(time_t timestamp)
{
	/* Log call with flags that are not enabled is not an error */
	if ((ekflog_common.logFlags & EKFLOG_TIME) == 0) {
		return 0;
	}

	return plog_write(ekflog_common.ctx, NULL, 0, TIME_LOG_INDICATOR, timestamp);
}


int ekflog_imuWrite(const sensor_event_t *accEvt, const sensor_event_t *gyrEvt, const sensor_event_t *magEvt)
{
	uint8_t buff[IMU_LOG_SIZE];

	/* Log call with flags that are not enabled is not an error */
	if ((ekflog_common.logFlags & EKFLOG_SENSC) == 0) {
		return 0;
	}

	memcpy(buff, &accEvt->accels, sizeof(accEvt->accels));
	memcpy(buff + sizeof(accEvt->accels), &gyrEvt->gyro, sizeof(gyrEvt->gyro));
	memcpy(buff + sizeof(accEvt->accels) + sizeof(gyrEvt->gyro), &magEvt->mag, sizeof(magEvt->mag));

	return plog_write(ekflog_common.ctx, buff, sizeof(buff), IMU_LOG_INDICATOR, accEvt->timestamp);
}


int ekflog_gpsWrite(const sensor_event_t *gpsEvt)
{
	/* Log call with flags that are not enabled is not an error */
	if ((ekflog_common.logFlags & EKFLOG_SENSC) == 0) {
		return 0;
	}

	return plog_write(ekflog_common.ctx, &gpsEvt->gps, sizeof(gpsEvt->gps), GPS_LOG_INDICATOR, gpsEvt->timestamp);
}


int ekflog_baroWrite(const sensor_event_t *baroEvt)
{
	/* Log call with flags that are not enabled is not an error */
	if ((ekflog_common.logFlags & EKFLOG_SENSC) == 0) {
		return 0;
	}

	return plog_write(ekflog_common.ctx, &baroEvt->baro, sizeof(baroEvt->baro), BARO_LOG_INDICATOR, baroEvt->timestamp);
}


int ekflog_stateWrite(const matrix_t *state, time_t timestamp)
{
	/* Log call with flags that are not enabled is not an error */
	if ((ekflog_common.logFlags & EKFLOG_STATE) == 0) {
		return 0;
	}

	return plog_write(ekflog_common.ctx, state->data, STATE_LOG_SIZE, STATE_LOG_INDICATOR, timestamp);
}


int ekflog_writerInit(plog_t *logger, uint32_t flags)
{
	if (flags == 0) {
		ekflog_common.logsEnabled = false;
		ekflog_common.logFlags = 0;
		return 0;
	}

	if (logger == NULL) {
		fprintf(stderr, "ekflog: failed to start plog\n");
		return -1;
	}

	ekflog_common.ctx = logger;
	ekflog_common.logFlags = flags;
	ekflog_common.logsEnabled = true;

	return 0;
}
