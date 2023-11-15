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
 *
 *
 * The actual implementation of collection logs uses a two buffers.
 * The log producer saves data to the first one as long as there is enough space for new logs.
 * When there is not enough free memory, the producer marks it as dirty and starts using the next one.
 *
 * A separate thread saves the dirty buffers to a file and clears dirty flag.
 *
 * This approach allows for collecting logs without blocking the EKF thread due to potentially
 * time-consuming file writes.
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


int ekflog_writerDone(void)
{
	int err = 0;

	if (ekflog_common.logsEnabled == false) {
		return 0;
	}

	plog_done(ekflog_common.ctx);

	return err;
}


int ekflog_writerInit(const char *path, uint32_t flags)
{
	if (flags == 0) {
		ekflog_common.logsEnabled = false;
		ekflog_common.logFlags = 0;
		return 0;
	}

	ekflog_common.ctx = plog_init(path, flags & EKFLOG_STRICT_MODE);
	if (ekflog_common.ctx == NULL) {
		fprintf(stderr, "ekflog: failed to start plog\n");
		return -1;
	}

	ekflog_common.logFlags = flags;
	ekflog_common.logsEnabled = true;

	return 0;
}
