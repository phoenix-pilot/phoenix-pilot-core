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
	int fd;

	ekflog_buff_t buffA;
	ekflog_buff_t buffB;

	ekflog_buff_t *actBuff;

	pthread_mutex_t lock;
	pthread_cond_t buffEvent;
	pthread_t tid;

	uint32_t logCnt; /* Number of requests to log a value */
	volatile int run;
	bool logsEnabled;

	int lost; /* Number of lost logs */
} ekflog_common;


static inline ekflog_buff_t *ekflog_nextBufferGet(ekflog_buff_t *actBuff)
{
	return actBuff == &ekflog_common.buffA ? &ekflog_common.buffB : &ekflog_common.buffA;
}


static void *ekflog_thread(void *args)
{
	ekflog_buff_t *outBuff = &ekflog_common.buffA;

	pthread_mutex_lock(&ekflog_common.lock);

#ifdef LOG_VOL_CHECK
	maxLog_start();
	maxLog_sleepReport();
#endif

	do {
		while (outBuff->dirty == false && ekflog_common.run != 0) {
			pthread_cond_wait(&ekflog_common.buffEvent, &ekflog_common.lock);
		}

		while (outBuff->dirty == true) {
			pthread_mutex_unlock(&ekflog_common.lock);

#ifdef LOG_VOL_CHECK
			maxLog_wakeUpReport();
			maxLog_writeReport(outBuff->size);
#endif

			if (write(ekflog_common.fd, outBuff->buff, outBuff->size) != outBuff->size) {
				fprintf(stderr, "ekflog: error while writing to file\n");
			}

#ifdef LOG_VOL_CHECK
			maxLog_sleepReport();
#endif

			pthread_mutex_lock(&ekflog_common.lock);

			outBuff->dirty = false;
			outBuff->size = 0;

			outBuff = ekflog_nextBufferGet(outBuff);
			pthread_cond_signal(&ekflog_common.buffEvent);
		}
	} while (ekflog_common.run != 0);

#ifdef LOG_VOL_CHECK
	maxLog_wakeUpReport();
	maxLog_end();
	maxLog_resultsPrint();
#endif

	printf("Logging finished\n");
	printf("Number of logs requests: %d\n", ekflog_common.logCnt);
	printf("Lost logs: %d\n", ekflog_common.lost);

	pthread_mutex_unlock(&ekflog_common.lock);

	return NULL;
}


static bool ekflog_actBuffWritable(void)
{
	if (ekflog_common.actBuff->dirty == false) {
		return true;
	}

	if ((ekflog_common.logFlags & EKFLOG_STRICT_MODE) != 0) {
		/* Waiting for a place to insert logs */
		do {
			pthread_cond_wait(&ekflog_common.buffEvent, &ekflog_common.lock);
		} while (ekflog_common.actBuff->dirty == true);

		return true;
	}

	return false;
}


static int ekflog_write(const void *msg, size_t msgLen, char logIndicator, time_t timestamp)
{
	size_t remainingBuffSize;

	pthread_mutex_lock(&ekflog_common.lock);

	remainingBuffSize = BUFFS_CAPACITY - ekflog_common.actBuff->size;

	if (remainingBuffSize < msgLen + LOG_PREFIX_SIZE) {
		/* Changing actual buffer for the next one */
		ekflog_common.actBuff->dirty = true;
		ekflog_common.actBuff = ekflog_nextBufferGet(ekflog_common.actBuff);
		pthread_cond_signal(&ekflog_common.buffEvent);
	}

	/* Adding log number */
	ekflog_common.logCnt++;

	if (!ekflog_actBuffWritable()) {
		/* Dropping the log */
		ekflog_common.lost++;
		pthread_mutex_unlock(&ekflog_common.lock);
		return -1;
	}

	memcpy(ekflog_common.actBuff->buff + ekflog_common.actBuff->size, &ekflog_common.logCnt, sizeof(ekflog_common.logCnt));
	ekflog_common.actBuff->size += sizeof(ekflog_common.logCnt);

	/* Adding log identifier */
	memcpy(ekflog_common.actBuff->buff + ekflog_common.actBuff->size, &logIndicator, sizeof(logIndicator));
	ekflog_common.actBuff->size += sizeof(logIndicator);

	/* Adding timestamp */
	memcpy(ekflog_common.actBuff->buff + ekflog_common.actBuff->size, &timestamp, sizeof(timestamp));
	ekflog_common.actBuff->size += sizeof(timestamp);

	if (msgLen > 0) {
		memcpy(ekflog_common.actBuff->buff + ekflog_common.actBuff->size, msg, msgLen);
		ekflog_common.actBuff->size += msgLen;
	}

	pthread_mutex_unlock(&ekflog_common.lock);

	return 0;
}


int ekflog_timeWrite(time_t timestamp)
{
	/* Log call with flags that are not enabled is not an error */
	if ((ekflog_common.logFlags & EKFLOG_TIME) == 0) {
		return 0;
	}

	return ekflog_write(NULL, 0, TIME_LOG_INDICATOR, timestamp);
}


int ekflog_imuWrite(const sensor_event_t *accEvt, const sensor_event_t *gyrEvt, const sensor_event_t *magEvt)
{
	uint8_t buff[IMU_LOG_SIZE - LOG_PREFIX_SIZE];

	/* Log call with flags that are not enabled is not an error */
	if ((ekflog_common.logFlags & EKFLOG_SENSC) == 0) {
		return 0;
	}

	memcpy(buff, &accEvt->accels, sizeof(accEvt->accels));
	memcpy(buff + sizeof(accEvt->accels), &gyrEvt->gyro, sizeof(gyrEvt->gyro));
	memcpy(buff + sizeof(accEvt->accels) + sizeof(gyrEvt->gyro), &magEvt->mag, sizeof(magEvt->mag));

	return ekflog_write(buff, sizeof(buff), IMU_LOG_INDICATOR, accEvt->timestamp);
}


int ekflog_gpsWrite(const sensor_event_t *gpsEvt)
{
	/* Log call with flags that are not enabled is not an error */
	if ((ekflog_common.logFlags & EKFLOG_SENSC) == 0) {
		return 0;
	}

	return ekflog_write(&gpsEvt->gps, sizeof(gpsEvt->gps), GPS_LOG_INDICATOR, gpsEvt->timestamp);
}


int ekflog_baroWrite(const sensor_event_t *baroEvt)
{
	/* Log call with flags that are not enabled is not an error */
	if ((ekflog_common.logFlags & EKFLOG_SENSC) == 0) {
		return 0;
	}

	return ekflog_write(&baroEvt->baro, sizeof(baroEvt->baro), BARO_LOG_INDICATOR, baroEvt->timestamp);
}


int ekflog_stateWrite(const matrix_t *state, time_t timestamp)
{
	/* Log call with flags that are not enabled is not an error */
	if ((ekflog_common.logFlags & EKFLOG_STATE) == 0) {
		return 0;
	}

	return ekflog_write(state->data, STATE_LOG_SIZE - LOG_PREFIX_SIZE, STATE_LOG_INDICATOR, timestamp);
}


int ekflog_writerDone(void)
{
	int err = 0;

	if (ekflog_common.logsEnabled == false) {
		return 0;
	}

	pthread_mutex_lock(&ekflog_common.lock);
	ekflog_common.run = 0;
	ekflog_common.actBuff->dirty = true;
	pthread_mutex_unlock(&ekflog_common.lock);

	pthread_cond_signal(&ekflog_common.buffEvent);

	if (pthread_join(ekflog_common.tid, NULL) != 0) {
		fprintf(stderr, "ekflog: cannot join logging thread\n");
		return -1;
	}

	err |= close(ekflog_common.fd);
	err |= pthread_mutex_destroy(&ekflog_common.lock);
	err |= pthread_cond_destroy(&ekflog_common.buffEvent);

	return err;
}


int ekflog_writerInit(const char *path, uint32_t flags)
{
	pthread_attr_t attr;
	int ret;

	if (flags == 0) {
		ekflog_common.logsEnabled = false;
		ekflog_common.logFlags = 0;
		return 0;
	}

	if (path == NULL) {
		fprintf(stderr, "ekflog: wrong file path\n");
		return -1;
	}

	ekflog_common.fd = open(path, O_WRONLY | O_CREAT | O_TRUNC, S_IRWXU);
	if (ekflog_common.fd == -1) {
		fprintf(stderr, "ekflog: can`t open %s to write\n", path);
		return -1;
	}

	if (pthread_mutex_init(&ekflog_common.lock, NULL) != 0) {
		fprintf(stderr, "ekflog: cannot initialize lock\n");
		close(ekflog_common.fd);
		return -1;
	}

	if (pthread_cond_init(&ekflog_common.buffEvent, NULL) != 0) {
		fprintf(stderr, "ekflog: cannot initialize conditional variable\n");
		close(ekflog_common.fd);
		pthread_mutex_destroy(&ekflog_common.lock);
		return -1;
	}

	if (pthread_attr_init(&attr) != 0) {
		fprintf(stderr, "ekflog: cannot initialize conditional variable\n");
		close(ekflog_common.fd);
		pthread_mutex_destroy(&ekflog_common.lock);
		pthread_cond_destroy(&ekflog_common.buffEvent);
		return -1;
	}

/* On Phoenix-RTOS we want to set thread priority */
#ifdef __phoenix__

	if (pthread_attr_setschedparam(&attr, &((struct sched_param) { .sched_priority = PHOENIX_THREAD_PRIO })) != 0) {
		printf("ekflog: cannot set thread priority\n");
		close(ekflog_common.fd);
		pthread_mutex_destroy(&ekflog_common.lock);
		pthread_cond_destroy(&ekflog_common.buffEvent);
		pthread_attr_destroy(&attr);
		return -1;
	}

#endif

	ekflog_common.logFlags = flags;

	ekflog_common.buffA.dirty = false;
	ekflog_common.buffA.size = 0;

	ekflog_common.buffB.dirty = false;
	ekflog_common.buffB.size = 0;

	ekflog_common.actBuff = &ekflog_common.buffA;
	ekflog_common.logCnt = 0;
	ekflog_common.run = 1;
	ekflog_common.logsEnabled = true;
	ekflog_common.lost = 0;

	ret = pthread_create(&ekflog_common.tid, &attr, ekflog_thread, NULL);
	pthread_attr_destroy(&attr);
	if (ret != 0) {
		fprintf(stderr, "ekflog: cannot start a log thread\n");
		close(ekflog_common.fd);
		pthread_mutex_destroy(&ekflog_common.lock);
		pthread_cond_destroy(&ekflog_common.buffEvent);
		return -1;
	}

	return 0;
}
