/*
 * Phoenix-Pilot
 *
 * Ekf-specific log module
 *
 * Copyright 2022, 2023 Phoenix Systems
 * Author: Mateusz Niewiadomski, Piotr Nieciecki
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#include "log.h"

#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>
#include <pthread.h>
#include <stdbool.h>
#include <string.h>

#define BUFF_LEN            1024 * 16
#define PHOENIX_THREAD_PRIO 4

#define PREFIX_LOG_LEN 13


struct {
	uint32_t logFlags;
	FILE *file;

	char buff[BUFF_LEN];
	int head;
	int tail;
	int buff_end;
	pthread_mutex_t lock;
	pthread_cond_t queueEvent;
	pthread_t tid;

	uint32_t logCnt;
	volatile int run;
	bool logsEnabled;

	int lost;
} ekflog_common;


static int ekflog_msgLen(void)
{
	if (ekflog_common.head > ekflog_common.tail) {
		return ekflog_common.head - ekflog_common.tail;
	}

	return ekflog_common.buff_end - ekflog_common.tail;
}


static void *ekflog_thread(void *args)
{
	int len;

	pthread_mutex_lock(&ekflog_common.lock);

	do {
		while (ekflog_common.head == ekflog_common.tail && ekflog_common.run != 0) {
			pthread_cond_wait(&ekflog_common.queueEvent, &ekflog_common.lock);
		}

		while (ekflog_common.head != ekflog_common.tail) {
			/* Writing thread assumes max message length */
			if (ekflog_common.tail == ekflog_common.buff_end) {
				ekflog_common.tail = 0;
			}
			else {
				ekflog_common.tail++;
			}

			len = ekflog_msgLen();

			pthread_mutex_unlock(&ekflog_common.lock);

			if (fwrite_unlocked(&ekflog_common.buff[ekflog_common.tail], len, 1, ekflog_common.file) < 1) {
				fprintf(stderr, "ekflogs: error while writing to file\n");
			}


			pthread_mutex_lock(&ekflog_common.lock);
			ekflog_common.tail += len;
			pthread_cond_signal(&ekflog_common.queueEvent);
		}
	} while (ekflog_common.run != 0);

	pthread_mutex_unlock(&ekflog_common.lock);

	return NULL;
}


/*
 * Sets head to the beginning of new message slot.
 * Assumes that the length of a message is equal to MAX_MSG_LEN.
 * Tries to find place for the longest possible message.
 */
static int ekflog_slotGet(void)
{
	if (ekflog_common.head < ekflog_common.tail) {
		if (ekflog_common.tail - ekflog_common.head - 1 > MAX_MSG_LEN) {
			/* Adding new message after previous one */
			ekflog_common.head++;
			return 0;
		}

		return -1;
	}

	/* ekflog_common.head >= ekflog_common.tail */

	if (BUFF_LEN - ekflog_common.head - 2 >= MAX_MSG_LEN) {
		/* Adding new message after previous one */
		ekflog_common.head++;
		return 0;
	}

	if (ekflog_common.tail - 1 > MAX_MSG_LEN) {
		/* New message goes to the beginning of buffer */
		ekflog_common.buff_end = ekflog_common.head;
		ekflog_common.head = 0;
		return 0;
	}

	return -1;
}


int ekflog_write(uint32_t flags, const char *format, ...)
{
	va_list args;
	int ret;
	static unsigned int lp = 0;
	static char buf[MAX_MSG_LEN + 1];

	return 0;

	/* Log call with flags that are not enabled is not an error */
	if ((flags & ekflog_common.logFlags) == 0) {
		return 0;
	}

	pthread_mutex_lock(&ekflog_common.lock);

	lp++;

	if (ekflog_slotGet() != 0) {
		if ((ekflog_common.logFlags & EKFLOG_STRICT_MODE) != 0) {
			/* Waiting for a place in buff */
			do {
				pthread_cond_wait(&ekflog_common.queueEvent, &ekflog_common.lock);
			} while (ekflog_slotGet() != 0);
		}
		else {
			/* Drop the log */
			ekflog_common.lost++;
			pthread_mutex_unlock(&ekflog_common.lock);
			return -1;
		}
	}

	va_start(args, format);
	ret = vsnprintf(buf, MAX_MSG_LEN + 1 - 10, format, args);
	va_end(args);

	ret = snprintf(&ekflog_common.buff[ekflog_common.head], MAX_MSG_LEN + 1, "%x,%s", lp, buf);

	if (ret < 0) {
		fprintf(stderr, "ekflogs: cannot log an event\n");
	}
	else {
		ekflog_common.head += ret;
	}

	pthread_cond_signal(&ekflog_common.queueEvent);
	pthread_mutex_unlock(&ekflog_common.lock);

	return ret < 0 ? -1 : 0;
}


int ekflog_writeBin(uint32_t flags, void *msg_buf, size_t buf_len)
{
	/* Log call with flags that are not enabled is not an error */
	if ((flags & ekflog_common.logFlags) == 0) {
		return 0;
	}

	pthread_mutex_lock(&ekflog_common.lock);

	if (ekflog_slotGet() != 0) {
		if ((ekflog_common.logFlags & EKFLOG_STRICT_MODE) != 0) {
			/* Waiting for a place in buff */
			do {
				pthread_cond_wait(&ekflog_common.queueEvent, &ekflog_common.lock);
			} while (ekflog_slotGet() != 0);
		}
		else {
			/* Dropping the log */
			ekflog_common.lost++;
			pthread_mutex_unlock(&ekflog_common.lock);
			return -1;
		}
	}

	memcpy(&ekflog_common.buff[ekflog_common.head], msg_buf, sizeof(char) * buf_len);
	ekflog_common.head += MAX_MSG_LEN;

	pthread_cond_signal(&ekflog_common.queueEvent);
	pthread_mutex_unlock(&ekflog_common.lock);

	return 0;
}


static int ekflog_writeLogPrefix(char *msg_buf, char msgType, time_t timestamp)
{
	uint64_t time = (uint64_t)timestamp;

	ekflog_common.logCnt++;

	memcpy(msg_buf, &ekflog_common.logCnt, sizeof(ekflog_common.logCnt));
	msg_buf[sizeof(ekflog_common.logCnt)] = msgType;
	memcpy(&msg_buf[sizeof(ekflog_common.logCnt) + sizeof(msgType)], &time, sizeof(time));

	return PREFIX_LOG_LEN;
}


static void ekflog_addLogField(char* msg_buf, int *msgEnd,const void *data, size_t dataSize)
{
	memcpy(&msg_buf[*msgEnd], data, dataSize);
	msgEnd += dataSize;
}


extern int ekflog_timeWrite(time_t timestamp)
{
	char msg_buf[PREFIX_LOG_LEN];
	int msg_len = ekflog_writeLogPrefix(msg_buf, 'T', timestamp);

	return ekflog_writeBin(EKFLOG_TIME, msg_buf, msg_len);
}


extern int ekflog_senscImuWrite(const sensor_event_t *accEvt, const sensor_event_t *gyrEvt, const sensor_event_t *magEvt)
{
	char msg_buf[PREFIX_LOG_LEN + sizeof(accel_data_t) + sizeof(gyro_data_t) + sizeof(mag_data_t)];
	int msg_len = ekflog_writeLogPrefix(msg_buf, 'I', accEvt->timestamp);

	ekflog_addLogField(msg_buf, &msg_len, &accEvt->accels.accelX, sizeof(accEvt->accels.accelX));
	ekflog_addLogField(msg_buf, &msg_len, &accEvt->accels.accelY, sizeof(accEvt->accels.accelY));
	ekflog_addLogField(msg_buf, &msg_len, &accEvt->accels.accelZ, sizeof(accEvt->accels.accelZ));

	ekflog_addLogField(msg_buf, &msg_len, &accEvt->gyro.gyroX, sizeof(accEvt->gyro.gyroX));
	ekflog_addLogField(msg_buf, &msg_len, &accEvt->gyro.gyroX, sizeof(accEvt->gyro.gyroX));
	ekflog_addLogField(msg_buf, &msg_len, &accEvt->gyro.gyroX, sizeof(accEvt->gyro.gyroX));

	ekflog_addLogField(msg_buf, &msg_len, &accEvt->gyro.dAngleX, sizeof(accEvt->gyro.dAngleX));
	ekflog_addLogField(msg_buf, &msg_len, &accEvt->gyro.dAngleY, sizeof(accEvt->gyro.dAngleY));
	ekflog_addLogField(msg_buf, &msg_len, &accEvt->gyro.dAngleZ, sizeof(accEvt->gyro.dAngleZ));

	ekflog_addLogField(msg_buf, &msg_len, &accEvt->mag.magX, sizeof(accEvt->mag.magX));
	ekflog_addLogField(msg_buf, &msg_len, &accEvt->mag.magY, sizeof(accEvt->mag.magY));
	ekflog_addLogField(msg_buf, &msg_len, &accEvt->mag.magZ, sizeof(accEvt->mag.magZ));

	return ekflog_writeBin(EKFLOG_SENSC, msg_buf, msg_len);
}


extern int ekflog_senscGpsWrite(const sensor_event_t *gpsEvt)
{
	char msg_buf[PREFIX_LOG_LEN + sizeof(gps_data_t)];
	int msg_len = ekflog_writeLogPrefix(msg_buf, 'P', gpsEvt->timestamp);

	ekflog_addLogField(msg_buf, &msg_len, &gpsEvt->gps.lat, sizeof(gpsEvt->gps.lat));
	ekflog_addLogField(msg_buf, &msg_len, &gpsEvt->gps.lon, sizeof(gpsEvt->gps.lon));
	ekflog_addLogField(msg_buf, &msg_len, &gpsEvt->gps.alt, sizeof(gpsEvt->gps.alt));

	ekflog_addLogField(msg_buf, &msg_len, &gpsEvt->gps.eph, sizeof(gpsEvt->gps.eph));
	ekflog_addLogField(msg_buf, &msg_len, &gpsEvt->gps.evel, sizeof(gpsEvt->gps.evel));

	ekflog_addLogField(msg_buf, &msg_len, &gpsEvt->gps.fix, sizeof(gpsEvt->gps.fix));

	ekflog_addLogField(msg_buf, &msg_len, &gpsEvt->gps.satsNb, sizeof(gpsEvt->gps.satsNb));

	ekflog_addLogField(msg_buf, &msg_len, &gpsEvt->gps.velNorth, sizeof(gpsEvt->gps.velNorth));
	ekflog_addLogField(msg_buf, &msg_len, &gpsEvt->gps.velEast, sizeof(gpsEvt->gps.velEast));
	ekflog_addLogField(msg_buf, &msg_len, &gpsEvt->gps.velDown, sizeof(gpsEvt->gps.velDown));

	return ekflog_writeBin(EKFLOG_SENSC, msg_buf, msg_len);
}


extern int ekflog_senscBaroWrite(const sensor_event_t *baroEvt)
{
	char msg_buf[PREFIX_LOG_LEN + sizeof(baro_data_t)];
	int msg_len = ekflog_writeLogPrefix(msg_buf, 'B', baroEvt->timestamp);

	ekflog_addLogField(msg_buf, &msg_len, &baroEvt->baro.pressure, sizeof(baroEvt->baro.pressure));
	ekflog_addLogField(msg_buf, &msg_len, &baroEvt->baro.temp, sizeof(baroEvt->baro.temp));

	return ekflog_writeBin(EKFLOG_SENSC, msg_buf, msg_len);
}


int ekflog_done(void)
{
	int err = 0;

	if (ekflog_common.logsEnabled == false) {
		return 0;
	}

	pthread_mutex_lock(&ekflog_common.lock);
	ekflog_common.run = 0;
	pthread_mutex_unlock(&ekflog_common.lock);

	pthread_cond_signal(&ekflog_common.queueEvent);

	if (pthread_join(ekflog_common.tid, NULL) != 0) {
		fprintf(stderr, "ekflog: cannot join logging thread\n");
		return -1;
	}

	if (ekflog_common.lost > 0) {
		// fprintf(ekflog_common.file, "\nWARNING\n");
		// fprintf(ekflog_common.file, "Logging module have missed some of the messages\n");
		// fprintf(ekflog_common.file, "Number of lost logs: %d\n", ekflog_common.lost);
		// fprintf(ekflog_common.file, "It is possible to use EKFLOG_STRICT_MODE to avoid this behaviour\n");

		printf("Lost logs: %d\n", ekflog_common.lost);
	}

	err |= fclose(ekflog_common.file);
	err |= pthread_mutex_destroy(&ekflog_common.lock);
	err |= pthread_cond_destroy(&ekflog_common.queueEvent);

	return err;
}


int ekflog_init(const char *path, uint32_t flags)
{
	pthread_attr_t attr;
	int ret;

	if (flags == 0) {
		ekflog_common.logsEnabled = false;
		return 0;
	}

	if (path == NULL) {
		fprintf(stderr, "ekflog: wrong file path\n");
		return -1;
	}

	ekflog_common.file = fopen(path, "wb");
	if (ekflog_common.file == NULL) {
		fprintf(stderr, "ekflog: can`t open %s to write\n", path);
		return -1;
	}

	// if ((flags & EKFLOG_STRICT_MODE) == 0) {
	// 	/* Line buffering results in the smallest max time between loops */
	// 	if (setvbuf(ekflog_common.file, NULL, _IOLBF, MAX_MSG_LEN + 1) != 0) {
	// 		fclose(ekflog_common.file);
	// 		fprintf(stderr, "ekflog: can't set file buffering\n");
	// 		return -1;
	// 	}
	// }
	// else {
	// 	/* If strict mode is not enabled, then module is tuned for minimizing lost logs */

	// }

	// if (setvbuf(ekflog_common.file, NULL, _IOFBF, 512) != 0) {
	// 	fclose(ekflog_common.file);
	// 	fprintf(stderr, "ekflog: can't set file buffering\n");
	// 	return -1;
	// }


	if (pthread_mutex_init(&ekflog_common.lock, NULL) != 0) {
		fprintf(stderr, "ekflog: cannot initialize lock\n");
		fclose(ekflog_common.file);
		return -1;
	}

	if (pthread_cond_init(&ekflog_common.queueEvent, NULL) != 0) {
		fprintf(stderr, "ekflog: cannot initialize conditional variable\n");
		fclose(ekflog_common.file);
		pthread_mutex_destroy(&ekflog_common.lock);
		return -1;
	}

	ekflog_common.logFlags = flags;
	ekflog_common.head = -1;
	ekflog_common.tail = -1;
	ekflog_common.buff_end = BUFF_LEN;
	ekflog_common.run = 1;
	ekflog_common.lost = 0;
	ekflog_common.logsEnabled = true;
	ekflog_common.logCnt = 0;

	if (pthread_attr_init(&attr) != 0) {
		fprintf(stderr, "ekflog: cannot initialize conditional variable\n");
		fclose(ekflog_common.file);
		pthread_mutex_destroy(&ekflog_common.lock);
		pthread_cond_destroy(&ekflog_common.queueEvent);
		return -1;
	}

/* On Phoenix-RTOS we want to set thread priority */
#ifdef __phoenix__

	if (pthread_attr_setschedparam(&attr, &((struct sched_param) { .sched_priority = PHOENIX_THREAD_PRIO })) != 0) {
		printf("ekflog: cannot set thread priority\n");
		fclose(ekflog_common.file);
		pthread_mutex_destroy(&ekflog_common.lock);
		pthread_cond_destroy(&ekflog_common.queueEvent);
		pthread_attr_destroy(&attr);
		return -1;
	}

#endif

	ret = pthread_create(&ekflog_common.tid, &attr, ekflog_thread, NULL);
	pthread_attr_destroy(&attr);
	if (ret != 0) {
		fprintf(stderr, "ekflog: cannot start a log thread\n");
		fclose(ekflog_common.file);
		pthread_mutex_destroy(&ekflog_common.lock);
		pthread_cond_destroy(&ekflog_common.queueEvent);
		return -1;
	}

	return 0;
}
