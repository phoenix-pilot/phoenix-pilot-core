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
#include <pthread.h>
#include <stdbool.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>

#define BUFF_LEN 1024 * 16

/* Actual implementation accepts only buffer with two parts */
#define BUFF_PARTS_NB 2
#define BUFF_PART_LEN (BUFF_LEN / BUFF_PARTS_NB)

#define PHOENIX_THREAD_PRIO 4


/*
 * The actual implementation of collection logs uses a single buffer split into two parts.
 * The log producer saves data to the first part as long as there is enough space for new logs.
 * When there is not enough free memory in the current part, the producer marks it as dirty and
 * starts using the next one.
 *
 * A separate thread saves the dirty parts of the buffer to a file and removes this special flag.
 *
 * This approach allows for collecting logs without blocking the EKF thread due to potentially
 * time-consuming file writes.
 */


struct {
	uint32_t logFlags;
	int fd;

	char buff[BUFF_LEN];
	char *buffPartsStarts[BUFF_PARTS_NB];
	char *buffEnds[BUFF_PARTS_NB]; /* Indicates the end of last log in buffer */
	int actBuffPart;

	char msg_buff[MAX_MSG_LEN];

	bool buffsDirty[BUFF_PARTS_NB];

	char *head;

	pthread_mutex_t lock;
	pthread_cond_t buffEvent;
	pthread_t tid;

	uint32_t logCnt;
	volatile int run;
	bool logsEnabled;

	int lost; /* Number of lost logs */
} ekflog_common;


static void *ekflog_thread(void *args)
{
	int bufferToClean = 0, dataLen;

	pthread_mutex_lock(&ekflog_common.lock);

	do {
		while (ekflog_common.buffsDirty[bufferToClean] == false && ekflog_common.run != 0) {
			pthread_cond_wait(&ekflog_common.buffEvent, &ekflog_common.lock);
		}

		while (ekflog_common.buffsDirty[bufferToClean] == true) {
			dataLen = ekflog_common.buffEnds[bufferToClean] - ekflog_common.buffPartsStarts[bufferToClean];
			pthread_mutex_unlock(&ekflog_common.lock);

			if (write(ekflog_common.fd, ekflog_common.buffPartsStarts[bufferToClean], dataLen) != dataLen) {
				fprintf(stderr, "ekflog: error while writing to file\n");
			}

			pthread_mutex_lock(&ekflog_common.lock);

			ekflog_common.buffsDirty[bufferToClean] = false;
			bufferToClean = (bufferToClean + 1) % BUFF_PARTS_NB;
			pthread_cond_signal(&ekflog_common.buffEvent);
		}
	} while (ekflog_common.run != 0);

	pthread_mutex_unlock(&ekflog_common.lock);

	return NULL;
}


int ekflog_write(uint32_t flags, const char *format, ...)
{
	return 0;
}


static int ekflog_actBuffReadyToWrite(void)
{
	if (ekflog_common.buffsDirty[ekflog_common.actBuffPart] == false) {
		return 0;
	}

	if ((ekflog_common.logFlags & EKFLOG_STRICT_MODE) != 0) {
		/* Waiting for a place to insert logs */
		do {
			pthread_cond_wait(&ekflog_common.buffEvent, &ekflog_common.lock);
		} while (ekflog_common.buffsDirty[ekflog_common.actBuffPart] == true);

		return 0;
	}

	return -1;
}


static int ekflog_writeBin(void *msg, size_t msg_len)
{
	size_t remainingBufPartLen;

	pthread_mutex_lock(&ekflog_common.lock);

	remainingBufPartLen = ekflog_common.buffPartsStarts[ekflog_common.actBuffPart] - ekflog_common.head + BUFF_PART_LEN;

	if (remainingBufPartLen < msg_len) {
		/* Changing actual buffer for the next one */
		ekflog_common.buffEnds[ekflog_common.actBuffPart] = ekflog_common.head;
		ekflog_common.buffsDirty[ekflog_common.actBuffPart] = true;
		ekflog_common.actBuffPart = (ekflog_common.actBuffPart + 1) % BUFF_PARTS_NB;
		ekflog_common.head = ekflog_common.buffPartsStarts[ekflog_common.actBuffPart];
		pthread_cond_signal(&ekflog_common.buffEvent);
	}

	if (ekflog_actBuffReadyToWrite() != 0) {
		/* Dropping the log */
		ekflog_common.lost++;
		pthread_mutex_unlock(&ekflog_common.lock);
		return -1;
	}

	memcpy(ekflog_common.head, msg, msg_len);
	ekflog_common.head += msg_len;

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

	return sizeof(uint32_t) + sizeof(char) + sizeof(uint64_t);
}


static inline void ekflog_addLogField(char *msg_buf, int *msgEnd, const void *data, size_t dataSize)
{
	memcpy(&msg_buf[*msgEnd], data, dataSize);
	*msgEnd += dataSize;
}


extern int ekflog_timeWrite(time_t timestamp)
{
	int msg_len;

	/* Log call with flags that are not enabled is not an error */
	if ((ekflog_common.logFlags & EKFLOG_TIME) == 0) {
		return 0;
	}

	msg_len = ekflog_writeLogPrefix(ekflog_common.msg_buff, 'T', timestamp);

	return ekflog_writeBin(ekflog_common.msg_buff, msg_len);
}


extern int ekflog_senscImuWrite(const sensor_event_t *accEvt, const sensor_event_t *gyrEvt, const sensor_event_t *magEvt)
{
	int msg_len;

	/* Log call with flags that are not enabled is not an error */
	if ((ekflog_common.logFlags & EKFLOG_SENSC) == 0) {
		return 0;
	}

	msg_len = ekflog_writeLogPrefix(ekflog_common.msg_buff, 'I', accEvt->timestamp);

	ekflog_addLogField(ekflog_common.msg_buff, &msg_len, &accEvt->accels.accelX, sizeof(accEvt->accels.accelX));
	ekflog_addLogField(ekflog_common.msg_buff, &msg_len, &accEvt->accels.accelY, sizeof(accEvt->accels.accelY));
	ekflog_addLogField(ekflog_common.msg_buff, &msg_len, &accEvt->accels.accelZ, sizeof(accEvt->accels.accelZ));

	ekflog_addLogField(ekflog_common.msg_buff, &msg_len, &accEvt->gyro.gyroX, sizeof(accEvt->gyro.gyroX));
	ekflog_addLogField(ekflog_common.msg_buff, &msg_len, &accEvt->gyro.gyroY, sizeof(accEvt->gyro.gyroY));
	ekflog_addLogField(ekflog_common.msg_buff, &msg_len, &accEvt->gyro.gyroZ, sizeof(accEvt->gyro.gyroZ));

	ekflog_addLogField(ekflog_common.msg_buff, &msg_len, &accEvt->gyro.dAngleX, sizeof(accEvt->gyro.dAngleX));
	ekflog_addLogField(ekflog_common.msg_buff, &msg_len, &accEvt->gyro.dAngleY, sizeof(accEvt->gyro.dAngleY));
	ekflog_addLogField(ekflog_common.msg_buff, &msg_len, &accEvt->gyro.dAngleZ, sizeof(accEvt->gyro.dAngleZ));

	ekflog_addLogField(ekflog_common.msg_buff, &msg_len, &accEvt->mag.magX, sizeof(accEvt->mag.magX));
	ekflog_addLogField(ekflog_common.msg_buff, &msg_len, &accEvt->mag.magY, sizeof(accEvt->mag.magY));
	ekflog_addLogField(ekflog_common.msg_buff, &msg_len, &accEvt->mag.magZ, sizeof(accEvt->mag.magZ));

	return ekflog_writeBin(ekflog_common.msg_buff, msg_len);
}


extern int ekflog_senscGpsWrite(const sensor_event_t *gpsEvt)
{
	int msg_len;

	/* Log call with flags that are not enabled is not an error */
	if ((ekflog_common.logFlags & EKFLOG_SENSC) == 0) {
		return 0;
	}

	msg_len = ekflog_writeLogPrefix(ekflog_common.msg_buff, 'P', gpsEvt->timestamp);

	ekflog_addLogField(ekflog_common.msg_buff, &msg_len, &gpsEvt->gps.lat, sizeof(gpsEvt->gps.lat));
	ekflog_addLogField(ekflog_common.msg_buff, &msg_len, &gpsEvt->gps.lon, sizeof(gpsEvt->gps.lon));
	ekflog_addLogField(ekflog_common.msg_buff, &msg_len, &gpsEvt->gps.alt, sizeof(gpsEvt->gps.alt));

	ekflog_addLogField(ekflog_common.msg_buff, &msg_len, &gpsEvt->gps.eph, sizeof(gpsEvt->gps.eph));
	ekflog_addLogField(ekflog_common.msg_buff, &msg_len, &gpsEvt->gps.evel, sizeof(gpsEvt->gps.evel));

	ekflog_addLogField(ekflog_common.msg_buff, &msg_len, &gpsEvt->gps.fix, sizeof(gpsEvt->gps.fix));
	ekflog_addLogField(ekflog_common.msg_buff, &msg_len, &gpsEvt->gps.satsNb, sizeof(gpsEvt->gps.satsNb));

	ekflog_addLogField(ekflog_common.msg_buff, &msg_len, &gpsEvt->gps.velNorth, sizeof(gpsEvt->gps.velNorth));
	ekflog_addLogField(ekflog_common.msg_buff, &msg_len, &gpsEvt->gps.velEast, sizeof(gpsEvt->gps.velEast));
	ekflog_addLogField(ekflog_common.msg_buff, &msg_len, &gpsEvt->gps.velDown, sizeof(gpsEvt->gps.velDown));

	return ekflog_writeBin(ekflog_common.msg_buff, msg_len);
}


extern int ekflog_senscBaroWrite(const sensor_event_t *baroEvt)
{
	int msg_len;

	/* Log call with flags that are not enabled is not an error */
	if ((ekflog_common.logFlags & EKFLOG_SENSC) == 0) {
		return 0;
	}

	msg_len = ekflog_writeLogPrefix(ekflog_common.msg_buff, 'B', baroEvt->timestamp);

	ekflog_addLogField(ekflog_common.msg_buff, &msg_len, &baroEvt->baro.pressure, sizeof(baroEvt->baro.pressure));
	ekflog_addLogField(ekflog_common.msg_buff, &msg_len, &baroEvt->baro.temp, sizeof(baroEvt->baro.temp));

	return ekflog_writeBin(ekflog_common.msg_buff, msg_len);
}


int ekflog_done(void)
{
	int err = 0;
	int remainingData;

	if (ekflog_common.logsEnabled == false) {
		return 0;
	}

	pthread_mutex_lock(&ekflog_common.lock);
	ekflog_common.run = 0;
	pthread_mutex_unlock(&ekflog_common.lock);

	pthread_cond_signal(&ekflog_common.buffEvent);

	if (pthread_join(ekflog_common.tid, NULL) != 0) {
		fprintf(stderr, "ekflog: cannot join logging thread\n");
		return -1;
	}

	remainingData = ekflog_common.head - ekflog_common.buffPartsStarts[ekflog_common.actBuffPart];

	if (write(ekflog_common.fd, ekflog_common.buffPartsStarts[ekflog_common.actBuffPart], remainingData) != remainingData) {
		fprintf(stderr, "ekflog: error while writing to file\n");
	}

	printf("Logging finished\n");
	printf("Number of logs requests: %d\n", ekflog_common.logCnt);
	printf("Lost logs: %d\n", ekflog_common.lost);

	err |= close(ekflog_common.fd);
	err |= pthread_mutex_destroy(&ekflog_common.lock);
	err |= pthread_cond_destroy(&ekflog_common.buffEvent);

	return err;
}


int ekflog_init(const char *path, uint32_t flags)
{
	pthread_attr_t attr;
	int ret, i;

	if (flags == 0) {
		ekflog_common.logsEnabled = false;
		ekflog_common.logFlags = 0;
		return 0;
	}

	if (path == NULL) {
		fprintf(stderr, "ekflog: wrong file path\n");
		return -1;
	}

	ekflog_common.fd = open(path, O_WRONLY | O_CREAT, S_IRWXU);
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

	for (i = 0; i < BUFF_PARTS_NB; i++) {
		ekflog_common.buffPartsStarts[i] = ekflog_common.buff + BUFF_PART_LEN * i;
		ekflog_common.buffEnds[i] = NULL;
		ekflog_common.buffsDirty[i] = false;
	}

	ekflog_common.actBuffPart = 0;
	ekflog_common.head = ekflog_common.buff;
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
