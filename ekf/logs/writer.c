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

#include <stdio.h>
#include <pthread.h>
#include <stdbool.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>

#define BUFFS_CAPACITY 1024 * 8

#define TIME_LOG_LEN 13
#define IMU_LOG_LEN  55
#define GPS_LOG_LEN  85
#define BARO_LOG_LEN 21

#define PHOENIX_THREAD_PRIO 4


typedef struct {
	uint8_t buff[BUFFS_CAPACITY];
	bool dirty;
	int size; /* Used capacity */
} ekflog_buff_t;


struct {
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

	do {
		while (outBuff->dirty == false && ekflog_common.run != 0) {
			pthread_cond_wait(&ekflog_common.buffEvent, &ekflog_common.lock);
		}

		while (outBuff->dirty == true) {
			pthread_mutex_unlock(&ekflog_common.lock);

			if (write(ekflog_common.fd, outBuff->buff, outBuff->size) != outBuff->size) {
				fprintf(stderr, "ekflog: error while writing to file\n");
			}

			pthread_mutex_lock(&ekflog_common.lock);

			outBuff->dirty = false;
			outBuff->size = 0;

			outBuff = ekflog_nextBufferGet(outBuff);
			pthread_cond_signal(&ekflog_common.buffEvent);
		}
	} while (ekflog_common.run != 0);

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


static int ekflog_write(void *msg, size_t msg_len)
{
	size_t remainingBuffSize;

	pthread_mutex_lock(&ekflog_common.lock);

	remainingBuffSize = BUFFS_CAPACITY - ekflog_common.actBuff->size;

	if (remainingBuffSize < msg_len) {
		/* Changing actual buffer for the next one */
		ekflog_common.actBuff->dirty = true;
		ekflog_common.actBuff = ekflog_nextBufferGet(ekflog_common.actBuff);
		pthread_cond_signal(&ekflog_common.buffEvent);
	}

	if (!ekflog_actBuffWritable()) {
		/* Dropping the log */
		ekflog_common.lost++;
		pthread_mutex_unlock(&ekflog_common.lock);
		return -1;
	}

	memcpy(ekflog_common.actBuff->buff + ekflog_common.actBuff->size, msg, msg_len);
	ekflog_common.actBuff->size += msg_len;

	pthread_mutex_unlock(&ekflog_common.lock);

	return 0;
}


static size_t ekflog_writeLogPrefix(uint8_t *msg_buf, char msgType, time_t timestamp)
{
	uint64_t time = (uint64_t)timestamp;

	ekflog_common.logCnt++;

	memcpy(msg_buf, &ekflog_common.logCnt, sizeof(ekflog_common.logCnt));
	msg_buf[sizeof(ekflog_common.logCnt)] = msgType;
	memcpy(&msg_buf[sizeof(ekflog_common.logCnt) + sizeof(msgType)], &time, sizeof(time));

	return sizeof(ekflog_common.logCnt) + sizeof(msgType) + sizeof(time);
}


static inline size_t ekflog_addLogField(uint8_t *msg_buf, const void *data, size_t dataSize)
{
	memcpy(msg_buf, data, dataSize);
	return dataSize;
}


int ekflog_timeWrite(time_t timestamp)
{
	uint8_t msgBuff[TIME_LOG_LEN];

	/* Log call with flags that are not enabled is not an error */
	if ((ekflog_common.logFlags & EKFLOG_TIME) == 0) {
		return 0;
	}

	ekflog_writeLogPrefix(msgBuff, 'T', timestamp);

	return ekflog_write(msgBuff, sizeof(msgBuff));
}


int ekflog_senscImuWrite(const sensor_event_t *accEvt, const sensor_event_t *gyrEvt, const sensor_event_t *magEvt)
{
	uint8_t msgBuff[IMU_LOG_LEN];
	uint8_t *msgHead = msgBuff;

	/* Log call with flags that are not enabled is not an error */
	if ((ekflog_common.logFlags & EKFLOG_SENSC) == 0) {
		return 0;
	}

	msgHead += ekflog_writeLogPrefix(msgHead, 'I', accEvt->timestamp);

	msgHead += ekflog_addLogField(msgHead, &accEvt->accels.accelX, sizeof(accEvt->accels.accelX));
	msgHead += ekflog_addLogField(msgHead, &accEvt->accels.accelY, sizeof(accEvt->accels.accelY));
	msgHead += ekflog_addLogField(msgHead, &accEvt->accels.accelZ, sizeof(accEvt->accels.accelZ));

	msgHead += ekflog_addLogField(msgHead, &accEvt->gyro.gyroX, sizeof(accEvt->gyro.gyroX));
	msgHead += ekflog_addLogField(msgHead, &accEvt->gyro.gyroY, sizeof(accEvt->gyro.gyroY));
	msgHead += ekflog_addLogField(msgHead, &accEvt->gyro.gyroZ, sizeof(accEvt->gyro.gyroZ));

	msgHead += ekflog_addLogField(msgHead, &accEvt->gyro.dAngleX, sizeof(accEvt->gyro.dAngleX));
	msgHead += ekflog_addLogField(msgHead, &accEvt->gyro.dAngleY, sizeof(accEvt->gyro.dAngleY));
	msgHead += ekflog_addLogField(msgHead, &accEvt->gyro.dAngleZ, sizeof(accEvt->gyro.dAngleZ));

	msgHead += ekflog_addLogField(msgHead, &accEvt->mag.magX, sizeof(accEvt->mag.magX));
	msgHead += ekflog_addLogField(msgHead, &accEvt->mag.magY, sizeof(accEvt->mag.magY));
	msgHead += ekflog_addLogField(msgHead, &accEvt->mag.magZ, sizeof(accEvt->mag.magZ));

	return ekflog_write(msgBuff, sizeof(msgBuff));
}


int ekflog_senscGpsWrite(const sensor_event_t *gpsEvt)
{
	uint8_t msgBuff[GPS_LOG_LEN];
	uint8_t *msgHead = msgBuff;

	/* Log call with flags that are not enabled is not an error */
	if ((ekflog_common.logFlags & EKFLOG_SENSC) == 0) {
		return 0;
	}

	msgHead += ekflog_writeLogPrefix(msgBuff, 'P', gpsEvt->timestamp);

	msgHead += ekflog_addLogField(msgHead, &gpsEvt->gps.lat, sizeof(gpsEvt->gps.lat));
	msgHead += ekflog_addLogField(msgHead, &gpsEvt->gps.lon, sizeof(gpsEvt->gps.lon));
	msgHead += ekflog_addLogField(msgHead, &gpsEvt->gps.alt, sizeof(gpsEvt->gps.alt));

	msgHead += ekflog_addLogField(msgHead, &gpsEvt->gps.utc, sizeof(gpsEvt->gps.utc));

	msgHead += ekflog_addLogField(msgHead, &gpsEvt->gps.hdop, sizeof(gpsEvt->gps.hdop));
	msgHead += ekflog_addLogField(msgHead, &gpsEvt->gps.vdop, sizeof(gpsEvt->gps.vdop));

	msgHead += ekflog_addLogField(msgHead, &gpsEvt->gps.altEllipsoid, sizeof(gpsEvt->gps.altEllipsoid));

	msgHead += ekflog_addLogField(msgHead, &gpsEvt->gps.groundSpeed, sizeof(gpsEvt->gps.groundSpeed));

	msgHead += ekflog_addLogField(msgHead, &gpsEvt->gps.velNorth, sizeof(gpsEvt->gps.velNorth));
	msgHead += ekflog_addLogField(msgHead, &gpsEvt->gps.velEast, sizeof(gpsEvt->gps.velEast));
	msgHead += ekflog_addLogField(msgHead, &gpsEvt->gps.velDown, sizeof(gpsEvt->gps.velDown));

	msgHead += ekflog_addLogField(msgHead, &gpsEvt->gps.eph, sizeof(gpsEvt->gps.eph));
	msgHead += ekflog_addLogField(msgHead, &gpsEvt->gps.epv, sizeof(gpsEvt->gps.epv));
	msgHead += ekflog_addLogField(msgHead, &gpsEvt->gps.evel, sizeof(gpsEvt->gps.evel));

	msgHead += ekflog_addLogField(msgHead, &gpsEvt->gps.heading, sizeof(gpsEvt->gps.heading));
	msgHead += ekflog_addLogField(msgHead, &gpsEvt->gps.headingOffs, sizeof(gpsEvt->gps.headingOffs));
	msgHead += ekflog_addLogField(msgHead, &gpsEvt->gps.headingAccur, sizeof(gpsEvt->gps.headingAccur));

	msgHead += ekflog_addLogField(msgHead, &gpsEvt->gps.satsNb, sizeof(gpsEvt->gps.satsNb));
	msgHead += ekflog_addLogField(msgHead, &gpsEvt->gps.fix, sizeof(gpsEvt->gps.fix));

	return ekflog_write(msgBuff, sizeof(msgBuff));
}


int ekflog_senscBaroWrite(const sensor_event_t *baroEvt)
{
	uint8_t msgBuff[BARO_LOG_LEN];
	uint8_t *msgHead = msgBuff;

	/* Log call with flags that are not enabled is not an error */
	if ((ekflog_common.logFlags & EKFLOG_SENSC) == 0) {
		return 0;
	}

	msgHead += ekflog_writeLogPrefix(msgHead, 'B', baroEvt->timestamp);

	msgHead += ekflog_addLogField(msgHead, &baroEvt->baro.pressure, sizeof(baroEvt->baro.pressure));
	msgHead += ekflog_addLogField(msgHead, &baroEvt->baro.temp, sizeof(baroEvt->baro.temp));

	return ekflog_write(msgBuff, sizeof(msgBuff));
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
