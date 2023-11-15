/*
 * Phoenix-Pilot
 *
 * Generic Phoenix-Pilot binary logging utility
 *
 * The actual implementation of collection logs uses a two buffers.
 * The log producer saves data to the first one as long as there is enough space for new logs.
 * When there is not enough free memory, the producer marks it as dirty and starts using the next one.
 * A separate thread saves dirty buffers to a file and clears dirty flag.
 * This approach allows for collecting logs without blocking the main thread due to potentially
 * time-consuming file writes.
 *
 * Copyright 2023 Phoenix Systems
 * Authors: Piotr Nieciecki, Mateusz Niewiadomski
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 *
 */

#include <stdio.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>

#include "plog.h"

#ifdef LOG_VOL_CHECK
#include "max_logs.h"
#endif

#define BUFFS_CAPACITY 1024 * 8

#define PHOENIX_THREAD_PRIO 4


typedef struct {
	uint8_t buff[BUFFS_CAPACITY];
	bool dirty;
	int size; /* Used capacity */
} plog_buff_t;


struct plog_ctx_t {
	uint32_t logFlags;
	int fd;

	plog_buff_t buffA;
	plog_buff_t buffB;

	plog_buff_t *actBuff;

	pthread_mutex_t lock;
	pthread_cond_t buffEvent;
	pthread_t tid;

	uint32_t logCnt; /* Number of requests to log a value */
	volatile int run;
	bool logsEnabled;

	int lost; /* Number of lost logs */
};


static inline plog_buff_t *plog_nextBufferGet(plog_t * const ctx, plog_buff_t *actBuff)
{
	return (actBuff == &ctx->buffA) ? &ctx->buffB : &ctx->buffA;
}


static void *plog_thread(void *args)
{
	plog_t * const ctx = (plog_t *)args;
	plog_buff_t *outBuff = &ctx->buffA;

	pthread_mutex_lock(&ctx->lock);

#ifdef LOG_VOL_CHECK
	maxLog_start();
	maxLog_sleepReport();
#endif

	do {
		while (outBuff->dirty == false && ctx->run != 0) {
			pthread_cond_wait(&ctx->buffEvent, &ctx->lock);
		}

		while (outBuff->dirty == true) {
			pthread_mutex_unlock(&ctx->lock);

#ifdef LOG_VOL_CHECK
			maxLog_wakeUpReport();
			maxLog_writeReport(outBuff->size);
#endif

			if (write(ctx->fd, outBuff->buff, outBuff->size) != outBuff->size) {
				fprintf(stderr, "ekflog: error while writing to file\n");
			}

#ifdef LOG_VOL_CHECK
			maxLog_sleepReport();
#endif

			pthread_mutex_lock(&ctx->lock);

			outBuff->dirty = false;
			outBuff->size = 0;

			outBuff = plog_nextBufferGet(ctx, outBuff);
			pthread_cond_signal(&ctx->buffEvent);
		}
	} while (ctx->run != 0);

#ifdef LOG_VOL_CHECK
	maxLog_wakeUpReport();
	maxLog_end();
	maxLog_resultsPrint();
#endif

	printf("Logging finished\n");
	printf("Number of logs requests: %d\n", ctx->logCnt);
	printf("Lost logs: %d\n", ctx->lost);

	pthread_mutex_unlock(&ctx->lock);

	return NULL;
}


static bool plog_actBuffWritable(plog_t *ctx)
{
	if (ctx->actBuff->dirty == false) {
		return true;
	}

	if ((ctx->logFlags & PLOG_STRICT_MODE) != 0) {
		/* Waiting for a place to insert logs */
		do {
			pthread_cond_wait(&ctx->buffEvent, &ctx->lock);
		} while (ctx->actBuff->dirty == true);

		return true;
	}

	return false;
}


int plog_write(plog_t * const ctx, const void *msg, size_t msgLen, char logIndicator, time_t timestamp)
{
	size_t remainingBuffSize;

	pthread_mutex_lock(&ctx->lock);

	remainingBuffSize = BUFFS_CAPACITY - ctx->actBuff->size;

	if (remainingBuffSize < msgLen + LOG_PREFIX_SIZE) {
		/* Changing actual buffer for the next one */
		ctx->actBuff->dirty = true;
		ctx->actBuff = plog_nextBufferGet(ctx, ctx->actBuff);
		pthread_cond_signal(&ctx->buffEvent);
	}

	/* Adding log number */
	ctx->logCnt++;

	if (!plog_actBuffWritable(ctx)) {
		/* Dropping the log */
		ctx->lost++;
		pthread_mutex_unlock(&ctx->lock);
		return -1;
	}

	memcpy(ctx->actBuff->buff + ctx->actBuff->size, &ctx->logCnt, sizeof(ctx->logCnt));
	ctx->actBuff->size += sizeof(ctx->logCnt);

	/* Adding log identifier */
	memcpy(ctx->actBuff->buff + ctx->actBuff->size, &logIndicator, sizeof(logIndicator));
	ctx->actBuff->size += sizeof(logIndicator);

	/* Adding timestamp */
	memcpy(ctx->actBuff->buff + ctx->actBuff->size, &timestamp, sizeof(timestamp));
	ctx->actBuff->size += sizeof(timestamp);

	if (msgLen > 0) {
		memcpy(ctx->actBuff->buff + ctx->actBuff->size, msg, msgLen);
		ctx->actBuff->size += msgLen;
	}

	pthread_mutex_unlock(&ctx->lock);

	return 0;
}


int plog_done(plog_t * const ctx)
{
	int err = 0;

	if (ctx->logsEnabled == false) {
		return 0;
	}

	pthread_mutex_lock(&ctx->lock);
	ctx->run = 0;
	ctx->actBuff->dirty = true;
	pthread_mutex_unlock(&ctx->lock);

	pthread_cond_signal(&ctx->buffEvent);

	if (pthread_join(ctx->tid, NULL) != 0) {
		fprintf(stderr, "ekflog: cannot join logging thread\n");
		return -1;
	}

	err |= close(ctx->fd);
	err |= pthread_mutex_destroy(&ctx->lock);
	err |= pthread_cond_destroy(&ctx->buffEvent);

	if (err == 0) {
		free(ctx);
	}

	return err;
}


plog_t *plog_init(const char *path, uint32_t flags)
{
	plog_t *ctx;
	pthread_attr_t attr;
	int ret;

	ctx = (plog_t *)malloc(sizeof(struct plog_ctx_t));
	if (ctx == NULL) {
		return NULL;
	}

	if (path == NULL) {
		fprintf(stderr, "ekflog: wrong file path\n");
		return NULL;
	}

	ctx->fd = open(path, O_WRONLY | O_CREAT | O_TRUNC, S_IRWXU);
	if (ctx->fd == -1) {
		fprintf(stderr, "ekflog: can`t open %s to write\n", path);
		free(ctx);
		return NULL;
	}

	if (pthread_mutex_init(&ctx->lock, NULL) != 0) {
		fprintf(stderr, "ekflog: cannot initialize lock\n");
		close(ctx->fd);
		free(ctx);
		return NULL;
	}

	if (pthread_cond_init(&ctx->buffEvent, NULL) != 0) {
		fprintf(stderr, "ekflog: cannot initialize conditional variable\n");
		close(ctx->fd);
		pthread_mutex_destroy(&ctx->lock);
		free(ctx);
		return NULL;
	}

	if (pthread_attr_init(&attr) != 0) {
		fprintf(stderr, "ekflog: cannot initialize conditional variable\n");
		close(ctx->fd);
		pthread_mutex_destroy(&ctx->lock);
		pthread_cond_destroy(&ctx->buffEvent);
		free(ctx);
		return NULL;
	}

/* On Phoenix-RTOS we want to set thread priority */
#ifdef __phoenix__

	if (pthread_attr_setschedparam(&attr, &((struct sched_param) { .sched_priority = PHOENIX_THREAD_PRIO })) != 0) {
		printf("ekflog: cannot set thread priority\n");
		close(ctx->fd);
		pthread_mutex_destroy(&ctx->lock);
		pthread_cond_destroy(&ctx->buffEvent);
		pthread_attr_destroy(&attr);
		free(ctx);
		return NULL;
	}

#endif

	ctx->logFlags = flags;

	ctx->buffA.dirty = false;
	ctx->buffA.size = 0;

	ctx->buffB.dirty = false;
	ctx->buffB.size = 0;

	ctx->actBuff = &ctx->buffA;
	ctx->logCnt = 0;
	ctx->run = 1;
	ctx->logsEnabled = true;
	ctx->lost = 0;

	ret = pthread_create(&ctx->tid, &attr, plog_thread, (void *)ctx);
	pthread_attr_destroy(&attr);
	if (ret != 0) {
		fprintf(stderr, "ekflog: cannot start a log thread\n");
		close(ctx->fd);
		pthread_mutex_destroy(&ctx->lock);
		pthread_cond_destroy(&ctx->buffEvent);
		free(ctx);
		return NULL;
	}

	return ctx;
}
