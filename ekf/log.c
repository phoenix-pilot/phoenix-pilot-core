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

#define BUFF_LEN            1024
#define PHOENIX_THREAD_PRIO 5


struct {
	uint32_t logFlags;
	FILE *file;

	char buff[BUFF_LEN];
	int head;
	int tail;
	pthread_mutex_t lock;
	pthread_cond_t queueEvent;
	pthread_t tid;

	volatile int run;
	bool logsEnabled;

	bool lost;
} ekflog_common;


static void *ekflog_thread(void *args)
{
	pthread_mutex_lock(&ekflog_common.lock);

	do {
		while (ekflog_common.head == ekflog_common.tail && ekflog_common.run != 0) {
			pthread_cond_wait(&ekflog_common.queueEvent, &ekflog_common.lock);
		}

		while (ekflog_common.head != ekflog_common.tail) {
			/* Writing thread assumes max message length */
			if (BUFF_LEN - ekflog_common.tail - 2 < MAX_MSG_LEN) {
				ekflog_common.tail = 0;
			}
			else {
				ekflog_common.tail++;
			}

			pthread_mutex_unlock(&ekflog_common.lock);

			if (fputs(&ekflog_common.buff[ekflog_common.tail], ekflog_common.file) <= 0) {
				fprintf(stderr, "ekflogs: error while writing to file\n");
			}

			pthread_mutex_lock(&ekflog_common.lock);
			ekflog_common.tail += strlen(&ekflog_common.buff[ekflog_common.tail]);
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
		ekflog_common.head = 0;
		return 0;
	}

	return -1;
}


int ekflog_write(uint32_t flags, const char *format, ...)
{
	va_list args;
	int ret;

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
			/* Drop the log */
			ekflog_common.lost = true;
			pthread_mutex_unlock(&ekflog_common.lock);
			return -1;
		}
	}

	va_start(args, format);
	ret = vsnprintf(&ekflog_common.buff[ekflog_common.head], MAX_MSG_LEN + 1, format, args);
	va_end(args);

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

	if (ekflog_common.lost == true) {
		fprintf(ekflog_common.file, "\nWARNING\n");
		fprintf(ekflog_common.file, "Logging module have missed some of the messages\n");
		fprintf(ekflog_common.file, "It is possible to use EKFLOG_STRICT_MODE to avoid this behaviour\n");
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

	ekflog_common.file = fopen(path, "w");
	if (ekflog_common.file == NULL) {
		fprintf(stderr, "ekflog: can`t open %s to write\n", path);
		return -1;
	}

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
	ekflog_common.run = 1;
	ekflog_common.lost = false;
	ekflog_common.logsEnabled = true;

	if (pthread_attr_init(&attr) != 0) {
		fprintf(stderr, "ekflog: cannot initialize conditional variable\n");
		fclose(ekflog_common.file);
		pthread_mutex_destroy(&ekflog_common.lock);
		pthread_cond_destroy(&ekflog_common.queueEvent);
		return -1;
	}

/* On Phoenix-RTOS we want to set thread priority */
#ifdef __phoenix__

	if (pthread_attr_setschedparam(&attr, &((struct sched_param) { .sched_priority = 5 })) != 0) {
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
