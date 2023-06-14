/*
 * Phoenix-Pilot
 *
 * Ekf-specific log module
 *
 * Copyright 2022 Phoenix Systems
 * Author: Mateusz Niewiadomski
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

#define BUFF_LEN    1024
#define MAX_MSG_LEN 128


struct {
	uint32_t logFlags;
	FILE *file;

	char buff[BUFF_LEN];
	int head;
	int tail;
	int buff_end;
	pthread_mutex_t lock;
	pthread_cond_t buff_event;
	pthread_t tid;

	int run;
	bool logs_enabled;

	bool missing_logs;
} ekflog_common;


static void *ekflog_thread(void *args)
{
	int next_msg;

	pthread_mutex_lock(&ekflog_common.lock);

	while (true) {
		while (ekflog_common.head == ekflog_common.tail && ekflog_common.run != 0) {
			pthread_cond_wait(&ekflog_common.buff_event, &ekflog_common.lock);
		}

		if (ekflog_common.head == ekflog_common.tail && ekflog_common.run == 0) {
			break;
		}

		if (ekflog_common.tail == ekflog_common.buff_end) {
			next_msg = 0;
		}
		else {
			next_msg = ekflog_common.tail + 1;
		}

		pthread_mutex_unlock(&ekflog_common.lock);

		if (fputs(&ekflog_common.buff[next_msg], ekflog_common.file) <= 0) {
			fprintf(stderr, "ekflogs: error while writing to file\n");
		}


		pthread_mutex_lock(&ekflog_common.lock);
		ekflog_common.tail = next_msg + strlen(&ekflog_common.buff[next_msg]);
	}

	pthread_mutex_unlock(&ekflog_common.lock);

	return NULL;
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

	if (ekflog_common.head + MAX_MSG_LEN < BUFF_LEN) {
		/* Adding new message after previous one */
		ekflog_common.head++;
	}
	else if (MAX_MSG_LEN <= ekflog_common.tail) {
		/* new message goes to the beginning of buffer */
		ekflog_common.buff_end = ekflog_common.head;
		ekflog_common.head = 0;
	}
	else {
		/* No place in buffer for new message */
		ekflog_common.missing_logs = true;
		pthread_mutex_unlock(&ekflog_common.lock);
		return -1;
	}

	va_start(args, format);
	ret = vsnprintf(&ekflog_common.buff[ekflog_common.head], MAX_MSG_LEN, format, args);
	va_end(args);

	if (ret < 0) {
		fprintf(stderr, "ekflogs: cannot log an event\n");
	}
	else {
		ekflog_common.head += ret;
	}

	pthread_cond_signal(&ekflog_common.buff_event);
	pthread_mutex_unlock(&ekflog_common.lock);

	return ret < 0 ? -1 : 0;
}


int ekflog_done(void)
{
	int err = 0;

	if (ekflog_common.logs_enabled == false) {
		return 0;
	}

	pthread_mutex_lock(&ekflog_common.lock);
	ekflog_common.run = 0;
	pthread_mutex_unlock(&ekflog_common.lock);

	pthread_cond_signal(&ekflog_common.buff_event);

	if (pthread_join(ekflog_common.tid, NULL) != 0) {
		fprintf(stderr, "ekflog: cannot join logging thread\n");
		return -1;
	}

	if (ekflog_common.missing_logs == true) {
		fprintf(ekflog_common.file, "\nWARNING\n");
		fprintf(ekflog_common.file, "Logging module have missed some of the messages\n");
	}

	err |= fclose(ekflog_common.file);
	err |= pthread_mutex_destroy(&ekflog_common.lock);
	err |= pthread_cond_destroy(&ekflog_common.buff_event);

	return err;
}


int ekflog_init(const char *path, uint32_t flags)
{
	pthread_attr_t attr;
	int ret;

	if (flags == 0) {
		ekflog_common.logs_enabled = false;
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

	if (pthread_cond_init(&ekflog_common.buff_event, NULL) != 0) {
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
	ekflog_common.missing_logs = false;
	ekflog_common.logs_enabled = true;

	if (pthread_attr_init(&attr) != 0) {
		fprintf(stderr, "ekflog: cannot initialize conditional variable\n");
		fclose(ekflog_common.file);
		pthread_mutex_destroy(&ekflog_common.lock);
		pthread_cond_destroy(&ekflog_common.buff_event);
		return -1;
	}

	ret = pthread_create(&ekflog_common.tid, &attr, ekflog_thread, NULL);
	pthread_attr_destroy(&attr);
	if (ret != 0) {
		fprintf(stderr, "ekflog: cannot start a log thread\n");
		fclose(ekflog_common.file);
		pthread_mutex_destroy(&ekflog_common.lock);
		pthread_cond_destroy(&ekflog_common.buff_event);
		return -1;
	}

	return 0;
}
