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

#include <stdio.h>
#include <stdint.h>

#include "log.h"


struct {
	uint32_t logFlags;
	FILE *file;
} ekflog_common;


int ekflog_write(uint32_t flags, const char *format, ...)
{
	va_list args;
	int ret = 0;

	if (ekflog_common.file == NULL) {
		return -1;
	}

	/* Log call with flags that are not enabled is not an error */
	if ((flags & ekflog_common.logFlags) != 0) {
		va_start(args, format);
		ret = vfprintf(ekflog_common.file, format, args);
		va_end(args);
	}

	return ret;
}


int ekflog_done(void)
{
	if (ekflog_common.file != NULL) {
		return fclose(ekflog_common.file);
	}

	return 0;
}


int ekflog_init(const char *path, uint32_t flags)
{
	if (path == NULL) {
		fprintf(stderr, "ekflog: wrong file path\n");
		return -1;
	}

	ekflog_common.file = fopen(path, "w");
	if (ekflog_common.file == NULL) {
		fprintf(stderr, "ekflog: can`t open %s to write\n", path);
		return -1;
	}

	ekflog_common.logFlags = flags;

	return 0;
}
