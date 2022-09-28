/*
 * Phoenix-RTOS
 *
 * quad-control logging utility
 *
 * Copyright 2022 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <stdio.h>
#include <stdbool.h>

#include "log.h"


struct {
	bool logEnable;
} log_common;


void log_enable(void)
{
	log_common.logEnable = true;
}


void log_disable(void)
{
	log_common.logEnable = false;
}


void log_print(const char *format, ...)
{
	va_list args;

	if (log_common.logEnable) {
		va_start(args, format);
		vprintf(format, args);
		va_end(args);
	}
}
