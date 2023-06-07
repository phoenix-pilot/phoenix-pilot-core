/*
 * Phoenix-Pilot
 *
 * Implementation of gettime function for host
 *
 * Copyright 2023 Phoenix Systems
 * Author: Piotr Nieciecki
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#ifndef __phoenix__


#include "gettime.h"
#include <sys/time.h>


int gettime(time_t *raw, time_t *offs)
{
	struct timeval tv;

	if (gettimeofday(&tv, NULL) != 0) {
		return -1;
	}

	*raw = 1000000 * tv.tv_sec + tv.tv_usec;

	return 0;
}


#endif
