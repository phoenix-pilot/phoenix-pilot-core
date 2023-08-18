/*
 * Phoenix-Pilot
 *
 * Max log volume estimation
 *
 * Copyright 2023 Phoenix Systems
 * Authors: Piotr Nieciecki
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#include "max_logs.h"

#include <stdio.h>
#include <stdint.h>
#include <sys/time.h>


#define SEC_2_USEC 1000000
#define USEC_2_SEC (1.0f / (float)SEC_2_USEC)

#define KILOBYTE 1000
#define MEGABYTE (1000 * KILOBYTE)
#define GIGABYTE (1000 * MEGABYTE)


static struct {
	uint64_t loggedDataSize;
	uint64_t sleptUSec;

	uint64_t checkStartTime;
	uint64_t sleepStartTime;

	float totalSec;
	float busySec;
} maxLogEs_common;


static uint64_t maxLog_timeGet(void)
{
	struct timeval tv;

	gettimeofday(&tv, NULL);

	return tv.tv_sec * SEC_2_USEC + tv.tv_usec;
}


void maxLog_writeReport(size_t size)
{
	maxLogEs_common.loggedDataSize += size;
}


void maxLog_sleepReport(void)
{
	maxLogEs_common.sleepStartTime = maxLog_timeGet();
}


void maxLog_wakeUpReport(void)
{
	maxLogEs_common.sleptUSec += maxLog_timeGet() - maxLogEs_common.sleepStartTime;
}


void maxLog_start(void)
{
	printf("Max volume check is enabled\n");

	maxLogEs_common.checkStartTime = maxLog_timeGet();

	maxLogEs_common.loggedDataSize = 0;
	maxLogEs_common.sleptUSec = 0;
}


void maxLog_end(void)
{
	maxLogEs_common.totalSec = (maxLog_timeGet() - maxLogEs_common.checkStartTime) * USEC_2_SEC;
	maxLogEs_common.busySec = maxLogEs_common.totalSec - maxLogEs_common.sleptUSec * USEC_2_SEC;
}


static void maxLog_bytesPrint(float bytes)
{
	if (bytes < KILOBYTE) {
		printf("%f B", bytes);
	}
	else if (bytes < MEGABYTE) {
		printf("%f kB", bytes / KILOBYTE);
	}
	else if (bytes < GIGABYTE) {
		printf("%f MB", bytes / MEGABYTE);
	}
	else {
		printf("%f GB", bytes / GIGABYTE);
	}
}


void maxLog_resultsPrint(void)
{
	float currRate = maxLogEs_common.loggedDataSize / maxLogEs_common.totalSec;
	float maxRate = maxLogEs_common.loggedDataSize / maxLogEs_common.busySec;

	printf("\nEkf-logs volume check finished\n\nResults:\n");

	printf(" - total running time: %f s\n", maxLogEs_common.totalSec);
	printf(" - busy time: %f s\n", maxLogEs_common.busySec);

	printf(" - logged data: ");
	maxLog_bytesPrint(maxLogEs_common.loggedDataSize);
	printf("\n");

	printf(" - current rate: ");
	maxLog_bytesPrint(currRate);
	printf("/s\n");

	printf(" - theoretical max rate: ");
	maxLog_bytesPrint(maxRate);
	printf("/s\n\n");
}
