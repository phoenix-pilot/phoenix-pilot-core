/*
 * Phoenix-Pilot
 *
 * extended kalman filter
 *
 * auxiliary data filters
 *
 * Copyright 2022 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */


#include <stdio.h>
#include <fcntl.h>
#include <stdlib.h>
#include <vec.h>
#include <errno.h>

#include "filters.h"


#define FLTR_WINDOW_LEN 256

#define GYRO_WINDOW_PATH  "etc/ekf_windows/gyro.txt"
#define ACCEL_WINDOW_PATH "etc/ekf_windows/accel.txt"
#define BARO_WINDOW_PATH  "etc/ekf_windows/baro.txt"


typedef struct {
	float window[FLTR_WINDOW_LEN]; /* filter window values */
	unsigned int len;              /* length of window read from file */
} fltr_t;

struct {
	fltr_t gyroFltr;
	fltr_t accelFltr;
	fltr_t baroFltr;
} fltr_common;


static void fltr_windowVec(vec_t *raw, vec_t *buf, int *bufPos, const float *window, int windowLen)
{
	vec_t part, full = { 0 };
	int i, j;

	if (raw == NULL) {
		for (i = 0; i < windowLen; i++) {
			buf[i].x = buf[i].y = buf[i].z = 0;
		}
		return;
	}

	buf[*bufPos] = *raw;

	for (i = 0; i < windowLen; i++) {
		j = (*bufPos) - i;
		if (j < 0) {
			j += windowLen;
		}

		part = (i > *bufPos) ? buf[windowLen + *bufPos - i] : buf[(*bufPos) - i];
		vec_times(&part, window[windowLen - 1 - i]);
		vec_add(&full, &part);
	}

	/* Cyclic increment */
	*bufPos += 1;
	if ((*bufPos) == windowLen) {
		*bufPos = 0;
	}

	*raw = full;
}


static void fltr_windowScl(float *raw, float *buf, int *bufPos, const float *window, int windowLen)
{
	float part, full = 0;
	int i, j;

	if (raw == NULL) {
		for (i = 0; i < windowLen; i++) {
			buf[i] = 0;
		}
		return;
	}

	buf[*bufPos] = *raw;

	for (i = 0; i < windowLen; i++) {
		j = (*bufPos) - i;
		if (j < 0) {
			j += windowLen;
		}

		part = (i > *bufPos) ? buf[windowLen + *bufPos - i] : buf[(*bufPos) - i];
		part *= window[windowLen - 1 - i];
		full += part;
	}

	/* Cyclic increment */
	*bufPos += 1;
	if ((*bufPos) == windowLen) {
		*bufPos = 0;
	}

	*raw = full;
}


void fltr_accLpf(vec_t *raw)
{
	static vec_t buf[FLTR_WINDOW_LEN] = { 0 };
	static int bufPos = 0;

	fltr_windowVec(raw, buf, &bufPos, fltr_common.accelFltr.window, fltr_common.accelFltr.len);
}


void fltr_vBaroLpf(float *raw)
{
	static float buf[FLTR_WINDOW_LEN] = { 0 };
	static int bufPos = 0;

	fltr_windowScl(raw, buf, &bufPos, fltr_common.baroFltr.window, fltr_common.baroFltr.len);
}


void fltr_gyroLpf(vec_t *raw)
{
	static vec_t buf = { 0 };

	vec_times(&buf, 0.5);
	vec_times(raw, 0.5);
	vec_add(&buf, raw);

	*raw = buf;
}


/*
 * Initializes given `filter` with data from file pointed by `file`.
 * `buf` and `bufSz` usage is getline-like.
 */
static int fltr_windowInit(const char *path, fltr_t *filter, char **buf, size_t *bufSz)
{
	FILE *fp;
	unsigned int i;
	float val, sum = 0;

	fp = fopen(path, "r");
	if (fp == NULL) {
		fprintf(stderr, "filter: failed to open %s\n", path);
		return -1;
	}

	filter->len = 0;
	for (i = 0; i < FLTR_WINDOW_LEN; i++) {
		if (getline(buf, bufSz, fp) < 0) {
			break;
		}

		errno = 0;
		val = strtof(*buf, NULL);
		if (val == 0 && errno != 0) {
			fprintf(stderr, "filter: failed to parse %s@%d: %s\n", path, i, *buf);
			fclose(fp);
			return -1;
		}
		sum += val;
		filter->window[i] = val; /* writing window value */
		filter->len++;
	}

	if (fclose(fp) != 0) {
		fprintf(stderr, "filter: failed to close the file %s\n", path);
		return -1;
	}

	if (filter->len == 0) {
		fprintf(stderr, "filter: failed to read filter %s\n", path);
		return -1;
	}

	/* Filtering window sum must be 1 to not change the amplitude of signal */
	if (sum > 1.01 || sum < 0.99) {
		fprintf(stderr, "filter: ubalanced window %s\n", path);
		return -1;
	}

	return 0;
}


int fltr_init(void)
{
	char *buf;
	size_t bufSz = 64;

	buf = malloc(bufSz);
	if (buf == NULL) {
		fprintf(stderr, "filter: failed to malloc\n");
		return -1;
	}

	if (fltr_windowInit(GYRO_WINDOW_PATH, &fltr_common.gyroFltr, &buf, &bufSz) < 0) {
		fprintf(stderr, "filter: failed to init gyro filter\n");
		free(buf);
		return -1;
	}

	if (fltr_windowInit(ACCEL_WINDOW_PATH, &fltr_common.accelFltr, &buf, &bufSz) < 0) {
		fprintf(stderr, "filter: failed to init accel filter\n");
		free(buf);
		return -1;
	}

	if (fltr_windowInit(BARO_WINDOW_PATH, &fltr_common.baroFltr, &buf, &bufSz) < 0) {
		fprintf(stderr, "filter: failed to init bari filter\n");
		free(buf);
		return -1;
	}

	free(buf);

	return 0;
}
