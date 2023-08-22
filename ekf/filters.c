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

typedef struct {
	float window[FLTR_WINDOW_LEN]; /* filter window values */
	unsigned int len;              /* length of window read from file */
} fltr_t;

struct {
	fltr_t gyroFltr;
	fltr_t accelFltr;
	fltr_t baroFltr;
} fltr_common;

/* 
* Accelerometer data is passed through windowed-sinc FIR filter of following parameters:
 - Cutoff frequency: 5z
 - Transition bandwidth: 10Hz
 - Window type: Hammin
 - window length: 87

 Source: https://fiiir.com/
*/
#define FLTR_ACCEL_LEN 155

static const float fltr_accWindow[FLTR_ACCEL_LEN] = {
	-0.000332882541154906,
	-0.000340895797836114,
	-0.000351066436005467,
	-0.000363431334232473,
	-0.000377949604158723,
	-0.000394498751771664,
	-0.000412871633990140,
	-0.000432774259111680,
	-0.000453824472230744,
	-0.000475551558806685,
	-0.000497396791210024,
	-0.000518714934380554,
	-0.000538776717769488,
	-0.000556772271592121,
	-0.000571815516171670,
	-0.000582949483895028,
	-0.000589152544114042,
	-0.000589345492298407,
	-0.000582399455964657,
	-0.000567144561454469,
	-0.000542379297596862,
	-0.000506880504741818,
	-0.000459413910672513,
	-0.000398745128559998,
	-0.000323651026483016,
	-0.000232931373155512,
	-0.000125420660437853,
	0.000000000000000000,
	0.000144391010806394,
	0.000308742560195164,
	0.000493963137077950,
	0.000700868396325029,
	0.000930170478911259,
	0.001182467889257775,
	0.001458236028688313,
	0.001757818479604313,
	0.002081419129776099,
	0.002429095220088173,
	0.002800751392212336,
	0.003196134805068472,
	0.003614831380631536,
	0.004056263230723900,
	0.004519687306969837,
	0.005004195306164022,
	0.005508714853003964,
	0.006032011971546113,
	0.006572694845959745,
	0.007129218860265736,
	0.007699892895855336,
	0.008282886854783683,
	0.008876240366219893,
	0.009477872623105777,
	0.010085593286121221,
	0.010697114382565919,
	0.011310063118831372,
	0.011921995516834760,
	0.012530410777194669,
	0.013132766265117396,
	0.013726493008995705,
	0.014309011596655622,
	0.014877748350069737,
	0.015430151656227591,
	0.015963708329746550,
	0.016475959881742955,
	0.016964518569475773,
	0.017427083102328298,
	0.017861453881801194,
	0.018265547656338237,
	0.018637411475969319,
	0.018975235836901326,
	0.019277366912273376,
	0.019542317772268551,
	0.019768778504581048,
	0.019955625154809399,
	0.020101927415610510,
	0.020206955003326144,
	0.020270182671199059,
	0.020291293819140282,
	0.020270182671199059,
	0.020206955003326144,
	0.020101927415610510,
	0.019955625154809403,
	0.019768778504581048,
	0.019542317772268551,
	0.019277366912273376,
	0.018975235836901326,
	0.018637411475969319,
	0.018265547656338237,
	0.017861453881801197,
	0.017427083102328298,
	0.016964518569475777,
	0.016475959881742955,
	0.015963708329746550,
	0.015430151656227593,
	0.014877748350069737,
	0.014309011596655619,
	0.013726493008995705,
	0.013132766265117399,
	0.012530410777194672,
	0.011921995516834762,
	0.011310063118831372,
	0.010697114382565921,
	0.010085593286121223,
	0.009477872623105779,
	0.008876240366219892,
	0.008282886854783683,
	0.007699892895855338,
	0.007129218860265734,
	0.006572694845959747,
	0.006032011971546116,
	0.005508714853003966,
	0.005004195306164022,
	0.004519687306969837,
	0.004056263230723903,
	0.003614831380631536,
	0.003196134805068471,
	0.002800751392212336,
	0.002429095220088174,
	0.002081419129776099,
	0.001757818479604313,
	0.001458236028688314,
	0.001182467889257775,
	0.000930170478911259,
	0.000700868396325029,
	0.000493963137077951,
	0.000308742560195164,
	0.000144391010806394,
	0.000000000000000000,
	-0.000125420660437853,
	-0.000232931373155512,
	-0.000323651026483016,
	-0.000398745128559998,
	-0.000459413910672513,
	-0.000506880504741818,
	-0.000542379297596862,
	-0.000567144561454470,
	-0.000582399455964657,
	-0.000589345492298406,
	-0.000589152544114042,
	-0.000582949483895028,
	-0.000571815516171671,
	-0.000556772271592121,
	-0.000538776717769488,
	-0.000518714934380554,
	-0.000497396791210024,
	-0.000475551558806685,
	-0.000453824472230744,
	-0.000432774259111680,
	-0.000412871633990140,
	-0.000394498751771664,
	-0.000377949604158723,
	-0.000363431334232473,
	-0.000351066436005467,
	-0.000340895797836114,
	-0.000332882541154906
};


/*
* Barometer speed data is passed through windowed-sinc FIR filter of following parameters:
 - Cutoff frequency: 1Hz
 - Transition bandwidth: 1Hz
 - Window type: Keiser
 - Stopband attenuation: -21 dB
 - window length: 25

 Source: https://fiiir.com/
*/
#define FLTR_VBARO_LEN 25

static const float fltr_vBaroWindow[FLTR_VBARO_LEN] = {
	0.002818592054366249,
	0.009031280354472482,
	0.015862290899262995,
	0.023103874815533786,
	0.030522690608008492,
	0.037869336163470380,
	0.044888815628454863,
	0.051331451628891314,
	0.056963725767400129,
	0.061578528814388821,
	0.065004326755273401,
	0.067112798739478188,
	0.067824575541997806,
	0.067112798739478188,
	0.065004326755273401,
	0.061578528814388821,
	0.056963725767400129,
	0.051331451628891314,
	0.044888815628454863,
	0.037869336163470380,
	0.030522690608008492,
	0.023103874815533786,
	0.015862290899262995,
	0.009031280354472482,
	0.002818592054366249
};


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

		errno = EOK;
		val = strtof(*buf, NULL);
		if (val == 0 && errno != EOK) {
			fprintf(stderr, "filter: failed to parse %s@%d: %s\n", path, i, *buf);
			fclose(fp);
			return -1;
		}
		sum += val;
		filter->window[i] = val; /* writing window value */
		filter->len++;
	}

	if (filter->len == 0) {
		fprintf(stderr, "filter: failed to read filter %s\n", path);
		fclose(fp);
		return -1;
	}

	/* Filtering window sum must be 1 to not change the amplitude of signal */
	if (sum > 1.01 || sum < 0.99) {
		fprintf(stderr, "filter: ubalanced window %s\n", path);
		fclose(fp);
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

	if (fltr_windowInit("/etc/ekf_windows/gyro.txt", &fltr_common.gyroFltr, &buf, &bufSz) < 0) {
		fprintf(stderr, "filter: failed to init gyro filter\n");
		free(buf);
		return -1;
	}

	if (fltr_windowInit("/etc/ekf_windows/accel.txt", &fltr_common.accelFltr, &buf, &bufSz) < 0) {
		fprintf(stderr, "filter: failed to init accel filter\n");
		free(buf);
		return -1;
	}

	if (fltr_windowInit("/etc/ekf_windows/baro.txt", &fltr_common.baroFltr, &buf, &bufSz) < 0) {
		fprintf(stderr, "filter: failed to init bari filter\n");
		free(buf);
		return -1;
	}
}
