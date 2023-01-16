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
#include <vec.h>

#include "filters.h"

/* 
* Accelerometer data is passed through windowed-sinc FIR filter of following parameters:
 - Cutoff frequency: 15Hz
 - Transition bandwidth: 35Hz
 - Window type: Keiser
 - Stopband attenuation: -40 dB
 - window length: 65

 Source: https://fiiir.com/
*/
#define FLTR_ACCEL_LEN 65

static const float fltr_accWindow[FLTR_ACCEL_LEN] = {
	0.000199210030239271,
	0.000424229608547700,
	0.000723297047419754,
	0.001104142046107193,
	0.001573712960730727,
	0.002137953110816643,
	0.002801586486658635,
	0.003567918798687632,
	0.004438659652200897,
	0.005413771309271377,
	0.006491349018560496,
	0.007667537261972304,
	0.008936485498099311,
	0.010290346094017595,
	0.011719316150862934,
	0.013211723869743890,
	0.014754159000534514,
	0.016331645796411526,
	0.017927855792147184,
	0.019525356664762756,
	0.021105892451041771,
	0.022650689515795901,
	0.024140781913360887,
	0.025557349184979403,
	0.026882059204929122,
	0.028097408442343864,
	0.029187051952518743,
	0.030136115554644904,
	0.030931482990494263,
	0.031562051383240454,
	0.032018949014774860,
	0.032295710296038962,
	0.032388403796089343,
	0.032295710296038962,
	0.032018949014774860,
	0.031562051383240454,
	0.030931482990494263,
	0.030136115554644904,
	0.029187051952518743,
	0.028097408442343864,
	0.026882059204929122,
	0.025557349184979403,
	0.024140781913360887,
	0.022650689515795901,
	0.021105892451041771,
	0.019525356664762756,
	0.017927855792147184,
	0.016331645796411526,
	0.014754159000534514,
	0.013211723869743890,
	0.011719316150862934,
	0.010290346094017595,
	0.008936485498099311,
	0.007667537261972304,
	0.006491349018560496,
	0.005413771309271377,
	0.004438659652200897,
	0.003567918798687632,
	0.002801586486658635,
	0.002137953110816643,
	0.001573712960730727,
	0.001104142046107193,
	0.000723297047419754,
	0.000424229608547700,
	0.000199210030239271
};


#define FLTR_GYRO_LEN 11

static const float fltr_gyroWindow[FLTR_GYRO_LEN] = {
	0.080867462624515052,
	0.086741445655463434,
	0.091476727498003868,
	0.094950333264530992,
	0.097071580192293394,
	0.097784901530386506,
	0.097071580192293394,
	0.094950333264530992,
	0.091476727498003868,
	0.086741445655463434,
	0.080867462624515052
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
	static vec_t buf[FLTR_ACCEL_LEN] = { 0 };
	static int bufPos = 0;

	fltr_windowVec(raw, buf, &bufPos, fltr_accWindow, FLTR_ACCEL_LEN);
}


void fltr_vBaroLpf(float *raw)
{
	static float buf[FLTR_VBARO_LEN] = { 0 };
	static int bufPos = 0;

	fltr_windowScl(raw, buf, &bufPos, fltr_vBaroWindow, FLTR_VBARO_LEN);
}


void fltr_gyroLpf(vec_t *raw)
{
	static vec_t buf[FLTR_GYRO_LEN] = { 0 };
	static int bufPos = 0;

	fltr_windowVec(raw, buf, &bufPos, fltr_gyroWindow, FLTR_GYRO_LEN);
}
<<<<<<< HEAD
=======

>>>>>>> ekf: gyro low pass filter
