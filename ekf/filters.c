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


void fltr_accLpf(vec_t *raw)
{
	static vec_t buf[FLTR_ACCEL_LEN] = { 0 };
	static int bufPos = 0;

	fltr_windowVec(raw, buf, &bufPos, fltr_accWindow, FLTR_ACCEL_LEN);
}
