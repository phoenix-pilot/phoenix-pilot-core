/*
 * Phoenix-Pilot
 *
 * Extended Kalman Filter
 * 
 * EKF update engine functions for barometer measurements
 *
 * Copyright 2022 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <time.h>

#include <sys/msg.h>

#include "kalman_implem.h"

#include "tools/rotas_dummy.h"
#include "tools/phmatrix.h"

/* declare static calculation memory bank with matrices for EKF */
DECLARE_STATIC_MEASUREMENT_MATRIX_BANK(STATE_ROWS, BAROMEAS_ROWS)

extern kalman_init_t init_values;

/* BARO MEMORY SECTION */
/* barometric height memory */
static float baroMemory[2][6] = { 0 };

enum baroDimension { value = 0,
	dtime = 1 };

static int memoryPoint = 0;

static void insertToBaroMemory(float x, float dtBaro)
{
	memoryPoint = (memoryPoint >= sizeof(baroMemory[0]) / sizeof(float)) ? 0 : memoryPoint + 1;
	baroMemory[value][memoryPoint] = x;
	baroMemory[dtime][memoryPoint] = dtBaro;
}


static float baroMemoryAt(int index, enum baroDimension dimension)
{
	if ((index + memoryPoint) < (sizeof(baroMemory[0]) / sizeof(float))) {
		return baroMemory[dimension][(index + memoryPoint)];
	}
	else {
		return baroMemory[dimension][(index + memoryPoint - (sizeof(baroMemory[0]) / sizeof(float)))];
	}
}


static float filterBaroSpeed(void)
{
	int i, filterSpan = sizeof(baroMemory[0]) / sizeof(float);
	float hStart, weights, hEnd, delta, factor = 0.4;

	hStart = hEnd = weights = delta = 0;

	for (i = 0; i < filterSpan; i++) {
		hStart += baroMemoryAt(i, value) * factor;
		hEnd += baroMemoryAt(filterSpan - i, value) * factor;
		weights += factor;
		factor *= factor;

		delta += baroMemoryAt(i, dtime);
	}
	hStart /= weights;
	hEnd /= weights;

	if (delta > 0.2) {
		return (hEnd - hStart) / (delta / 1000000);
	}
	else {
		return 0;
	}
}

/* Rerurns pointer to passed Z matrix filled with newest measurements vector */
static phmatrix_t *getMeasurement(phmatrix_t *Z, phmatrix_t *state, phmatrix_t *R, time_t timeStep)
{
	float pressure, temp;
	uint64_t curr_tstamp;
	static uint64_t last_tstamp;

	/* if there is no pressure measurement available return NULL */
	if (acquireBaroMeasurements(&pressure, &temp, &curr_tstamp) < 0) {
		return NULL;
	}

	if (curr_tstamp <= last_tstamp) {
		return NULL;
	}

	insertToBaroMemory(hz, (float)(curr_tstamp - last_tstamp));
	last_tstamp = curr_tstamp;

	phx_zeroes(Z);
	Z->data[imhz] = 8453.669 * log(base_pressure / pressure);
	Z->data[imxz] = 0.8 * hz + 0.2 * xz;
	Z->data[imhv] = filterBaroSpeed();
	Z->data[imvz] = hv;

	return Z;
}


static phmatrix_t *getMeasurementPrediction(phmatrix_t *state_est, phmatrix_t *hx)
{
	phmatrix_t *state = state_est; /* aliasing for macros usage */
	phx_zeroes(hx);

	hx->data[imhz] = hz;
	hx->data[imxz] = xz;
	hx->data[imhv] = hv;
	hx->data[imvz] = vz;

	return hx;
}


static void getMeasurementPredictionJacobian(phmatrix_t *H, phmatrix_t *state, time_t timeStep)
{
	H->data[H->cols * imhz + ihz] = 1;
	H->data[H->cols * imxz + ixz] = 1;
	H->data[H->cols * imhv + ihv] = 1;
	H->data[H->cols * imvz + ivz] = 1;
}


/* initialization function for barometer update step matrices values */
static void baroUpdateInitializations(phmatrix_t *H, phmatrix_t *R)
{
	//R->data[R->cols * impx + impx] = init_values.R_pcov;
	R->data[R->cols * imhz + imhz] = init_values.R_hcov;
	R->data[R->cols * imxz + imxz] = init_values.R_xzcov;
	R->data[R->cols * imhv + imhv] = init_values.R_hvcov;
	R->data[R->cols * imvz + imvz] = init_values.R_vzcov;
}


update_engine_t setupBaroUpdateEngine(phmatrix_t *H, phmatrix_t *R)
{
	update_engine_t e;

	baroUpdateInitializations(&ekf_H, &ekf_R);

	POPULATE_MEASUREMENT_ENGINE_STATIC_MATRICES(e)

	e.getData = getMeasurement;
	e.getJacobian = getMeasurementPredictionJacobian;
	e.predictMeasurements = getMeasurementPrediction;

	return e;
}
