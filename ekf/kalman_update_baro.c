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

#include <vec.h>
#include <quat.h>
#include <matrix.h>


/* declare static calculation memory bank with matrices for EKF */
DECLARE_STATIC_MEASUREMENT_MATRIX_BANK(STATE_ROWS, BAROMEAS_ROWS)


enum baroDimension { value = 0, dtime = 1 };


/* Rerurns pointer to passed Z matrix filled with newest measurements vector */
static matrix_t *getMeasurement(matrix_t *Z, matrix_t *state, matrix_t *R, time_t timeStep)
{
	static uint64_t lastTstamp = 0;
	static float lastAlt = 0;

	float pressure, temp;
	uint64_t currTstamp;

	/* if there is no pressure measurement available return NULL */
	if (meas_baroGet(&pressure, &temp, &currTstamp) < 0) {
		return NULL;
	}

	if (currTstamp <= lastTstamp) {
		return NULL;
	}

	/* Make measurements negative to account for NED <-> ENU frame conversion */
	Z->data[IMBXZ] = -8453.669 * log(meas_calibPressGet() / pressure); /* Calculate barometric height */
	Z->data[IMBVZ] = -(XZ - lastAlt) / ((float)timeStep / 1000000.f);  /* Calculate derivative of barometric height */

	/* Update filter/derivative variables */
	lastAlt = XZ;

	return Z;
}


static matrix_t *getMeasurementPrediction(matrix_t *state_est, matrix_t *hx, time_t timestep)
{
	matrix_t *state = state_est; /* aliasing for macros usage */

	matrix_zeroes(hx);

	hx->data[IMBXZ] = XZ;
	hx->data[IMBVZ] = VZ;

	return hx;
}


static void getMeasurementPredictionJacobian(matrix_t *H, matrix_t *state, time_t timeStep)
{
	H->data[H->cols * IMBXZ + IXZ] = 1;
	H->data[H->cols * IMBVZ + IVZ] = 1;
}


/* initialization function for barometer update step matrices values */
static void baroUpdateInitializations(matrix_t *H, matrix_t *R, const kalman_init_t *inits)
{
	R->data[R->cols * IMBXZ + IMBXZ] = inits->R_xzcov;
	R->data[R->cols * IMBVZ + IMBVZ] = inits->R_vzcov;
}


void kmn_baroEngInit(update_engine_t *engine, const kalman_init_t *inits)
{
	baroUpdateInitializations(&ekf_H, &ekf_R, inits);

	POPULATE_MEASUREMENT_ENGINE_STATIC_MATRICES(engine)

	engine->getData = getMeasurement;
	engine->getJacobian = getMeasurementPredictionJacobian;
	engine->predictMeasurements = getMeasurementPrediction;
}
