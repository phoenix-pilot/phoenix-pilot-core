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
#include "filters.h"

#include <vec.h>
#include <quat.h>
#include <matrix.h>


/* Rerurns pointer to passed Z matrix filled with newest measurements vector */
static matrix_t *getMeasurement(matrix_t *Z, matrix_t *state, matrix_t *R, time_t timeStep)
{
	static uint64_t lastTstamp = 0;
	static float lastAlt = 0;
	static float lpfAltChange = 0;

	float pressure, temp, alt;
	uint64_t currTstamp;

	/* if there is no pressure measurement available return NULL */
	if (meas_baroGet(&pressure, &temp, &currTstamp) < 0) {
		return NULL;
	}

	if (currTstamp <= lastTstamp) {
		return NULL;
	}

	/* Make measurements negative to account for NED frame convention */
	alt = -8453.669 * log(meas_calibPressGet() / pressure);
	lpfAltChange = (alt - lastAlt);
	fltr_vBaroLpf(&lpfAltChange);

	Z->data[MDZ] = lpfAltChange;

	lastAlt = alt;

	return Z;
}


static matrix_t *getMeasurementPrediction(matrix_t *state_est, matrix_t *hx, time_t timestep)
{
	const float dt = (float)timestep / 1000000.f;

	vec_t deltaR = { .x = kmn_vecAt(state_est, VX), .y = kmn_vecAt(state_est, VY), .z = kmn_vecAt(state_est, VZ) };

	vec_times(&deltaR, dt);

	hx->data[MDZ] = deltaR.z;

	return hx;
}


static void getMeasurementPredictionJacobian(matrix_t *H, matrix_t *state, time_t timeStep)
{
	const float dt = (float)timeStep / 1000000.f;

	matrix_zeroes(H);

	/* d(dz)/d(v) calculations */
	*matrix_at(H, MDZ, VZ) = dt;
}


/* initialization function for barometer update step matrices values */
static void baroUpdateInitializations(matrix_t *H, matrix_t *R, const kalman_init_t *inits)
{
	matrix_zeroes(R);
	*matrix_at(R, MDZ, MDZ) = inits->R_dzstdev * inits->R_dzstdev;
}


void kmn_baroEngInit(update_engine_t *engine, const kalman_init_t *inits)
{
	baroUpdateInitializations(&engine->H, &engine->R, inits);

	engine->getData = getMeasurement;
	engine->getJacobian = getMeasurementPredictionJacobian;
	engine->predictMeasurements = getMeasurementPrediction;
}
