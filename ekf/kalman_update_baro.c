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


/* Returns pointer to passed Z matrix filled with newest measurements vector */
static matrix_t *getMeasurement(matrix_t *Z, matrix_t *state, matrix_t *R, time_t timeStep)
{
	float pressure, temp, alt;
	uint64_t currTstamp;

	/* if there is no pressure measurement available return NULL */
	if (meas_baroGet(&pressure, &temp, &currTstamp) < 0) {
		return NULL;
	}

	/* Make measurements negative to account for NED frame convention */
	alt = -8453.669 * log(meas_calibPressGet() / pressure);
	fltr_vBaroLpf(&alt);

	Z->data[MRZ] = alt;

	return Z;
}


static matrix_t *getMeasurementPrediction(matrix_t *state_est, matrix_t *hx, time_t timestep)
{
	hx->data[MRZ] = kmn_vecAt(state_est, RZ);

	return hx;
}


static void getMeasurementPredictionJacobian(matrix_t *H, matrix_t *state, time_t timeStep)
{
	matrix_zeroes(H);

	*matrix_at(H, MRZ, RZ) = 1;
}


/* initialization function for barometer update step matrices values */
static void baroUpdateInitializations(matrix_t *H, matrix_t *R, const kalman_init_t *inits)
{
	matrix_zeroes(R);

	*matrix_at(R, MRZ, MRZ) = inits->R_hstdev * inits->R_hstdev;
}


void kmn_baroEngInit(update_engine_t *engine, const kalman_init_t *inits)
{
	if (!engine->active) {
		return;
	}

	baroUpdateInitializations(&engine->H, &engine->R, inits);

	engine->getData = getMeasurement;
	engine->getJacobian = getMeasurementPredictionJacobian;
	engine->predictMeasurements = getMeasurementPrediction;
}
