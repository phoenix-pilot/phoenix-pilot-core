/*
 * Phoenix-Pilot
 *
 * Extended Kalman Filter
 *
 * EKF update engine functions for GPS measurements
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
#include "log.h"

#include <vec.h>
#include <quat.h>
#include <matrix.h>


/* Rerurns pointer to passed Z matrix filled with newest measurements vector */
static matrix_t *getMeasurement(matrix_t *Z, matrix_t *state, matrix_t *R, time_t timeStep)
{
	meas_gps_t gpsData;
	time_t timeGps;

	/* if there is no gps measurement available return NULL */
	if (meas_gpsGet(&gpsData, &timeGps) < 0) {
		return NULL;
	}

	/* NED frame of reference is used by both `gpsData` and ekf measurement model */
	Z->data[MGPSRX] = gpsData.pos.x;
	Z->data[MGPSRY] = gpsData.pos.y;

	Z->data[MGPSVX] = gpsData.vel.x;
	Z->data[MGPSVY] = gpsData.vel.y;

	return Z;
}


static matrix_t *getMeasurementPrediction(matrix_t *state_est, matrix_t *hx, time_t timestep)
{
	hx->data[MGPSRX] = kmn_vecAt(state_est, RX);
	hx->data[MGPSRY] = kmn_vecAt(state_est, RY);

	hx->data[MGPSVX] = kmn_vecAt(state_est, VX);
	hx->data[MGPSVY] = kmn_vecAt(state_est, VY);

	return hx;
}


static void getMeasurementPredictionJacobian(matrix_t *H, matrix_t *state, time_t timeStep)
{
	*matrix_at(H, MGPSRX, RX) = 1;
	*matrix_at(H, MGPSRY, RY) = 1;

	*matrix_at(H, MGPSVX, VX) = 1;
	*matrix_at(H, MGPSVY, VY) = 1;

	return;
}


static void gpsUpdateInitializations(matrix_t *H, matrix_t *R, const kalman_init_t *inits)
{
	matrix_zeroes(R);
	matrix_zeroes(H);

	/* Ugly and hardcoded - use hdop from inits */
	*matrix_at(R, MGPSRX, MGPSRX) = inits->R_gpsxstdev * inits->R_gpsxstdev;
	*matrix_at(R, MGPSRY, MGPSRY) = inits->R_gpsxstdev * inits->R_gpsxstdev;

	*matrix_at(R, MGPSVX, MGPSVX) = inits->R_gpsvstdev * inits->R_gpsvstdev;
	*matrix_at(R, MGPSVY, MGPSVY) = inits->R_gpsvstdev * inits->R_gpsvstdev;

	return;
}


void kmn_gpsEngInit(update_engine_t *engine, const kalman_init_t *inits)
{
	gpsUpdateInitializations(&engine->H, &engine->R, inits);

	engine->getData = getMeasurement;
	engine->getJacobian = getMeasurementPredictionJacobian;
	engine->predictMeasurements = getMeasurementPrediction;
}
