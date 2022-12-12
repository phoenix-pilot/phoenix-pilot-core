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

#include <vec.h>
#include <quat.h>
#include <matrix.h>


/* Rerurns pointer to passed Z matrix filled with newest measurements vector */
static matrix_t *getMeasurement(matrix_t *Z, matrix_t *state, matrix_t *R, time_t timeStep)
{
	vec_t enu_pos, enu_speed;
	float hdop;

	/* if there is no gps measurement available return NULL */
	if (meas_gpsGet(&enu_pos, &enu_speed, &hdop) < 0) {
		return NULL;
	}

	Z->data[IMGPSXX] = enu_pos.x;
	Z->data[IMGPSXY] = enu_pos.y;
	Z->data[IMGPSVX] = enu_speed.x;
	Z->data[IMGPSVY] = enu_speed.y;

	memset(R->data, 0, sizeof(R->rows * R->cols * sizeof(float)));
	R->data[R->cols * IMGPSXX + IMGPSXX] = 3 * hdop;
	R->data[R->cols * IMGPSXY + IMGPSXY] = 3 * hdop;
	R->data[R->cols * IMGPSVX + IMGPSVX] = 2;
	R->data[R->cols * IMGPSVY + IMGPSVY] = 2;

	return Z;
}


static matrix_t *getMeasurementPrediction(matrix_t *state_est, matrix_t *hx, time_t timestep)
{
	matrix_t *state = state_est; /* aliasing for macros usage */
	matrix_zeroes(hx);

	hx->data[IMGPSXX] = XX;
	hx->data[IMGPSXY] = XY;
	hx->data[IMGPSVX] = VX;
	hx->data[IMGPSVY] = VY;

	return hx;
}


static void getMeasurementPredictionJacobian(matrix_t *H, matrix_t *state, time_t timeStep)
{
	memset(H->data, 0, sizeof(H->rows * H->cols * sizeof(float)));
	H->data[H->cols * IMGPSXX + IXX] = 1;
	H->data[H->cols * IMGPSXY + IXY] = 1;
	H->data[H->cols * IMGPSVX + IVX] = 1;
	H->data[H->cols * IMGPSVY + IVY] = 1;
}


static void gpsUpdateInitializations(matrix_t *H, matrix_t *R, const kalman_init_t *inits)
{
	/* covariance is live-updated with each gps measurement */
	return;
}


void kmn_gpsEngInit(update_engine_t *engine, const kalman_init_t *inits)
{
	gpsUpdateInitializations(&engine->H, &engine->R, inits);

	engine->getData = getMeasurement;
	engine->getJacobian = getMeasurementPredictionJacobian;
	engine->predictMeasurements = getMeasurementPrediction;
}
