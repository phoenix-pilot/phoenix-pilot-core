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

/* declare static calculation memory bank with matrices for EKF */
DECLARE_STATIC_MEASUREMENT_MATRIX_BANK(STATE_ROWS, GPSMEAS_ROWS)

extern kalman_init_t init_values;

/* Rerurns pointer to passed Z matrix filled with newest measurements vector */
static phmatrix_t *getMeasurement(phmatrix_t *Z, phmatrix_t *state, phmatrix_t *R, time_t timeStep)
{
	vec_t enu_pos, enu_speed;
	float hdop;

	/* if there is no gps measurement available return NULL */
	if (meas_gpsGet(&enu_pos, &enu_speed, &hdop) < 0) {
		return NULL;
	}
	//printf("GPS: x=%f, y=%f, hdop: %f\n", enu_pos.x, enu_pos.y, hdop);
	Z->data[imgpsxx] = enu_pos.x;
	Z->data[imgpsxy] = enu_pos.y;
	Z->data[imgpsvx] = enu_speed.x;
	Z->data[imgpsvy] = enu_speed.y;

	memset(R->data, 0, sizeof(R->rows * R->cols * sizeof(float)));
	R->data[R->cols * imgpsxx + imgpsxx] = 3 * hdop;
	R->data[R->cols * imgpsxy + imgpsxy] = 3 * hdop;
	R->data[R->cols * imgpsvx + imgpsvx] = 2;
	R->data[R->cols * imgpsvy + imgpsvy] = 2;

	return Z;
}


static phmatrix_t *getMeasurementPrediction(phmatrix_t *state_est, phmatrix_t *hx)
{
	phmatrix_t *state = state_est; /* aliasing for macros usage */
	phx_zeroes(hx);

	hx->data[imgpsxx] = xx;
	hx->data[imgpsxy] = xy;
	hx->data[imgpsvx] = vx;
	hx->data[imgpsvy] = vy;

	return hx;
}


static void getMeasurementPredictionJacobian(phmatrix_t *H, phmatrix_t *state, time_t timeStep)
{
	memset(H->data, 0, sizeof(H->rows * H->cols * sizeof(float)));
	H->data[H->cols * imgpsxx + ixx] = 1;
	H->data[H->cols * imgpsxy + ixy] = 1;
	H->data[H->cols * imgpsvx + ivx] = 1;
	H->data[H->cols * imgpsvy + ivy] = 1;
}


static void gpsUpdateInitializations(phmatrix_t *H, phmatrix_t *R)
{
	/* covariance is live-updated with each gps measurement */
	return;
}


void kmn_gpsEngInit(update_engine_t *engine)
{
	gpsUpdateInitializations(&ekf_H, &ekf_R);

	POPULATE_MEASUREMENT_ENGINE_STATIC_MATRICES(engine)

	engine->getData = getMeasurement;
	engine->getJacobian = getMeasurementPredictionJacobian;
	engine->predictMeasurements = getMeasurementPrediction;
}
