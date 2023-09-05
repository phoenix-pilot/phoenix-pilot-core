/*
 * Phoenix-Pilot
 *
 * Extended Kalman Filter
 *
 * EKF update engine functions for inertial (IMU) measurements
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

#include <vec.h>
#include <quat.h>
#include <matrix.h>

#include "kalman_implem.h"


struct {
	const kalman_init_t *inits;
} imu_common;


/* Returns pointer to passed Z matrix filled with newest measurements vector */
static matrix_t *getMeasurement(matrix_t *Z, matrix_t *state, matrix_t *R, time_t timeStep)
{
	vec_t accel, accelRaw, mag, nedMeasE;
	float accelSigma, accLen;

	/* Get current sensor readings */
	meas_accelGet(&accel, &accelRaw);
	meas_magGet(&mag);

	/* earth acceleration is measured by accelerometer UPWARD, which in NED is negative */
	accel.x = -accel.x;
	accel.y = -accel.y;
	accel.z = -accel.z;
	accelRaw.x = -accelRaw.x;
	accelRaw.y = -accelRaw.y;
	accelRaw.z = -accelRaw.z;

	/* calculate acceleration uncertainty. Bloat the uncertainty if acceleration value is beyond threshold */
	accelSigma = imu_common.inits->R_astdev * imu_common.inits->R_astdev / EARTH_G * EARTH_G;
	accLen = vec_len(&accel);
	if (fabs(accLen - EARTH_G) > ACC_SIGMA_STEP_THRESHOLD) {
		accelSigma *= ACC_SIGMA_STEP_FACTOR;
	}

	*matrix_at(R, MGX, MGX) = accelSigma;
	*matrix_at(R, MGY, MGY) = accelSigma;
	*matrix_at(R, MGZ, MGZ) = accelSigma;

	/* east versor calculations */
	vec_cross(&accel, &mag, &nedMeasE);
	vec_normalize(&nedMeasE);
	vec_normalize(&accel);

	Z->data[MGX] = accel.x;
	Z->data[MGY] = accel.y;
	Z->data[MGZ] = accel.z;

	Z->data[MEX] = nedMeasE.x;
	Z->data[MEY] = nedMeasE.y;
	Z->data[MEZ] = nedMeasE.z;

	return Z;
}


static matrix_t *getMeasurementPrediction(matrix_t *state_est, matrix_t *hx, time_t timestep)
{
	/* gravity versor and east versor in NED frame of reference */
	vec_t nedMeasG = { .x = 0, .y = 0, .z = 1 };
	vec_t nedMeasE = { .x = 0, .y = 1, .z = 0 };

	/* Taking conjugation of quaternion as it should rotate from inertial frame to body frame */
	const quat_t qState = {.a = kmn_vecAt(state_est, QA), .i = -kmn_vecAt(state_est, QB), .j = -kmn_vecAt(state_est, QC), .k = -kmn_vecAt(state_est, QD)};

	matrix_zeroes(hx);

	quat_vecRot(&nedMeasG, &qState);
	quat_vecRot(&nedMeasE, &qState);

	hx->data[MGX] = nedMeasG.x;
	hx->data[MGY] = nedMeasG.y;
	hx->data[MGZ] = nedMeasG.z;

	hx->data[MEX] = nedMeasE.x;
	hx->data[MEY] = nedMeasE.y;
	hx->data[MEZ] = nedMeasE.z;

	return hx;
}


static void getMeasurementPredictionJacobian(matrix_t *H, matrix_t *state, time_t timeStep)
{
	const quat_t qState = {.a = kmn_vecAt(state, QA), .i = kmn_vecAt(state, QB), .j = kmn_vecAt(state, QC), .k = kmn_vecAt(state, QD)};

	float dgdqData[3 * 4];
	matrix_t dgdq = { .data = dgdqData, .rows = 3, .cols = 4, .transposed = 0 };

	float dedqData[3 * 4];
	matrix_t dedq = { .data = dedqData, .rows = 3, .cols = 4, .transposed = 0 };

	matrix_zeroes(H);

	/* Derivative of rotated earth acceleration with respect to quaternion */
	dgdq.data[5] = dgdq.data[8] = qState.a;
	dgdq.data[2] = -qState.a;
	dgdq.data[3] = dgdq.data[4] = qState.i;
	dgdq.data[9] = -qState.i;
	dgdq.data[7] = qState.j;
	dgdq.data[10] = dgdq.data[0] = -qState.j;
	dgdq.data[1] = dgdq.data[6] = dgdq.data[11] = qState.k;
	matrix_times(&dgdq, 2);

	/* Derivative of rotated east versor with respect to quaternion */
	dedq.data[3] = dedq.data[4] = qState.a;
	dedq.data[9] = -qState.a;
	dedq.data[2] = qState.i;
	dedq.data[5] = dedq.data[8] = -qState.i;
	dedq.data[1] = dedq.data[6] = dedq.data[11] = qState.j;
	dedq.data[0] = dedq.data[10] = qState.k;
	dedq.data[7] = -qState.k;
	matrix_times(&dedq, 2);

	matrix_writeSubmatrix(H, MGX, QA, &dgdq);
	matrix_writeSubmatrix(H, MEX, QA, &dedq);
}


/* initialization function for IMU update step matrices values */
static void imuUpdateInitializations(matrix_t *H, matrix_t *R)
{
	matrix_zeroes(R);

	/* Noise terms of acceleration measurement */
	*matrix_at(R, MGX, MGX) = *matrix_at(R, MGY, MGY) = *matrix_at(R, MGZ, MGZ) = imu_common.inits->R_astdev * imu_common.inits->R_astdev / (EARTH_G * EARTH_G);

	/* Noise terms of east versor measurement */
	*matrix_at(R, MEX, MEX) = *matrix_at(R, MEY, MEY) = *matrix_at(R, MEZ, MEZ) = imu_common.inits->R_mstdev * imu_common.inits->R_mstdev;
}


void kmn_imuEngInit(update_engine_t *engine, const kalman_init_t *inits)
{
	if (!engine->active) {
		return;
	}

	imu_common.inits = inits;

	imuUpdateInitializations(&engine->H, &engine->R);

	engine->getData = getMeasurement;
	engine->getJacobian = getMeasurementPredictionJacobian;
	engine->predictMeasurements = getMeasurementPrediction;
}
