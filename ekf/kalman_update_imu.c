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
#include <qdiff.h>

#include "kalman_implem.h"


struct {
	const kalman_init_t *inits;
} imu_common;


/* Returns pointer to passed Z matrix filled with newest measurements vector */
static matrix_t *getMeasurement(matrix_t *Z, matrix_t *state, matrix_t *R, time_t timeStep)
{
	vec_t accel, accelRaw, mag;
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

	Z->data[MGX] = accel.x;
	Z->data[MGY] = accel.y;
	Z->data[MGZ] = accel.z;

	Z->data[MMX] = mag.x;
	Z->data[MMY] = mag.y;
	Z->data[MMZ] = mag.z;

	return Z;
}


static matrix_t *getMeasurementPrediction(matrix_t *state_est, matrix_t *hx, time_t timestep)
{
	/* gravity versor and east versor in NED frame of reference */
	vec_t nedMeasG = { .x = 0, .y = 0, .z = 1 };
	vec_t nedMeasM = { .x = kmn_vecAt(state_est, MX), .y = kmn_vecAt(state_est, MY), .z = kmn_vecAt(state_est, MZ) };

	/* Taking conjugation of quaternion as it should rotate from inertial frame to body frame */
	const quat_t qState = {.a = kmn_vecAt(state_est, QA), .i = -kmn_vecAt(state_est, QB), .j = -kmn_vecAt(state_est, QC), .k = -kmn_vecAt(state_est, QD)};

	matrix_zeroes(hx);

	quat_vecRot(&nedMeasG, &qState);
	quat_vecRot(&nedMeasM, &qState);

	hx->data[MGX] = nedMeasG.x;
	hx->data[MGY] = nedMeasG.y;
	hx->data[MGZ] = nedMeasG.z;

	hx->data[MMX] = nedMeasM.x;
	hx->data[MMY] = nedMeasM.y;
	hx->data[MMZ] = nedMeasM.z;

	return hx;
}


static void getMeasurementPredictionJacobian(matrix_t *H, matrix_t *state, time_t timeStep)
{
	/* This is conjugation of state quaternion as it was used to rotate from earth NED to body frame */
	const quat_t qState = {.a = kmn_vecAt(state, QA), .i = kmn_vecAt(state, QB), .j = kmn_vecAt(state, QC), .k = kmn_vecAt(state, QD)};
	const vec_t mState = { .x = kmn_vecAt(state, MX), .y = kmn_vecAt(state, MY), .z = kmn_vecAt(state, MZ) };

	float dgdqData[3 * 4];
	matrix_t dgdq = { .data = dgdqData, .rows = 3, .cols = 4, .transposed = 0 };

	float dmdqData[3 * 4];
	matrix_t dmdq = { .data = dmdqData, .rows = 3, .cols = 4, .transposed = 0 };

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

	/* Derivative of inversly rotated mag vector with respect to quaternion */
	qvdiff_cqvqDiffQ(&qState, &mState, &dmdq);

	matrix_writeSubmatrix(H, MGX, QA, &dgdq);
	matrix_writeSubmatrix(H, MMX, QA, &dmdq);
}


/* initialization function for IMU update step matrices values */
static void imuUpdateInitializations(matrix_t *H, matrix_t *R)
{
	matrix_zeroes(R);

	/* Noise terms of acceleration measurement */
	*matrix_at(R, MGX, MGX) = *matrix_at(R, MGY, MGY) = *matrix_at(R, MGZ, MGZ) = imu_common.inits->R_astdev * imu_common.inits->R_astdev / (EARTH_G * EARTH_G);

	/* Noise terms of east versor measurement */
	*matrix_at(R, MMX, MMX) = *matrix_at(R, MMY, MMY) = *matrix_at(R, MMZ, MMZ) = imu_common.inits->R_mstdev * imu_common.inits->R_mstdev;
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
