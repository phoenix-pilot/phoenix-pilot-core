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
#include <time.h>

#include <sys/msg.h>

#include <vec.h>
#include <quat.h>
#include <matrix.h>

#include "kalman_implem.h"

/* declare static calculation memory bank with matrices for EKF */
DECLARE_STATIC_MEASUREMENT_MATRIX_BANK(STATE_ROWS, IMUMEAS_ROWS)

extern kalman_init_t init_values;

static const vec_t nedG = { .x = 0, .y = 0, .z = -1 }; /* earth acceleration versor in NED frame of reference */
static const vec_t nedY = { .x = 0, .y = 1, .z = 0 };  /* earth y versor (east) in NED frame of reference */

vec_t imu_memory[5];
int imu_mem_entry = 0;


/* Rerurns pointer to passed Z matrix filled with newest measurements vector */
static matrix_t *getMeasurement(matrix_t *Z, matrix_t *state, matrix_t *R, time_t timeStep)
{
	vec_t accel, gyro, mag;
	vec_t magFltrd, accelFltrd;
	vec_t bodyY, gDiff;
	quat_t qEst, rot = { .a = QA, .i = -QB, .j = -QC, .k = -QD };
	float qEstErr;
	uint64_t timestamp;

	/* 
		Sensors API wrapper call 

		Accelerations in m/s^2
		Angular rates in rad/s
		magnetic flux field in 10^-7 T
	*/
	meas_imuGet(&accel, &gyro, &mag, &timestamp);

	/* estimate rotation quaternion with assumption that imu is stationary */
	magFltrd = mag;

	/* prepare unrotated accelerometer measurement with earth_g added for prefiltered quaternion estimation */
	/* TODO: we can consider storing direct accel measurements in state vector and rotate it on demand (change in jacobians required!) */
	accelFltrd = (vec_t) { .x = AX, .y = AY, .z = AZ - EARTH_G };
	quat_vecRot(&accelFltrd, &rot);
	gDiff.x = -accelFltrd.x;
	gDiff.y = -accelFltrd.y;
	gDiff.z = accelFltrd.z + EARTH_G;
	quat_cjg(&rot);
	/* calculate quaternion estimation error based on its nonstationarity. Empirically fitted parameters! */
	qEstErr = 0.1 + 100 * vec_len(&gDiff) * vec_len(&gDiff) + 10 * vec_len(&gyro);

	/* calculate rotation quaternion based on current magnetometer reading and ekf filtered acceleration */
	vec_normalize(&magFltrd);
	vec_normalize(&accelFltrd);
	vec_cross(&magFltrd, &accelFltrd, &bodyY);
	vec_normalize(&bodyY);
	quat_frameRot(&accelFltrd, &bodyY, &nedG, &nedY, &qEst, &rot);

	/* rotate measurements of a and w */
	quat_vecRot(&accel, &rot);
	quat_vecRot(&gyro, &rot);

	/* remove earth acceleration from measurements */
	accel.z += EARTH_G;

	matrix_zeroes(Z);
	Z->data[IMAX] = accel.x;
	Z->data[IMAY] = accel.y;
	Z->data[IMAZ] = accel.z;

	Z->data[IMWX] = gyro.x;
	Z->data[IMWY] = gyro.y;
	Z->data[IMWZ] = gyro.z;

	Z->data[IMMX] = mag.x;
	Z->data[IMMY] = mag.y;
	Z->data[IMMZ] = mag.z;

	/*
	* qEst rotates vectors from body frame base to NED base.
	* We store the body frame rotation which is conjugation of qEst.
	*/
	Z->data[IMQA] = qEst.a;
	Z->data[IMQB] = qEst.i;
	Z->data[IMQC] = qEst.j;
	Z->data[IMQD] = qEst.k;

	/* update measurement error */
	R->data[R->cols * IMQA + IMQA] = qEstErr;
	R->data[R->cols * IMQB + IMQB] = qEstErr;
	R->data[R->cols * IMQC + IMQC] = qEstErr;
	R->data[R->cols * IMQD + IMQD] = qEstErr;

	return Z;
}


static matrix_t *getMeasurementPrediction(matrix_t *state_est, matrix_t *hx, time_t timestep)
{
	matrix_t *state = state_est; /* aliasing for macros usage */
	matrix_zeroes(hx);

	hx->data[IMAX] = AX;
	hx->data[IMAY] = AY;
	hx->data[IMAZ] = AZ;

	hx->data[IMWX] = WX;
	hx->data[IMWY] = WY;
	hx->data[IMWZ] = WZ;

	hx->data[IMMX] = MX;
	hx->data[IMMY] = MY;
	hx->data[IMMZ] = MZ;

	hx->data[IMQA] = QA;
	hx->data[IMQB] = QB;
	hx->data[IMQC] = QC;
	hx->data[IMQD] = QD;

	return hx;
}


static void getMeasurementPredictionJacobian(matrix_t *H, matrix_t *state, time_t timeStep)
{
	float I33_data[9] = { 0 };
	matrix_t I33 = { .rows = 3, .cols = 3, .transposed = 0, .data = I33_data };
	matrix_diag(&I33);

	matrix_zeroes(H);
	matrix_writeSubmatrix(H, IMAX, IAX, &I33);
	matrix_writeSubmatrix(H, IMWX, IWX, &I33);
	matrix_writeSubmatrix(H, IMMX, IMX, &I33);
	/* using I33 and one direct write to write I44 */
	matrix_writeSubmatrix(H, IMQA, IQA, &I33);
	H->data[H->cols * IMQD + IQD] = 1;
}


/* initialization function for IMU update step matrices values */
void imuUpdateInitializations(matrix_t *H, matrix_t *R)
{
	/* init of measurement noise matrix R */
	R->data[R->cols * IMAX + IMAX] = init_values.R_acov;
	R->data[R->cols * IMAY + IMAY] = init_values.R_acov;
	R->data[R->cols * IMAZ + IMAZ] = init_values.R_acov;

	R->data[R->cols * IMWX + IMWX] = init_values.R_wcov;
	R->data[R->cols * IMWY + IMWY] = init_values.R_wcov;
	R->data[R->cols * IMWZ + IMWZ] = init_values.R_wcov;

	R->data[R->cols * IMMX + IMMX] = init_values.R_mcov;
	R->data[R->cols * IMMY + IMMY] = init_values.R_mcov;
	R->data[R->cols * IMMZ + IMMZ] = init_values.R_mcov;

	R->data[R->cols * IMQA + IMQA] = init_values.R_qcov;
	R->data[R->cols * IMQB + IMQB] = init_values.R_qcov;
	R->data[R->cols * IMQC + IMQC] = init_values.R_qcov;
	R->data[R->cols * IMQD + IMQD] = init_values.R_qcov;
}


void kmn_imuEngInit(update_engine_t *engine)
{
	imuUpdateInitializations(&ekf_H, &ekf_R);

	POPULATE_MEASUREMENT_ENGINE_STATIC_MATRICES(engine)

	engine->getData = getMeasurement;
	engine->getJacobian = getMeasurementPredictionJacobian;
	engine->predictMeasurements = getMeasurementPrediction;
}
