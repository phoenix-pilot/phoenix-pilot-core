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

#include "kalman_implem.h"

#include "tools/rotas_dummy.h"
#include "tools/phmatrix.h"

/* declare static calculation memory bank with matrices for EKF */
DECLARE_STATIC_MEASUREMENT_MATRIX_BANK(STATE_ROWS, IMUMEAS_ROWS)

extern kalman_init_t init_values;

/* constants */
static vec_t true_g = { .x = 0, .y = 0, .z = 1 };
static vec_t x_versor = { .x = 1, .y = 0, .z = 0 };

vec_t imu_memory[5];
int imu_mem_entry = 0;


/* Rerurns pointer to passed Z matrix filled with newest measurements vector */
static phmatrix_t *getMeasurement(phmatrix_t *Z, phmatrix_t *state, phmatrix_t *R, time_t timeStep)
{
	vec_t accel, gyro, mag;
	vec_t magFltrd, accelFltrd;
	vec_t xp, gDiff;
	quat_t qEst, rot = { .a = qa, .i = -qb, .j = -qc, .k = -qd };
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
	accelFltrd = (vec_t) { .x = ax, .y = ay, .z = az };
	accelFltrd.z += EARTH_G;
	quat_vecRot(&accelFltrd, &rot);
	gDiff.x = -accelFltrd.x;
	gDiff.y = -accelFltrd.y;
	gDiff.z = EARTH_G - accelFltrd.z;
	quat_cjg(&rot);
	/* calculate quaternion estimation error based on its nonstationarity. Empirically fitted parameters! */
	qEstErr = 0.1 + 100 * vec_len(&gDiff) * vec_len(&gDiff) + 10 * vec_len(&gyro);

	/* calculate rotation quaternion based on current magnetometer reading and ekf filtered acceleration */
	vec_normalize(&magFltrd);
	vec_normalize(&accelFltrd);
	vec_cross(&magFltrd, &accelFltrd, &xp);
	vec_normalize(&xp);
	quat_frameRot(&accelFltrd, &xp, &true_g, &x_versor, &qEst, &rot);

	/* rotate measurements of a and w */
	quat_vecRot(&accel, &rot);
	quat_vecRot(&gyro, &rot);

	/* remove earth acceleration from measurements */
	accel.z -= EARTH_G;

	phx_zeroes(Z);
	Z->data[imax] = accel.x;
	Z->data[imay] = accel.y;
	Z->data[imaz] = accel.z;

	Z->data[imwx] = gyro.x;
	Z->data[imwy] = gyro.y;
	Z->data[imwz] = gyro.z;

	Z->data[immx] = mag.x;
	Z->data[immy] = mag.y;
	Z->data[immz] = mag.z;

	Z->data[imqa] = qEst.a;
	Z->data[imqb] = qEst.i;
	Z->data[imqc] = qEst.j;
	Z->data[imqd] = qEst.k;

	/* update measurement error */
	R->data[R->cols * imqa + imqa] = qEstErr;
	R->data[R->cols * imqb + imqb] = qEstErr;
	R->data[R->cols * imqc + imqc] = qEstErr;
	R->data[R->cols * imqd + imqd] = qEstErr;

	return Z;
}


static phmatrix_t *getMeasurementPrediction(phmatrix_t *state_est, phmatrix_t *hx)
{
	phmatrix_t *state = state_est; /* aliasing for macros usage */
	phx_zeroes(hx);

	hx->data[imax] = ax;
	hx->data[imay] = ay;
	hx->data[imaz] = az;

	hx->data[imwx] = wx;
	hx->data[imwy] = wy;
	hx->data[imwz] = wz;

	hx->data[immx] = mx;
	hx->data[immy] = my;
	hx->data[immz] = mz;

	hx->data[imqa] = qa;
	hx->data[imqb] = qb;
	hx->data[imqc] = qc;
	hx->data[imqd] = qd;

	return hx;
}


static void getMeasurementPredictionJacobian(phmatrix_t *H, phmatrix_t *state, time_t timeStep)
{
	float I33_data[9] = { 0 };
	phmatrix_t I33 = { .rows = 3, .cols = 3, .transposed = 0, .data = I33_data };
	phx_diag(&I33);

	phx_zeroes(H);
	phx_writesubmatrix(H, imax, iax, &I33);
	phx_writesubmatrix(H, imwx, iwx, &I33);
	phx_writesubmatrix(H, immx, imx, &I33);
	/* using I33 and one direct write to write I44 */
	phx_writesubmatrix(H, imqa, iqa, &I33);
	H->data[H->cols * imqd + iqd] = 1;
}


/* initialization function for IMU update step matrices values */
void imuUpdateInitializations(phmatrix_t *H, phmatrix_t *R)
{
	/* init of measurement noise matrix R */
	R->data[R->cols * imax + imax] = init_values.R_acov;
	R->data[R->cols * imay + imay] = init_values.R_acov;
	R->data[R->cols * imaz + imaz] = init_values.R_acov;

	R->data[R->cols * imwx + imwx] = init_values.R_wcov;
	R->data[R->cols * imwy + imwy] = init_values.R_wcov;
	R->data[R->cols * imwz + imwz] = init_values.R_wcov;

	R->data[R->cols * immx + immx] = init_values.R_mcov;
	R->data[R->cols * immy + immy] = init_values.R_mcov;
	R->data[R->cols * immz + immz] = init_values.R_mcov;

	R->data[R->cols * imqa + imqa] = init_values.R_qcov;
	R->data[R->cols * imqb + imqb] = init_values.R_qcov;
	R->data[R->cols * imqc + imqc] = init_values.R_qcov;
	R->data[R->cols * imqd + imqd] = init_values.R_qcov;
}


void kmn_imuEngInit(update_engine_t *engine)
{
	imuUpdateInitializations(&ekf_H, &ekf_R);

	POPULATE_MEASUREMENT_ENGINE_STATIC_MATRICES(engine)

	engine->getData = getMeasurement;
	engine->getJacobian = getMeasurementPredictionJacobian;
	engine->predictMeasurements = getMeasurementPrediction;
}
