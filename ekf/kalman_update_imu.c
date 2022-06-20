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

#include <tools/rotas_dummy.h>
#include <tools/phmatrix.h>

/* DATA MATRICES */

// /* measurements matrix */
// static float Z_data[IMUMEAS_ROWS * STATE_COLS] = { 0 };
// static phmatrix_t Z = { .rows = IMUMEAS_ROWS, .cols = STATE_COLS, .transposed = 0, .data = Z_data };

// /* innovation matrix */
// static float Y_data[IMUMEAS_ROWS * STATE_COLS] = { 0 };
// static phmatrix_t Y = { .rows = IMUMEAS_ROWS, .cols = STATE_COLS, .transposed = 0, .data = Y_data };

// /* innovation covariance matrix */
// static float S_data[IMUMEAS_ROWS * IMUMEAS_ROWS] = { 0 };
// static phmatrix_t S = { .rows = IMUMEAS_ROWS, .cols = IMUMEAS_ROWS, .transposed = 0, .data = S_data };

// /* kalman gain matrix */ /* 16x6 */
// static float K_data[STATE_ROWS * IMUMEAS_ROWS] = { 0 };
// static phmatrix_t K = { .rows = STATE_ROWS, .cols = IMUMEAS_ROWS, .transposed = 0, .data = K_data };


// /* TEMPORAL CALCULATION MATRICES */

// /* square identity matrix */ /* 16x16 */
// static float I_data[STATE_ROWS * STATE_ROWS] = { 0 };
// static phmatrix_t I = { .rows = STATE_ROWS, .cols = STATE_ROWS, .transposed = 0, .data = I_data };

// /* h(x) matrix */
// static float hx_data[IMUMEAS_ROWS * STATE_COLS] = { 0 };
// static phmatrix_t hx = { .rows = IMUMEAS_ROWS, .cols = STATE_COLS, .transposed = 0, .data = hx_data };

// /* temporary matrix #1: small square */
// static float tmp1_data[IMUMEAS_ROWS * IMUMEAS_ROWS] = { 0 };
// static phmatrix_t tmp1 = { .rows = IMUMEAS_ROWS, .cols = IMUMEAS_ROWS, .transposed = 0, .data = tmp1_data };

// /* temporary matrix #2: high rectangular */
// static float tmp2_data[STATE_ROWS * IMUMEAS_ROWS] = { 0 };
// static phmatrix_t tmp2 = { .rows = STATE_ROWS, .cols = IMUMEAS_ROWS, .transposed = 0, .data = tmp2_data };

// /* temporary matrix #3: wide rectangular UNUSED */
// static float tmp3_data[IMUMEAS_ROWS * STATE_ROWS] = { 0 };
// static phmatrix_t tmp3 = { .rows = IMUMEAS_ROWS, .cols = STATE_ROWS, .transposed = 0, .data = tmp3_data };

// /* temporary matrix #4: big square */
// static float tmp4_data[STATE_ROWS * STATE_ROWS] = { 0 };
// static phmatrix_t tmp4 = { .rows = STATE_ROWS, .cols = STATE_ROWS, .transposed = 0, .data = tmp4_data };

// /* temporary matrix #4: state length column vector */
// static float tmp5_data[STATE_ROWS * STATE_COLS] = { 0 };
// static phmatrix_t tmp5 = { .rows = STATE_ROWS, .cols = STATE_COLS, .transposed = 0, .data = tmp5_data };

// /* S inversion buffer */
// static float S_inv_buff[IMUMEAS_ROWS * IMUMEAS_ROWS * 2];
// static unsigned int S_inv_buff_len = IMUMEAS_ROWS * IMUMEAS_ROWS * 2;

/* declare static calculation memory bank with matrices for EKF */
DECLARE_STATIC_MEASUREMENT_MATRIX_BANK(STATE_ROWS, IMUMEAS_ROWS)

extern kalman_init_t init_values;

/* constants */
static vec_t true_g = { .x = 0, .y = 0, .z = 1 };
static vec_t x_versor = { .x = 1, .y = 0, .z = 0 };

vec_t imu_memory[5];
int imu_mem_entry = 0;

static void addMemEntry(vec_t * a)
{
	imu_memory[imu_mem_entry] = *a;

	imu_mem_entry = (imu_mem_entry >= (sizeof(imu_memory) / sizeof(imu_memory[0]) - 1)) ? 0 : imu_mem_entry + 1;
}

static vec_t getMemoryMean(void)
{
	int i;
	vec_t amean = {0};

	for (i = 0; i < (sizeof(imu_memory) / sizeof(imu_memory[0])); i++) {
		amean = vec_add(&amean, &(imu_memory[i]));
	}

	return vec_times(&amean, 1. / (sizeof(imu_memory) / sizeof(imu_memory[0])));
}

/* Rerurns pointer to passed Z matrix filled with newest measurements vector */
static phmatrix_t *getMeasurement(phmatrix_t *Z, phmatrix_t *state, phmatrix_t *R, float dt)
{
	vec_t ameas, wmeas, mmeas;
	vec_t mmeas_unit, ameas_unit;
	vec_t xp, diff;
	quat_t q_est, rot = { .a = qa, .i = qb, .j = qc, .k = qd };
	float err_q_est;
	uint64_t timestamp;

	/* 
		Sensors API wrapper call 

		Accelerations in g
		Angular rates in rad/s
		magnetic flux field in uT 
	*/
	acquireImuMeasurements(&ameas, &wmeas, &mmeas, &timestamp);
	addMemEntry(&ameas);
	ameas = getMemoryMean();
	//printf("ameas: %.3f %.3f %.3f ", ameas.x, ameas.y, ameas.z);

	/* estimate rotation quaternion with assumption that imu is stationary */
	mmeas_unit = mmeas;
	ameas_unit = ameas;
	vec_normalize(&mmeas_unit);
	vec_normalize(&ameas_unit);
	xp = vec_cross(&mmeas_unit, &ameas_unit);
	vec_normalize(&xp);
	q_est = quat_framerot(&ameas_unit, &xp, &true_g, &x_versor, &rot);

	/* rotate measurements of a and w */
	quat_vecrot(&ameas, &rot);
	quat_vecrot(&wmeas, &rot);

	/* calculate quaternion estimation error based on its nonstationarity. Empirically fitted parameters! */
	diff = vec_sub(&true_g, &ameas);
	err_q_est = 0.1 + 100 * vec_len(&diff) * vec_len(&diff) + 10 * vec_len(&wmeas);

	/* trimming data from imu */
	ameas = vec_times(&ameas, EARTH_G);

	/* remove earth acceleration from measurements */
	ameas.z -= EARTH_G;

	//printf("%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f\n", xx, xy, xz, vx, vy, vz, ax, ay, az, ameas.x, ameas.y, ameas.z);

	phx_zeroes(Z);
	Z->data[imax] = ameas.x;
	Z->data[imay] = ameas.y;
	Z->data[imaz] = ameas.z;

	Z->data[imwx] = wmeas.x;
	Z->data[imwy] = wmeas.y;
	Z->data[imwz] = wmeas.z;

	Z->data[immx] = mmeas.x;
	Z->data[immy] = mmeas.y;
	Z->data[immz] = mmeas.z;

	Z->data[imqa] = q_est.a;
	Z->data[imqb] = q_est.i;
	Z->data[imqc] = q_est.j;
	Z->data[imqd] = q_est.k;

	/* update measurement error */
	R->data[R->cols * imqa + imqa] = err_q_est;
	R->data[R->cols * imqb + imqb] = err_q_est;
	R->data[R->cols * imqc + imqc] = err_q_est;
	R->data[R->cols * imqd + imqd] = err_q_est;

	return Z;
}


static phmatrix_t *getMeasurementPrediction(phmatrix_t *state_est, phmatrix_t * hx)
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


static void getMeasurementPredictionJacobian(phmatrix_t *H, phmatrix_t *state, float dt)
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


update_engine_t setupImuUpdateEngine(phmatrix_t *H, phmatrix_t *R)
{
	update_engine_t e;

	imuUpdateInitializations(&ekf_H, &ekf_R);

	POPULATE_MEASUREMENT_ENGINE_STATIC_MATRICES(e)

	e.getData = getMeasurement;
	e.getJacobian = getMeasurementPredictionJacobian;
	e.predictMeasurements = getMeasurementPrediction;

	return e;
}
