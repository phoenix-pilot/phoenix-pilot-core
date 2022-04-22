/*
 * Phoenix-Pilot
 *
 * extended kalman filter 
 * 
 * update step formulas 
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
#include <time.h>

#include <sys/msg.h>

#include "kalman.h"

#include <tools/rotas_dummy.h>
#include <tools/phmatrix.h>

/* DATA MATRICES */

/* measurements matrix */
static float Z_data[MEAS_ROWS * STATE_COLS] = { 0 };
static phmatrix_t Z = { .rows = MEAS_ROWS, .cols = STATE_COLS, .transposed = 0, .data = Z_data };

/* innovation matrix */
static float Y_data[MEAS_ROWS * STATE_COLS] = { 0 };
static phmatrix_t Y = { .rows = MEAS_ROWS, .cols = STATE_COLS, .transposed = 0, .data = Y_data };

/* innovation covariance matrix */
static float S_data[MEAS_ROWS * MEAS_ROWS] = { 0 };
static phmatrix_t S = { .rows = MEAS_ROWS, .cols = MEAS_ROWS, .transposed = 0, .data = S_data };

/* kalman gain matrix */ /* 16x6 */
static float K_data[STATE_ROWS * MEAS_ROWS] = { 0 };
static phmatrix_t K = { .rows = STATE_ROWS, .cols = MEAS_ROWS, .transposed = 0, .data = K_data };


/* TEMPORAL CALCULATION MATRICES */

/* square identity matrix */ /* 16x16 */
static float I_data[STATE_ROWS * STATE_ROWS] = { 0 };
static phmatrix_t I = { .rows = STATE_ROWS, .cols = STATE_ROWS, .transposed = 0, .data = I_data };

/* h(x) matrix */
static float hx_data[MEAS_ROWS * STATE_COLS] = { 0 };
static phmatrix_t hx = { .rows = MEAS_ROWS, .cols = STATE_COLS, .transposed = 0, .data = hx_data };

/* temporary matrix #1: small square */
static float tmp1_data[MEAS_ROWS * MEAS_ROWS] = { 0 };
static phmatrix_t tmp1 = { .rows = MEAS_ROWS, .cols = MEAS_ROWS, .transposed = 0, .data = tmp1_data };

/* temporary matrix #2: high rectangular */
static float tmp2_data[STATE_ROWS * MEAS_ROWS] = { 0 };
static phmatrix_t tmp2 = { .rows = STATE_ROWS, .cols = MEAS_ROWS, .transposed = 0, .data = tmp2_data };

/* temporary matrix #3: wide rectangular UNUSED */
static float tmp3_data[MEAS_ROWS * STATE_ROWS] = { 0 };
static phmatrix_t tmp3 = { .rows = MEAS_ROWS, .cols = STATE_ROWS, .transposed = 0, .data = tmp3_data };

/* temporary matrix #4: big square */
static float tmp4_data[STATE_ROWS * STATE_ROWS] = { 0 };
static phmatrix_t tmp4 = { .rows = STATE_ROWS, .cols = STATE_ROWS, .transposed = 0, .data = tmp4_data };

/* temporary matrix #4: big square */
static float tmp5_data[STATE_ROWS * STATE_COLS] = { 0 };
static phmatrix_t tmp5 = { .rows = STATE_ROWS, .cols = STATE_COLS, .transposed = 0, .data = tmp5_data };

/* S inversion buffer */
static float S_inv_buff[MEAS_ROWS * MEAS_ROWS * 2];
static unsigned int S_inv_buff_len = MEAS_ROWS * MEAS_ROWS * 2;


/* Rerurns pointer to passed Z matrix filled with newest measurements vector */
static phmatrix_t *get_measurements(phmatrix_t *Z, phmatrix_t *state)
{
	vec_t *imu_meas;
	vec_t ameas;
	vec_t wmeas;
	quat_t rot = { .a = qa, .i = qb, .j = qc, .k = qd };


	imu_meas = imu_measurements();
	ameas = imu_meas[0];
	wmeas = imu_meas[1];

	quat_vecrot(&ameas, &rot);
	quat_vecrot(&wmeas, &rot);

	/* trimming data from imu */
	ameas = vec_scl(&ameas, EARTH_G);
	wmeas = vec_scl(&wmeas, DEG2RAD);

	/* remove earth acceleration from measurements */
	ameas.z -= EARTH_G;

	phx_zeroes(Z);
	Z->data[imax] = (float)((int)(ameas.x * 100)) / 100;
	Z->data[imay] = (float)((int)(ameas.y * 100)) / 100;
	Z->data[imaz] = (float)((int)(ameas.z * 100)) / 100;
	Z->data[imwx] = wmeas.x;
	Z->data[imwy] = wmeas.y;
	Z->data[imwz] = wmeas.z;

	// phx_print(Z);
	// printf("\n");

	return Z;
}


static phmatrix_t *get_hx(phmatrix_t *state_est)
{
	phmatrix_t *state = state_est; /* aliasing for macros usage */
	memset(hx.data, 0, sizeof(hx_data));

	hx.data[imax] = ax;
	hx.data[imay] = ay;
	hx.data[imaz] = az;
	hx.data[imwx] = wx;
	hx.data[imwy] = wy;
	hx.data[imwz] = wz;

	return &hx;
}


void kalman_update(phmatrix_t *state, phmatrix_t *cov, phmatrix_t *state_est, phmatrix_t *cov_est, phmatrix_t *H, phmatrix_t *R, float dt, int verbose)
{
	/* prepare diag */
	phx_diag(&I);

	/* y_k = z_k - h(x_(k|k-1)) */
	phx_sub(get_measurements(&Z, state), get_hx(state_est), &Y);

	/* S_k = H_k * P_(k|k-1) * transpose(H_k) + R*/
	phx_sadwitch_product(H, cov_est, &S, &tmp3);
	phx_add(&S, R, NULL);

	/* only for debug purposes */
	if (verbose) {
		printf("tmp3:\n");
		phx_print(&tmp3);
		printf("Z:\n");
		phx_print(&Z);
		printf("S:\n");
		phx_print(&S);
		printf("hx:\n");
		phx_print(&hx);
		printf("H:\n");
		phx_print(H);
		printf("cov_est:\n");
		phx_print(cov_est);
	}

	/* K_k = P_(k|k-1) * transpose(H_k) * inverse(S_k) */
	phx_transpose(H);
	phx_product(cov_est, H, &tmp2);
	phx_transpose(H);
	phx_inverse(&S, &tmp1, S_inv_buff, S_inv_buff_len);
	phx_product(&tmp2, &tmp1, &K);

	/* only for debug purposes */
	if (verbose) {
		printf("PkHt:\n");
		phx_print(&tmp2);
		printf("S-1:\n");
		phx_print(&tmp1);
		printf("K:\n");
		phx_print(&K);
	}

	/* x_(k|k) = x_(k|k-1) + K_k * y_k */
	phx_product(&K, &Y, &tmp5);
	phx_add(state_est, &tmp5, state);

	/* P_(k|k) = (I - K_k * H_k) * P_(k|k-1) */
	phx_product(&K, H, &tmp4);
	phx_sub(&I, &tmp4, NULL);
	phx_product(&I, cov_est, cov);
}
