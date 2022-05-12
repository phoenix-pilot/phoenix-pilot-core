/*
 * Phoenix-Pilot
 *
 * extended kalman filter 
 * 
 * update step formulas for barometer update
 * 
 * NOTE: 
 * This file previews the use of multiple update procedures. 
 * There are plans of renaming it to slowUpdate as update matrices of measurments not as frequent as IMU
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

#include "kalman.h"

#include <tools/rotas_dummy.h>
#include <tools/phmatrix.h>

/* DATA MATRICES */

/* measurements matrix */
static float Z_data[BAROMEAS_ROWS * STATE_COLS] = { 0 };
static phmatrix_t Z = { .rows = BAROMEAS_ROWS, .cols = STATE_COLS, .transposed = 0, .data = Z_data };

/* innovation matrix */
static float Y_data[BAROMEAS_ROWS * STATE_COLS] = { 0 };
static phmatrix_t Y = { .rows = BAROMEAS_ROWS, .cols = STATE_COLS, .transposed = 0, .data = Y_data };

/* innovation covariance matrix */
static float S_data[BAROMEAS_ROWS * BAROMEAS_ROWS] = { 0 };
static phmatrix_t S = { .rows = BAROMEAS_ROWS, .cols = BAROMEAS_ROWS, .transposed = 0, .data = S_data };

/* kalman gain matrix */ /* 16x6 */
static float K_data[STATE_ROWS * BAROMEAS_ROWS] = { 0 };
static phmatrix_t K = { .rows = STATE_ROWS, .cols = BAROMEAS_ROWS, .transposed = 0, .data = K_data };


/* TEMPORAL CALCULATION MATRICES */

/* square identity matrix */ /* 16x16 */
static float I_data[STATE_ROWS * STATE_ROWS] = { 0 };
static phmatrix_t I = { .rows = STATE_ROWS, .cols = STATE_ROWS, .transposed = 0, .data = I_data };

/* h(x) matrix */
static float hx_data[BAROMEAS_ROWS * STATE_COLS] = { 0 };
static phmatrix_t hx = { .rows = BAROMEAS_ROWS, .cols = STATE_COLS, .transposed = 0, .data = hx_data };

/* temporary matrix #1: small square */
static float tmp1_data[BAROMEAS_ROWS * BAROMEAS_ROWS] = { 0 };
static phmatrix_t tmp1 = { .rows = BAROMEAS_ROWS, .cols = BAROMEAS_ROWS, .transposed = 0, .data = tmp1_data };

/* temporary matrix #2: high rectangular */
static float tmp2_data[STATE_ROWS * BAROMEAS_ROWS] = { 0 };
static phmatrix_t tmp2 = { .rows = STATE_ROWS, .cols = BAROMEAS_ROWS, .transposed = 0, .data = tmp2_data };

/* temporary matrix #3: wide rectangular UNUSED */
static float tmp3_data[BAROMEAS_ROWS * STATE_ROWS] = { 0 };
static phmatrix_t tmp3 = { .rows = BAROMEAS_ROWS, .cols = STATE_ROWS, .transposed = 0, .data = tmp3_data };

/* temporary matrix #4: big square */
static float tmp4_data[STATE_ROWS * STATE_ROWS] = { 0 };
static phmatrix_t tmp4 = { .rows = STATE_ROWS, .cols = STATE_ROWS, .transposed = 0, .data = tmp4_data };

/* temporary matrix #4: state length column vector */
static float tmp5_data[STATE_ROWS * STATE_COLS] = { 0 };
static phmatrix_t tmp5 = { .rows = STATE_ROWS, .cols = STATE_COLS, .transposed = 0, .data = tmp5_data };

/* S inversion buffer */
static float S_inv_buff[BAROMEAS_ROWS * BAROMEAS_ROWS * 2];
static unsigned int S_inv_buff_len = BAROMEAS_ROWS * BAROMEAS_ROWS * 2;



/* Rerurns pointer to passed Z matrix filled with newest measurements vector */
static phmatrix_t *get_measurements(phmatrix_t *Z, phmatrix_t *state, phmatrix_t *R)
{
	float pressure;

	/* if there is no pressure measurement available return NULL */
	if (acquireBaroMeasurements(&pressure) < 0) {
		return NULL;
	}

	phx_zeroes(Z);
	Z->data[imhz] = 8453.669 * log(base_pressure / pressure);
	Z->data[imxz] = 0.2 * hz + 0.8 * xz;

	return Z;
}


static phmatrix_t *get_hx(phmatrix_t *state_est)
{
	phmatrix_t *state = state_est; /* aliasing for macros usage */
	memset(hx.data, 0, sizeof(hx_data));

	hx.data[imhz] = hz;
	hx.data[imxz] = xz;

	return &hx;
}


int kalman_updateBaro(phmatrix_t *state, phmatrix_t *cov, phmatrix_t *state_est, phmatrix_t *cov_est, phmatrix_t *H, phmatrix_t *R, float dt, int verbose)
{
	/* no new pressure measurement available */
	if (get_measurements(&Z, state, R) == NULL) {
		return -1;
	}

	/* prepare diag */
	phx_diag(&I);

	/* y_k = z_k - h(x_(k|k-1)) */
	phx_sub(&Z, get_hx(state_est), &Y);

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

	return 0;
}
