/*
 * Phoenix-Pilot
 *
 * extended kalman filter 
 * 
 * update step formulas for gps update
 * 
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
#include "gpsserver.h"

#include <tools/rotas_dummy.h>
#include <tools/phmatrix.h>


/* DATA MATRICES */

/* measurements matrix */
static float Z_data[GPSMEAS_ROWS * STATE_COLS] = { 0 };
static phmatrix_t Z = { .rows = GPSMEAS_ROWS, .cols = STATE_COLS, .transposed = 0, .data = Z_data };

/* innovation matrix */
static float Y_data[GPSMEAS_ROWS * STATE_COLS] = { 0 };
static phmatrix_t Y = { .rows = GPSMEAS_ROWS, .cols = STATE_COLS, .transposed = 0, .data = Y_data };

/* innovation covariance matrix */
static float S_data[GPSMEAS_ROWS * GPSMEAS_ROWS] = { 0 };
static phmatrix_t S = { .rows = GPSMEAS_ROWS, .cols = GPSMEAS_ROWS, .transposed = 0, .data = S_data };

/* kalman gain matrix */ /* 16x6 */
static float K_data[STATE_ROWS * GPSMEAS_ROWS] = { 0 };
static phmatrix_t K = { .rows = STATE_ROWS, .cols = GPSMEAS_ROWS, .transposed = 0, .data = K_data };


/* TEMPORAL CALCULATION MATRICES */

/* square identity matrix */ /* 16x16 */
static float I_data[STATE_ROWS * STATE_ROWS] = { 0 };
static phmatrix_t I = { .rows = STATE_ROWS, .cols = STATE_ROWS, .transposed = 0, .data = I_data };

/* h(x) matrix */
static float hx_data[GPSMEAS_ROWS * STATE_COLS] = { 0 };
static phmatrix_t hx = { .rows = GPSMEAS_ROWS, .cols = STATE_COLS, .transposed = 0, .data = hx_data };

/* temporary matrix #1: small square */
static float tmp1_data[GPSMEAS_ROWS * GPSMEAS_ROWS] = { 0 };
static phmatrix_t tmp1 = { .rows = GPSMEAS_ROWS, .cols = GPSMEAS_ROWS, .transposed = 0, .data = tmp1_data };

/* temporary matrix #2: high rectangular */
static float tmp2_data[STATE_ROWS * GPSMEAS_ROWS] = { 0 };
static phmatrix_t tmp2 = { .rows = STATE_ROWS, .cols = GPSMEAS_ROWS, .transposed = 0, .data = tmp2_data };

/* temporary matrix #3: wide rectangular UNUSED */
static float tmp3_data[GPSMEAS_ROWS * STATE_ROWS] = { 0 };
static phmatrix_t tmp3 = { .rows = GPSMEAS_ROWS, .cols = STATE_ROWS, .transposed = 0, .data = tmp3_data };

/* temporary matrix #4: big square */
static float tmp4_data[STATE_ROWS * STATE_ROWS] = { 0 };
static phmatrix_t tmp4 = { .rows = STATE_ROWS, .cols = STATE_ROWS, .transposed = 0, .data = tmp4_data };

/* temporary matrix #4: state length column vector */
static float tmp5_data[STATE_ROWS * STATE_COLS] = { 0 };
static phmatrix_t tmp5 = { .rows = STATE_ROWS, .cols = STATE_COLS, .transposed = 0, .data = tmp5_data };

/* S inversion buffer */
static float S_inv_buff[GPSMEAS_ROWS * GPSMEAS_ROWS * 2];
static unsigned int S_inv_buff_len = GPSMEAS_ROWS * GPSMEAS_ROWS * 2;


/* Rerurns pointer to passed Z matrix filled with newest measurements vector */
static phmatrix_t *get_measurements(phmatrix_t *Z, phmatrix_t *state, phmatrix_t *R, float dt)
{
	vec_t neu_pos, neu_speed;
	float hdop;

	/* if there is no gps measurement available return NULL */
	if (acquireGpsMeasurement(&neu_pos, &neu_speed, &hdop) < 0) {
		return NULL;
	}
	printf("GPS: x=%f, y=%f, hdop: %f\n", neu_pos.x, neu_pos.y, hdop);
	Z->data[imgpsxx] = neu_pos.x;
	Z->data[imgpsxy] = neu_pos.y;
	Z->data[imgpsvx] = neu_speed.x;
	Z->data[imgpsvy] = neu_speed.y;

	memset(R->data, 0, sizeof(R->rows * R->cols * sizeof(float)));
	R->data[R->cols * imgpsxx + imgpsxx] = 3 * hdop;
	R->data[R->cols * imgpsxy + imgpsxy] = 3 * hdop;
	R->data[R->cols * imgpsvx + imgpsvx] = 2;
	R->data[R->cols * imgpsvy + imgpsvy] = 2;

	return Z;
}


static phmatrix_t *get_hx(phmatrix_t *state_est)
{
	phmatrix_t *state = state_est; /* aliasing for macros usage */
	memset(hx.data, 0, sizeof(hx_data));

	hx.data[imgpsxx] = xx;
	hx.data[imgpsxy] = xy;
	hx.data[imgpsvx] = vx;
	hx.data[imgpsvy] = vy;

	return &hx;
}


static void calcGpsJacobian(phmatrix_t *H, phmatrix_t *state, float dt)
{
	memset(H->data, 0, sizeof(H->rows * H->cols * sizeof(float)));
	H->data[H->cols * imgpsxx + ixx] = 1;
	H->data[H->cols * imgpsxy + ixy] = 1;
	H->data[H->cols * imgpsvx + ivx] = 1;
	H->data[H->cols * imgpsvy + ivy] = 1;
}


update_engine_t setupGpsUpdateEngine(phmatrix_t *H, phmatrix_t *R)
{
	update_engine_t e;

	e.H = H;
	e.R = R;

	e.Z = &Z;
	e.Y = &Y;
	e.S = &S;
	e.K = &K;
	e.I = &I;
	e.hx = &hx;
	e.invBuf = S_inv_buff;
	e.invBufLen = S_inv_buff_len;
	e.tmp1 = &tmp1;
	e.tmp2 = &tmp2;
	e.tmp3 = &tmp3;
	e.tmp4 = &tmp4;
	e.tmp5 = &tmp5;
	e.getData = get_measurements;
	e.getJacobian = calcGpsJacobian;
	e.predictMeasurements = get_hx;

	return e;
}
