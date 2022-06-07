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

#include "kalman_implem.h"

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

/* BARO MEMORY SECTION */
/* barometric height memory */
static float baroMemory[2][6] = { 0 };

static enum baroDimension {value = 0, dt = 1};

static int memoryPoint = 0;

void insertToBaroMemory(float x, float dtBaro)
{
	memoryPoint = (memoryPoint >= sizeof(baroMemory[0]) / sizeof(float)) ? 0 : memoryPoint + 1;
	baroMemory[value][memoryPoint] = x;
	baroMemory[dt][memoryPoint] = dtBaro;
}


float baroMemoryAt(int index, enum baroDimension dimension)
{
	if ((index + memoryPoint) < (sizeof(baroMemory[0]) / sizeof(float))) {
		return baroMemory[dimension][(index + memoryPoint)];
	}
	else {
		return baroMemory[dimension][(index + memoryPoint - (sizeof(baroMemory[0]) / sizeof(float)))];
	}
}


float filterBaroSpeed(void)
{
	int i, filterSpan = sizeof(baroMemory[0]) / sizeof(float);
	float hStart, weights, hEnd, delta, factor = 0.4;

	hStart = hEnd = weights = delta = 0;

	for (i = 0; i < filterSpan; i++) {
		hStart += baroMemoryAt(i, value) * factor;
		hEnd += baroMemoryAt(filterSpan - i, value) * factor;
		weights += factor;
		factor *= factor;

		//printf("%f ", baroMemoryAt(i, value));
		delta += baroMemoryAt(i, dt);
	}
	hStart /= weights;
	hEnd /= weights;

	if (delta > 0.2) {
		return (hEnd - hStart) / (delta/1000000);
	}
	else {
		return 0;
	}
}

/* Rerurns pointer to passed Z matrix filled with newest measurements vector */
static phmatrix_t *get_measurements(phmatrix_t *Z, phmatrix_t *state, phmatrix_t *R, float dt)
{
	float pressure, temp, dtBaroUs;

	/* if there is no pressure measurement available return NULL */
	if (acquireBaroMeasurements(&pressure, &temp, &dtBaroUs) < 0) {
		return NULL;
	}

	insertToBaroMemory(hz, dtBaroUs);

	phx_zeroes(Z);
	Z->data[imhz] = 8453.669 * log(base_pressure / pressure);
	Z->data[imxz] = hz;//0.2 * hz + 0.8 * xz;
	Z->data[imhv] = filterBaroSpeed();
	Z->data[imvz] = hv;

	return Z;
}


static phmatrix_t *get_hx(phmatrix_t *state_est)
{
	phmatrix_t *state = state_est; /* aliasing for macros usage */
	memset(hx.data, 0, sizeof(hx_data));

	hx.data[imhz] = hz;
	hx.data[imxz] = xz;
	hx.data[imhv] = hv;
	hx.data[imvz] = vz;

	return &hx;
}


static void calcBaroJacobian(phmatrix_t *H, phmatrix_t *state, float dt)
{
	H->data[H->cols * imhz + ihz] = 1;
	H->data[H->cols * imxz + ixz] = 1;
	H->data[H->cols * imhv + ihv] = 1;
	H->data[H->cols * imvz + ivz] = 1;
}


update_engine_t setupBaroUpdateEngine(phmatrix_t *H, phmatrix_t *R)
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
	e.getJacobian = calcBaroJacobian;
	e.predictMeasurements = get_hx;

	return e;
}
