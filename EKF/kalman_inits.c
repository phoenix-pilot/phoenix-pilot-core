/*
 * Phoenix-Pilot
 *
 * extended kalman filter 
 * 
 * matrix (memory and values) initialization 
 *
 * Copyright 2022 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "kalman.h"

#include <tools/rotas_dummy.h>
#include <tools/phmatrix.h>

/* state values init */
void init_state_vector(phmatrix_t * state) {
	state->data[ixx] = state->data[ixy] = state->data[ixz] = 0;	/* start position at [0,0,0] */
	state->data[ivx] = state->data[ivy] = state->data[ivz] = 0;	/* start velocity at [0,0,0] */
	state->data[iax] = state->data[iay] = state->data[iaz] = 0;	/* start acceleration at [0,0,0] */
	state->data[iwx] = state->data[iwy] = state->data[iwz] = 0;	/* start angular speed at [0,0,0] */
	state->data[iqa] = 1; 										/* start rotation at identity quaternion */
	state->data[iqb] = state->data[iqc] = state->data[iqd] = 0; /* start rotation at identity quaternion */
}

void init_cov_vector(phmatrix_t * cov)
{
	/* covariance matrix P values init */
	float errx = 0.1; /* err(x) = (10cm)^2 */
	float errv = 0.1; /* err(v) = (10cm/s)^2 */
	float erra = 0.1; /* err(a) = (10cm/s^2)^2 */
	float errw = 0.1; /* err(w) = 0.1 rad/s */ /* FIXME: thats a lot! */
	float errqa = DEG2RAD; /* err(qa) set to 1 degree */
	float errqbcd = DEG2RAD; /* err(qb/qc/qd) set to 1 degree */

	phx_zeroes(cov);
	cov->data[cov->cols * ixx + ixx] = errx * errx;
	cov->data[cov->cols * ixy + ixy] = errx * errx;
	cov->data[cov->cols * ixz + ixz] = errx * errx;
	cov->data[cov->cols * ivx + ivx] = errv * errv;
	cov->data[cov->cols * ivy + ivy] = errv * errv;
	cov->data[cov->cols * ivz + ivz] = errv * errv;
	cov->data[cov->cols * iax + iax] = erra * erra;
	cov->data[cov->cols * iay + iay] = erra * erra;
	cov->data[cov->cols * iaz + iaz] = erra * erra;
	cov->data[cov->cols * iwx + iwx] = errw * errw;
	cov->data[cov->cols * iwy + iwy] = errw * errw;
	cov->data[cov->cols * iwz + iwz] = errw * errw;
	cov->data[cov->cols * iqa + iqa] = errqa * errqa;
	cov->data[cov->cols * iqb + iqb] = errqbcd * errqbcd;
	cov->data[cov->cols * iqc + iqc] = errqbcd * errqbcd;
	cov->data[cov->cols * iqd + iqd] = errqbcd * errqbcd;
}


void init_prediction_matrices(phmatrix_t * state, phmatrix_t * state_est, phmatrix_t * cov, phmatrix_t * cov_est, phmatrix_t * F, phmatrix_t * Q, float dt)
{
	/* process noise Q matrix value init */
	float anoise = 0.1;	/* noise(a) = (1cm/s^2)^2 */
	float wnoise = (5 * DEG2RAD) * (5 * DEG2RAD);	/* noise(w) = (5 dps)^2 */

	/* Q_meas noise matrix */
	float Qm_data[STATE_ROWS * STATE_ROWS];
	phmatrix_t Qm = {.cols = STATE_ROWS, .rows = STATE_ROWS, .transposed = 0, .data = Qm_data};
	float tmp_data[STATE_ROWS * STATE_ROWS];
	phmatrix_t tmp = {.cols = STATE_ROWS, .rows = STATE_ROWS, .transposed = 0, .data = tmp_data};

	/* matrix initialization */
	phx_newmatrix(state, 	STATE_ROWS,	STATE_COLS);	/* 16x1 */
	phx_newmatrix(state_est,STATE_ROWS,	STATE_COLS);	/* 16x1 */

	phx_newmatrix(cov, 		STATE_ROWS,	STATE_ROWS);	/* 16x16 */
	phx_newmatrix(cov_est, 	STATE_ROWS,	STATE_ROWS);	/* 16x16 */
	phx_newmatrix(F, 		STATE_ROWS,	STATE_ROWS);	/* 16x16 */
	phx_newmatrix(Q,		STATE_ROWS,	STATE_ROWS);	/* 16x16 */

	init_state_vector(state);
	init_cov_vector(cov);

	jacobian_F(state, F, dt); /* F needed for Q matrix */
	phx_zeroes(&Qm);
	Qm.data[Qm.cols * iax + iax] = Qm.data[Qm.cols * iay + iay] = Qm.data[Qm.cols * iaz + iaz] = anoise;
	Qm.data[Qm.cols * iwx + iwx] = Qm.data[Qm.cols * iwy + iwy] = Qm.data[Qm.cols * iwz + iwz] = wnoise;
	phx_sadwitch_product(F, &Qm, Q, &tmp);
	phx_zeroes(Q);
}

void init_update_matrices(phmatrix_t * H, phmatrix_t * R)
{
	float meas_anoise = 1; /* noise of accel reading */
	float meas_wnoise = 1; /* noise of gyro reading */

	/* matrix initialization */
	phx_newmatrix(H, 	MEAS_ROWS,	STATE_ROWS);	/* 6x16 */
	phx_newmatrix(R, 	MEAS_ROWS,	MEAS_ROWS);		/* 6x6 */

	/* init of measurement noise matrix R */
	R->data[R->cols * imax + imax] = meas_anoise;
	R->data[R->cols * imay + imay] = meas_anoise;
	R->data[R->cols * imaz + imaz] = meas_anoise;
	R->data[R->cols * imwx + imwx] = meas_wnoise;
	R->data[R->cols * imwy + imwy] = meas_wnoise;
	R->data[R->cols * imwz + imwz] = meas_wnoise;
}