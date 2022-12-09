/*
 * Phoenix-Pilot
 *
 * extended kalman filter
 * 
 * EKF math algorithms for prediction and update steps
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
#include <math.h>
#include <time.h>

#include "kalman_implem.h"

#include <vec.h>
#include <quat.h>
#include <matrix.h>

/* buffer matrix for calculations */
/* TODO: move this matrix to stateEngine to remove macros and globals from core */
static float predict_tmp_data[STATE_ROWS * STATE_ROWS];
static matrix_t predict_tmp = { .cols = STATE_ROWS, .rows = STATE_ROWS, .transposed = 0, .data = predict_tmp_data };


static void predict_covar_estimate(matrix_t *F, matrix_t *P, matrix_t *P_estimate, matrix_t *Q)
{
	matrix_sandwitch(F, P, P_estimate, &predict_tmp);
	matrix_add(P_estimate, Q, NULL);
}


/* performs kalman prediction step given state engine */
void kalman_predict(state_engine_t *engine, time_t timeStep, int verbose)
{
	/* calculate current state transition jacobian */
	engine->getJacobian(&engine->F, &engine->state, timeStep);

	/* apriori estimation of state before measurement */
	engine->estimateState(&engine->state, &engine->state_est, timeStep);

	if (verbose) {
		printf("stat_est:\n");
		matrix_print(&engine->state_est);
		printf("F:\n");
		matrix_print(&engine->F);
	}

	/* apriori estimation of covariance matrix */
	predict_covar_estimate(&engine->F, &engine->cov, &engine->cov_est, &engine->Q);

	if (verbose) {
		printf("cov:\n");
		matrix_print(&engine->cov);
		printf("covest:\n");
		matrix_print(&engine->cov_est);
	}
}

/* performs kalman update step calculations */
int kalman_update(time_t timeStep, int verbose, update_engine_t *updateEngine, state_engine_t *stateEngine)
{
	/* no new measurement available = exit step */
	if (updateEngine->getData(&updateEngine->Z, &stateEngine->state, &updateEngine->R, timeStep) == NULL) {
		return -1;
	}

	updateEngine->getJacobian(&updateEngine->H, &stateEngine->state_est, timeStep);

	/* prepare diag */
	matrix_diag(&updateEngine->I);

	/* y_k = z_k - h(x_(k|k-1)) */
	matrix_sub(&updateEngine->Z, updateEngine->predictMeasurements(&stateEngine->state_est, &updateEngine->hx, timeStep), &updateEngine->Y);

	/* S_k = H_k * P_(k|k-1) * transpose(H_k) + R*/
	matrix_sandwitch(&updateEngine->H, &stateEngine->cov_est, &updateEngine->S, &updateEngine->tmp3);
	matrix_add(&updateEngine->S, &updateEngine->R, NULL);

	/* only for debug purposes */
	if (verbose) {
		printf("tmp3:\n");
		matrix_print(&updateEngine->tmp3);
		printf("Z:\n");
		matrix_print(&updateEngine->Z);
		printf("S:\n");
		matrix_print(&updateEngine->S);
		printf("hx:\n");
		matrix_print(&updateEngine->hx);
		printf("H:\n");
		matrix_print(&updateEngine->H);
		printf("cov_est:\n");
		matrix_print(&stateEngine->cov_est);
	}

	/* K_k = P_(k|k-1) * transpose(H_k) * inverse(S_k) */
	matrix_trp(&updateEngine->H);
	matrix_prod(&stateEngine->cov_est, &updateEngine->H, &updateEngine->tmp2);
	matrix_trp(&updateEngine->H);
	matrix_inv(&updateEngine->S, &updateEngine->tmp1, updateEngine->invBuf, updateEngine->invBufLen);
	matrix_prod(&updateEngine->tmp2, &updateEngine->tmp1, &updateEngine->K);

	/* only for debug purposes */
	if (verbose) {
		printf("K:\n");
		matrix_print(&updateEngine->K);

		printf("y:\n");
		matrix_print(&updateEngine->Y);
	}

	/* x_(k|k) = x_(k|k-1) + K_k * y_k */
	matrix_prod(&updateEngine->K, &updateEngine->Y, &updateEngine->tmp5);
	matrix_add(&stateEngine->state_est, &updateEngine->tmp5, &stateEngine->state);

	/* P_(k|k) = (I - K_k * H_k) * P_(k|k-1) */
	matrix_prod(&updateEngine->K, &updateEngine->H, &updateEngine->tmp4);
	matrix_sub(&updateEngine->I, &updateEngine->tmp4, NULL);
	matrix_prod(&updateEngine->I, &stateEngine->cov_est, &stateEngine->cov);

	return 0;
}


void kalman_updateDealloc(update_engine_t *engine)
{
	matrix_bufFree(&engine->Z);
	matrix_bufFree(&engine->Y);
	matrix_bufFree(&engine->S);
	matrix_bufFree(&engine->K);
	matrix_bufFree(&engine->I);
	matrix_bufFree(&engine->H);
	matrix_bufFree(&engine->R);
	matrix_bufFree(&engine->hx);
	matrix_bufFree(&engine->tmp1);
	matrix_bufFree(&engine->tmp2);
	matrix_bufFree(&engine->tmp3);
	matrix_bufFree(&engine->tmp4);
	matrix_bufFree(&engine->tmp5);

	free(engine->invBuf);
}


int kalman_updateAlloc(update_engine_t *engine, unsigned int stateLen, unsigned int measLen)
{
	int err = 0;

	/* Calculation matrices initializations */
	err |= matrix_bufAlloc(&engine->Z, measLen, 1);
	err |= matrix_bufAlloc(&engine->Y, measLen, 1);
	err |= matrix_bufAlloc(&engine->S, measLen, measLen);
	err |= matrix_bufAlloc(&engine->K, stateLen, measLen);
	err |= matrix_bufAlloc(&engine->I, stateLen, stateLen);
	err |= matrix_bufAlloc(&engine->H, measLen, stateLen);
	err |= matrix_bufAlloc(&engine->R, measLen, measLen);
	err |= matrix_bufAlloc(&engine->hx, measLen, 1);

	/* temporary/helper matrices initialization */
	err |= matrix_bufAlloc(&engine->tmp1, measLen, measLen);
	err |= matrix_bufAlloc(&engine->tmp2, stateLen, measLen);
	err |= matrix_bufAlloc(&engine->tmp3, measLen, stateLen);
	err |= matrix_bufAlloc(&engine->tmp4, stateLen, stateLen);
	err |= matrix_bufAlloc(&engine->tmp5, stateLen, 1);

	/* non-matrix buffers */
	engine->invBufLen = measLen * measLen * 2;
	engine->invBuf = calloc(engine->invBufLen, sizeof(float));
	if (engine->invBuf == NULL) {
		err |= -1;
	}

	if (err != 0) {
		kalman_updateDealloc(engine);
		return -1;
	}

	return 0;
}


void kalman_predictDealloc(state_engine_t *engine)
{
	matrix_bufFree(&engine->state);
	matrix_bufFree(&engine->state_est);
	matrix_bufFree(&engine->cov);
	matrix_bufFree(&engine->cov_est);
	matrix_bufFree(&engine->F);
	matrix_bufFree(&engine->Q);
}


int kalman_predictAlloc(state_engine_t *engine, int stateLen)
{
	int err = 0;

	/* State and covariance matrices */
	err |= matrix_bufAlloc(&engine->state, stateLen, 1);
	err |= matrix_bufAlloc(&engine->cov, stateLen, stateLen);

	/* State and covariance estimates matrices */
	err |= matrix_bufAlloc(&engine->state_est, stateLen, 1);
	err |= matrix_bufAlloc(&engine->cov_est, stateLen, stateLen);

	/* Noise and state transition jacobian matrices */
	err |= matrix_bufAlloc(&engine->F, stateLen, stateLen);
	err |= matrix_bufAlloc(&engine->Q, stateLen, stateLen);

	if (err != 0) {
		kalman_predictDealloc(engine);
		return -1;
	}

	return 0;
}
