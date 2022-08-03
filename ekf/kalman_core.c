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
void kalmanPredictionStep(state_engine_t *engine, time_t timeStep, int verbose)
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
int kalmanUpdateStep(time_t timeStep, int verbose, update_engine_t *updateEngine, state_engine_t *stateEngine)
{
	/* no new measurement available = exit step */
	if (updateEngine->getData(updateEngine->Z, &stateEngine->state, updateEngine->R, timeStep) == NULL) {
		return -1;
	}

	updateEngine->getJacobian(updateEngine->H, &stateEngine->state_est, timeStep);

	/* prepare diag */
	matrix_diag(updateEngine->I);

	/* y_k = z_k - h(x_(k|k-1)) */
	matrix_sub(updateEngine->Z, updateEngine->predictMeasurements(&stateEngine->state_est, updateEngine->hx), updateEngine->Y);

	/* S_k = H_k * P_(k|k-1) * transpose(H_k) + R*/
	matrix_sandwitch(updateEngine->H, &stateEngine->cov_est, updateEngine->S, updateEngine->tmp3);
	matrix_add(updateEngine->S, updateEngine->R, NULL);

	/* only for debug purposes */
	if (verbose) {
		printf("tmp3:\n");
		matrix_print(updateEngine->tmp3);
		printf("Z:\n");
		matrix_print(updateEngine->Z);
		printf("S:\n");
		matrix_print(updateEngine->S);
		printf("hx:\n");
		matrix_print(updateEngine->hx);
		printf("H:\n");
		matrix_print(updateEngine->H);
		printf("cov_est:\n");
		matrix_print(&stateEngine->cov_est);
	}

	/* K_k = P_(k|k-1) * transpose(H_k) * inverse(S_k) */
	matrix_trp(updateEngine->H);
	matrix_prod(&stateEngine->cov_est, updateEngine->H, updateEngine->tmp2);
	matrix_trp(updateEngine->H);
	matrix_inv(updateEngine->S, updateEngine->tmp1, updateEngine->invBuf, updateEngine->invBufLen);
	matrix_prod(updateEngine->tmp2, updateEngine->tmp1, updateEngine->K);

	/* only for debug purposes */
	if (verbose) {
		//printf("PkHt:\n");
		//matrix_print(updateEngine->tmp2);
		//printf("S-1:\n");
		//matrix_print(updateEngine->tmp1);
		printf("K:\n");
		matrix_print(updateEngine->K);

		printf("y:\n");
		matrix_print(updateEngine->Y);
	}

	/* x_(k|k) = x_(k|k-1) + K_k * y_k */
	matrix_prod(updateEngine->K, updateEngine->Y, updateEngine->tmp5);
	matrix_add(&stateEngine->state_est, updateEngine->tmp5, &stateEngine->state);

	/* P_(k|k) = (I - K_k * H_k) * P_(k|k-1) */
	matrix_prod(updateEngine->K, updateEngine->H, updateEngine->tmp4);
	matrix_sub(updateEngine->I, updateEngine->tmp4, NULL);
	matrix_prod(updateEngine->I, &stateEngine->cov_est, &stateEngine->cov);

	return 0;
}
