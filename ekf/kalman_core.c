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

#include "tools/rotas_dummy.h"
#include "tools/phmatrix.h"

/* buffer matrix for calculations */
/* TODO: move this matrix to stateEngine to remove macros from core */
float predict_tmp_data[STATE_ROWS * STATE_ROWS];
phmatrix_t predict_tmp = { .cols = STATE_ROWS, .rows = STATE_ROWS, .transposed = 0, .data = predict_tmp_data };


static void predict_covar_estimate(phmatrix_t *F, phmatrix_t *P, phmatrix_t *P_estimate, phmatrix_t *Q)
{
	phx_sadwitch_product(F, P, P_estimate, &predict_tmp);
	phx_add(P_estimate, Q, NULL);
}


/* performs kalman prediction step given state engine */
void kalmanPredictionStep(state_engine_t *engine, float dt, int verbose)
{
	/* calculate current state transition jacobian */
	engine->getJacobian(engine->F, engine->state, dt);

	/* apriori estimation of state before measurement */
	engine->estimateState(engine->state, engine->state_est, dt);

	if (verbose) {
		printf("stat_est:\n");
		phx_print(engine->state_est);
		printf("F:\n");
		phx_print(engine->F);
	}

	/* apriori estimation of covariance matrix */
	predict_covar_estimate(engine->F, engine->cov, engine->cov_est, engine->Q);

	if (verbose) {
		printf("cov:\n");
		phx_print(engine->cov);
		printf("covest:\n");
		phx_print(engine->cov_est);
	}
}

/* performs kalman update step calculations */
int kalmanUpdateStep(float dt, int verbose, update_engine_t *updateEngine, state_engine_t *stateEngine)
{
	/* no new measurement available = exit step */
	if (updateEngine->getData(updateEngine->Z, stateEngine->state, updateEngine->R, dt) == NULL) {
		return -1;
	}

	updateEngine->getJacobian(updateEngine->H, stateEngine->state_est, dt);

	/* prepare diag */
	phx_diag(updateEngine->I);

	/* y_k = z_k - h(x_(k|k-1)) */
	phx_sub(updateEngine->Z, updateEngine->predictMeasurements(stateEngine->state_est, updateEngine->hx), updateEngine->Y);

	/* S_k = H_k * P_(k|k-1) * transpose(H_k) + R*/
	phx_sadwitch_product(updateEngine->H, stateEngine->cov_est, updateEngine->S, updateEngine->tmp3);
	phx_add(updateEngine->S, updateEngine->R, NULL);

	/* only for debug purposes */
	if (verbose) {
		printf("tmp3:\n");
		phx_print(updateEngine->tmp3);
		printf("Z:\n");
		phx_print(updateEngine->Z);
		printf("S:\n");
		phx_print(updateEngine->S);
		printf("hx:\n");
		phx_print(updateEngine->hx);
		printf("H:\n");
		phx_print(updateEngine->H);
		printf("cov_est:\n");
		phx_print(stateEngine->cov_est);
	}

	/* K_k = P_(k|k-1) * transpose(H_k) * inverse(S_k) */
	phx_transpose(updateEngine->H);
	phx_product(stateEngine->cov_est, updateEngine->H, updateEngine->tmp2);
	phx_transpose(updateEngine->H);
	phx_inverse(updateEngine->S, updateEngine->tmp1, updateEngine->invBuf, updateEngine->invBufLen);
	phx_product(updateEngine->tmp2, updateEngine->tmp1, updateEngine->K);

	/* only for debug purposes */
	if (verbose) {
		//printf("PkHt:\n");
		//phx_print(updateEngine->tmp2);
		//printf("S-1:\n");
		//phx_print(updateEngine->tmp1);
		printf("K:\n");
		phx_print(updateEngine->K);

		printf("y:\n");
		phx_print(updateEngine->Y);
	}

	/* x_(k|k) = x_(k|k-1) + K_k * y_k */
	phx_product(updateEngine->K, updateEngine->Y, updateEngine->tmp5);
	phx_add(stateEngine->state_est, updateEngine->tmp5, stateEngine->state);

	/* P_(k|k) = (I - K_k * H_k) * P_(k|k-1) */
	phx_product(updateEngine->K, updateEngine->H, updateEngine->tmp4);
	phx_sub(updateEngine->I, updateEngine->tmp4, NULL);
	phx_product(updateEngine->I, stateEngine->cov_est, stateEngine->cov);

	return 0;
}
