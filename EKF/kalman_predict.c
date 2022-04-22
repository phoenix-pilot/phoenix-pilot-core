/*
 * Phoenix-Pilot
 *
 * extended kalman filter
 * 
 * prediction step formulas
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
#include <time.h>

#include "kalman.h"

#include <tools/rotas_dummy.h>
#include <tools/phmatrix.h>

/* buffer matrix for calculations */
float predict_tmp_data[STATE_ROWS * STATE_ROWS];
phmatrix_t predict_tmp = { .cols = STATE_ROWS, .rows = STATE_ROWS, .transposed = 0, .data = predict_tmp_data };


/* estimate system state */
static void kalman_estimate_state(phmatrix_t *state, phmatrix_t *state_est, float dt)
{
	float dt2 = dt * dt;
	quat_t quat_q, quat_w, /*quat_ap, quat_wp,*/ res;

	quat_q = quat(qa, qb, qc, qd);
	quat_w = quat(0, wx, wy, wz);

	state_est->data[ixx] = xx + vx * dt + ax * dt2 / 2;
	state_est->data[ixy] = xy + vy * dt + ay * dt2 / 2;
	state_est->data[ixz] = xz + vz * dt + az * dt2 / 2;

	state_est->data[ivx] = vx + ax * dt;
	state_est->data[ivy] = vy + ay * dt;
	state_est->data[ivz] = vz + az * dt;

	res = quat_mlt(&quat_w, &quat_q);
	quat_times(&res, dt / 2);
	quat_q = quat_add(&quat_q, &res);
	quat_normalize(&quat_q);
	state_est->data[iqa] = quat_q.a;
	state_est->data[iqb] = quat_q.i;
	state_est->data[iqc] = quat_q.j;
	state_est->data[iqd] = quat_q.k;

	state_est->data[iax] = ax;
	state_est->data[iay] = ay;
	state_est->data[iaz] = az;

	state_est->data[iwx] = wx;
	state_est->data[iwy] = wy;
	state_est->data[iwz] = wz;
}


static void predict_covar_estimate(phmatrix_t *F, phmatrix_t *P, phmatrix_t *P_estimate, phmatrix_t *Q)
{
	phx_sadwitch_product_sparse(F, P, P_estimate, &predict_tmp);
	phx_add(P_estimate, Q, NULL);
}


/* performs kalman prediction step given:
	'state' - current system state vector
	'state_est' - matrix to store system state estimate
	'cov' - process covariance matrix
	'cov_est' - matrix to store covariance matrix estimate
	'F' - transition matrix
	'Q' - process noise 
	dt - time step
*/
void kalman_predict(phmatrix_t *state, phmatrix_t *cov, phmatrix_t *state_est, phmatrix_t *cov_est, phmatrix_t *F, phmatrix_t *Q, float dt, int verbose)
{
	/* apriori estimation of state before measurement */
	kalman_estimate_state(state, state_est, dt);

	if (verbose) {
		printf("stat_est:\n");
		phx_print(state_est);
		printf("F:\n");
		phx_print(F);
	}

	/* apriori estimation of covariance matrix */
	predict_covar_estimate(F, cov, cov_est, Q);

	if (verbose) {
		printf("cov:\n");
		phx_print(cov);
		printf("covest:\n");
		phx_print(cov_est);
	}
}
