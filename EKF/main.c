/*
 * Phoenix-Pilot
 *
 * extended kalman filter
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
#include <unistd.h>

#include "kalman.h"

#include <tools/rotas_dummy.h>
#include <tools/phmatrix.h>

float t = 0, dt = 0.01;
int verbose = 0, lastprint = -1;

/* prints current state in readable format with possible interval set in seconds*/
int print_state(phmatrix_t * state, float t, float interval)
{
	t *= 1000;
	interval *= 1000;

	/* decide if interval of time has passed to print the state to console */
	if (interval != 0) {
		if ((int)t / (int)interval == lastprint) {
			return 1;
		}
		lastprint = (int)t / (int)interval;
	}

	/* detailed state output */

	printf("X: [%.7f, %.7f, %.7f] | V:  [%.3f, %.3f, %.3f] | A:  [%.3f, %.3f, %.3f]\n", xx, xy, xz, vx, vy, vz, ax, ay, az);
	printf("W: [%.3f, %.3f, %.3f] | Q: [%.3f, %.3f, %.3f, %.3f]\n", wx, wy, wz, qa, qb, qc, qd);
	printf("t: %.3fs\n\n", t/1000);

	/* short state output */
//	printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n", t/1000, xx, xy, xz, vx, vy, vz);
	getchar();
	return 0;
}


int main(int argc, char ** argv)
{
	phmatrix_t state, state_est, cov, cov_est, F, Q, H, R;

	/* imu_calibrate_acc_gyr(); work in progress */
	init_prediction_matrices(&state, &state_est, &cov, &cov_est, &F, &Q, dt);
	init_update_matrices(&H, &R);

	/* Kalman loop */
	while (1) {

		jacobian_F(&state, &F, dt);
		kalman_predict(&state, &cov, &state_est, &cov_est, &F, &Q, dt, 0);
		jacobian_H(&state_est, &H, dt);
		kalman_update(&state, &cov, &state_est, &cov_est, &H, &R, dt, 0);

		t += dt;
		print_state(&state, t, 1); /* print state after 1s of simulation */
	}
}