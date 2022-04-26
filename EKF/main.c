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
#include <math.h>
#include "sys/time.h"

#include "kalman.h"

#include <tools/rotas_dummy.h>
#include <tools/phmatrix.h>

float t = 0, dt = 0.001;
int verbose = 0, lastprint = -1;
struct timeval last_time;

/* prints current state in readable format with possible interval set in seconds*/
int print_state(phmatrix_t *state, phmatrix_t *cov, float t, float interval)
{
	quat_t q = quat(qa, qb, qc, qd);
	vec_t euler = quat_quat2euler(q);
	euler = vec_scl(&euler, 180 * M_1_PI);
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
	printf("X: [%.3f, %.3f, %.7f] | V:  [%.3f, %.3f, %.3f] | A:  [%.3f, %.3f, %.3f]\n", xx, xy, xz, vx, vy, vz, ax, ay, az);
	printf("W: [%.3f, %.3f, %.3f] | Q: [%.5f, %.5f, %.5f, %.5f]\n", wx, wy, wz, qa, qb, qc, qd);
	printf("E: [%.3f, %.3f, %.3f]\n", euler.x, euler.y, euler.z);
	printf("t: %.3fs\n\n", t / 1000);

	/* plain state output */
	/*
	printf("%.1f", t/1000);
	printf(" %.3f %.3f %.3f", xx, xy, xz);
	printf(" %.3f %.3f %.3f", vx, vy, vz);
	printf(" %.3f %.3f %.3f", ax, ay, az);
	printf(" %.3f %.3f %.3f", wx, wy, wz);
	printf(" %.3f %.3f %.3f %.3f", qa, qb, qc, qd);
	printf("\n");
	*/
	/*
	printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n", t/1000, xx, xy, xz, vx, vy, vz);
	*/
	return 0;
}


float get_dt(void)
{
	struct timeval current_time;
	usleep(800);

	gettimeofday(&current_time, NULL);
	long diff = current_time.tv_sec - last_time.tv_sec;
	diff = current_time.tv_usec + diff * 1000000 - last_time.tv_usec;
	last_time = current_time;

	return (float)diff / 1000000;
}


int main(int argc, char **argv)
{
	phmatrix_t state, state_est, cov, cov_est, F, Q, H, R;

	read_config();
	imu_calibrate_acc_gyr();
	init_prediction_matrices(&state, &state_est, &cov, &cov_est, &F, &Q, dt);
	init_update_matrices(&H, &R);

	/* Kalman loop */
	gettimeofday(&last_time, NULL);
	while (1) {
		dt = get_dt();

		jacobian_F(&state, &F, dt);
		kalman_predict(&state, &cov, &state_est, &cov_est, &F, &Q, dt, 0);
		jacobian_H(&state_est, &H, dt);
		kalman_update(&state, &cov, &state_est, &cov_est, &H, &R, dt, 0);

		t += dt;
		print_state(&state, &cov, t, 1); /* print state after 1s of simulation */
	}
}