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
#include <sys/threads.h>

#include "kalman_core.h"
#include "kalman_implem.h"

#include "tools/rotas_dummy.h"
#include "tools/phmatrix.h"

kalman_common_t kalman_common;
phmatrix_t *TR;

void print_UAV_versors(quat_t q, vec_t start, vec_t a)
{
	vec_t x = vec(1, 0, 0), y = vec(0, 1, 0), z = vec(0, 0, 1);

	quat_vecrot(&x, &q);
	quat_vecrot(&y, &q);
	quat_vecrot(&z, &q);

	printf("%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f\n", x.x, x.y, x.z, y.x, y.y, y.z, z.x, z.y, z.z, start.x, start.y, start.z, a.x, a.y, a.z);
}

/* prints current state in readable format with possible interval set in seconds*/
int print_state(phmatrix_t *state, phmatrix_t *cov, float t, float interval)
{
	quat_t q;
	vec_t euler;

	/* decide if interval of time has passed to print the state to console */
	t *= 1000;
	interval *= 1000;
	if (interval != 0) {
		if ((int)t / (int)interval == kalman_common.lastprint) {
			return 1;
		}
		kalman_common.lastprint = (int)t / (int)interval;
	}
	t /= 1000;

	/* compute euler angles */
	q = quat(qa, qb, qc, qd);
	euler = quat_quat2euler(q);
	euler = vec_times(&euler, 180 * M_1_PI);

	/* detailed state output */
	if (verbose) {
		printf("X: [%.3f, %.3f, %.7f] | V:  [%.3f, %.3f, %.3f] | A:  [%.3f, %.3f, %.3f]\n", xx, xy, xz, vx, vy, vz, ax, ay, az);
		printf("covX: [%.3f, %.3f, %.7f]\n", cov->data[cov->cols * ixx + ixx], cov->data[cov->cols * ixy + ixy], cov->data[cov->cols * ixz + ixz]);
		printf("covX: [%.3f, %.3f, %.7f]\n", cov->data[cov->cols * ivx + ivx], cov->data[cov->cols * ivy + ivy], cov->data[cov->cols * ivz + ivz]);
		printf("covX: [%.3f, %.3f, %.7f]\n", cov->data[cov->cols * iax + iax], cov->data[cov->cols * iay + iay], cov->data[cov->cols * iaz + iaz]);
		printf("W: [%.3f, %.3f, %.3f] | Q: [%.5f, %.5f, %.5f, %.5f]\n", wx, wy, wz, qa, qb, qc, qd);
		printf("M: [%.3f, %.3f, %.3f]\n", mx, my, mz);
		printf("E: [%.3f, %.3f, %.3f]\n", euler.x, euler.y, euler.z);
		printf("hz/hv: [%.3f / %.3f] %f t: %.3f\n\n", hz, hv, cov->data[ixz * cov->cols + ixz], t);
		printf("%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f\n", xx, xy, xz, vx, vy, vz, ax, ay, az);
	}
	print_UAV_versors(quat(qa, qb, qc, qd), vec(xx, xy, xz), vec(ax, ay, az));
	return 0;
}

void reset_state(phmatrix_t * state, phmatrix_t * cov) {
	xx = xy = 0;
	vx = vy = 0;
	ax = ay = az = 0;
	wx = wy = wz = 0;

	phx_zeroes(cov);
}


static float get_dt(void)
{
	time_t diff;

	usleep(1000);
	gettime(&kalman_common.current_time, NULL);
	diff = kalman_common.current_time - kalman_common.last_time;
	kalman_common.last_time = kalman_common.current_time;

	return (float)diff / 1000000;
}


int main(int argc, char **argv)
{
	update_engine_t imuEngine, baroEngine;
	state_engine_t stateEngine;

	phmatrix_t state, state_est, cov, cov_est, F, Q; /* state prediction matrices */
	phmatrix_t imuH, imuR;                           /* imu measurements update matrices */
	phmatrix_t baroH, baroR;                         /* barometer mesurement matices */
	TR = &imuR;
	int reset = 1;

	read_config();
	imu_calibrate_acc_gyr_mag();
	//gps_calibrate();

	stateEngine = init_prediction_matrices(&state, &state_est, &cov, &cov_est, &F, &Q, kalman_common.dt);
	imuEngine = setupImuUpdateEngine(&imuH, &imuR);
	baroEngine = setupBaroUpdateEngine(&baroH, &baroR);

	/* Kalman loop */
	gettime(&kalman_common.last_time, NULL);
	while (1) {
		kalman_common.dt = get_dt();

		/* state prediction procedure */
		kalmanPredictionStep(&stateEngine, kalman_common.dt, 0);
			if (kalmanUpdateStep(kalman_common.dt, 0, &baroEngine, &stateEngine) < 0) { /* barometer measurements update procedure */
				kalmanUpdateStep(kalman_common.dt, 0, &imuEngine, &stateEngine);        /* imu measurements update procedure */
			}

		kalman_common.t += kalman_common.dt;
		print_state(&state, &cov_est, kalman_common.t, 0.05); /* print state after 1s of simulation */
	
		if ((int)kalman_common.t > 1 && ((int)kalman_common.t)%10 == 0 && reset == 0) {
			if (reset == 0) {
				reset = 1;
				reset_state(&state, &cov);
			}
		}
		else {
			reset = 0;
		}
	}
}