#include <stdio.h>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <unistd.h>
#include <math.h>
#include <errno.h>
#include "sys/time.h"
#include <sys/threads.h>

#include "kalman_core.h"
#include "kalman_implem.h"

#include "tools/rotas_dummy.h"
#include "tools/phmatrix.h"

#include "ekflib.h"

kalman_common_t kalman_common;

update_engine_t imuEngine, baroEngine;

state_engine_t stateEngine;

phmatrix_t state, state_est, cov, cov_est, F, Q; /* state prediction matrices */

int ekfCalib(void)
{
	read_config();
	imu_calibrate_acc_gyr_mag();
	//gps_calibrate();

	stateEngine = init_prediction_matrices(&state, &state_est, &cov, &cov_est, &F, &Q, kalman_common.dt);
	imuEngine = setupImuUpdateEngine(NULL, NULL);
	baroEngine = setupBaroUpdateEngine(NULL, NULL);

	return 0;
}


static float get_dt(void)
{
	time_t diff;

	usleep(1000);
	gettime(kalman_common.current_time);
	diff = kalman_common.current_time - kalman_common.last_time;
	kalman_common.last_time = kalman_common.current_time;

	return (float)diff / 1000000;
}


static void ekfThread(void)
{
	kalman_common.run = 1;

	/* Kalman loop */
	gettimeofday(&kalman_common.last_time, NULL);
	while (kalman_common.run == 1) {
		kalman_common.dt = get_dt();

		/* state prediction procedure */
		kalmanPredictionStep(&stateEngine, kalman_common.dt, 0);
			if (kalmanUpdateStep(kalman_common.dt, 0, &baroEngine, &stateEngine) < 0) { /* barometer measurements update procedure */
				kalmanUpdateStep(kalman_common.dt, 0, &imuEngine, &stateEngine);        /* imu measurements update procedure */
			}
	}

	kalman_common.run = 0;
}


int ekfRun(void)
{
	int *stack, stacksz = 1024;

	stack = malloc(stacksz);
	if (stack == NULL) {
		return -ENOMEM;
	}

	return beginthread(ekfThread, 4, stack, stacksz, NULL);
}


int ekfReset(void)
{
	/* variables for satysfying macro usage */
	phmatrix_t *state, *cov;
	state = stateEngine.state;
	cov = stateEngine.cov;

	xx = xy = 0;
	vx = vy = 0;
	ax = ay = az = 0;
	wx = wy = wz = 0;

	phx_zeroes(cov);
}

void ekfStop(void)
{
	if (kalman_common.run == 1) {
		kalman_common.run = -1;
		while(kalman_common.run != 0) {
			usleep(100);
		}
	}
}


void getPos(float *xeast, float *xnorth, float *xalt) {
	*xeast = stateEngine.state->data[ixx];
	*xnorth = stateEngine.state->data[ixy];
	*xalt = stateEngine.state->data[ixz];
}


void getAtt(float *yaw, float *pitch, float *roll)
{
	phmatrix_t * state = stateEngine.state;
	vec_t euler = quat_quat2euler(quat(state->data[iqa], state->data[iqb], state->data[iqc], state->data[iqd]));

	/* quat_quat2euler() returns vector of (heading, pitch, bank) */
	*yaw = euler.x;
	*pitch = euler.y;
	*roll = euler.z;
}