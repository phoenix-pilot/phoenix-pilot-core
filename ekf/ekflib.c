#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include <errno.h>
#include "sys/time.h"
#include <sys/threads.h>

#include "kalman_core.h"
#include "kalman_implem.h"

#include "tools/rotas_dummy.h"
#include "tools/phmatrix.h"

#include "ekflib.h"

static kalman_common_t kalman_common;

static update_engine_t imuEngine, baroEngine;

static state_engine_t stateEngine;

static phmatrix_t state, state_est, cov, cov_est, F, Q; /* state prediction matrices */


int ekf_init(void)
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
	gettime(&kalman_common.current_time, NULL);
	diff = kalman_common.current_time - kalman_common.last_time;
	kalman_common.last_time = kalman_common.current_time;

	return (float)diff / 1000000;
}


static void ekf_thread(void *arg)
{
	kalman_common.run = 1;

	/* Kalman loop */
	gettime(&kalman_common.last_time, NULL);
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


int ekf_run(void)
{
	int *stack, stacksz = 1024;

	stack = malloc(stacksz);
	if (stack == NULL) {
		return -ENOMEM;
	}

	return beginthread(ekf_thread, 4, stack, stacksz, NULL);
}


void ekf_done(void)
{
	/* TODO: use atomics */
	if (kalman_common.run == 1) {
		kalman_common.run = -1;
		while (kalman_common.run != 0) {
			usleep(100);
		}
	}
}


void ekf_getstate(ekf_state_t *ekf_state)
{
	vec_t euler;

	euler = quat_quat2euler(quat(stateEngine.state->data[iqa], stateEngine.state->data[iqb], stateEngine.state->data[iqc], stateEngine.state->data[iqd]));

	/* TODO: shared memory read without any access maagement */
	ekf_state->enuX = stateEngine.state->data[ixx];
	ekf_state->enuY = stateEngine.state->data[ixy];
	ekf_state->enuZ = stateEngine.state->data[ixz];
	ekf_state->yaw = euler.x;
	ekf_state->pitch = euler.y;
	ekf_state->roll = euler.z;
}
