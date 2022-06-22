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
	return 0;
}


void ekf_done(void)
{
	return;
}


void ekf_getstate(ekf_state_t *ekf_state)
{
	return;
}
