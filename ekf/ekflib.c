/*
 * Phoenix-Pilot
 *
 * extended kalman filter library
 *
 * Copyright 2022 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include <errno.h>
#include <sys/time.h>
#include <sys/threads.h>

#include "kalman_core.h"
#include "kalman_implem.h"

#include "tools/rotas_dummy.h"
#include "tools/phmatrix.h"

#include "ekflib.h"

typedef struct {
	update_engine_t imuEngine;
	update_engine_t baroEngine;
	state_engine_t stateEngine;

	phmatrix_t state;
	phmatrix_t state_est;
	phmatrix_t cov;
	phmatrix_t cov_est;
	phmatrix_t F;
	phmatrix_t Q;

	float dt;            /* current time step length */
	unsigned int run;             /* proceed with ekf loop */
	time_t last_time;    /* last kalman loop time */
	time_t current_time; /* current kalman loop time */
} ekf_common_t;

ekf_common_t ekf_common;


int ekf_init(void)
{
	ekf_common.run = 0;
	return 0;
}


static float get_dt(void)
{
	time_t diff;

	usleep(1000);
	gettime(&ekf_common.current_time, NULL);
	diff = ekf_common.current_time - ekf_common.last_time;
	ekf_common.last_time = ekf_common.current_time;

	return (float)diff / 1000000.0f;
}


static void ekf_thread(void *arg)
{
	__atomic_add_fetch(&(ekf_common.run), 1, __ATOMIC_RELAXED);

	/* Kalman loop */
	gettime(&ekf_common.last_time, NULL);
	while (ekf_common.run == 1) {
		ekf_common.dt = get_dt();

		/* state prediction procedure */
		kalmanPredictionStep(&ekf_common.stateEngine, ekf_common.dt, 0);
		if (kalmanUpdateStep(ekf_common.dt, 0, &ekf_common.baroEngine, &ekf_common.stateEngine) < 0) { /* barometer measurements update procedure */
			kalmanUpdateStep(ekf_common.dt, 0, &ekf_common.imuEngine, &ekf_common.stateEngine);        /* imu measurements update procedure */
		}
	}

	ekf_common.run = 0;
}


int ekf_run(void)
{
	return 0;
}


void ekf_done(void)
{
	__atomic_sub_fetch(&(ekf_common.run), 1, __ATOMIC_RELAXED);
	return;
}


void ekf_getstate(ekf_state_t *ekf_state)
{
	return;
}
