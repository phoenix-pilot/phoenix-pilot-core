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

struct {
	update_engine_t imuEngine;
	update_engine_t baroEngine;
	state_engine_t stateEngine;

	phmatrix_t state;
	phmatrix_t state_est;
	phmatrix_t cov;
	phmatrix_t cov_est;
	phmatrix_t F;
	phmatrix_t Q;

	unsigned int run;             /* proceed with ekf loop */
	time_t lastTime;    /* last kalman loop time */
	time_t currTime;    /* current kalman loop time */
} ekf_common;


int ekf_init(void)
{
	ekf_common.run = 0;

	return 0;
}


static time_t ekf_dtGet(void)
{
	time_t diff;

	gettime(&ekf_common.currTime, NULL);
	diff = ekf_common.currTime - ekf_common.lastTime;
	ekf_common.lastTime = ekf_common.currTime;

	return diff;
}


static void ekf_thread(void *arg)
{
	time_t timeStep;

	__atomic_store_n(&(ekf_common.run), 1, __ATOMIC_RELAXED);

	/* Kalman loop */
	gettime(&ekf_common.lastTime, NULL);
	while (ekf_common.run == 1) {
		usleep(1000);
		timeStep = ekf_dtGet();

		/* state prediction procedure */
		kalmanPredictionStep(&ekf_common.stateEngine, timeStep, 0);
		if (kalmanUpdateStep(timeStep, 0, &ekf_common.baroEngine, &ekf_common.stateEngine) < 0) { /* barometer measurements update procedure */
			kalmanUpdateStep(timeStep, 0, &ekf_common.imuEngine, &ekf_common.stateEngine);        /* imu measurements update procedure */
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
	__atomic_store_n(&(ekf_common.run), 0, __ATOMIC_RELAXED);
}


void ekf_stateGet(ekf_state_t *ekf_state)
{
	/* empty */
}
