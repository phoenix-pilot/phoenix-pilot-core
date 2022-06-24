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
#include "sensc.h"

#include "tools/rotas_dummy.h"
#include "tools/phmatrix.h"

#include "ekflib.h"

struct {
	update_engine_t imuEngine;
	update_engine_t baroEngine;
	state_engine_t stateEngine;

	kalman_calib_t calib;

	phmatrix_t state;
	phmatrix_t state_est;
	phmatrix_t cov;
	phmatrix_t cov_est;
	phmatrix_t F;
	phmatrix_t Q;

	volatile unsigned int run; /* proceed with ekf loop */
	time_t lastTime;           /* last kalman loop time */
	time_t currTime;           /* current kalman loop time */

	char stack[8192];
} ekf_common;


int ekf_init(void)
{
	const kalman_calib_t * calib;

	ekf_common.run = 0;
	if (sensc_init("/dev/sensors") < 0) {
		return -1;
	}

	meas_imuCalib();
	meas_baroCalib();

	if (kmn_predInit(&ekf_common.stateEngine, meas_calibGet()) < 0) {
		printf("failed to initialize prediction matrices\n");
		return -1;
	}
	kmn_imuEngInit(&ekf_common.imuEngine);
	kmn_baroEngInit(&ekf_common.baroEngine);

	// printf("%f\n", ekf_common.calib.base_pressure);
	// printf("%f %f %f\n", ekf_common.calib.init_m.x, ekf_common.calib.init_m.y, ekf_common.calib.init_m.z);
	// printf("%f %f %f\n", ekf_common.calib.gyr_nivel.x, ekf_common.calib.gyr_nivel.y, ekf_common.calib.gyr_nivel.z);

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

	printf("ekf: starting ekf thread\n");

	ekf_common.run = 1;

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

	printf("ekf: ekf thread stopped!\n");
	ekf_common.run = -1;
}


int ekf_run(void)
{
	return beginthread(ekf_thread, 4, ekf_common.stack, sizeof(ekf_common.stack), NULL);
}


void ekf_done(void)
{
	if (ekf_common.run == 1) {
		ekf_common.run = 0;
		while (ekf_common.run == 0) {
			usleep(1000);
		}
		usleep(1000);
		kmn_predDeinit(&ekf_common.stateEngine);
	}
}


void ekf_stateGet(ekf_state_t *ekf_state)
{
	quat_t q;
	vec_t e;

	/* save quaternion attitude */
	q.a = ekf_state->q0 = ekf_common.stateEngine.state.data[iqa];
	q.i = ekf_state->q1 = ekf_common.stateEngine.state.data[iqb];
	q.j = ekf_state->q2 = ekf_common.stateEngine.state.data[iqc];
	q.k = ekf_state->q3 = ekf_common.stateEngine.state.data[iqd];

	/* calculate and save euler attitude */
	e = quat_quat2euler(q);
	ekf_state->yaw = e.x;
	ekf_state->pitch = e.y;
	ekf_state->roll = e.z;

	/* save position */
	ekf_state->enuX = ekf_common.stateEngine.state.data[ixx];
	ekf_state->enuX = ekf_common.stateEngine.state.data[ixy];
	ekf_state->enuX = ekf_common.stateEngine.state.data[ixz];
}
