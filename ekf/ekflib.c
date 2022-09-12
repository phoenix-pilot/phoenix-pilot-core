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
#include <math.h>
#include <errno.h>
#include <sys/time.h>
#include <sys/threads.h>

#include "kalman_core.h"
#include "kalman_implem.h"

#include <sensc.h>
#include <vec.h>
#include <quat.h>
#include <matrix.h>

#include "ekflib.h"

struct {
	update_engine_t imuEngine;
	update_engine_t baroEngine;
	state_engine_t stateEngine;

	matrix_t state;
	matrix_t state_est;
	matrix_t cov;
	matrix_t cov_est;
	matrix_t F;
	matrix_t Q;

	volatile unsigned int run; /* proceed with ekf loop */
	time_t lastTime;           /* last kalman loop time */
	time_t currTime;           /* current kalman loop time */

	handle_t lock;

	char stack[8192];
} ekf_common;


int ekf_init(void)
{
	if (mutexCreate(&(ekf_common.lock)) < 0) {
		printf("Cannot create mutex for ekf\n");
		return -1;
	}

	ekf_common.run = 0;
	if (sensc_init("/dev/sensors", corrEnable) < 0) {
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
		/* TODO: make critical section smaller and only on accesses to state and cov matrices */
		mutexLock(ekf_common.lock);
		if (kalmanUpdateStep(timeStep, 0, &ekf_common.baroEngine, &ekf_common.stateEngine) < 0) { /* barometer measurements update procedure */
			kalmanUpdateStep(timeStep, 0, &ekf_common.imuEngine, &ekf_common.stateEngine);        /* imu measurements update procedure */
		}
		mutexUnlock(ekf_common.lock);
	}

	ekf_common.run = -1;
	endthread();
}


int ekf_run(void)
{
	return beginthread(ekf_thread, 4, ekf_common.stack, sizeof(ekf_common.stack), NULL);
}


void ekf_done(void)
{
	if (ekf_common.run == 1) {
		ekf_common.run = 0;
	}
	threadJoin(0);
	kmn_predDeinit(&ekf_common.stateEngine);
	resourceDestroy(ekf_common.lock);
}


void ekf_boundsGet(float *bYaw, float *bRoll, float *bPitch)
{
	*bYaw = M_PI;
	*bRoll = M_PI;
	*bPitch = M_PI_2;
}


void ekf_stateGet(ekf_state_t *ekfState)
{
	quat_t q;
	vec_t angRates = { 0 }; /* (rollDot, pitchDot, yawDot) */

	mutexLock(ekf_common.lock);

	/* save quaternion attitude */
	q.a = ekfState->q0 = ekf_common.stateEngine.state.data[iqa];
	q.i = ekfState->q1 = ekf_common.stateEngine.state.data[iqb];
	q.j = ekfState->q2 = ekf_common.stateEngine.state.data[iqc];
	q.k = ekfState->q3 = ekf_common.stateEngine.state.data[iqd];

	/* save position */
	ekfState->enuX = ekf_common.stateEngine.state.data[ixx];
	ekfState->enuY = ekf_common.stateEngine.state.data[ixy];
	/* as long as inertial reckoning is not trustable enough, return barometer height as enuZ */
	ekfState->enuZ = ekf_common.stateEngine.state.data[ihz];

	/* save accelerations */
	ekfState->accelX = ekf_common.stateEngine.state.data[iax];
	ekfState->accelY = ekf_common.stateEngine.state.data[iay];
	ekfState->accelZ = ekf_common.stateEngine.state.data[iaz];

	angRates.x = ekf_common.stateEngine.state.data[iwx];
	angRates.y = ekf_common.stateEngine.state.data[iwy];
	angRates.z = ekf_common.stateEngine.state.data[iwz];

	mutexUnlock(ekf_common.lock);

	/* calculate and save euler attitude */
	quat_quat2euler(&q, &ekfState->roll, &ekfState->pitch, &ekfState->yaw);

	/* rotate angular rates back to UAV frame of reference */
	quat_cjg(&q);
	quat_vecRot(&angRates, &q);
	ekfState->rollDot = angRates.x;
	ekfState->pitchDot = angRates.y;
	ekfState->yawDot = angRates.z;
}
