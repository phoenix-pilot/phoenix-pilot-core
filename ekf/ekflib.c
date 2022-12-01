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
#include <stdbool.h>
#include <time.h>
#include <unistd.h>
#include <math.h>
#include <errno.h>
#include <sys/time.h>
#include <sys/threads.h>

#include "kalman_core.h"
#include "kalman_implem.h"
#include "log.h"

#include <sensc.h>
#include <vec.h>
#include <quat.h>
#include <matrix.h>

#include "ekflib.h"

struct {
	kalman_init_t initVals;

	update_engine_t imuEngine;
	update_engine_t baroEngine;
	state_engine_t stateEngine;

	matrix_t state;
	matrix_t state_est;
	matrix_t cov;
	matrix_t cov_est;
	matrix_t F;
	matrix_t Q;

	unsigned int tid;
	volatile unsigned int run; /* proceed with ekf loop */
	time_t lastTime;           /* last kalman loop time */
	time_t currTime;           /* current kalman loop time */

	handle_t lock;

	char stack[8192] __attribute__((aligned(8)));
} ekf_common;


int ekf_init(void)
{
	uint32_t logFlags;
	int err;

	if (mutexCreate(&ekf_common.lock) < 0) {
		printf("Cannot create mutex for ekf\n");
		return -1;
	}

	ekf_common.run = 0;
	if (sensc_init("/dev/sensors", true) < 0) {
		resourceDestroy(ekf_common.lock);
		return -1;
	}

	err = 0;
	err |= kalman_predictAlloc(&ekf_common.stateEngine, STATE_ROWS);
	err |= kalman_updateAlloc(&ekf_common.imuEngine, STATE_ROWS, IMUMEAS_ROWS);
	err |= kalman_updateAlloc(&ekf_common.baroEngine, STATE_ROWS, BAROMEAS_ROWS);

	if (err != 0) {
		sensc_deinit();
		resourceDestroy(ekf_common.lock);
		return -1;
	}

	/* TODO: config read should utilize parser, and default values should be stored in /etc/calib.conf */
	kmn_configRead(&ekf_common.initVals); /* only for development process */

	/* TODO: reimplement this nasty cast below */
	ekf_common.initVals.log = (int)(*((float*)(&ekf_common.initVals.log)));

	/* Choose ekf log mode */
	switch (ekf_common.initVals.log) {
		case 1:
			logFlags = EKFLOG_SENSC;
			break;
		case 2:
			logFlags = EKFLOG_MEAS;
			break;
		case 3:
			logFlags = EKFLOG_EKF_IMU;
			break;
		case 4:
			logFlags = EKFLOG_EKF_POS;
			break;
		default:
			logFlags = 0;
	}

	if (ekflog_init("ekf_log.txt", logFlags) != 0) {
		resourceDestroy(ekf_common.lock);
		sensc_deinit();

		kalman_predictDealloc(&ekf_common.stateEngine);
		kalman_updateDealloc(&ekf_common.imuEngine);
		kalman_updateDealloc(&ekf_common.baroEngine);

		return -1;
	}

	meas_imuCalib();
	meas_baroCalib();

	kmn_predInit(&ekf_common.stateEngine, meas_calibGet(), &ekf_common.initVals);
	kmn_imuEngInit(&ekf_common.imuEngine, &ekf_common.initVals);
	kmn_baroEngInit(&ekf_common.baroEngine, &ekf_common.initVals);

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
	time_t timeStep, lastBaro;

	printf("ekf: starting ekf thread\n");

	ekf_common.run = 1;

	/* Kalman loop */
	gettime(&ekf_common.lastTime, NULL);
	lastBaro = ekf_common.lastTime;

	while (ekf_common.run == 1) {
		usleep(1000);
		timeStep = ekf_dtGet();

		/* state prediction procedure */
		kalman_predict(&ekf_common.stateEngine, timeStep, 0);

		/* TODO: make critical section smaller and only on accesses to state and cov matrices */
		mutexLock(ekf_common.lock);
		if (ekf_common.currTime - lastBaro > BARO_UPDATE_PERIOD) {
			/* Perform barometer update step if 'BARO_UPDATE_PERIOD' time has passed since last such update */
			kalman_update(ekf_common.currTime - lastBaro, 0, &ekf_common.baroEngine, &ekf_common.stateEngine);
			lastBaro = ekf_common.currTime;
		}
		else {
			kalman_update(timeStep, 0, &ekf_common.imuEngine, &ekf_common.stateEngine); /* imu measurements update procedure */
		}
		mutexUnlock(ekf_common.lock);

		ekflog_write(
			EKFLOG_EKF_IMU,
			"EI %lli %.3f %.3f %.3f %.3f %.3f %.3f %.3f\n",
			ekf_common.currTime,
			*matrix_at(&ekf_common.stateEngine.state, IAX, 0),
			*matrix_at(&ekf_common.stateEngine.state, IAY, 0),
			*matrix_at(&ekf_common.stateEngine.state, IAZ, 0),
			*matrix_at(&ekf_common.stateEngine.state, IWX, 0),
			*matrix_at(&ekf_common.stateEngine.state, IWY, 0),
			*matrix_at(&ekf_common.stateEngine.state, IWZ, 0),
			*matrix_at(&ekf_common.stateEngine.state, IBAZ, 0));
	}

	ekf_common.run = -1;
	endthread();
}


int ekf_run(void)
{
	int res =  beginthreadex(ekf_thread, 3, ekf_common.stack, sizeof(ekf_common.stack), NULL, &ekf_common.tid);

	/* Wait to stabilize data in covariance matrixes */
	sleep(3);

	return res;
}


int ekf_stop(void)
{
	int err = EOK;

	if (ekf_common.run == 1) {
		ekf_common.run = 0;
	}

	do {
		err = threadJoin(ekf_common.tid, 0);
	} while (err == -EINTR);

	return err;
}


void ekf_done(void)
{
	kalman_predictDealloc(&ekf_common.stateEngine);
	kalman_updateDealloc(&ekf_common.imuEngine);
	kalman_updateDealloc(&ekf_common.baroEngine);

	sensc_deinit();
	ekflog_done();
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
	q.a = ekfState->q0 = ekf_common.stateEngine.state.data[IQA];
	q.i = ekfState->q1 = ekf_common.stateEngine.state.data[IQB];
	q.j = ekfState->q2 = ekf_common.stateEngine.state.data[IQC];
	q.k = ekfState->q3 = ekf_common.stateEngine.state.data[IQD];

	/* save newtonian motion parameters with frame change from NED to ENU */
	ekfState->enuX = ekf_common.stateEngine.state.data[IXY];
	ekfState->enuY = ekf_common.stateEngine.state.data[IXX];
	ekfState->enuZ = -ekf_common.stateEngine.state.data[IXZ];

	ekfState->veloX = ekf_common.stateEngine.state.data[IVY];
	ekfState->veloY = ekf_common.stateEngine.state.data[IVX];
	ekfState->veloZ = -ekf_common.stateEngine.state.data[IVZ];

	ekfState->accelX = ekf_common.stateEngine.state.data[IAY];
	ekfState->accelY = ekf_common.stateEngine.state.data[IAX];
	ekfState->accelZ = -ekf_common.stateEngine.state.data[IAZ];

	/* Save position quaternion */
	angRates.x = ekf_common.stateEngine.state.data[IWX];
	angRates.y = ekf_common.stateEngine.state.data[IWY];
	angRates.z = ekf_common.stateEngine.state.data[IWZ];

	ekfState->accelBiasZ = ekf_common.stateEngine.state.data[IBAZ];

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
