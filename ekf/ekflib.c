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
#include <pthread.h>
#include <sched.h>
#include <errno.h>

#include "kalman_core.h"
#include "kalman_implem.h"
#include "log.h"
#include "meas.h"

#include <sensc.h>
#include <vec.h>
#include <quat.h>
#include <matrix.h>
#include <gettime.h>

#include "ekflib.h"

#define EKF_CONFIG_FILE "etc/ekf.conf"

#if defined PTHREAD_STACK_MIN && 8192 < PTHREAD_STACK_MIN
#define STACK_SIZE PTHREAD_STACK_MIN
#else
#define STACK_SIZE 8192
#endif


struct {
	kalman_init_t initVals;

	update_engine_t imuEngine;
	update_engine_t baroEngine;
	update_engine_t gpsEngine;
	state_engine_t stateEngine;

	matrix_t state;
	matrix_t state_est;
	matrix_t cov;
	matrix_t cov_est;
	matrix_t F;
	matrix_t Q;

	pthread_t tid;
	volatile unsigned int run; /* proceed with ekf loop */
	time_t lastTime;           /* last kalman loop time */
	time_t currTime;           /* current kalman loop time */

	pthread_mutex_t lock;
	pthread_attr_t threadAttr;

	char stack[STACK_SIZE] __attribute__((aligned(8)));
} ekf_common;


static int ekf_threadAttrInit(void)
{
#ifdef __phoenix__

	struct sched_param sched = { .sched_priority = 3 };

	if (pthread_attr_setschedparam(&ekf_common.threadAttr, &sched) != 0) {
		printf("Set schedule failed\n");
		pthread_attr_destroy(&ekf_common.threadAttr);
		return -1;
	}

#endif

	if (pthread_attr_init(&ekf_common.threadAttr) != 0) {
		printf("Attribute init failed\n");
		pthread_attr_destroy(&ekf_common.threadAttr);
		return -1;
	}

	if (pthread_attr_setstack(&ekf_common.threadAttr, ekf_common.stack, sizeof(ekf_common.stack)) != 0) {
		pthread_attr_destroy(&ekf_common.threadAttr);
		return -1;
	}

	return 0;
}


int ekf_init(void)
{
	uint32_t logFlags;
	int err;

	if (ekf_threadAttrInit() != 0) {
		return -1;
	}

	if (pthread_mutex_init(&ekf_common.lock, NULL) < 0) {
		printf("Cannot create mutex for ekf\n");
		pthread_attr_destroy(&ekf_common.threadAttr);
		return -1;
	}

	ekf_common.run = 0;
	if (sensc_init("/dev/sensors", true) < 0) {
		pthread_mutex_destroy(&ekf_common.lock);
		pthread_attr_destroy(&ekf_common.threadAttr);
		return -1;
	}

	err = 0;
	err |= kalman_predictAlloc(&ekf_common.stateEngine, STATE_LENGTH, CTRL_LENGTH);
	err |= kalman_updateAlloc(&ekf_common.imuEngine, STATE_LENGTH, MEAS_IMU_LENGTH);
	err |= kalman_updateAlloc(&ekf_common.baroEngine, STATE_LENGTH, MEAS_BARO_LENGTH);
	err |= kalman_updateAlloc(&ekf_common.gpsEngine, STATE_LENGTH, MEAS_GPS_LENGTH);

	err |= kmn_configRead(EKF_CONFIG_FILE, &ekf_common.initVals);

	if (err != 0) {
		sensc_deinit();
		pthread_mutex_destroy(&ekf_common.lock);
		pthread_attr_destroy(&ekf_common.threadAttr);

		kalman_predictDealloc(&ekf_common.stateEngine);
		kalman_updateDealloc(&ekf_common.imuEngine);
		kalman_updateDealloc(&ekf_common.baroEngine);
		kalman_updateDealloc(&ekf_common.gpsEngine);

		return -1;
	}

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
		case 5:
			logFlags = EKFLOG_GPS_POS;
			break;
		case 6:
			logFlags = EKFLOG_GPS_MEAS;
			break;
		default:
			logFlags = 0;
	}

	if (ekflog_init("ekf_log.txt", logFlags) != 0) {
		pthread_mutex_destroy(&ekf_common.lock);
		pthread_attr_destroy(&ekf_common.threadAttr);
		sensc_deinit();

		kalman_predictDealloc(&ekf_common.stateEngine);
		kalman_updateDealloc(&ekf_common.imuEngine);
		kalman_updateDealloc(&ekf_common.baroEngine);
		kalman_updateDealloc(&ekf_common.gpsEngine);

		return -1;
	}

	meas_imuCalib();
	meas_baroCalib();
	meas_gpsCalib();

	kmn_predInit(&ekf_common.stateEngine, meas_calibGet(), &ekf_common.initVals);
	kmn_imuEngInit(&ekf_common.imuEngine, &ekf_common.initVals);
	kmn_baroEngInit(&ekf_common.baroEngine, &ekf_common.initVals);
	kmn_gpsEngInit(&ekf_common.gpsEngine, &ekf_common.initVals);

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


static void *ekf_thread(void *arg)
{
	static time_t lastBaroUpdate = 0, lastGpsUpdate = 0;
	time_t loopStep, updateStep;
	update_engine_t *currUpdate;
	int i = 0;

	printf("ekf: starting ekf thread\n");

	ekf_common.run = 1;

	/* Kalman loop */
	gettime(&ekf_common.lastTime, NULL);

	lastBaroUpdate = ekf_common.lastTime;
	lastGpsUpdate = ekf_common.lastTime;

	while (ekf_common.run == 1) {
		usleep(1000);
		loopStep = ekf_dtGet();

		/* IMU polling is done regardless on update procedure */
		meas_imuPoll();
		currUpdate = &ekf_common.imuEngine;
		updateStep = loopStep;

		/* Update step selection */
		if (ekf_common.currTime - lastGpsUpdate > GPS_UPDATE_TIMEOUT) {
			if (meas_gpsPoll() == 0) {
				currUpdate = &ekf_common.gpsEngine;
				updateStep = ekf_common.currTime - lastGpsUpdate;
				lastGpsUpdate = ekf_common.currTime;
			}
		}
		else if (ekf_common.currTime - lastBaroUpdate > BARO_UPDATE_TIMEOUT) {
			if (meas_baroPoll() == 0) {
				currUpdate = &ekf_common.baroEngine;
				updateStep = ekf_common.currTime - lastBaroUpdate;
				lastBaroUpdate = ekf_common.currTime;
			}
		}

		/* State prediction procedure */
		kalman_predict(&ekf_common.stateEngine, loopStep, 0);

		/* TODO: make critical section smaller and only on accesses to state and cov matrices */
		pthread_mutex_lock(&ekf_common.lock);
		kalman_update(updateStep, 0, currUpdate, &ekf_common.stateEngine); /* baro measurements update procedure */
		pthread_mutex_unlock(&ekf_common.lock);

		if (i++ > 50) {
			quat_t q = { .a = kmn_vecAt(&ekf_common.stateEngine.state, QA), .i = kmn_vecAt(&ekf_common.stateEngine.state, QB), .j = kmn_vecAt(&ekf_common.stateEngine.state, QC), .k = kmn_vecAt(&ekf_common.stateEngine.state, QD) };
			vec_t a = { .x = kmn_vecAt(&ekf_common.stateEngine.U, UAX), .y = kmn_vecAt(&ekf_common.stateEngine.U, UAY), .z = kmn_vecAt(&ekf_common.stateEngine.U, UAZ) };
			vec_t pos = { .x = kmn_vecAt(&ekf_common.stateEngine.state, RX), .y = kmn_vecAt(&ekf_common.stateEngine.state, RY), .z = kmn_vecAt(&ekf_common.stateEngine.state, RZ) };
			float yaw, pitch, roll;

			quat_quat2euler(&q, &roll, &pitch, &yaw);

			quat_vecRot(&a, &q);
			a.z += EARTH_G;

			ekflog_write(
				EKFLOG_EKF_IMU,
				"%lli %.3f %.3f %.3f %.2f %.2f %.2f %.3f %.3f  %.3f\n",
				ekf_common.currTime, a.x, a.y, a.z, pos.x, pos.y, pos.z, yaw, pitch, roll);
			i = 0;
		}
	}

	ekf_common.run = -1;

	return NULL;
}


int ekf_run(void)
{
	int res = pthread_create(&ekf_common.tid, &ekf_common.threadAttr, ekf_thread, NULL);

	pthread_attr_destroy(&ekf_common.threadAttr);

	/* Wait to stabilize data in covariance matrixes */
	sleep(3);

	return res;
}


int ekf_stop(void)
{
	if (ekf_common.run == 1) {
		ekf_common.run = 0;
	}

	return pthread_join(ekf_common.tid, NULL);
}


void ekf_done(void)
{
	kalman_predictDealloc(&ekf_common.stateEngine);
	kalman_updateDealloc(&ekf_common.imuEngine);
	kalman_updateDealloc(&ekf_common.baroEngine);
	kalman_updateDealloc(&ekf_common.gpsEngine);

	sensc_deinit();
	ekflog_done();
	pthread_mutex_destroy(&ekf_common.lock);
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
	pthread_mutex_lock(&ekf_common.lock);

	/* save quaternion attitude */
	q.a = ekfState->q0 = ekf_common.stateEngine.state.data[QA];
	q.i = ekfState->q1 = ekf_common.stateEngine.state.data[QB];
	q.j = ekfState->q2 = ekf_common.stateEngine.state.data[QC];
	q.k = ekfState->q3 = ekf_common.stateEngine.state.data[QD];

	/* save newtonian motion parameters with frame change from NED to ENU */
	ekfState->enuX = kmn_vecAt(&ekf_common.stateEngine.state, RY);
	ekfState->enuY = kmn_vecAt(&ekf_common.stateEngine.state, RX);
	ekfState->enuZ = -kmn_vecAt(&ekf_common.stateEngine.state, RZ);

	ekfState->veloX = kmn_vecAt(&ekf_common.stateEngine.state, VY);
	ekfState->veloY = kmn_vecAt(&ekf_common.stateEngine.state, VX);
	ekfState->veloZ = -kmn_vecAt(&ekf_common.stateEngine.state, VZ);

	ekfState->accelX = kmn_vecAt(&ekf_common.stateEngine.U, UAX);
	ekfState->accelY = kmn_vecAt(&ekf_common.stateEngine.U, UAY);
	ekfState->accelZ = kmn_vecAt(&ekf_common.stateEngine.U, UAZ);

	ekfState->rollDot = ekf_common.stateEngine.U.data[UWX] - ekf_common.stateEngine.state.data[BWX];
	ekfState->pitchDot = ekf_common.stateEngine.U.data[UWY] - ekf_common.stateEngine.state.data[BWY];
	ekfState->yawDot = ekf_common.stateEngine.U.data[UWZ] - ekf_common.stateEngine.state.data[BWZ];

	ekfState->accelBiasZ = 0;

	pthread_mutex_unlock(&ekf_common.lock);

	/* calculate and save euler attitude */
	quat_quat2euler(&q, &ekfState->roll, &ekfState->pitch, &ekfState->yaw);
}
