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
#include <string.h>

#include "kalman_core.h"
#include "kalman_implem.h"
#include "logs/writer.h"
#include "meas.h"

#include <sensc.h>
#include <vec.h>
#include <quat.h>
#include <matrix.h>

#include "ekflib.h"
#include "filters.h"

#define EKF_CONFIG_FILE "etc/ekf.conf"
#define EKF_LOG_FILE    "ekf_log.bin"
#define SENSOR_FILE     "/dev/sensors"

#define STACK_SIZE 16384


struct {
	kalman_init_t initVals;
	int status;

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

	/* benchmarking */
	time_t stateTime; /* last state estimation timestamp */
	time_t imuTime;   /* timestamp of last used IMU sample */

	char stack[STACK_SIZE] __attribute__((aligned(8)));
} ekf_common;


static int ekf_threadAttrInit(void)
{

/* On Phoenix-RTOS we want to set thread priority */
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


/* Function wraps meas initialization. Adds some additional security checks. */
static int ekf_measGate(int initFlags)
{
	switch (ekf_common.initVals.measSource) {
		case srcSens:
			/*
			 * Initializing with sensors as data source only if EKF_INIT_SENC_SCR init flag is specified
			 * and data source from `ekf.conf` is equal to SENSORS.
			 */

			if ((initFlags & EKF_INIT_LOG_SRC) != 0) {
				fprintf(stderr, "Ekf config: inconsistent data source specifiers\n");
				return -1;
			}

			return meas_init(srcSens, SENSOR_FILE, SENSC_INIT_IMU | SENSC_INIT_BARO | SENSC_INIT_GPS);

		case srcLog:
			/*
			 * Initializing with logs as data source only if EKF_INIT_LOG_SCR init flag is specified,
			 * data source from `ekf.conf` is equal to LOGS and file from `ekf.conf` is different
			 * than file to which logs are collected.
			 */

			if ((initFlags & EKF_INIT_LOG_SRC) == 0) {
				fprintf(stderr, "Ekf config: inconsistent data source specifiers\n");
				return -1;
			}

			if (strcmp(EKF_LOG_FILE, ekf_common.initVals.sourceFile) == 0) {
				fprintf(stderr, "Ekf config: %s cannot be a data source file\n", EKF_LOG_FILE);
				return -1;
			}

			return meas_init(srcLog, ekf_common.initVals.sourceFile, SENSC_INIT_IMU | SENSC_INIT_BARO | SENSC_INIT_GPS);

		default:
			fprintf(stderr, "Ekf config: unknown meas source type\n");
			return -1;
	}
}


int ekf_init(int initFlags)
{
	int err;

	if (ekf_threadAttrInit() != 0) {
		return -1;
	}

	if (fltr_init() != 0) {
		fprintf(stderr, "ekf: filter init failed\n");
		return -1;
	}

	if (pthread_mutex_init(&ekf_common.lock, NULL) < 0) {
		printf("Cannot create mutex for ekf\n");
		pthread_attr_destroy(&ekf_common.threadAttr);
		return -1;
	}

	err = 0;
	err |= kalman_predictAlloc(&ekf_common.stateEngine, STATE_LENGTH, CTRL_LENGTH);
	err |= kalman_updateAlloc(&ekf_common.imuEngine, STATE_LENGTH, MEAS_IMU_LENGTH);
	err |= kalman_updateAlloc(&ekf_common.baroEngine, STATE_LENGTH, MEAS_BARO_LENGTH);
	err |= kalman_updateAlloc(&ekf_common.gpsEngine, STATE_LENGTH, MEAS_GPS_LENGTH);

	err |= kmn_configRead(EKF_CONFIG_FILE, &ekf_common.initVals);

	/* activate update models selected in `initVals` */
	ekf_common.imuEngine.active = ((ekf_common.initVals.modelFlags & KMN_UPDT_IMU) != 0);
	ekf_common.baroEngine.active = ((ekf_common.initVals.modelFlags & KMN_UPDT_BARO) != 0);
	ekf_common.gpsEngine.active = ((ekf_common.initVals.modelFlags & KMN_UPDT_GPS) != 0);

	/* IMU calibration is obligatory */
	if (!ekf_common.imuEngine.active) {
		fprintf(stderr, "ekf: imu update not enabled\n");
		err = -1;
	}

	if (err != 0) {
		pthread_mutex_destroy(&ekf_common.lock);
		pthread_attr_destroy(&ekf_common.threadAttr);

		kalman_predictDealloc(&ekf_common.stateEngine);
		kalman_updateDealloc(&ekf_common.imuEngine);
		kalman_updateDealloc(&ekf_common.baroEngine);
		kalman_updateDealloc(&ekf_common.gpsEngine);

		return -1;
	}

	ekf_common.run = 0;
	ekf_common.status = 0;

	if (ekf_measGate(initFlags) != 0) {
		pthread_mutex_destroy(&ekf_common.lock);
		pthread_attr_destroy(&ekf_common.threadAttr);

		kalman_predictDealloc(&ekf_common.stateEngine);
		kalman_updateDealloc(&ekf_common.imuEngine);
		kalman_updateDealloc(&ekf_common.baroEngine);
		kalman_updateDealloc(&ekf_common.gpsEngine);

		return -1;
	}

	if (ekflog_writerInit(EKF_LOG_FILE, ekf_common.initVals.log | ekf_common.initVals.logMode) != 0) {
		pthread_mutex_destroy(&ekf_common.lock);
		pthread_attr_destroy(&ekf_common.threadAttr);
		meas_done();

		kalman_predictDealloc(&ekf_common.stateEngine);
		kalman_updateDealloc(&ekf_common.imuEngine);
		kalman_updateDealloc(&ekf_common.baroEngine);
		kalman_updateDealloc(&ekf_common.gpsEngine);

		return -1;
	}

	if (meas_imuCalib() != 0) {
		printf("ekf: error during IMU calibration\n");
		err = -1;
	}

	if (ekf_common.baroEngine.active) {
		if (meas_baroCalib() != 0) {
			printf("ekf: error during baro calibration\n");
			err = -1;
		}
	}
	if (ekf_common.gpsEngine.active) {
		if (meas_gpsCalib() != 0) {
			printf("ekf: error during GPS calibration\n");
			err = -1;
		}
	}

	if (err != 0) {
		ekf_done();
		return -1;
	}

	/* obligatory engines initialization */
	kmn_predInit(&ekf_common.stateEngine, meas_calibGet(), &ekf_common.initVals);
	kmn_imuEngInit(&ekf_common.imuEngine, &ekf_common.initVals);

	/* supplementary engines initialization */
	kmn_baroEngInit(&ekf_common.baroEngine, &ekf_common.initVals);
	kmn_gpsEngInit(&ekf_common.gpsEngine, &ekf_common.initVals);

	return 0;
}


static int ekf_dtGet(time_t *result)
{
	time_t tmp;

	if (meas_timeGet(&tmp) != 0) {
		return -1;
	}
	ekf_common.currTime = tmp;
	*result = ekf_common.currTime - ekf_common.lastTime;
	ekf_common.lastTime = ekf_common.currTime;

	return 0;
}


static int ekf_pollErrHandle(void)
{
	ekf_common.status |= (errno == 0) ? EKF_MEAS_EOF : EKF_ERROR;

	/* If EKF is running from logs, then EOF results in stopping main loop */
	return (ekf_common.initVals.measSource == srcLog) ? -1 : 1;
}


static void *ekf_thread(void *arg)
{
	static time_t lastBaroUpdate = 0, lastGpsUpdate = 0;
	time_t imuTime;
	time_t loopStep, updateStep;
	update_engine_t *currUpdate;
	int i = 0;

	printf("ekf: starting ekf thread\n");

	errno = 0;
	ekf_common.run = 1;

	/* Kalman loop */
	if (meas_timeGet(&ekf_common.lastTime) != 0) {
		ekf_common.run = ekf_pollErrHandle();
	}

	lastBaroUpdate = ekf_common.lastTime;
	lastGpsUpdate = ekf_common.lastTime;

	while (ekf_common.run == 1) {
		usleep(1000);

		if (ekf_dtGet(&loopStep) != 0) {
			ekf_common.run = ekf_pollErrHandle();
		}

		/* IMU polling is done regardless on update procedure */
		if (meas_imuPoll(&imuTime) != 0) {
			ekf_common.run = ekf_pollErrHandle();
		}
		currUpdate = &ekf_common.imuEngine;
		updateStep = loopStep;

		/* Update step selection */
		if (ekf_common.currTime - lastBaroUpdate > BARO_UPDATE_TIMEOUT && ekf_common.baroEngine.active) {
			if (meas_baroPoll() == 0) {
				currUpdate = &ekf_common.baroEngine;
				updateStep = ekf_common.currTime - lastBaroUpdate;
				lastBaroUpdate = ekf_common.currTime;
			}
			else {
				ekf_common.run = ekf_pollErrHandle();
			}
		}

		if (ekf_common.currTime - lastGpsUpdate > GPS_UPDATE_TIMEOUT && ekf_common.gpsEngine.active) {
			if (meas_gpsPoll() == 0) {
				currUpdate = &ekf_common.gpsEngine;
				updateStep = ekf_common.currTime - lastGpsUpdate;
				lastGpsUpdate = ekf_common.currTime;
			}
			else {
				ekf_common.run = ekf_pollErrHandle();
			}
		}

		if (ekf_common.run != 1) {
			break;
		}

		/* State prediction procedure */
		kalman_predict(&ekf_common.stateEngine, loopStep, 0);

		/* TODO: make critical section smaller and only on accesses to state and cov matrices */
		pthread_mutex_lock(&ekf_common.lock);
		kalman_update(updateStep, 0, currUpdate, &ekf_common.stateEngine); /* baro measurements update procedure */
		ekf_common.imuTime = imuTime;                                      /* assigning here not at gettime to utilize locked mutex */
		pthread_mutex_unlock(&ekf_common.lock);

		if (i++ > 50) {
			quat_t q = { .a = kmn_vecAt(&ekf_common.stateEngine.state, QA), .i = kmn_vecAt(&ekf_common.stateEngine.state, QB), .j = kmn_vecAt(&ekf_common.stateEngine.state, QC), .k = kmn_vecAt(&ekf_common.stateEngine.state, QD) };
			vec_t a = { .x = kmn_vecAt(&ekf_common.stateEngine.U, UAX), .y = kmn_vecAt(&ekf_common.stateEngine.U, UAY), .z = kmn_vecAt(&ekf_common.stateEngine.U, UAZ) };
			float yaw, pitch, roll;

			quat_quat2euler(&q, &roll, &pitch, &yaw);

			quat_vecRot(&a, &q);
			a.z += EARTH_G;

			i = 0;
		}

		/* using pre-calculation time as to not call meas_timeGet() */
		ekf_common.stateTime = ekf_common.currTime;
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

	meas_done();
	ekflog_writerDone();
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

	ekfState->status = 0;
	ekfState->status |= ekf_common.status;

	if (ekf_common.run == 1) {
		ekfState->status |= EKF_RUNNING;
	}

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

	ekfState->stateTime = ekf_common.stateTime;
	ekfState->imuTime = ekf_common.imuTime;

	pthread_mutex_unlock(&ekf_common.lock);

	/* calculate and save euler attitude */
	quat_quat2euler(&q, &ekfState->roll, &ekfState->pitch, &ekfState->yaw);
}
