/*
 * Phoenix-Pilot
 *
 * Extended kalman Filter 
 * 
 * EKF implementation specific code. Defines:
 *  - prediction engine functions and initializations
 *  - measurement engines initializations
 *
 * Copyright 2022 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "kalman_implem.h"
#include "meas.h"

#include <vec.h>
#include <quat.h>
#include <matrix.h>


/* NOTE: must be kept in the same order as 'configNames' */
static const kalman_init_t initTemplate = {
	.verbose = 0,

	.P_xerr = 1,               /* 1 m */
	.P_verr = 0.1,             /* 0.1 m/s */
	.P_aerr = 0.1,             /* 0.001 m/s^2 */
	.P_werr = DEG2RAD,         /* 1 degree */
	.P_merr = 300,             /* 300 uT */
	.P_qaerr = 10 * DEG2RAD,   /* 10 degrees */
	.P_qijkerr = 10 * DEG2RAD, /* 10 degrees */
	.P_pxerr = 0.01,           /* 10 hPa */

	.R_acov = 10,               /* 10 m/s^2 with engines on */
	.R_wcov = 0.01,            /* 1 milliradian/s, overtrust gyroscope for fast response */
	.R_mcov = 0.1,
	.R_qcov = 500,

	.R_xzcov = 0.075,
	.R_vzcov = 0.2,

	/* better to keep Q low */
	.Q_xcov = 2,
	.Q_vcov = 2,
	.Q_hcov = 0.001,
	.Q_avertcov = 1,
	.Q_ahoricov = 0.1,
	.Q_wcov = 0.001,
	.Q_mcov = 0.001,
	.Q_qcov = 0.001
};


/* NOTE: must be kept in the same order as 'init_values' */
static const char *configNames[] = {
	"verbose",
	"P_xerr", "P_verr", "P_aerr", "P_werr", "P_merr", "P_qaerr", "P_qijkerr", "P_pxerr",
	"R_acov", "R_wcov", "R_mcov", "R_qcov", "R_xzcov", "R_vzcov",
	"Q_xcov", "Q_vcov", "Q_hcov", "Q_avertcov", "Q_ahoricov", "Q_wcov", "Q_mcov", "Q_qcov"
};


/* reads config file named "config" from filesystem */
void kmn_configRead(kalman_init_t *initVals)
{
	char buf[32], *p, *v;
	int i;
	float val;
	FILE *fd = fopen("config", "r");
	kalman_init_t inits = initTemplate;

	if (fd == NULL) {
		printf("No config file found!\n");
	}
	else {
		while (fgets(buf, sizeof(buf), fd)) {
			p = strtok(buf, " ");
			v = strtok(NULL, " ");
			val = atof(v);
			for (i = 0; i < sizeof(inits) / sizeof(float); i++) {
				if (memcmp(p, configNames[i], strlen(configNames[i])) == 0) {
					((float *)&inits)[i] = (float)val;
					break;
				}
			}
		}
		fclose(fd);
	}

	printf("config:\n");
	for (i = 0; i < sizeof(inits) / sizeof(float); i++) {
		printf("%s = %f\n", configNames[i], ((float *)&inits)[i]);
	}

	*initVals = inits;
}


/* state vectors values init */
static void init_state_vector(matrix_t *state, const meas_calib_t *calib)
{
	state->data[IXX] = state->data[IXY] = state->data[IXZ] = 0; /* start position at [0,0,0] */
	state->data[IVX] = state->data[IVY] = state->data[IVZ] = 0; /* start velocity at [0,0,0] */
	state->data[IAX] = state->data[IAY] = state->data[IAZ] = 0; /* start acceleration at [0,0,0] */
	state->data[IWX] = state->data[IWY] = state->data[IWZ] = 0; /* start angular speed at [0,0,0] */

	/* start rotation at identity quaternion */
	state->data[IQA] = calib->imu.initQuat.a;
	state->data[IQB] = calib->imu.initQuat.i;
	state->data[IQC] = calib->imu.initQuat.j;
	state->data[IQD] = calib->imu.initQuat.k;

	/* start magnetic field as calibrated */
	state->data[IMX] = calib->imu.initMag.x;
	state->data[IMX] = calib->imu.initMag.y;
	state->data[IMX] = calib->imu.initMag.z;
}


/* covariance matrox values inits */
static void init_cov_vector(matrix_t *cov, const kalman_init_t *inits)
{
	matrix_zeroes(cov);
	cov->data[cov->cols * IXX + IXX] = inits->P_xerr * inits->P_xerr;
	cov->data[cov->cols * IXY + IXY] = inits->P_xerr * inits->P_xerr;
	cov->data[cov->cols * IXZ + IXZ] = inits->P_xerr * inits->P_xerr;

	cov->data[cov->cols * IVX + IVX] = inits->P_verr * inits->P_verr;
	cov->data[cov->cols * IVY + IVY] = inits->P_verr * inits->P_verr;
	cov->data[cov->cols * IVZ + IVZ] = inits->P_verr * inits->P_verr;

	cov->data[cov->cols * IAX + IAX] = inits->P_aerr * inits->P_aerr;
	cov->data[cov->cols * IAY + IAY] = inits->P_aerr * inits->P_aerr;
	cov->data[cov->cols * IAZ + IAZ] = inits->P_aerr * inits->P_aerr;

	cov->data[cov->cols * IWX + IWX] = inits->P_werr * inits->P_werr;
	cov->data[cov->cols * IWY + IWY] = inits->P_werr * inits->P_werr;
	cov->data[cov->cols * IWZ + IWZ] = inits->P_werr * inits->P_werr;

	cov->data[cov->cols * IQA + IQA] = inits->P_qaerr * inits->P_qaerr;
	cov->data[cov->cols * IQB + IQB] = inits->P_qijkerr * inits->P_qijkerr;
	cov->data[cov->cols * IQC + IQC] = inits->P_qijkerr * inits->P_qijkerr;
	cov->data[cov->cols * IQD + IQD] = inits->P_qijkerr * inits->P_qijkerr;

	cov->data[cov->cols * IMX + IMX] = inits->P_merr * inits->P_merr;
	cov->data[cov->cols * IMY + IMY] = inits->P_merr * inits->P_merr;
	cov->data[cov->cols * IMZ + IMZ] = inits->P_merr * inits->P_merr;
}

/* State estimation function definition */
static void calcStateEstimation(matrix_t *state, matrix_t *state_est, time_t timeStep)
{
	static vec_t last_a = { 0 };
	static vec_t last_v = { 0 };

	float dt, dt2;
	quat_t quat_q, quat_w, res;

	dt = timeStep / 1000000.;
	dt2 = dt * dt / 2;

	quat_q = (quat_t) { .a = QA, .i = QB, .j = QC, .k = QD };
	quat_w = (quat_t) { .a = 0, .i = WX, .j = WY, .k = WZ };

	/* trapezoidal integration */
	state_est->data[IXX] = XX + (VX + last_v.x) * 0.5 * dt + AX * dt2;
	state_est->data[IXY] = XY + (VY + last_v.y) * 0.5 * dt + AY * dt2;
	state_est->data[IXZ] = XZ + (VZ + last_v.z) * 0.5 * dt + AZ * dt2;

	/* trapezoidal integration */
	/* as no direct velocity measurements are done, time corelation is introduced to velocity with assumption that velocity always decreases */
	state_est->data[IVX] = (VX + (AX + last_a.x) * 0.5 * dt) * 0.999;
	state_est->data[IVY] = (VY + (AY + last_a.y) * 0.5 * dt) * 0.999;
	state_est->data[IVZ] = (VZ + (AZ + last_a.z) * 0.5 * dt);

	last_a.x = AX;
	last_a.y = AY;
	last_a.z = AZ;

	last_v.x = VX;
	last_v.y = VY;
	last_v.z = VZ;

	/* predition from w */
	quat_mlt(&quat_w, &quat_q, &res);
	quat_times(&res, dt / 2);
	quat_add(&quat_q, &res);
	quat_normalize(&quat_q);

	state_est->data[IQA] = quat_q.a;
	state_est->data[IQB] = quat_q.i;
	state_est->data[IQC] = quat_q.j;
	state_est->data[IQD] = quat_q.k;

	state_est->data[IAX] = AX;
	state_est->data[IAY] = AY;
	state_est->data[IAZ] = AZ;

	state_est->data[IWX] = WX;
	state_est->data[IWY] = WY;
	state_est->data[IWZ] = WZ;

	state_est->data[IMX] = MX;
	state_est->data[IMY] = MY;
	state_est->data[IMZ] = MZ;
}


/* prediction step jacobian calculation function */
static void calcPredictionJacobian(matrix_t *F, matrix_t *state, time_t timeStep)
{
	float dt, dt2;
	/* differentials matrices */
	matrix_t dfqdq, dfqdw;
	/* diagonal matrix */
	float I33_data[9] = { 0 };
	matrix_t I33 = { .rows = 3, .cols = 3, .transposed = 0, .data = I33_data };

	dt = timeStep / 1000000.F;
	dt2 = dt / 2; /* helper value */

	/* derrivative submatrix of (dfq / dq) of size 4x4 */
	float wxdt2 = WX * dt2, wydt2 = WY * dt2, wzdt2 = WZ * dt2;
	float data_dfqdq[16] = {
		1, -wxdt2, -wydt2, -wzdt2,
		wxdt2, 1, -wzdt2, wydt2,
		wydt2, wzdt2, 1, -wxdt2,
		wzdt2, -wydt2, wxdt2, 1
	};
	/* derrivative submatrix of (dfq / dw) of size 4x3 */
	float qadt2 = QA * dt2, qbdt2 = QB * dt2, qcdt2 = QC * dt2, qddt2 = QD * dt2;
	float data_dfqdw[12] = {
		-qbdt2, -qcdt2, -qddt2,
		qadt2, qddt2, -qcdt2,
		-qddt2, qadt2, qbdt2,
		qcdt2, -qbdt2, qadt2
	};

	dfqdq = (matrix_t) { .rows = 4, .cols = 4, .transposed = 0, .data = data_dfqdq };
	dfqdw = (matrix_t) { .rows = 4, .cols = 3, .transposed = 0, .data = data_dfqdw };

	matrix_diag(&I33);

	/* set F to zeroes and add ones on diag */
	matrix_zeroes(F);
	matrix_diag(F);

	/* change I33 to (I * dt) matrix and write it to appropriate places */
	matrix_times(&I33, dt);
	matrix_writeSubmatrix(F, IXX, IVX, &I33); /* dfx / dv */
	matrix_writeSubmatrix(F, IVX, IAX, &I33); /* dfv / da */

	/* change I33 to (I * dt^2 / 2) matrix and write it to appropriate places */
	matrix_times(&I33, dt / 2);
	//matrix_times(&I33, 0.5);
	//matrix_writeSubmatrix(F, IXX, IAX, &I33); /* dfx / dv */

	/* write differentials matrices */
	matrix_writeSubmatrix(F, IQA, IQA, &dfqdq);
	matrix_writeSubmatrix(F, IQA, IWX, &dfqdw);
}


/* initialization of prediction step matrix values */
int kmn_predInit(state_engine_t *engine, const meas_calib_t *calib, const kalman_init_t *inits)
{
	matrix_t *Q;


	if (matrix_bufAlloc(&engine->state, STATE_ROWS, STATE_COLS) != 0) {
		return -1;
	}

	if (matrix_bufAlloc(&engine->state_est, STATE_ROWS, STATE_COLS) != 0) {
		kmn_predDeinit(engine);
		return -1;
	}

	if (matrix_bufAlloc(&engine->cov, STATE_ROWS, STATE_ROWS) != 0) {
		kmn_predDeinit(engine);
		return -1;
	}

	if (matrix_bufAlloc(&engine->cov_est, STATE_ROWS, STATE_ROWS) != 0) {
		kmn_predDeinit(engine);
		return -1;
	}

	if (matrix_bufAlloc(&engine->F, STATE_ROWS, STATE_ROWS) != 0) {
		kmn_predDeinit(engine);
		return -1;
	}

	if (matrix_bufAlloc(&engine->Q, STATE_ROWS, STATE_ROWS) != 0) {
		kmn_predDeinit(engine);
		return -1;
	}

	init_state_vector(&engine->state, calib);
	init_cov_vector(&engine->cov, inits);

	/* prepare noise matrix Q */
	matrix_zeroes(&engine->Q);
	Q = &engine->Q;
	Q->data[Q->cols * IXX + IXX] = Q->data[Q->cols * IXY + IXY] = inits->Q_xcov;
	Q->data[Q->cols * IVX + IVX] = Q->data[Q->cols * IVY + IVY] = inits->Q_vcov;

	Q->data[Q->cols * IAX + IAX] = Q->data[Q->cols * IAY + IAY] = inits->Q_ahoricov;
	Q->data[Q->cols * IAZ + IAZ] = inits->Q_avertcov;

	Q->data[Q->cols * IWX + IWX] = Q->data[Q->cols * IWY + IWY] = Q->data[Q->cols * IWZ + IWZ] = inits->Q_wcov;
	Q->data[Q->cols * IMX + IMX] = Q->data[Q->cols * IMY + IMY] = Q->data[Q->cols * IMZ + IMZ] = inits->Q_mcov;
	Q->data[Q->cols * IQA + IQA] = inits->Q_qcov;
	Q->data[Q->cols * IQB + IQB] = inits->Q_qcov;
	Q->data[Q->cols * IQC + IQC] = inits->Q_qcov;
	Q->data[Q->cols * IQD + IQD] = inits->Q_qcov;
	Q->data[Q->cols * IXZ + IXZ] = inits->Q_hcov;

	/* save function pointers */
	engine->estimateState = calcStateEstimation;
	engine->getJacobian = calcPredictionJacobian;

	return 0;
}


void kmn_predDeinit(state_engine_t *engine)
{
	matrix_bufFree(&engine->state);
	matrix_bufFree(&engine->state_est);
	matrix_bufFree(&engine->cov);
	matrix_bufFree(&engine->cov_est);
	matrix_bufFree(&engine->F);
	matrix_bufFree(&engine->Q);
}
