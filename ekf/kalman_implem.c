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

#include <vec.h>
#include <quat.h>
#include <matrix.h>


/* NOTE: must be kept in the same order as 'config_names' */
kalman_init_t init_values = {
	.verbose = 0,

	.P_xerr = 0.1,             /* 0.1 m */
	.P_verr = 0.1,             /* 0.1 m/s */
	.P_aerr = 0.01,            /* 0.001 m/s^2 */
	.P_werr = DEG2RAD,         /* 1 degree */
	.P_merr = 300,             /* 300 uT */
	.P_qaerr = 10 * DEG2RAD,   /* 10 degrees */
	.P_qijkerr = 10 * DEG2RAD, /* 10 degrees */
	.P_pxerr = 0.01,           /* 10 hPa */

	.R_acov = 0.1,
	.R_wcov = 0.01,
	.R_mcov = 10,
	.R_qcov = 0.1,

	.R_pcov = 0.1,
	.R_hcov = 1,
	.R_xzcov = 1,
	.R_hvcov = 0.1,
	.R_vzcov = 2,

	/* better to keep Q low */
	.Q_xcov = 0.001,
	.Q_vcov = 0.001,
	.Q_hcov = 0.001,
	.Q_avertcov = 0.0001,
	.Q_ahoricov = 0.0001,
	.Q_wcov = 0.0001,
	.Q_mcov = 0.001,
	.Q_qcov = 0.001,
	.Q_pcov = 0.01,
	.Q_pvcov = 0.001
};


/* NOTE: must be kept in the same order as 'init_values' */
char *config_names[] = {
	"verbose",
	"P_xerr", "P_verr", "P_aerr", "P_werr", "P_merr", "P_qaerr", "P_qijkerr", "P_pxerr",
	"R_acov", "R_wcov", "R_mcov", "R_qcov", "R_pcov", "R_hcov", "R_xzcov", "R_hvcov", "R_vzcov",
	"Q_xcov", "Q_vcov", "Q_hcov", "Q_avertcov", "Q_ahoricov", "Q_wcov", "Q_mcov", "Q_qcov", "Q_pcov", "Q_pvcov"
};


/* reads config file named "config" from filesystem */
void read_config(void)
{
	char buf[32], *p, *v;
	int i;
	float val;
	FILE *fd = fopen("config", "r");
	if (fd == NULL) {
		printf("No config file found!\n");
	}
	else {
		while (fgets(buf, sizeof(buf), fd)) {
			p = strtok(buf, " ");
			v = strtok(NULL, " ");
			val = atof(v);
			for (i = 0; i < sizeof(init_values) / sizeof(float); i++) {
				if (memcmp(p, config_names[i], strlen(config_names[i])) == 0) {
					((float *)&init_values)[i] = (float)val;
					break;
				}
			}
		}
		fclose(fd);
	}
	verbose = init_values.verbose;

	printf("config:\n");
	for (i = 0; i < sizeof(init_values) / sizeof(float); i++) {
		printf("%s = %f\n", config_names[i], ((float *)&init_values)[i]);
	}
}


/* state vectors values init */
static void init_state_vector(matrix_t *state, const kalman_calib_t *calib)
{
	state->data[ixx] = state->data[ixy] = state->data[ixz] = 0; /* start position at [0,0,0] */
	state->data[ivx] = state->data[ivy] = state->data[ivz] = 0; /* start velocity at [0,0,0] */
	state->data[iax] = state->data[iay] = state->data[iaz] = 0; /* start acceleration at [0,0,0] */
	state->data[iwx] = state->data[iwy] = state->data[iwz] = 0; /* start angular speed at [0,0,0] */

	/* start rotation at identity quaternion */
	state->data[iqa] = calib->init_q.a;
	state->data[iqb] = calib->init_q.i;
	state->data[iqc] = calib->init_q.j;
	state->data[iqd] = calib->init_q.k;

	/* start magnetic field as calibrated */
	state->data[imx] = calib->init_m.x;
	state->data[imx] = calib->init_m.y;
	state->data[imx] = calib->init_m.z;

	/* start pressure set to 1013 hPa */
	state->data[ihz] = 0;
	state->data[ihv] = 0;
}


/* covariance matrox values inits */
static void init_cov_vector(matrix_t *cov)
{
	matrix_zeroes(cov);
	cov->data[cov->cols * ixx + ixx] = init_values.P_xerr * init_values.P_xerr;
	cov->data[cov->cols * ixy + ixy] = init_values.P_xerr * init_values.P_xerr;
	cov->data[cov->cols * ixz + ixz] = init_values.P_xerr * init_values.P_xerr;

	cov->data[cov->cols * ivx + ivx] = init_values.P_verr * init_values.P_verr;
	cov->data[cov->cols * ivy + ivy] = init_values.P_verr * init_values.P_verr;
	cov->data[cov->cols * ivz + ivz] = init_values.P_verr * init_values.P_verr;

	cov->data[cov->cols * iax + iax] = init_values.P_aerr * init_values.P_aerr;
	cov->data[cov->cols * iay + iay] = init_values.P_aerr * init_values.P_aerr;
	cov->data[cov->cols * iaz + iaz] = init_values.P_aerr * init_values.P_aerr;

	cov->data[cov->cols * iwx + iwx] = init_values.P_werr * init_values.P_werr;
	cov->data[cov->cols * iwy + iwy] = init_values.P_werr * init_values.P_werr;
	cov->data[cov->cols * iwz + iwz] = init_values.P_werr * init_values.P_werr;

	cov->data[cov->cols * iqa + iqa] = init_values.P_qaerr * init_values.P_qaerr;
	cov->data[cov->cols * iqb + iqb] = init_values.P_qijkerr * init_values.P_qijkerr;
	cov->data[cov->cols * iqc + iqc] = init_values.P_qijkerr * init_values.P_qijkerr;
	cov->data[cov->cols * iqd + iqd] = init_values.P_qijkerr * init_values.P_qijkerr;

	cov->data[cov->cols * imx + imx] = init_values.P_merr * init_values.P_merr;
	cov->data[cov->cols * imy + imy] = init_values.P_merr * init_values.P_merr;
	cov->data[cov->cols * imz + imz] = init_values.P_merr * init_values.P_merr;

	cov->data[cov->cols * ihz + ihz] = init_values.P_pxerr * init_values.P_pxerr;
	cov->data[cov->cols * ihv + ihv] = init_values.P_verr * init_values.P_verr;
}

vec_t last_a = { 0 };
vec_t last_v = { 0 };

/* State estimation function definition */
static void calcStateEstimation(matrix_t *state, matrix_t *state_est, time_t timeStep)
{
	float dt, dt2;
	quat_t quat_q, quat_w, res;

	dt = timeStep / 1000000.;
	dt2 = dt * dt / 2;

	quat_q = (quat_t) { .a = qa, .i = qb, .j = qc, .k = qd };
	quat_w = (quat_t) { .a = 0, .i = wx, .j = wy, .k = wz };

	/* trapezoidal integration */
	state_est->data[ixx] = xx + (vx + last_v.x) * 0.5 * dt + ax * dt2;
	state_est->data[ixy] = xy + (vy + last_v.y) * 0.5 * dt + ay * dt2;
	state_est->data[ixz] = xz + (vz + last_v.z) * 0.5 * dt + az * dt2;

	/* trapezoidal integration */
	/* as no direct velocity measurements are done, time corelation is introduced to velocity with assumption that velocity always decreases */
	state_est->data[ivx] = (vx + (ax + last_a.x) * 0.5 * dt) * 0.9995;
	state_est->data[ivy] = (vy + (ay + last_a.y) * 0.5 * dt) * 0.9995;
	state_est->data[ivz] = (vz + (az + last_a.z) * 0.5 * dt) * 0.9995;

	last_a.x = ax;
	last_a.y = ay;
	last_a.z = az;

	last_v.x = vx;
	last_v.y = vy;
	last_v.z = vz;

	/* predition from w */
	quat_mlt(&quat_w, &quat_q, &res);
	quat_times(&res, dt / 2);
	quat_add(&quat_q, &res);
	quat_normalize(&quat_q);

	state_est->data[iqa] = quat_q.a;
	state_est->data[iqb] = quat_q.i;
	state_est->data[iqc] = quat_q.j;
	state_est->data[iqd] = quat_q.k;

	state_est->data[iax] = ax * 0.9995;
	state_est->data[iay] = ay * 0.9995;
	state_est->data[iaz] = az * 0.9995;

	state_est->data[iwx] = wx;
	state_est->data[iwy] = wy;
	state_est->data[iwz] = wz;

	state_est->data[imx] = mx;
	state_est->data[imy] = my;
	state_est->data[imz] = mz;

	state_est->data[ihz] = hz;
	state_est->data[ihv] = hv;
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
	float wxdt2 = wx * dt2, wydt2 = wy * dt2, wzdt2 = wz * dt2;
	float data_dfqdq[16] = {
		1, -wxdt2, -wydt2, -wzdt2,
		wxdt2, 1, -wzdt2, wydt2,
		wydt2, wzdt2, 1, -wxdt2,
		wzdt2, -wydt2, wxdt2, 1
	};
	/* derrivative submatrix of (dfq / dw) of size 4x3 */
	float qadt2 = qa * dt2, qbdt2 = qb * dt2, qcdt2 = qc * dt2, qddt2 = qd * dt2;
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
	matrix_writeSubmatrix(F, ixx, ivx, &I33); /* dfx / dv */
	matrix_writeSubmatrix(F, ivx, iax, &I33); /* dfv / da */

	/* change I33 to (I * dt^2 / 2) matrix and write it to appropriate places */
	matrix_times(&I33, dt / 2);
	//matrix_times(&I33, 0.5);
	//matrix_writeSubmatrix(F, ixx, iax, &I33); /* dfx / dv */

	/* write differentials matrices */
	matrix_writeSubmatrix(F, iqa, iqa, &dfqdq);
	matrix_writeSubmatrix(F, iqa, iwx, &dfqdw);

	F->data[ihz * F->cols + ihz] = 1;
	F->data[ihz * F->cols + ivz] = dt;
	F->data[ihv * F->cols + ihv] = 1;
}


/* initialization of prediction step matrix values */
int kmn_predInit(state_engine_t *engine, const kalman_calib_t *calib)
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
	init_cov_vector(&engine->cov);

	/* prepare noise matrix Q */
	matrix_zeroes(&engine->Q);
	Q = &engine->Q;
	Q->data[Q->cols * ixx + ixx] = Q->data[Q->cols * ixy + ixy] = init_values.Q_xcov;
	Q->data[Q->cols * ivx + ivx] = Q->data[Q->cols * ivy + ivy] = init_values.Q_vcov;

	Q->data[Q->cols * iax + iax] = Q->data[Q->cols * iay + iay] = init_values.Q_ahoricov;
	Q->data[Q->cols * iaz + iaz] = init_values.Q_avertcov;

	Q->data[Q->cols * iwx + iwx] = Q->data[Q->cols * iwy + iwy] = Q->data[Q->cols * iwz + iwz] = init_values.Q_wcov;
	Q->data[Q->cols * imx + imx] = Q->data[Q->cols * imy + imy] = Q->data[Q->cols * imz + imz] = init_values.Q_mcov;
	Q->data[Q->cols * iqa + iqa] = init_values.Q_qcov;
	Q->data[Q->cols * iqb + iqb] = init_values.Q_qcov;
	Q->data[Q->cols * iqc + iqc] = init_values.Q_qcov;
	Q->data[Q->cols * iqd + iqd] = init_values.Q_qcov;
	Q->data[Q->cols * ihz + ihz] = init_values.Q_hcov;
	Q->data[Q->cols * ixz + ixz] = init_values.Q_hcov;
	Q->data[Q->cols * ihv + ihv] = init_values.Q_pvcov;

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
