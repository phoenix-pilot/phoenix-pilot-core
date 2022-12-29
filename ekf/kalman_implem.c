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


struct {
	const kalman_init_t *inits;
	vec_t gyroBiasBypass;
	float gLength;
} pred_common;


/* NOTE: must be kept in the same order as 'configNames' */
static const kalman_init_t initTemplate = {
	.verbose = 0,
	.log = 0,

	.P_qerr = 10 * DEG2RAD,   /* 10 degrees */
	.P_verr = 1,
	.P_baerr = 1,
	.P_bwerr = 1,

	.R_astdev = 0.1,     /* standard deviation of accelerometer reading in m/s */
	.R_mstdev = 1,       /* standard deviation of magnetometer reading in milligauss */
	.R_bwstdev = 0.0001, /* standard deviation of gyroscope bias estimation in radians */

	.R_dzstdev = 0.02, /* standard deviation of change in height in meters */

	.Q_astdev = 1, /* 1m/s^2 process noise of velocity */
	.Q_wstdev = 0.9,
	.Q_baDotstdev = 0.001,
	.Q_bwDotstdev = 0.001
};


/* NOTE: must be kept in the same order as 'init_values' */
static const char *configNames[] = {
	"verbose", "log",
	"P_qerr", "P_verr", "P_baerr", "P_bwerr",
	"R_astdev", "R_mstdev", "R_bwstdev",
	"R_dzstdev",
	"Q_astdev", "Q_wstdev", "Q_baDotstdev", "Q_bwDotstdev"
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


static matrix_t *kmn_getCtrl(matrix_t *U)
{
	vec_t accel, gyro, mag;
	time_t tstamp;

	meas_imuGet(&accel, &gyro, &mag, &tstamp);

	*matrix_at(U, UWX, 0) = gyro.x;
	*matrix_at(U, UWY, 0) = gyro.y;
	*matrix_at(U, UWZ, 0) = gyro.z;
	*matrix_at(U, UAX, 0) = accel.x;
	*matrix_at(U, UAY, 0) = accel.y;
	*matrix_at(U, UAZ, 0) = accel.z;

	return U;
}


/* State estimation function definition */
static void kmn_stateEst(matrix_t *state, matrix_t *state_est, matrix_t *U, time_t timeStep)
{
	/* values from state vector */
	const quat_t qState = {.a = kmn_vecAt(state, QA), .i = kmn_vecAt(state, QB), .j = kmn_vecAt(state, QC), .k = kmn_vecAt(state, QD)};
	const quat_t bwState = {.a = 0, .i = kmn_vecAt(state, BWX), .j = kmn_vecAt(state, BWY), .k = kmn_vecAt(state, BWZ)}; /* quaternionized vector */
	const vec_t vState = {.x = kmn_vecAt(state, VX), .y = kmn_vecAt(state, VY), .z = kmn_vecAt(state, VZ)};
	const vec_t baState = {.x = kmn_vecAt(state, BAX), .y = kmn_vecAt(state, BAY), .z = kmn_vecAt(state, BAZ)};

	/* quaternionized angular rate from U vector */
	quat_t wMeas = {.a = 0, .i = kmn_vecAt(U, UWX), .j = kmn_vecAt(U, UWY), .k = kmn_vecAt(U, UWZ)}; /* quaternionized vector */
	vec_t aMeas = {.x = kmn_vecAt(U, UAX), .y = kmn_vecAt(U, UAY), .z = kmn_vecAt(U, UAZ)};

	/* quaternionized angular rate and rotation quaternion estimates */
	quat_t qEst, qtmp;
	vec_t vEst;

	const float dt = (float)timeStep / 1000000.f;

	/* quaternion estimation: q = q * ( q_iden + h/2 * (wMeas - bw) ) */
	quat_dif(&wMeas, &bwState, &qtmp);
	quat_times(&qtmp, dt / 2);
	qtmp.a += 1;
	quat_mlt(&qState, &qtmp, &qEst);
	quat_normalize(&qEst);

	*matrix_at(state_est, QA, 0) = qEst.a;
	*matrix_at(state_est, QB, 0) = qEst.i;
	*matrix_at(state_est, QC, 0) = qEst.j;
	*matrix_at(state_est, QD, 0) = qEst.k;

	/* gyroscope bias estimation: SIMPLIFICATION: we use constant value as prediction */
	*matrix_at(state_est, BWX, 0) = pred_common.gyroBiasBypass.x;
	*matrix_at(state_est, BWY, 0) = pred_common.gyroBiasBypass.y;
	*matrix_at(state_est, BWZ, 0) = pred_common.gyroBiasBypass.z;

	/* velocity estimation */
	vec_dif(&aMeas, &baState, &vEst);
	quat_vecRot(&vEst, &qState);
	vEst.z += pred_common.gLength; /* Cancel out earth acceleration */
	vec_times(&vEst, dt);
	vec_add(&vEst, &vState);

	*matrix_at(state_est, VX, 0) = 0;
	*matrix_at(state_est, VY, 0) = 0;
	*matrix_at(state_est, VZ, 0) = vEst.z;

	/* accelerometer bias estimation: SIMPLIFICATION: we use constant value as prediction */
	*matrix_at(state_est, BAX, 0) = 0;
	*matrix_at(state_est, BAY, 0) = 0;
	*matrix_at(state_est, BAZ, 0) = 0;
}

/* Calculates cross product matrix for vector v so that when left-hand-multiplied by a vector p produces cross product (v x p).
* `out` is assumed to be 3x3, untransposed matrix.
*/
static inline void kmn_crossMat(const vec_t *v, matrix_t *out)
{
	out->data[0] = out->data[4] = out->data[8] = 0;
	out->data[7] = v->x;
	out->data[5] = -v->x;
	out->data[2] = v->y;
	out->data[6] = -v->y;
	out->data[3] = v->z;
	out->data[1] = -v->z;
}

/* Calculates half of derivative: d(q * p *cjg(q)) / dq with assumptions:
* 1) `out` is 3x4 matrix
* 2) `q` is rotation quaternion
* 3) `p` is vector (or pure quaternion vectorial part)
*/
static void kmn_qpqDiffQ(const quat_t *q, const vec_t *p, matrix_t *out)
{
	float pCrossData[3 * 3] = { 0 };
	float buffData[3 * 3] = { 0 };

	/* Matrices for cross matrix of p and general matrix for the part of derivative over imaginary components of q */
	matrix_t pCross = { .data = pCrossData, .rows = 3, .cols = 3, .transposed = 0 };
	matrix_t buf = { .data = buffData, .rows = 3, .cols = 3, .transposed = 0 };

	/* Vector for cross product (p x q) */
	const vec_t qVec = { .x = q->i, .y = q->j, .z = q->k };
	vec_t pxq;

	vec_cross(p, &qVec, &pxq);

	kmn_crossMat(p, &pCross);
	kmn_crossMat(&pxq, &buf);

	matrix_times(&pCross, q->a);
	matrix_sub(&buf, &pCross, NULL);
	buf.data[0] = buf.data[4] = buf.data[8] = vec_dot(p, &qVec);

	matrix_writeSubmatrix(out, 0, 1, &buf);
	*matrix_at(out, 0, 0) = q->a * p->x - pxq.x;
	*matrix_at(out, 1, 0) = q->a * p->y - pxq.y;
	*matrix_at(out, 2, 0) = q->a * p->z - pxq.z;
}

/* Calculates derivative: d(q * p * cjg(q)) / d(p) with assumptions:
* 1) `out` is 3x3, untransposed matrix
* 2) q is quaternion
* Note: this derivative does not need value of `p`
*/
static void kmn_qpqDiffP(const quat_t *q, matrix_t *out)
{
	float qqtData[3 * 3] = { 0 };
	matrix_t qqt = { .data = qqtData, .rows = 3, .cols = 3, .transposed = 0 };
	const vec_t qVec = { .x = q->i, .y = q->j, .z = q->k };

	kmn_crossMat(&qVec, out);
	matrix_times(out, 2 * q->a);
	/* Diagonal terms of `out` are now set to zeroes by `kmn_crossMat`. Proceed with diagonal terms */
	out->data[0] = out->data[4] = out->data[8] = q->a * q->a - (q->i * q->i + q->j * q->j + q->k * q->k);

	qqt.data[0] = q->i * q->i;
	qqt.data[4] = q->j * q->j;
	qqt.data[8] = q->k * q->k;

	qqt.data[1] = qqt.data[3] = q->i * q->j;
	qqt.data[2] = qqt.data[6] = q->i * q->k;
	qqt.data[5] = qqt.data[7] = q->j * q->k;

	matrix_times(&qqt, 2);
	matrix_add(out, &qqt, NULL);
}


/* Calculates derivative: d(q * p) / d(q) with assumptions:
 * 1) `out` is 4x4 matrix
 * 2) 'q' and 'p' are quaternions
*/
static void kmn_qpDiffQ(quat_t *p, matrix_t *out)
{
	out->data[0] = out->data[5] = out->data[10] = out->data[15] = p->a;

	out->data[4] = out->data[11] = p->i;
	out->data[1] = out->data[14] = -p->i;

	out->data[8] = out->data[13] = p->j;
	out->data[2] = out->data[7] = -p->j;

	out->data[12] = out->data[6] = p->k;
	out->data[3] = out->data[9] = -p->k;
}


/* Calculates derivative: d(q * w) / d(q) with assumptions:
 * 1) `out` is 4x3 matrix
 * 2) 'q' is a quaternion
 * 3) 'w' is a quaternionized vector (only imaginary terms are significant)
*/
static void kmn_qwDiffW(const quat_t *q, matrix_t *out)
{
	out->data[3] = out->data[7] = out->data[11] = q->a;

	out->data[10] = q->i;
	out->data[0] = out->data[8] = -q->i;

	out->data[5] = q->j;
	out->data[1] = out->data[9] = -q->j;

	out->data[6] = q->k;
	out->data[2] = out->data[4] = -q->k;
}


/* prediction step jacobian calculation function */
static void kmn_predJcb(matrix_t *F, matrix_t *state, matrix_t *U, time_t timeStep)
{
	const quat_t qState = {.a = kmn_vecAt(state, QA), .i = kmn_vecAt(state, QB), .j = kmn_vecAt(state, QC), .k = kmn_vecAt(state, QD)};
	const quat_t bwState = {.a = 0, .i = kmn_vecAt(state, BWX), .j = kmn_vecAt(state, BWY), .k = kmn_vecAt(state, BWZ)};
	const vec_t baState = {.x = kmn_vecAt(state, BAX), .y = kmn_vecAt(state, BAY), .z = kmn_vecAt(state, BAZ)};
	const quat_t wMeas = {.a = 0, .i = kmn_vecAt(U, UWX), .j = kmn_vecAt(U, UWY), .k = kmn_vecAt(U, UWZ)};
	const vec_t aMeas = {.x = kmn_vecAt(U, VX), .y = kmn_vecAt(U, VY), .z = kmn_vecAt(U, VZ)};

	const float dt = (float)timeStep / 1000000;

	/* d(f_q)/d(q) variables */
	float dfqdqData[4 * 4];
	matrix_t dfqdq = { .data = dfqdqData, .rows = 4, .cols = 4, .transposed = 0 };
	quat_t p; /* quaternion derivative product second term */

	/* d(f_q)/d(bw) variables */
	float dfqdbwData[4 * 3];
	matrix_t dfqdbw = { .data = dfqdbwData, .rows = 4, .cols = 3, .transposed = 0 };
	vec_t aTrue; /* measured acceleration without bias */

	/* d(f_v)/d(q) variables */
	float dfvdqData[3 * 4];
	matrix_t dfvdq = { .data = dfvdqData, .rows = 3, .cols = 4, .transposed = 0 };

	/* d(f_v)/d(ba) variables */
	float dfvdbaData[3 * 3];
	matrix_t dfvdba = { .data = dfvdbaData, .rows = 3, .cols = 3, .transposed = 0 };

	/* d(f_q)/d(q) calculations */
	quat_dif(&wMeas, &bwState, &p);
	quat_times(&p, dt / 2);
	p.a += 1;
	kmn_qpDiffQ(&p, &dfqdq);
	/* d(f_q)/d(q) write into F */
	matrix_writeSubmatrix(F, QA, QA, &dfqdq);

	/* d(f_q)/d(bw) calculations */
	kmn_qwDiffW(&qState, &dfqdbw);
	matrix_times(&dfqdbw, -dt / 2);
	/* d(f_q)/d(b_w) write into F */
	matrix_writeSubmatrix(F, QA, BWX, &dfqdbw);

	/* d(f_bw)/d(bw) calculations */
	*matrix_at(F, BWX, BWX) = *matrix_at(F, BWY, BWY) = *matrix_at(F, BWZ, BWZ) = 1;

	/* d(f_v)/d(q) calculations */
	vec_dif(&aMeas, &baState, &aTrue);
	kmn_qpqDiffQ(&qState, &aTrue, &dfvdq);
	matrix_times(&dfvdq, 2 * dt);
	/* d(f_v)/d(q) write into F */
	matrix_writeSubmatrix(F, VX, QA, &dfvdq);

	/* d(f_v)/d(ba) calculations */
	kmn_qpqDiffP(&qState, &dfvdba);
	matrix_times(&dfvdba, -dt);
	/* d(f_v)/d(ba) write into F */
	matrix_writeSubmatrix(F, VX, BAX, &dfvdba);

	/* d(f_v)/d(v) calculations */
	*matrix_at(F, VX, VX) = *matrix_at(F, VY, VY) = *matrix_at(F, VZ, VZ) = 1;

	/* d(f_ba)/d(ba) calculations */
	*matrix_at(F, BAX, BAX) = *matrix_at(F, BAY, BAY) = *matrix_at(F, BAZ, BAZ) = 1;
}


static void kmn_getNoiseQ(matrix_t *state, matrix_t *U, matrix_t *Q, time_t timestep)
{
	/* Submatrix for quaternion process noise */
	float qNoiseData[4 * 4] = { 0 };
	matrix_t qNoise = {.data = qNoiseData, .rows = 4, .cols = 4, .transposed = 0};

	const quat_t q = {.a = kmn_vecAt(state, QA), .i = kmn_vecAt(state, QB), .j = kmn_vecAt(state, QC), .k = kmn_vecAt(state, QD)};
	const float dtSq = ((float)timestep / 1000000.f) * ((float)timestep / 1000000.f);

	matrix_zeroes(Q);

	/* QUATERNION PROCESS NOISE: diagonal terms */
	*matrix_at(&qNoise, 0, 0) = 1 - q.a * q.a;
	*matrix_at(&qNoise, 1, 1) = 1 - q.i * q.i;
	*matrix_at(&qNoise, 2, 2) = 1 - q.j * q.j;
	*matrix_at(&qNoise, 3, 3) = 1 - q.k * q.k;

	/* QUATERNION PROCESS NOISE: upper and lower triangle terms */
	*matrix_at(&qNoise, 0, 1) = *matrix_at(&qNoise, 1, 0) = -q.a * q.i;
	*matrix_at(&qNoise, 0, 2) = *matrix_at(&qNoise, 2, 0) = -q.a * q.j;
	*matrix_at(&qNoise, 0, 3) = *matrix_at(&qNoise, 3, 0) = -q.a * q.k;
	*matrix_at(&qNoise, 1, 2) = *matrix_at(&qNoise, 2, 1) = -q.i * q.j;
	*matrix_at(&qNoise, 1, 3) = *matrix_at(&qNoise, 3, 1) = -q.i * q.k;
	*matrix_at(&qNoise, 2, 3) = *matrix_at(&qNoise, 3, 2) = -q.j * q.k;

	matrix_times(&qNoise, pred_common.inits->Q_wstdev * pred_common.inits->Q_wstdev * dtSq / 4);
	matrix_writeSubmatrix(Q, QA, QA, &qNoise);

	/* GYRO BIAS PROCESS NOISE */
	*matrix_at(Q, BWX, BWX) = *matrix_at(Q, BWY, BWY) = *matrix_at(Q, BWZ, BWZ) = pred_common.inits->Q_bwDotstdev * pred_common.inits->Q_bwDotstdev * dtSq;

	/* VELOCITY PROCESS NOISE: only diagonal terms */
	*matrix_at(Q, VX, VX) = *matrix_at(Q, VY, VY) = *matrix_at(Q, VZ, VZ) = dtSq * pred_common.inits->Q_astdev * pred_common.inits->Q_astdev;

	/* ACCEL BIAS PROCESS NOISE */
	*matrix_at(Q, BWX, BWX) = *matrix_at(Q, BWY, BWY) = *matrix_at(Q, BWZ, BWZ) = pred_common.inits->Q_baDotstdev * pred_common.inits->Q_baDotstdev * dtSq;
}


static void kmn_initState(matrix_t *state, const meas_calib_t *calib)
{
	*matrix_at(state, QA, 0) = calib->imu.initQuat.a;
	*matrix_at(state, QB, 0) = calib->imu.initQuat.i;
	*matrix_at(state, QC, 0) = calib->imu.initQuat.j;
	*matrix_at(state, QD, 0) = calib->imu.initQuat.k;

	*matrix_at(state, VX, 0) = 0;
	*matrix_at(state, VY, 0) = 0;
	*matrix_at(state, VZ, 0) = 0;

	*matrix_at(state, BWX, 0) = pred_common.gyroBiasBypass.x;
	*matrix_at(state, BWY, 0) = pred_common.gyroBiasBypass.y;
	*matrix_at(state, BWZ, 0) = pred_common.gyroBiasBypass.z;

	*matrix_at(state, BAX, 0) = 0;
	*matrix_at(state, BAY, 0) = 0;
	*matrix_at(state, BAZ, 0) = 0;
}


/* initialization of prediction step matrix values */
void kmn_predInit(state_engine_t *engine, const meas_calib_t *calib, const kalman_init_t *inits)
{
	pred_common.inits = inits;

	/* Storing calibration bias as constant value of gyroscope bias */
	pred_common.gyroBiasBypass = calib->imu.gyroBias;

	/* Storing length of calibration acceleration measured as measured earth acceleration. This imposes small but negligible error on velocity */
	pred_common.gLength = vec_len(&calib->imu.initAcc);

	kmn_initState(&engine->state, calib);

	/* prepare noise matrix Q */
	matrix_zeroes(&engine->Q);

	/* save function pointers */
	engine->estimateState = kmn_stateEst;
	engine->getJacobian = kmn_predJcb;
	engine->getControl = kmn_getCtrl;
	engine->getNoiseQ = kmn_getNoiseQ;
}
