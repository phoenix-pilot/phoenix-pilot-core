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
#include <math.h>
#include <string.h>
#include <time.h>

#include "kalman_implem.h"
#include "meas.h"

#include <vec.h>
#include <quat.h>
#include <matrix.h>
#include <parser.h>

#define KMN_CONFIG_HEADERS_CNT    4
#define KMN_CONFIG_MAX_FIELDS_CNT 9


struct {
	const kalman_init_t *inits;
	vec_t gyroBiasBypass;
	float gLength;
} pred_common;


static kalman_init_t *converterResult;


static int kmn_PMatrixConverter(const hmap_t *h)
{
	int err = 0;

	err |= parser_fieldGetFloat(h, "qerr", &converterResult->P_qerr);
	err |= parser_fieldGetFloat(h, "verr", &converterResult->P_verr);
	err |= parser_fieldGetFloat(h, "baerr", &converterResult->P_baerr);
	err |= parser_fieldGetFloat(h, "rerr", &converterResult->P_rerr);

	return err;
}


static int kmn_RMatrixConverter(const hmap_t *h)
{
	int err = 0;

	err |= parser_fieldGetFloat(h, "astdev", &converterResult->R_astdev);
	err |= parser_fieldGetFloat(h, "mstdev", &converterResult->R_mstdev);
	err |= parser_fieldGetFloat(h, "bwstdev", &converterResult->R_bwstdev);
	err |= parser_fieldGetFloat(h, "hstdev", &converterResult->R_hstdev);

	return err;
}


static int kmn_QMatrixConverter(const hmap_t *h)
{
	int err = 0;

	err |= parser_fieldGetFloat(h, "astdev", &converterResult->Q_astdev);
	err |= parser_fieldGetFloat(h, "wstdev", &converterResult->Q_wstdev);
	err |= parser_fieldGetFloat(h, "baDotstdev", &converterResult->Q_baDotstdev);
	err |= parser_fieldGetFloat(h, "bwDotstdev", &converterResult->Q_bwDotstdev);

	return err;
}


static int kmn_loggingConverter(const hmap_t *h)
{
	int err = 0;

	err |= parser_fieldGetInt(h, "verbose", &converterResult->verbose);
	err |= parser_fieldGetInt(h, "log", &converterResult->log);

	return err;
}


/* reads config file named "config" from filesystem */
int kmn_configRead(const char *configFile, kalman_init_t *initVals)
{
	int err = 0;
	parser_t *p;

	converterResult = initVals;

	p = parser_alloc(KMN_CONFIG_HEADERS_CNT, KMN_CONFIG_MAX_FIELDS_CNT);
	if (p == NULL) {
		return -1;
	}

	err |= parser_headerAdd(p, "P_MATRIX", kmn_PMatrixConverter);
	err |= parser_headerAdd(p, "R_MATRIX", kmn_RMatrixConverter);
	err |= parser_headerAdd(p, "Q_MATRIX", kmn_QMatrixConverter);
	err |= parser_headerAdd(p, "LOGGING", kmn_loggingConverter);

	if (err != 0) {
		parser_free(p);
		return -1;
	}

	err = parser_execute(p, configFile, PARSER_IGN_UNKNOWN_HEADERS);
	parser_free(p);

	if (err != 0) {
		return -1;
	}

	return 0;
}


static matrix_t *kmn_getCtrl(matrix_t *U)
{
	vec_t accel, gyro, mag, accelRaw;
	time_t tstamp;

	meas_imuGet(&accel, &accelRaw, &gyro, &mag, &tstamp);

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

	static quat_t gyroBiasLpf = { 0 };

	/* quaternionized angular rate from U vector */
	quat_t wMeas = {.a = 0, .i = kmn_vecAt(U, UWX), .j = kmn_vecAt(U, UWY), .k = kmn_vecAt(U, UWZ)}; /* quaternionized vector */
	vec_t aMeas = {.x = kmn_vecAt(U, UAX), .y = kmn_vecAt(U, UAY), .z = kmn_vecAt(U, UAZ)};

	/* quaternionized angular rate and rotation quaternion estimates */
	quat_t qEst, qTmp;
	vec_t aEst;

	const float dt = (float)timeStep / 1000000.f;

	/* update gyro bias lpf value. Gyro bias decreases when there are bad estimation conditions */
	gyroBiasLpf.i *= GYRO_BIAS_IIR_FACTOR;
	gyroBiasLpf.j *= GYRO_BIAS_IIR_FACTOR;
	if (fabs(wMeas.i) > 0.05 && fabs(gyroBiasLpf.i) < 0.01) {
		gyroBiasLpf.i += (1.f - GYRO_BIAS_IIR_FACTOR) * wMeas.i;
	}
	if (fabs(wMeas.j) > 0.05 && fabs(gyroBiasLpf.j) < 0.01) {
		gyroBiasLpf.j += (1.f - GYRO_BIAS_IIR_FACTOR) * wMeas.j;
	}
	quat_sub(&wMeas, &gyroBiasLpf);

	/* quaternion estimation: q = q * ( q_iden + h/2 * (wMeas - bw) ) */
	quat_dif(&wMeas, &bwState, &qTmp);
	quat_times(&qTmp, dt / 2);
	qTmp.a += 1;
	quat_mlt(&qState, &qTmp, &qEst);
	quat_normalize(&qEst);

	*matrix_at(state_est, QA, 0) = qEst.a;
	*matrix_at(state_est, QB, 0) = qEst.i;
	*matrix_at(state_est, QC, 0) = qEst.j;
	*matrix_at(state_est, QD, 0) = qEst.k;

	/* gyroscope bias estimation: SIMPLIFICATION: we use constant value as prediction */
	*matrix_at(state_est, BWX, 0) = kmn_vecAt(state, BWX);
	*matrix_at(state_est, BWY, 0) = kmn_vecAt(state, BWY);
	*matrix_at(state_est, BWZ, 0) = kmn_vecAt(state, BWZ);

	/* velocity estimation */
	vec_dif(&aMeas, &baState, &aEst);
	quat_vecRot(&aEst, &qState);
	aEst.z += pred_common.gLength; /* Cancel out earth acceleration */

	*matrix_at(state_est, VX, 0) = kmn_vecAt(state, VX) + aEst.x * dt;
	*matrix_at(state_est, VY, 0) = kmn_vecAt(state, VY) + aEst.y * dt;
	*matrix_at(state_est, VZ, 0) = kmn_vecAt(state, VZ) + aEst.z * dt;

	/* accelerometer bias estimation: SIMPLIFICATION: we use constant value as prediction */
	*matrix_at(state_est, BAX, 0) = kmn_vecAt(state, BAX);
	*matrix_at(state_est, BAY, 0) = kmn_vecAt(state, BAY);
	*matrix_at(state_est, BAZ, 0) = kmn_vecAt(state, BAZ);

	/* position estimation */
	*matrix_at(state_est, RX, 0) = kmn_vecAt(state, RX) + dt * vState.x;
	*matrix_at(state_est, RY, 0) = kmn_vecAt(state, RY) + dt * vState.y;
	*matrix_at(state_est, RZ, 0) = kmn_vecAt(state, RZ) + dt * vState.z;
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
	const vec_t aMeas = {.x = kmn_vecAt(U, UAX), .y = kmn_vecAt(U, UAY), .z = kmn_vecAt(U, UAZ)};

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
	p.a = 1;
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

	/*
	* SOMETHING IS WRONG in dfvdq or dfvdba!!! Disabling it as it is not position crucial derivative.
	* It must be taken care of!!
	*/

	/* d(f_v)/d(q) calculations */
	vec_dif(&aMeas, &baState, &aTrue);
	kmn_qpqDiffQ(&qState, &aTrue, &dfvdq);
	matrix_times(&dfvdq, 2 * dt);
	/* d(f_v)/d(q) write into F */
	/* matrix_writeSubmatrix(F, VX, QA, &dfvdq); */

	/* d(f_v)/d(ba) calculations */
	kmn_qpqDiffP(&qState, &dfvdba);
	matrix_times(&dfvdba, -dt);
	/* d(f_v)/d(ba) write into F */
	/* matrix_writeSubmatrix(F, VX, BAX, &dfvdba); */

	/* d(f_v)/d(v) calculations */
	*matrix_at(F, VX, VX) = *matrix_at(F, VY, VY) = *matrix_at(F, VZ, VZ) = 1;

	/* d(f_ba)/d(ba) calculations */
	*matrix_at(F, BAX, BAX) = *matrix_at(F, BAY, BAY) = *matrix_at(F, BAZ, BAZ) = 1;

	/* d(f_r)/d(r) and d(f_r)/d(v) */
	*matrix_at(F, RX, RX) = *matrix_at(F, RY, RY) = *matrix_at(F, RZ, RZ) = 1;
	*matrix_at(F, RX, VX) = *matrix_at(F, RY, VY) = *matrix_at(F, RZ, VZ) = dt;
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

	*matrix_at(Q, RX, RX) = *matrix_at(Q, RY, RY) = *matrix_at(Q, RZ, RZ) = (dtSq * pred_common.inits->Q_astdev) * (dtSq * pred_common.inits->Q_astdev);
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

	*matrix_at(state, BWX, 0) = 0;
	*matrix_at(state, BWY, 0) = 0;
	*matrix_at(state, BWZ, 0) = 0;

	*matrix_at(state, BAX, 0) = 0;
	*matrix_at(state, BAY, 0) = 0;
	*matrix_at(state, BAZ, 0) = 0;

	*matrix_at(state, RX, 0) = 0;
	*matrix_at(state, RY, 0) = 0;
	*matrix_at(state, RZ, 0) = 0;
}


static void kmn_initCov(matrix_t *cov, const kalman_init_t *inits)
{
	matrix_zeroes(cov);

	*matrix_at(cov, QA, QA) = inits->P_qerr;
	*matrix_at(cov, QB, QB) = inits->P_qerr;
	*matrix_at(cov, QC, QC) = inits->P_qerr;
	*matrix_at(cov, QD, QD) = inits->P_qerr;

	*matrix_at(cov, BWX, BWX) = inits->P_bwerr;
	*matrix_at(cov, BWY, BWY) = inits->P_bwerr;
	*matrix_at(cov, BWZ, BWZ) = inits->P_bwerr;

	*matrix_at(cov, BAX, BAX) = inits->P_baerr;
	*matrix_at(cov, BAY, BAY) = inits->P_baerr;
	*matrix_at(cov, BAZ, BAZ) = inits->P_baerr;

	*matrix_at(cov, RX, RX) = inits->P_rerr;
	*matrix_at(cov, RY, RY) = inits->P_rerr;
	*matrix_at(cov, RZ, RZ) = inits->P_rerr;

	*matrix_at(cov, RX, RX) = inits->P_rerr;
	*matrix_at(cov, RY, RY) = inits->P_rerr;
	*matrix_at(cov, RZ, RZ) = inits->P_rerr;
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
	kmn_initCov(&engine->cov, inits);

	/* prepare noise matrix Q */
	matrix_zeroes(&engine->Q);

	/* save function pointers */
	engine->estimateState = kmn_stateEst;
	engine->getJacobian = kmn_predJcb;
	engine->getControl = kmn_getCtrl;
	engine->getNoiseQ = kmn_getNoiseQ;
}
