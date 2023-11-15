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
#include "logs/writer.h"

#include <vec.h>
#include <quat.h>
#include <qdiff.h>
#include <matrix.h>
#include <parser.h>

#define KMN_CONFIG_HEADERS_CNT    7
#define KMN_CONFIG_MAX_FIELDS_CNT 9


struct {
	const kalman_init_t *inits;
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

	err |= parser_fieldGetFloat(h, "gpsxstdev", &converterResult->R_gpsxstdev);
	err |= parser_fieldGetFloat(h, "gpsvstdev", &converterResult->R_gpsvstdev);

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
	char *str;
	const char *separators = ",";

	/* Parsing field `verbose` */
	if (parser_fieldGetInt(h, "verbose", &converterResult->verbose) != 0) {
		return -1;
	}

	/* Parsing field `log` */
	str = hmap_get(h, "log");
	if (str == NULL) {
		fprintf(stderr, "Ekf config: cannot find \"log\" field in config file\n");
		return -1;
	}

	converterResult->log = 0;

	str = strtok(str, separators);
	while (str != NULL) {
		if (strcmp(str, "SENSC") == 0) {
			converterResult->log |= EKFLOG_SENSC;
		}
		else if (strcmp(str, "TIME") == 0) {
			converterResult->log |= EKFLOG_TIME;
		}
		else if (strcmp(str, "STATE") == 0) {
			converterResult->log |= EKFLOG_STATE;
		}
		else if (strcmp(str, "ALL") == 0) {
			converterResult->log |= EKFLOG_SENSC;
			converterResult->log |= EKFLOG_TIME;
			converterResult->log |= EKFLOG_STATE;
			break;
		}
		else if (strcmp(str, "NONE") == 0) {
			converterResult->log = 0;
			break;
		}
		else {
			fprintf(stderr, "EKF config: invalid log specifier: %s\n", str);
			return -1;
		}

		str = strtok(NULL, separators);
	}

	/* Parsing field `mode` */
	str = hmap_get(h, "mode");
	if (str == NULL) {
		converterResult->logMode = 0;
	}
	else if (strcmp(str, "DEFAULT") == 0) {
		converterResult->logMode = 0;
	}
	else if (strcmp(str, "STRICT") == 0) {
		converterResult->logMode = PLOG_STRICT_MODE;
	}
	else {
		fprintf(stderr, "EKF config: Invalid mode specifier: %s\n", str);
		return -1;
	}

	return 0;
}


static int kmn_dataSourceConverter(const hmap_t *h)
{
	char *str;

	str = hmap_get(h, "source");
	if (str == NULL) {
		return -1;
	}

	if (strcmp(str, "SENSORS") == 0) {
		converterResult->measSource = srcSens;
	}
	else if (strcmp(str, "LOGS") == 0) {
		converterResult->measSource = srcLog;
	}
	else {
		fprintf(stderr, "Ekf config: Unknown source specifier: %s\n", str);
		return -1;
	}

	if (converterResult->measSource == srcLog) {
		str = hmap_get(h, "file");
		if (str == NULL) {
			fprintf(stderr, "Ekf config: no file as data source for ekf is specified\n");
			return -1;
		}

		if (strlen(str) > MAX_PATH_LEN) {
			fprintf(stderr, "Ekf config: file specification is too long\n");
			return -1;
		}

		strcpy(converterResult->sourceFile, str);
	}

	return 0;
}


static int kmn_modelConverter(const hmap_t *h)
{
	int flag, err = 0, modelFlags = 0;

	err |= parser_fieldGetInt(h, "imu", &flag);
	modelFlags |= (flag != 0) ? KMN_UPDT_IMU : 0;

	err |= parser_fieldGetInt(h, "baro", &flag);
	modelFlags |= (flag != 0) ? KMN_UPDT_BARO : 0;

	err |= parser_fieldGetInt(h, "gps", &flag);
	modelFlags |= (flag != 0) ? KMN_UPDT_GPS : 0;

	if (err != 0) {
		return err;
	}

	converterResult->modelFlags = modelFlags;

	return 0;
}


static int kmn_miscConverter(const hmap_t *h)
{
	int err = 0;
	float magDecl = 0;

	err |= parser_fieldGetFloat(h, "magDecl", &magDecl);

	if (err != 0) {
		return err;
	}

	if (magDecl > 45 || magDecl < -45) {
		fprintf(stderr, "Ekf config: magDecl outside of [-45 deg, +45 deg]");
		return -1;
	}

	converterResult->magDeclCos = cos(M_PI * magDecl / 180);
	converterResult->magDeclSin = sin(M_PI * magDecl / 180);

	return 0;
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
	err |= parser_headerAdd(p, "DATA_SOURCE", kmn_dataSourceConverter);
	err |= parser_headerAdd(p, "MODEL", kmn_modelConverter);
	err |= parser_headerAdd(p, "MISC", kmn_miscConverter);

	if (err != 0) {
		parser_free(p);
		return -1;
	}

	err = parser_execute(p, configFile, PARSER_IGN_UNKNOWN_HEADERS);
	parser_free(p);

	return err == 0 ? 0 : -1;
}


static matrix_t *kmn_getCtrl(matrix_t *U)
{
	vec_t accel, gyro, accelRaw, gyroRaw;

	meas_accelGet(&accel, &accelRaw);
	meas_gyroGet(&gyro, &gyroRaw);

	*matrix_at(U, UWX, 0) = gyro.x;
	*matrix_at(U, UWY, 0) = gyro.y;
	*matrix_at(U, UWZ, 0) = gyro.z;
	*matrix_at(U, UAX, 0) = accel.x;
	*matrix_at(U, UAY, 0) = accel.y;
	*matrix_at(U, UAZ, 0) = accel.z;

	return U;
}


/* Damping acceleration magnitude if its length is below ACC_DAMP_THRESHOLD */
float kmn_accelDamp(vec_t *accel)
{
	float len, damp;

	len = vec_len(accel);

	if (len < ACC_DAMP_THRESHOLD) {
		len /= ACC_DAMP_THRESHOLD;
		damp = len * len * len * len;
	}
	else {
		damp = 1.0;
	}

	return damp;
}


/* State estimation function definition */
static void kmn_stateEst(matrix_t *state, matrix_t *state_est, matrix_t *U, time_t timeStep)
{
	/* values from state vector */
	const quat_t qState = { .a = kmn_vecAt(state, QA), .i = kmn_vecAt(state, QB), .j = kmn_vecAt(state, QC), .k = kmn_vecAt(state, QD) };
	const quat_t bwState = { .a = 0, .i = kmn_vecAt(state, BWX), .j = kmn_vecAt(state, BWY), .k = kmn_vecAt(state, BWZ) }; /* quaternionized vector */
	const vec_t vState = { .x = kmn_vecAt(state, VX), .y = kmn_vecAt(state, VY), .z = kmn_vecAt(state, VZ) };
	const vec_t baState = { .x = kmn_vecAt(state, BAX), .y = kmn_vecAt(state, BAY), .z = kmn_vecAt(state, BAZ) };

	/* quaternionized angular rate from U vector */
	quat_t wMeas = { .a = 0, .i = kmn_vecAt(U, UWX), .j = kmn_vecAt(U, UWY), .k = kmn_vecAt(U, UWZ) }; /* quaternionized vector */
	vec_t aMeas = { .x = kmn_vecAt(U, UAX), .y = kmn_vecAt(U, UAY), .z = kmn_vecAt(U, UAZ) };

	/* quaternionized angular rate and rotation quaternion estimates */
	quat_t qEst, qTmp;
	vec_t aEst;

	const float dt = (float)timeStep / 1000000.f;

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
	aEst.z += EARTH_G;

	/* Integrating z acceleration as is. Low impact from attitude error */
	*matrix_at(state_est, VZ, 0) = kmn_vecAt(state, VZ) + aEst.z * dt;
	/* Integrating damped x/y acceleration. High impact from attitude error */
	aEst.z = 0;
	vec_times(&aEst, kmn_accelDamp(&aEst));
	*matrix_at(state_est, VX, 0) = kmn_vecAt(state, VX) + aEst.x * dt;
	*matrix_at(state_est, VY, 0) = kmn_vecAt(state, VY) + aEst.y * dt;

	/* accelerometer bias estimation: SIMPLIFICATION: we use constant value as prediction */
	*matrix_at(state_est, BAX, 0) = kmn_vecAt(state, BAX);
	*matrix_at(state_est, BAY, 0) = kmn_vecAt(state, BAY);
	*matrix_at(state_est, BAZ, 0) = kmn_vecAt(state, BAZ);

	/* position estimation */
	*matrix_at(state_est, RX, 0) = kmn_vecAt(state, RX) + dt * vState.x;
	*matrix_at(state_est, RY, 0) = kmn_vecAt(state, RY) + dt * vState.y;
	*matrix_at(state_est, RZ, 0) = kmn_vecAt(state, RZ) + dt * vState.z;
}


/* prediction step jacobian calculation function */
static void kmn_predJcb(matrix_t *F, matrix_t *state, matrix_t *U, time_t timeStep)
{
	const quat_t qState = { .a = kmn_vecAt(state, QA), .i = kmn_vecAt(state, QB), .j = kmn_vecAt(state, QC), .k = kmn_vecAt(state, QD) };
	const quat_t bwState = { .a = 0, .i = kmn_vecAt(state, BWX), .j = kmn_vecAt(state, BWY), .k = kmn_vecAt(state, BWZ) };
	const vec_t baState = { .x = kmn_vecAt(state, BAX), .y = kmn_vecAt(state, BAY), .z = kmn_vecAt(state, BAZ) };

	const quat_t wMeas = { .a = 0, .i = kmn_vecAt(U, UWX), .j = kmn_vecAt(U, UWY), .k = kmn_vecAt(U, UWZ) };
	const vec_t aMeas = { .x = kmn_vecAt(U, UAX), .y = kmn_vecAt(U, UAY), .z = kmn_vecAt(U, UAZ) };

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
	qvdiff_qpDiffQ(&p, &dfqdq);
	/* d(f_q)/d(q) write into F */
	matrix_writeSubmatrix(F, QA, QA, &dfqdq);

	/* d(f_q)/d(bw) calculations */
	qvdiff_qpDiffP(&qState, &dfqdbw);
	matrix_times(&dfqdbw, -dt / 2);
	/* d(f_q)/d(b_w) write into F */
	matrix_writeSubmatrix(F, QA, BWX, &dfqdbw);

	/* d(f_bw)/d(bw) calculations */
	*matrix_at(F, BWX, BWX) = *matrix_at(F, BWY, BWY) = *matrix_at(F, BWZ, BWZ) = 1;

	/*
	* DISABLED USE OF POSITION DERIVATIVES
	* Not yet tested in flight!
	*/

	/* d(f_v)/d(q) calculations */
	vec_dif(&aMeas, &baState, &aTrue);
	qvdiff_qvqDiffQ(&qState, &aTrue, &dfvdq);
	matrix_times(&dfvdq, 2 * dt);
	/* d(f_v)/d(q) write into F */
	/* matrix_writeSubmatrix(F, VX, QA, &dfvdq); */

	/* d(f_v)/d(ba) calculations */
	qvdiff_qvqDiffV(&qState, &dfvdba);
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
	matrix_t qNoise = { .data = qNoiseData, .rows = 4, .cols = 4, .transposed = 0 };

	const quat_t q = { .a = kmn_vecAt(state, QA), .i = kmn_vecAt(state, QB), .j = kmn_vecAt(state, QC), .k = kmn_vecAt(state, QD) };
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
	*matrix_at(Q, BAX, BAX) = *matrix_at(Q, BAY, BAY) = *matrix_at(Q, BAZ, BAZ) = pred_common.inits->Q_baDotstdev * pred_common.inits->Q_baDotstdev * dtSq;

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

	*matrix_at(cov, VX, VX) = inits->P_verr;
	*matrix_at(cov, VY, VY) = inits->P_verr;
	*matrix_at(cov, VZ, VZ) = inits->P_verr;

	*matrix_at(cov, RX, RX) = inits->P_rerr;
	*matrix_at(cov, RY, RY) = inits->P_rerr;
	*matrix_at(cov, RZ, RZ) = inits->P_rerr;
}


/* initialization of prediction step matrix values */
void kmn_predInit(state_engine_t *engine, const meas_calib_t *calib, const kalman_init_t *inits)
{
	pred_common.inits = inits;

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
