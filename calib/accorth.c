/*
 * Phoenix-Pilot
 *
 * Drone accelerometer initial rotation calibration
 *
 * SUMMARY:
 *
 * Accelerometer calibration has three phases:
 * (1) Rough ellipsoidal (rotational)
 * (2) Fine ellipsoidal (stable six-point)
 * (3) Accelerometer rotation
 * (4) Save parameters
 * The goal of (1) and (2) is to produce matrix S3 (3x3) and H3 (3x1) which
 * satisfy the calibration equation:
 * accel_calib = S3 ( accel_uncalib - H3)
 * The goal of (3) is to obtain a base rotation quaternion of accelerometer,
 * which tells the device where the true `down` and `front` are.
 *
 * CALIBRATION STEPS DESCRIPTION:
 *
 * (1) Rough ellipsoidal calibration:
 * This calibration consist of slowly rotating the device in the air so that
 * the external accelerations are minimized to only earth acceleration and then
 * fitting an ellipsoid to these measurements.
 * Ellipsoidal parameters produced by `ellcal` in  (1) are S1 and H1 which roughly remove
 * scaling factors, nonorthogonality and accelerometer biases.
 *
 * (2) Fine ellipsoidal calibration
 * This calibration consist of taking multiple samples of earth accelerations taken form
 * steadily placed drone in multiple rotations.
 * During sampling of these the drone should not be disturbed in any way!
 * These samples are corrected using S1 and H1 and new ellipsoidal parameters S2 and H2
 * are fitted to these corrected measurements.
 * Final S3 and H3 parameters are calculated using S1, S2, H1, H2 matrices as follow:
 * S3 = S2 * S1
 * H3 = H1 + inv(S1) * H2
 *
 * (3) Accelerometer rotation calibration:
 * This calibration consist of taking samples of the drone placed as much leveled
 * as possible and as much vertical (nose up) as possible.
 * This produces the rotation quaternion that transforms measurements from
 * IMU frame of reference to the drone frame of reference.
 * This calibration is a baseline for the navigation system
 * to determine "where is up and front of me". Should be done very precisely!
 *
 * Copyright 2023 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#include <stdio.h>
#include <errno.h>
#include <math.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include <libsensors.h>

#include <sensc.h>
#include <vec.h>
#include <quat.h>
#include <calib.h>
#include <lma.h>

#include "calibtool.h"
#include "ellcal.h"

#define DANGLE_MIN         (M_PI / 36)       /* Minimum delta angle to take sample */
#define MAX_SAMPLES        256               /* Samples to be taken for ellipsoid fitting */
#define MAX_SAMPLES_FINE   (MAX_SAMPLES / 8) /* Samples to be taken for ellipsoid fine fitting. Must be smaller or equal to MAX_SAMPLES */
#define MAX_ACCEL_OFFSET   10000             /* Maximum acceptable accelerometer offset (mm/s^2) */
#define LMA_FITTING_EPOCHS 100               /* LMA epochs for parameters fitting */
#define LMA_JACOBIAN_STEP  0.0001f           /* Step for jacobian calculation in LMA */
#define MIN_TILT_COSINE    (M_SQRT2 / 2.f)   /* minimal cosine of tilt angle between down and front measurement (45 degrees) */

#define EARTH_G_MM 9806.65 /* Earth acceleration (in mm/s^2) according to 1901 General Conference on Weights and Measures */

struct {
	calib_data_t data;
	vec_t *meas;

	vec_t ellCenter;
	float ellMeanAxis;
} accorth_common;


/* Returns pointer to internal parameters for read purposes */
static calib_data_t *accorth_dataGet(void)
{
	return &accorth_common.data;
}


const char *accorth_help(void)
{
	return "Accelerometer nonorthogonality calibration\n";
}


static int accorth_write(FILE *file)
{
	unsigned int row, col;

	for (row = 0; row < ACC_OFFSET_ROWSPAN; row++) {
		for (col = 0; col < ACC_OFFSET_COLSPAN; col++) {
			fprintf(file, "%c%u%u %f\n", ACC_CHAR_OFFSET, row, col, MATRIX_DATA(&accorth_common.data.params.accorth.offset, row, col));
		}
	}

	for (row = 0; row < ACC_ORTHO_ROWSPAN; row++) {
		for (col = 0; col < ACC_ORTHO_COLSPAN; col++) {
			fprintf(file, "%c%u%u %f\n", ACC_CHAR_ORTHO, row, col, MATRIX_DATA(&accorth_common.data.params.accorth.ortho, row, col));
		}
	}

	fprintf(file, "%c%u%u %f\n", ACC_CHAR_QUAT, 0, 0, accorth_common.data.params.accorth.frameQ.a);
	fprintf(file, "%c%u%u %f\n", ACC_CHAR_QUAT, 1, 0, accorth_common.data.params.accorth.frameQ.i);
	fprintf(file, "%c%u%u %f\n", ACC_CHAR_QUAT, 2, 0, accorth_common.data.params.accorth.frameQ.j);
	fprintf(file, "%c%u%u %f\n", ACC_CHAR_QUAT, 3, 0, accorth_common.data.params.accorth.frameQ.k);

	return 0;
}


/* Accorth prompt utility, `prompt` must be nul-terminated */
static void accorth_prompt(const char *prompt, bool block)
{
	printf("%s", prompt);
	fflush(stdout);

	if (block) {
		fflush(stdin);
		getchar();
	}
}


/* Get average accelerometer reading over `n` samples */
static void accorth_accelAvg(vec_t *out, unsigned int n)
{
	unsigned int i;
	sensor_event_t accelEvt, gyroEvt, magEvt;

	out->x = out->y = out->z = 0;
	for (i = 0; i < n; i++) {
		sensc_imuGet(&accelEvt, &gyroEvt, &magEvt);
		out->x += accelEvt.accels.accelX;
		out->y += accelEvt.accels.accelY;
		out->z += accelEvt.accels.accelZ;
		usleep(1000);
	}
	vec_times(out, 1. / n);
}


/*
* Applies calbration: calibed = S * (raw - H)
* Uses internal buffer for `raw` so it is possible to pass the same vector as `raw` and `calibed`.
*/
static void accorth_calibApply(const matrix_t *S, const matrix_t *H, const vec_t *raw, vec_t *calibed)
{
	float dataVecMat[3], dataVecTmp[3];
	matrix_t vecMat = { .data = dataVecMat, .rows = 3, .cols = 1, .transposed = 0 };
	matrix_t vecTmp = { .data = dataVecTmp, .rows = 3, .cols = 1, .transposed = 0 };

	MATRIX_DATA(&vecTmp, 0, 0) = raw->x;
	MATRIX_DATA(&vecTmp, 1, 0) = raw->y;
	MATRIX_DATA(&vecTmp, 2, 0) = raw->z;

	/* y = S(x - H) */
	matrix_sub(&vecTmp, H, NULL);     /* vecTmp -= H */
	matrix_prod(S, &vecTmp, &vecMat); /* vecMat = S * vecTmp */

	calibed->x = MATRIX_DATA(&vecMat, 0, 0);
	calibed->y = MATRIX_DATA(&vecMat, 1, 0);
	calibed->z = MATRIX_DATA(&vecMat, 2, 0);
}


/*
* Calculates rotation quaternion that transforms measurements from IMU frame of reference
* to the droe frame of reference and saves it to `rotQuat`. Returns 0 on success, -1 otherwise.
*/
static int accorth_accRot(matrix_t *S, matrix_t *H, quat_t *rotQuat)
{
	static const vec_t nedZ = { .x = 0, .y = 0, .z = 1 }; /* z versor of NED frame of reference */
	static const vec_t nedY = { .x = 0, .y = 1, .z = 0 }; /* y versor of NED frame of reference */
	static const quat_t idenQ = { .a = 1, .i = 0, .j = 0, .k = 0 };

	vec_t bodyZ, accX, bodyY = { 0 };
	quat_t q;

	/*
	* Body frame placed on flat surface should provide a measurement of acceleration pointing upward.
	* Using NED frame of reference means that earth acceleration is parallel to NED z versor, but of opposite direction.
	* Taking negative earth acceleration as NED z versor.
	*/
	accorth_prompt("Place drone precisely horizontal and press [Enter]", true);

	sleep(1); /* Sleep for any vibration to disperse */
	accorth_accelAvg(&bodyZ, 1500);
	accorth_calibApply(S, H, &bodyZ, &bodyZ);
	vec_times(&bodyZ, -1);

	/*
	* Body frame pointing upward (+90 degree pitch angle) experience earth acceleration that is:
	* - parallel to NED x axis versor
	* - of common direction to NED x axis versor.
	*/
	accorth_prompt("Tilt drone precisely nose up and press [Enter]", true);

	sleep(1); /* Sleep for any vibration to disperse */
	accorth_accelAvg(&accX, 1500);
	accorth_calibApply(S, H, &accX, &accX);

	/* Non-allignment check by calculating cosine between measured accelerations using dot product */
	vec_normalize(&bodyZ);
	vec_normalize(&accX);
	if (vec_dot(&bodyZ, &accX) > MIN_TILT_COSINE) {
		printf("Calibration failed. Too small angle of front tilt\n");
		return -1;
	}

	/* Calculate cross vector to acquire frame of reference with orthogonal axis */
	vec_cross(&bodyZ, &accX, &bodyY);
	vec_normalize(&bodyY);

	/* Calculate rotation */
	quat_frameRot(&bodyZ, &bodyY, &nedZ, &nedY, &q, &idenQ);

	/* Quaternion validity check */
	if ((1 - quat_len(&q)) >= ACC_QUAT_ERR) {
		printf("Too big error in quaternion. Calibration aborted\n");
		return -1;
	}

	*rotQuat = q;

	return EOK;
}


/* Data acquisition of earth accelerations in steady positions */
static void accorth_meshGet(vec_t *buf, size_t n)
{
	size_t i;
	char msg[128];

	sprintf(msg, "Place the drone in %u unique positions. Positions should be very stable.\n", n);
	accorth_prompt(msg, false);

	for (i = 0; i < n; i++) {
		sprintf(msg, "Stored samples: %u/%u. Press [Enter] to sample...", i, n);
		accorth_prompt(msg, true);
		accorth_accelAvg(&buf[i], 1000);
	}
}


/* Calculating S3 and H3 from S1, S2, H1, H2. Returns -1 on calculation error, 0 otherwise. */
static int accorth_paramsCombine(const matrix_t *S1, const matrix_t *H1, const matrix_t *S2, const matrix_t *H2, matrix_t *S3, matrix_t *H3)
{
	float invBuf[18], dataTmp[3];
	matrix_t tmp = { .data = dataTmp, .rows = 3, .cols = 1, .transposed = 0 };

	/* Using S3 as temporary storage for inv(S1) */
	if (matrix_inv(S1, S3, invBuf, sizeof(invBuf) / sizeof(*invBuf)) < 0) {
		printf("Cannot calculate S1 inverse\n");
		return -1;
	}

	matrix_prod(S3, H2, &tmp);
	matrix_add(&tmp, H1, H3);
	matrix_prod(S2, S1, S3);

	return 0;
}


/* Fits ellipsoid to `n` samples in `data` and stores calibration parameters to `S` and `H` */
static int accorth_ellipsoidFit(vec_t *data, size_t n, matrix_t *S, matrix_t *H)
{
	fit_lma_t lma;
	int i;

	if (lma_init(3, 12, n, ellcal_lmaJacobian, ellcal_lmaResiduum, ellcal_lmaGuess, &lma) < 0) {
		fprintf(stderr, "%s: failed to init LMA", ACCORTH_TAG);
		return -1;
	}

	/* Write measurements to lma `samples` matrix */
	for (i = 0; i < n; i++) {
		MATRIX_DATA(&lma.samples, i, 0) = data[i].x;
		MATRIX_DATA(&lma.samples, i, 1) = data[i].y;
		MATRIX_DATA(&lma.samples, i, 2) = data[i].z;
	}

	if (lma_fit(LMA_FITTING_EPOCHS, &lma, LMALOG_NONE) < 0) {
		fprintf(stderr, "%s: ellipsoid fitting error\n", ACCORTH_TAG);
		return -1;
	}

	ellcal_lma2matrices(&lma.paramsVec, S, H);
	lma_done(&lma);

	return 0;
}


static int accorth_run(void)
{
	vec_t measAvg;
	float measAvgLen;
	quat_t frameQ;
	int i;

	/* Matrices preparation */
	float dataS1[9], dataS2[9], dataS3[9];
	matrix_t S1 = { .data = dataS1, .rows = 3, .cols = 3, .transposed = 0 };
	matrix_t S2 = { .data = dataS2, .rows = 3, .cols = 3, .transposed = 0 };
	matrix_t S3 = { .data = dataS3, .rows = 3, .cols = 3, .transposed = 0 };

	float dataH1[3], dataH2[3], dataH3[3];
	matrix_t H1 = { .data = dataH1, .rows = 3, .cols = 1, .transposed = 0 };
	matrix_t H2 = { .data = dataH2, .rows = 3, .cols = 1, .transposed = 0 };
	matrix_t H3 = { .data = dataH3, .rows = 3, .cols = 1, .transposed = 0 };

	/*
	* (1) ROUGH ELLIPSOIDAL CALIBRATION
	*/

	/* Acquire rotatinal measurements */
	if (ellcal_rotDataGet(accorth_common.meas, MAX_SAMPLES, DANGLE_MIN, SENSOR_TYPE_ACCEL) < 0) {
		return -1;
	}

	/* Calculate mean offset of samples (rough estimate of ellipsoid center) */
	measAvg = (vec_t) { .x = 0, .y = 0, .z = 0 };
	for (i = 0; i < MAX_SAMPLES; i++) {
		vec_add(&measAvg, &accorth_common.meas[i]);
	}
	vec_times(&measAvg, 1.f / MAX_SAMPLES);

	/* Shift samples by 'measAvg' and calculate their new mean distance from (0,0,0) */
	for (i = 0; i < MAX_SAMPLES; i++) {
		vec_sub(&accorth_common.meas[i], &measAvg);
		measAvgLen += vec_len(&accorth_common.meas[i]);
	}

	/* Normalize shifted measurements to be spaced "around" (0,0,0)+-(1,1,1) */
	for (i = 0; i < MAX_SAMPLES; i++) {
		vec_times(&accorth_common.meas[i], 1.f / EARTH_G_MM);
	}

	if (accorth_ellipsoidFit(accorth_common.meas, MAX_SAMPLES, &S1, &H1) < 0) {
		printf("%s: failed at rough ellipsoid fit\n", ACCORTH_TAG);
		return -1;
	}

	/*
	* Data was scaled and shoftef. H matrix need to be scaled/shifted as follow:
	* h_final = avg + h * avgLen(m_shift)
	*/
	matrix_times(&H1, EARTH_G_MM);
	MATRIX_DATA(&H1, 0, 0) += measAvg.x;
	MATRIX_DATA(&H1, 1, 0) += measAvg.y;
	MATRIX_DATA(&H1, 2, 0) += measAvg.z;

	/*
	* (2) FINE ELLIPSOIDAL CALIBRATION
	*/

	/* Acquire six measurements */
	accorth_meshGet(accorth_common.meas, MAX_SAMPLES_FINE);

	/* Rough-calibrate measurements and scale them by EARTH_G */
	for (i = 0; i < MAX_SAMPLES_FINE; i++) {
		accorth_calibApply(&S1, &H1, &accorth_common.meas[i], &accorth_common.meas[i]);
		vec_times(&accorth_common.meas[i], 1.f / EARTH_G_MM);
	}

	if (accorth_ellipsoidFit(accorth_common.meas, MAX_SAMPLES_FINE, &S2, &H2) < 0) {
		printf("%s: failed at fine ellipsoid fit\n", ACCORTH_TAG);
		return -1;
	}

	/* Scale back H matrix produced by lma so it is in [mm/s^2] */
	matrix_times(&H2, EARTH_G_MM);

	if (accorth_paramsCombine(&S1, &H1, &S2, &H2, &S3, &H3) < 0) {
		return -1;
	}

	/* Vaidity check: offset should be inside expected limits (len(H) below threshold) */
	measAvgLen = MATRIX_DATA(&H3, 0, 0) * MATRIX_DATA(&H3, 0, 0);
	measAvgLen += MATRIX_DATA(&H3, 1, 0) * MATRIX_DATA(&H3, 1, 0);
	measAvgLen += MATRIX_DATA(&H3, 2, 0) * MATRIX_DATA(&H3, 2, 0);
	measAvgLen = sqrtf(measAvgLen);
	if (measAvgLen > MAX_ACCEL_OFFSET) {
		fprintf(stderr, "%s: hard iron exceeds expectations: x:%f y:%f z:%f\n\n", ACCORTH_TAG, MATRIX_DATA(&H3, 0, 0), MATRIX_DATA(&H3, 1, 0), MATRIX_DATA(&H3, 2, 0));
		return -1;
	}

	/* Vaidity check: S matrix should not invert data */
	if (MATRIX_DATA(&S3, 0, 0) <= 0 || MATRIX_DATA(&S3, 1, 1) <= 0 || MATRIX_DATA(&S3, 2, 2) <= 0) {
		fprintf(stderr, "%s: invalid transformation! Diag: %f %f %f\n", ACCORTH_TAG, MATRIX_DATA(&S3, 0, 0), MATRIX_DATA(&S3, 1, 1), MATRIX_DATA(&S3, 2, 2));
		return -1;
	}

	/*
	* (3) ACCELEROMETER ROTATION CALIBRATION
	*/

	if (accorth_accRot(&S3, &H3, &frameQ) < 0) {
		printf("%s: accelerometer rotation failed\n", ACCORTH_TAG);
	}

	/*
	* (4) SAVE PARAMETERS
	*/

	printf("%s: frameQ: %f %f %f %f\n", ACCORTH_TAG, frameQ.a, frameQ.i, frameQ.j, frameQ.k);
	printf("%s: offset: %f %f %f\n", ACCORTH_TAG, MATRIX_DATA(&H3, 0, 0), MATRIX_DATA(&H3, 1, 0), MATRIX_DATA(&H3, 2, 0));
	printf("%s: nonortho:\n", ACCORTH_TAG);
	matrix_print(&S3);

	accorth_common.data.params.accorth.frameQ = frameQ;
	matrix_writeSubmatrix(&accorth_common.data.params.accorth.offset, 0, 0, &H3);
	matrix_writeSubmatrix(&accorth_common.data.params.accorth.ortho, 0, 0, &S3);

	return 0;
}


static int accorth_done(void)
{
	sensc_deinit();
	free(accorth_common.meas);

	return EOK;
}


static int accorth_init(int argc, const char **argv)
{
	if (sensc_init(SENSOR_PATH, false) < 0) {
		return -1;
	}

	accorth_common.meas = calloc(MAX_SAMPLES, sizeof(vec_t));
	if (accorth_common.meas == NULL) {
		fprintf(stderr, "%s: memory allocation fail\n", ACCORTH_TAG);
		sensc_deinit();
		return -1;
	}

	return EOK;
}


__attribute__((constructor(102))) static void accorth_register(void)
{
	static calib_ops_t cal = {
		.name = ACCORTH_TAG,
		.init = accorth_init,
		.run = accorth_run,
		.done = accorth_done,
		.write = accorth_write,
		.help = accorth_help,
		.dataGet = accorth_dataGet
	};

	calib_register(&cal);

	accorth_common.data.type = typeAccorth;
}
