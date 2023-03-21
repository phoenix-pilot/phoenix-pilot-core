/*
 * Phoenix-Pilot
 *
 * Drone magnetometer calibration module
 * Calibration of magnetometer against hard/soft iron interference
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
#include <string.h>
#include <errno.h>
#include <math.h>
#include <unistd.h>
#include <sys/time.h>

#include <matrix.h>
#include <vec.h>
#include <calib.h>
#include <lma.h>
#include <sensc.h>

#include "calibtool.h"


#define DANGLE_MIN          (M_PI / 9) /* Minimum delta angle to take sample */
#define MAX_SAMPLES         512        /* Samples to be taken for ellipsoid fitting */
#define MAX_HARDIRON_LENGTH 5000       /* Maximum acceptable hard iron offset */
#define LMA_FITTING_EPOCHS  20         /* LMA epochs for parameters fitting */
#define LMA_JACOBIAN_STEP   0.0001f    /* Step for jacobian calculation in LMA */

#define MAX_U32_DELTAANGLE 0x7fffffff


struct {
	calib_data_t data;
	vec_t *meas;
} magiron_common;


/*
* Target function has a form  of: || S(X - H) || = 1 where:
*
* X -> 3x1 matrix of (x,y,z) uncalibrated measurement
* S -> 3x3 soft iron calibration matrix of form:
*    | p0, p1, p2 |
*    | p3, p4, p5 |
*    | p6, p7, p8 |
* H -> 3x1 hard iron offset calibration matrix of form: (p9, p10, p11)^T
*
* We want this function to transform data from some ellipsoid into a unit radius sphere.
* This function does not check any conic function equalities for ellipsoid.
*
* Works well if data is placed around (0,0,0) +- (1,1,1) and the biggest ellipsoid semiaxis is of length ~1.
*/
static int magiron_lmaResiduum(const matrix_t *P, const matrix_t *V, float *res, bool log)
{
	const matrix_t S = { .data = &P->data[0], .rows = 3, .cols = 3, .transposed = 0 };
	const matrix_t H = { .data = &P->data[9], .rows = 3, .cols = 1, .transposed = 0 };

	int i;
	float r = 0;
	float dataX[3], dataTmpX[3];
	matrix_t X = { .data = dataX, .rows = 3, .cols = 1, .transposed = 0 };
	matrix_t tmpX = { .data = dataTmpX, .rows = 3, .cols = 1, .transposed = 0 };

	for (i = 0; i < 3; i++) {
		MATRIX_DATA(&X, i, 0) = MATRIX_DATA(V, 0, i); /* Copy V into X matrix */
	}

	matrix_sub(&X, &H, NULL);   /* apply hard iron offset */
	matrix_prod(&S, &X, &tmpX); /* apply soft iron matrix */

	/* calculate length of produced vector */
	for (i = 0; i < 3; i++) {
		r += MATRIX_DATA(&tmpX, i, 0) * MATRIX_DATA(&tmpX, i, 0);
	}

	/* Length difference between transformed vector and a unit sphere radius is the residuum */
	*res = (float)(sqrt(r) - 1);

	return 0;
}


/*
* Jacobian is calculated numerically by calculating differences of residuum after step in parameters.
* Steps are taken for each parameter at a time. Step size is
*/
static int magiron_lmaJacobian(const matrix_t *P, const matrix_t *V, matrix_t *J, bool log)
{
	float dataPtmp[12] = { 0 };
	matrix_t Ptmp = { .data = dataPtmp, .rows = 1, .cols = 12, .transposed = 0 };
	float baseRes, newRes;
	int i;

	magiron_lmaResiduum(P, V, &baseRes, false);

	/* Numeric jacobian solving for each parameter */
	for (i = 0; i < 12; i++) {
		memcpy(Ptmp.data, P->data, 12 * sizeof(float));

		/* Increment, get new residuum and calculate the slope of change */
		MATRIX_DATA(&Ptmp, 0, i) += LMA_JACOBIAN_STEP;
		magiron_lmaResiduum(&Ptmp, V, &newRes, false);
		MATRIX_DATA(J, 0, i) = (newRes - baseRes) / LMA_JACOBIAN_STEP;
	}

	return 0;
}


/* Initial guess is unit radius sphere at (0,0,0) */
void magiron_lmaGuess(matrix_t *P)
{
	/* soft iron calibration guess */
	MATRIX_DATA(P, 0, 0) = 1;
	MATRIX_DATA(P, 0, 1) = 0;
	MATRIX_DATA(P, 0, 2) = 0;
	MATRIX_DATA(P, 0, 3) = 0;
	MATRIX_DATA(P, 0, 4) = 1;
	MATRIX_DATA(P, 0, 5) = 0;
	MATRIX_DATA(P, 0, 6) = 0;
	MATRIX_DATA(P, 0, 7) = 0;
	MATRIX_DATA(P, 0, 8) = 1;

	/* hard iron calibration guess */
	MATRIX_DATA(P, 0, 9) = 0;
	MATRIX_DATA(P, 0, 10) = 0;
	MATRIX_DATA(P, 0, 11) = 0;
}


/* Returns pointer to internal parameters for read purposes */
static calib_data_t *magiron_dataGet(void)
{
	return &magiron_common.data;
}


/* Prints to `file` data from `mat` as calibtype `type` */
static void magiron_printIron(FILE *file, char type, matrix_t *mat)
{
	unsigned int rows, cols, r, c;

	cols = matrix_colsGet(mat);
	rows = matrix_rowsGet(mat);

	for (r = 0; r < rows; r++) {
		for (c = 0; c < cols; c++) {
			fprintf(file, "%c%i%i %f\n", type, r, c, MATRIX_DATA(mat, r, c));
		}
	}
}


static int magiron_write(FILE *file)
{
	/* Printing hard iron calibration parameters */
	magiron_printIron(file, CHAR_HARDIRON, &magiron_common.data.params.magiron.hardCal);

	/* Printing soft iron calibration parameters */
	magiron_printIron(file, CHAR_SOFTIRON, &magiron_common.data.params.magiron.softCal);

	return 0;
}


static const char *magiron_help(void)
{
	return "Magnetometer calibration against soft/hard iron interference.\n";
}


static int magiron_done(void)
{
	sensc_deinit();
	free(magiron_common.meas);

	return EOK;
}


/*
* The goal of `magiron_run` is to collect magnetometer readings from various positions of the device,
* calculate calibration ellipsoid coefficients and save them to the common structure.
*
* Sample magnetometer readings are taken while device is randomly rotated in space.
* Sampling is dependent on device rotation which is calculated using strapdown gyroscope integration.
* This connects the measurements with space rotation of the device.
*
* Data sampling may be improved by ensuring that points are evenly spaced on the ellipsoid/sphere.
* This would need some spatial checks which are too complex for first implementation.
*/
static int magiron_run(void)
{
	/* sensor-interfacing variables */
	sensor_event_t accelEvt, magEvt, gyroEvt;
	vec_t gyroBias, angle = { 0 };

	/* measurement/fitting variables */
	fit_lma_t lma;
	vec_t measAvg;
	time_t lastT, currT;
	float deltaT, measAvgLen;
	int i;

	/* final data variables */
	float dataSfinal[9];
	matrix_t Sfinal = { .data = dataSfinal, .rows = 3, .cols = 3, .transposed = 0 };
	vec_t hFinal;

	printf("Do not rotate the device for 1s after pressing [Enter]\n");
	printf("Press [Enter] to continue...");
	fflush(stdout);
	getchar();

	/* Calculating gyroscope bias */
	for (i = 0; i < 1000; i++) {
		if (sensc_imuGet(&accelEvt, &gyroEvt, &magEvt) < 0) {
			fprintf(stderr, "%s: sensc_imuGet() fail\n", MAGIRON_TAG);
			return -1;
		}

		/* Conversion from mrad/s -> rad/s */
		gyroBias.x += gyroEvt.gyro.gyroX / 1000.f;
		gyroBias.y += gyroEvt.gyro.gyroY / 1000.f;
		gyroBias.z += gyroEvt.gyro.gyroZ / 1000.f;
		usleep(1000);
	}
	vec_times(&gyroBias, 0.001);

	printf("Rotate the device until all samples are taken\n");
	printf("Press [Enter] to begin sampling...");
	fflush(stdout);
	getchar();
	printf("Rotate...\n");

	/*
	* Taking samples each time we rotate more than DANGLE_MIN radians.
	* Angle integration implemented to assure correct angles of rotation.
	*
	* Assumed constant gyroscope bias and neglecting gyroscope drift.
	*/
	gettime(&lastT, NULL);
	currT = lastT;
	i = 0;
	while (i < MAX_SAMPLES) {
		usleep(1000);
		gettime(&currT, NULL);

		if (sensc_imuGet(&accelEvt, &gyroEvt, &magEvt) < 0) {
			fprintf(stderr, "%s: sensc_imuGet() fail\n", MAGIRON_TAG);
		}

		deltaT = (float)(currT - lastT) / 1000000.f;

		/* Conversion from mrad/s -> rad/s */
		angle.x += (gyroEvt.gyro.gyroX / 1000.f - gyroBias.x) * deltaT;
		angle.y += (gyroEvt.gyro.gyroY / 1000.f - gyroBias.y) * deltaT;
		angle.z += (gyroEvt.gyro.gyroZ / 1000.f - gyroBias.z) * deltaT;

		if (vec_len(&angle) > DANGLE_MIN) {
			magiron_common.meas[i].x = magEvt.mag.magX;
			magiron_common.meas[i].y = magEvt.mag.magY;
			magiron_common.meas[i].z = magEvt.mag.magZ;

			if (i % (MAX_SAMPLES / 100) == (MAX_SAMPLES / 100) - 1) {
				printf("%s: Taken samples: %u/%u\n", MAGIRON_TAG, i, MAX_SAMPLES);
			}

			angle = (vec_t) { .x = 0, .y = 0, .z = 0 };
			i++;
		}

		lastT = currT;
	}

	/* Calculate mean offset of samples (rough estimate of ellipsoid center) */
	measAvg = (vec_t) { .x = 0, .y = 0, .z = 0 };
	for (i = 0; i < MAX_SAMPLES; i++) {
		vec_add(&measAvg, &magiron_common.meas[i]);
	}
	vec_times(&measAvg, 1.f / MAX_SAMPLES);

	/* Shift samples by 'measAvg' and calculate their new mean distance from (0,0,0) */
	measAvgLen = 0;
	for (i = 0; i < MAX_SAMPLES; i++) {
		vec_sub(&magiron_common.meas[i], &measAvg);
		measAvgLen += vec_len(&magiron_common.meas[i]);
	}
	measAvgLen /= (float)MAX_SAMPLES;

	/* Normalize shifted measurements to be spaced "around" (0,0,0)+-(1,1,1) */
	for (i = 0; i < MAX_SAMPLES; i++) {
		vec_times(&magiron_common.meas[i], 1.f / measAvgLen);
	}

	if (lma_init(3, 12, MAX_SAMPLES, magiron_lmaJacobian, magiron_lmaResiduum, magiron_lmaGuess, &lma) < 0) {
		printf("%s: failed to init LMA\n", MAGIRON_TAG);
		return -1;
	}

	/* Write prepared measurements to lma `samples` matrix */
	for (i = 0; i < MAX_SAMPLES; i++) {
		MATRIX_DATA(&lma.samples, i, 0) = magiron_common.meas[i].x;
		MATRIX_DATA(&lma.samples, i, 1) = magiron_common.meas[i].y;
		MATRIX_DATA(&lma.samples, i, 2) = magiron_common.meas[i].z;
	}

	lma_fit(LMA_FITTING_EPOCHS, &lma, LMALOG_NONE);

	/*
	* Measurements used in fitting were shifted and scaled:
	* m_fit = (m_raw - avg) / avgLen(m_shift)
	*
	* Fitting the ellipsoid gives us:
	* - soft iron correction matrix S
	* - hard iron correction vector h
	*
	* Final correction parameters have to include `avg` and `avgLen(m_shift)` values:
	* S_final = S / avgLen(m_shift)
	* h_final = avg + h * avgLen(m_shift)
	*
	* DISCLAIMER: we want S matrix to scale data as low as possible
	* so we IGNORE division by avgLen(m_shift), thus preserving the magnitude.
	*/
	MATRIX_DATA(&Sfinal, 0, 0) = MATRIX_DATA(&lma.paramsVec, 0, 0);
	MATRIX_DATA(&Sfinal, 0, 1) = MATRIX_DATA(&lma.paramsVec, 0, 1);
	MATRIX_DATA(&Sfinal, 0, 2) = MATRIX_DATA(&lma.paramsVec, 0, 2);
	MATRIX_DATA(&Sfinal, 1, 0) = MATRIX_DATA(&lma.paramsVec, 0, 3);
	MATRIX_DATA(&Sfinal, 1, 1) = MATRIX_DATA(&lma.paramsVec, 0, 4);
	MATRIX_DATA(&Sfinal, 1, 2) = MATRIX_DATA(&lma.paramsVec, 0, 5);
	MATRIX_DATA(&Sfinal, 2, 0) = MATRIX_DATA(&lma.paramsVec, 0, 6);
	MATRIX_DATA(&Sfinal, 2, 1) = MATRIX_DATA(&lma.paramsVec, 0, 7);
	MATRIX_DATA(&Sfinal, 2, 2) = MATRIX_DATA(&lma.paramsVec, 0, 8);

	hFinal.x = MATRIX_DATA(&lma.paramsVec, 0, 9);
	hFinal.y = MATRIX_DATA(&lma.paramsVec, 0, 10);
	hFinal.z = MATRIX_DATA(&lma.paramsVec, 0, 11);

	/* 
	 * This is left commented intentionally - we should scale S matrix by avgLen(m_shift) 
	 * but we ignore it according to DISCLAIMER above.
	*/
	/* matrix_times(&Sfinal, 1.f / measAvgLen); */

	vec_times(&hFinal, measAvgLen);
	vec_add(&hFinal, &measAvg);

	/* Vaidity check: offset should be inside expected limits */
	if (vec_len(&hFinal) > MAX_HARDIRON_LENGTH) {
		fprintf(stderr, "%s: hard iron exceeds expectations: x:%f y:%f z:%f\n\n", MAGIRON_TAG, hFinal.x, hFinal.y, hFinal.z);
		return -1;
	}

	/* Vaidity check: S matrix should not invert data */
	if (MATRIX_DATA(&Sfinal, 0, 0) <= 0 || MATRIX_DATA(&Sfinal, 1, 1) <= 0 || MATRIX_DATA(&Sfinal, 2, 2) <= 0) {
		fprintf(stderr, "%s: invalid transformation! Diag: %f %f %f\n", MAGIRON_TAG, MATRIX_DATA(&Sfinal, 0, 0), MATRIX_DATA(&Sfinal, 1, 1), MATRIX_DATA(&Sfinal, 2, 2));
		return -1;
	}

	/* Saving calibration parameters to common structure */
	matrix_writeSubmatrix(&magiron_common.data.params.magiron.softCal, 0, 0, &Sfinal);
	MATRIX_DATA(&magiron_common.data.params.magiron.hardCal, 0, 0) = hFinal.x;
	MATRIX_DATA(&magiron_common.data.params.magiron.hardCal, 1, 0) = hFinal.y;
	MATRIX_DATA(&magiron_common.data.params.magiron.hardCal, 2, 0) = hFinal.z;

	printf("Hard iron: %fmG %fmG %fmG\n", hFinal.x, hFinal.y, hFinal.z);
	printf("Soft iron:\n");
	matrix_print(&Sfinal);

	return 0;
}


static int magiron_init(int argc, const char **argv)
{
	if (sensc_init(SENSOR_PATH, false) < 0) {
		return -1;
	}

	magiron_common.meas = calloc(MAX_SAMPLES, sizeof(vec_t));
	if (magiron_common.meas == NULL) {
		printf("%s: memory allocation fail\n", MAGIRON_TAG);
		sensc_deinit();
		return -1;
	}

	return EOK;
}


__attribute__((constructor(102))) static void magiron_register(void)
{
	static calib_ops_t cal = {
		.name = MAGIRON_TAG,
		.init = magiron_init,
		.run = magiron_run,
		.done = magiron_done,
		.write = magiron_write,
		.help = magiron_help,
		.dataGet = magiron_dataGet
	};

	calib_register(&cal);

	magiron_common.data.type = typeMagiron;
}
