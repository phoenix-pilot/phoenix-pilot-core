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
#include "ellcal.h"


#define DANGLE_MIN          (M_PI / 18) /* Minimum delta angle to take sample */
#define MAX_SAMPLES         256         /* Samples to be taken for ellipsoid fitting */
#define MAX_HARDIRON_LENGTH 5000        /* Maximum acceptable hard iron offset */
#define LMA_FITTING_EPOCHS  20          /* LMA epochs for parameters fitting */
#define LMA_JACOBIAN_STEP   0.0001f     /* Step for jacobian calculation in LMA */

#define MAX_U32_DELTAANGLE 0x7fffffff


struct {
	calib_data_t data;
	vec_t *meas;
} magiron_common;


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
	/* measurement/fitting variables */
	fit_lma_t lma;
	vec_t measAvg;
	float measAvgLen;
	int i;

	/* final data variables */
	float dataSfinal[9], dataHfinal[3];
	matrix_t Sfinal = { .data = dataSfinal, .rows = 3, .cols = 3, .transposed = 0 };
	matrix_t hFinal = { .data = dataHfinal, .rows = 3, .cols = 1, .transposed = 0 };

	/* Obtain rotational samples of magnetometer */
	if (ellcal_rotDataGet(magiron_common.meas, MAX_SAMPLES, DANGLE_MIN, SENSOR_TYPE_MAG) < 0) {
		return -1;
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

	if (lma_init(3, 12, MAX_SAMPLES, ellcal_lmaJacobian, ellcal_lmaResiduum, ellcal_lmaGuess, &lma) < 0) {
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

	ellcal_lma2matrices(&lma.paramsVec, &Sfinal, &hFinal);

	lma_done(&lma);

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
	* The scaling of matrix S is left commented intentionally.
	*/
	/* matrix_times(&Sfinal, 1.f / measAvgLen); */
	matrix_times(&hFinal, measAvgLen);
	MATRIX_DATA(&hFinal, 0, 0) += measAvg.x;
	MATRIX_DATA(&hFinal, 1, 0) += measAvg.y;
	MATRIX_DATA(&hFinal, 2, 0) += measAvg.z;

	measAvgLen = MATRIX_DATA(&hFinal, 0, 0) * MATRIX_DATA(&hFinal, 0, 0);
	measAvgLen += MATRIX_DATA(&hFinal, 1, 0) * MATRIX_DATA(&hFinal, 1, 0);
	measAvgLen += MATRIX_DATA(&hFinal, 2, 0) * MATRIX_DATA(&hFinal, 2, 0);
	measAvgLen = sqrt(measAvgLen);

	/* Vaidity check: offset should be inside expected limits */
	if (measAvgLen > MAX_HARDIRON_LENGTH) {
		fprintf(stderr, "%s: hard iron exceeds expectations: x:%f y:%f z:%f\n\n", MAGIRON_TAG, MATRIX_DATA(&hFinal, 0, 0), MATRIX_DATA(&hFinal, 1, 0), MATRIX_DATA(&hFinal, 2, 0));
		return -1;
	}

	/* Vaidity check: S matrix should not invert data */
	if (MATRIX_DATA(&Sfinal, 0, 0) <= 0 || MATRIX_DATA(&Sfinal, 1, 1) <= 0 || MATRIX_DATA(&Sfinal, 2, 2) <= 0) {
		fprintf(stderr, "%s: invalid transformation! Diag: %f %f %f\n", MAGIRON_TAG, MATRIX_DATA(&Sfinal, 0, 0), MATRIX_DATA(&Sfinal, 1, 1), MATRIX_DATA(&Sfinal, 2, 2));
		return -1;
	}

	/* Saving calibration parameters to common structure */
	matrix_writeSubmatrix(&magiron_common.data.params.magiron.softCal, 0, 0, &Sfinal);
	matrix_writeSubmatrix(&magiron_common.data.params.magiron.hardCal, 0, 0, &hFinal);

	printf("%s: Hard iron: %fmG %fmG %fmG\n", MAGIRON_TAG, MATRIX_DATA(&hFinal, 0, 0), MATRIX_DATA(&hFinal, 1, 0), MATRIX_DATA(&hFinal, 2, 0));
	printf("%s: Soft iron:\n", MAGIRON_TAG);
	matrix_print(&Sfinal);

	return 0;
}


static int magiron_init(int argc, const char **argv)
{
	if (sensc_init(SENSOR_PATH, CORR_ENBL_NONE, SENSC_INIT_IMU) < 0) {
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
