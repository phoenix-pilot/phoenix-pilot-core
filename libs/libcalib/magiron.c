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

#include <board_config.h>
#include <libsensors.h>

#include <matrix.h>
#include <vec.h>

#include "calib.h"


#define CHAR_HARDIRON 'h'
#define CHAR_SOFTIRON 's'


struct {
	/* Calibration parameters */
	matrix_t softCal;
	matrix_t hardCal;

	/* Utility variables */
	float softCalBuf[9];
	float hardCalBuf[3];
} magiron_common;


/* returns pointer do data slot named as 'paramName' */
static float *magiron_paramSlot(const char *paramName)
{
	matrix_t *mat;
	unsigned int row, col;

	if (strlen(paramName) != 3) {
		return NULL;
	}

	/* variable casting for MISRA compliance */
	row = (uint8_t)(paramName[1] - '0'); /* convert character to unsigned int digit */
	col = (uint8_t)(paramName[2] - '0'); /* convert character to unsigned int digit */
	if (row > (unsigned int)'9' || col > (unsigned int)'9') {
		return NULL;
	}

	/* matrix type get through character check */
	switch (paramName[0]) {
		case CHAR_SOFTIRON:
			mat = &magiron_common.softCal;
			break;
		case CHAR_HARDIRON:
			mat = &magiron_common.softCal;
			break;
		default:
			return NULL;
	}

	/* matrix boundary checks performed by matrix_at() */
	return matrix_at(mat, row, col);
}


/* Prints to `file` data from `mat` as calibtype `type` */
static void magiron_printIron(FILE *file, char type, matrix_t *mat)
{
	unsigned int rows, cols, r, c;

	cols = matrix_colsGet(mat);
	rows = matrix_rowsGet(mat);

	for (r = 0; r < rows; r++) {
		for (c = 0; c < cols; c++) {
			fprintf(file, "%c%i%i %f\n", type, r, c, *matrix_at(mat, r, c));
		}
	}
}


static int cal_magironWrite(FILE *file)
{
	/* Printing hard iron calibration parameters */
	magiron_printIron(file, CHAR_HARDIRON, &magiron_common.hardCal);

	/* Printing soft iron calibration parameters */
	magiron_printIron(file, CHAR_SOFTIRON, &magiron_common.softCal);

	return 0;
}


static int cal_magironInterpret(const char *valName, float val)
{
	float *valSlot;

	valSlot = magiron_paramSlot(valName);
	if (valSlot == NULL) {
		return -ENOENT;
	}

	*valSlot = val;

	return EOK;
}


static const char *cal_magironHelp(void)
{
	return "  Magnetometer calibration against soft/hard iron interference.\n";
}


static int cal_magironDone(void)
{
	/* Stub implementation. Calibration procedure returns precompiled data */
	return EOK;
}


static int cal_magironRun(void)
{
	printf("This calibration procedure is not implemented and it returns precalculated values!\n Press enter to continue...\n");
	getchar();
	fflush(stdin);

	return EOK;
}


static int cal_magironInit(int argc, const char **argv)
{
	/* Stub implementation. Calibration procedure returns precompiled data */
	return EOK;
}


static int corr_magironDo(sensor_event_t *evt)
{
	float measBuf[3] = { evt->mag.magX, evt->mag.magY, evt->mag.magZ};
	float corrBuff[3];
	matrix_t meas = { .data = measBuf, .rows = 3, .cols = 1, .transposed = 0};
	matrix_t corr = { .data = corrBuff, .rows = 3, .cols = 1, .transposed = 0};

	/* precompiled values need to be subtracted, as they are error, not correction */
	matrix_sub(&meas, &magiron_common.hardCal, NULL);
	matrix_prod(&magiron_common.softCal, &measBuf, &corr);

	evt->mag.magX = *matrix_at(&corr, 0, 0);
	evt->mag.magY = *matrix_at(&corr, 1, 0);
	evt->mag.magZ = *matrix_at(&corr, 2, 0);

	return 0;
}


__attribute__((constructor(102))) static void cal_magironRegister(void)
{
	static calib_t cal = {
		.name = "magiron",
		.init = cal_magironInit,
		.run = cal_magironRun,
		.done = cal_magironDone,
		.interpret = cal_magironInterpret,
		.write = cal_magironWrite,
		.help = cal_magironHelp,

		.cInit = NULL,
		.cRecalc = NULL,
		.cDone = NULL,
		.cDo = corr_magironDo,
		.delay = 0
	};

	calib_register(&cal);

	/* Soft iron calibration matrix init */
	magiron_common.softCal.cols = 3;
	magiron_common.softCal.rows = 3;
	magiron_common.softCal.transposed = 0;
	magiron_common.softCal.data = magiron_common.softCalBuf;
	matrix_diag(&magiron_common.softCal);

	/* hard iron calibration matrix init */
	magiron_common.hardCal.rows = 3;
	magiron_common.hardCal.cols = 1;
	magiron_common.hardCal.transposed = 0;
	magiron_common.hardCal.data = magiron_common.hardCalBuf;
	matrix_zeroes(&magiron_common.hardCal);

	/*
	* FIXME: Precalibrated data injection.
	* This must be removed when full procedure is implemented
	*/

	/* hard iron vector matrix */
	*matrix_at(&magiron_common.hardCal, 0, 0) = 42.47503636;
	*matrix_at(&magiron_common.hardCal, 1, 0) = 1084.20661751;
	*matrix_at(&magiron_common.hardCal, 2, 0) = -111.58247011;

	/* soft iron matrix */
	*matrix_at(&magiron_common.softCal, 0, 0) = 0.9409439;
	*matrix_at(&magiron_common.softCal, 0, 1) = 0.09766692;
	*matrix_at(&magiron_common.softCal, 0, 2) = -0.01307758;
	*matrix_at(&magiron_common.softCal, 1, 0) = 0.09766692;
	*matrix_at(&magiron_common.softCal, 1, 1) = 1.01364504;
	*matrix_at(&magiron_common.softCal, 1, 2) = -0.01144832;
	*matrix_at(&magiron_common.softCal, 2, 0) = -0.01307758;
	*matrix_at(&magiron_common.softCal, 2, 1) = -0.01144832;
	*matrix_at(&magiron_common.softCal, 2, 2) = 1.0593312;
}
