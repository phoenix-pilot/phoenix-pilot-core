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

#include <matrix.h>
#include <vec.h>

#include "calib.h"


#define CHAR_HARDIRON 'h'
#define CHAR_SOFTIRON 's'

#define SOFTCAL_ROWSPAN 3
#define SOFTCAL_COLSPAN 3
#define HARDCAL_ROWSPAN 3
#define HARDCAL_COLSPAN 1


struct {
	/* Calibration parameters */
	matrix_t softCal;
	matrix_t hardCal;

	/* Utility variables */
	float softCalBuf[SOFTCAL_ROWSPAN * SOFTCAL_COLSPAN];
	float hardCalBuf[HARDCAL_ROWSPAN * HARDCAL_COLSPAN];
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


static int cal_magironInterpret(const char *name, float val)
{
	float *valSlot;

	valSlot = magiron_paramSlot(name);
	if (valSlot == NULL) {
		return -ENOENT;
	}

	*valSlot = val;

	return EOK;
}


static const char *cal_magironHelp(void)
{
	return "Magnetometer calibration against soft/hard iron interference.\n";
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


static void cal_magironPreinit(void)
{
	/* Soft iron calibration matrix init */
	magiron_common.softCal.cols = SOFTCAL_COLSPAN;
	magiron_common.softCal.rows = SOFTCAL_ROWSPAN;
	magiron_common.softCal.transposed = 0;
	magiron_common.softCal.data = magiron_common.softCalBuf;
	matrix_diag(&magiron_common.softCal);

	/* hard iron calibration matrix init */
	magiron_common.hardCal.cols = HARDCAL_COLSPAN;
	magiron_common.hardCal.rows = HARDCAL_ROWSPAN;
	magiron_common.hardCal.transposed = 0;
	magiron_common.hardCal.data = magiron_common.hardCalBuf;
	matrix_zeroes(&magiron_common.hardCal);

	/*
	* FIXME: Precalibrated data injection.
	* This must be removed when full procedure is implemented
	*/

	/* hard iron vector matrix */
	magiron_common.hardCal.data[0] = 42.47503636;
	magiron_common.hardCal.data[1] = 1084.20661751;
	magiron_common.hardCal.data[2] = -111.58247011;

	/* soft iron matrix */
	magiron_common.softCal.data[0] = 0.9409439;
	magiron_common.softCal.data[1] = 0.09766692;
	magiron_common.softCal.data[2] = -0.01307758;
	magiron_common.softCal.data[3] = 0.09766692;
	magiron_common.softCal.data[4] = 1.01364504;
	magiron_common.softCal.data[5] = -0.01144832;
	magiron_common.softCal.data[6] = -0.01307758;
	magiron_common.softCal.data[7] = -0.01144832;
	magiron_common.softCal.data[8] = 1.0593312;
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
		.help = cal_magironHelp
	};

	calib_register(&cal);

	cal_magironPreinit();
}
