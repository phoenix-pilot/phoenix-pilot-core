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

#include "../calibcore.h"
#include "magiron.h"


#define CHAR_HARDIRON 'h'
#define CHAR_SOFTIRON 's'


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


int magiron_write(FILE *file)
{
	/* Printing hard iron calibration parameters */
	magiron_printIron(file, CHAR_HARDIRON, &magiron_common.hardCal);

	/* Printing soft iron calibration parameters */
	magiron_printIron(file, CHAR_SOFTIRON, &magiron_common.softCal);

	return 0;
}


int magiron_interpret(const char *name, float val)
{
	float *valSlot;

	valSlot = magiron_paramSlot(name);
	if (valSlot == NULL) {
		return -ENOENT;
	}

	*valSlot = val;

	return EOK;
}


void magiron_preinit(void)
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


const char *magiron_help(void)
{
	return "Magnetometer calibration against soft/hard iron interference.\n";
}
